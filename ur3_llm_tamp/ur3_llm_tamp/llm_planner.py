"""LLM Planner: natural-language command + camera scene  ->  structured skill plan.

The LLM is ONLY used to (1) understand the command, (2) choose skills,
(3) fill in symbolic parameters and (4) order the steps.  It never sees or
produces joint values / trajectories; motion is produced by MoveIt 2 inside
the robot skills.  Its output is always checked by plan_validator before
execution, and validator errors are fed back to the LLM for re-planning.

Backends
--------
- "openai"  : any OpenAI-compatible /chat/completions endpoint
              (OpenRouter (default), OpenAI, Groq, Gemini-OpenAI, Ollama ...)
              env: LLM_API_KEY (or OPENROUTER_API_KEY / OPENAI_API_KEY),
                   LLM_BASE_URL, LLM_MODEL
- "offline" : tiny rule-based parser for testing WITHOUT network.  It is
              NOT an LLM and is clearly labelled as such in the logs.
"""
import json
import os
import re
from typing import Callable, List, Optional

import requests

from .world_model import KNOWN_OBJECTS, KNOWN_ZONES, SceneState

SYSTEM_PROMPT = f"""You are the task planner of a UR3 robot arm with a parallel gripper and an
overhead camera, working on a table with 3 zones and 5 cubes.

You must translate the user's command into a sequence of HIGH-LEVEL robot
skills. You never output joint values, coordinates or trajectories - the
robot computes motions itself with MoveIt 2.

Objects: {', '.join(KNOWN_OBJECTS)}
Zones:   {', '.join(KNOWN_ZONES)}   (each zone holds at most ONE cube)

Available skills (JSON step format):
  {{"skill": "detect_objects"}}                         refresh camera scene
  {{"skill": "check_zone", "zone": "<zone>"}}           verify zone state with the camera
  {{"skill": "find_object", "object": "<object>"}}      locate a cube with the camera
  {{"skill": "find_free_position", "name": "<temp_name>", "object": "<object>"}}
        compute an empty spot on the table (outside all zones) to park <object>;
        <temp_name> (e.g. "temp_1") can then be used as a place target
  {{"skill": "pick", "object": "<object>"}}             grasp a cube (gripper must be empty)
  {{"skill": "place", "object": "<object>", "target": "<zone or temp_name>"}}
        put the held cube down; a zone target MUST be empty at that moment
  {{"skill": "home"}}                                   go to the safe home pose

Rules:
1. Start with detect_objects. Use check_zone on every zone you will place into.
2. If a target zone is OCCUPIED by another cube, first clear it: find_free_position
   for the blocking cube, pick it, place it at that temp position. Only then
   pick and place the requested cube. If the requested cube is already in the
   target zone, do nothing except home.
3. Never pick while holding a cube. Always place what you picked.
4. Only manipulate cubes that the camera reports as visible.
5. Finish with {{"skill": "home"}}.
6. Answer with ONLY one JSON object, no markdown, in this exact format:
{{
  "reasoning": "<one short sentence>",
  "goal": [{{"object": "<object>", "zone": "<zone>"}}],
  "plan": [ <steps> ]
}}
"goal" lists the final object-in-zone relations the user asked for.
"""

EXAMPLE_USER = """Scene (from camera):
Objects detected by the camera:
  - blue_cube: zone_b
  - red_cube: table (0.21, 0.11)
Zones:
  - zone_a: EMPTY
  - zone_b: OCCUPIED by blue_cube
  - zone_c: EMPTY

Command: Put the red cube in Zone B."""

EXAMPLE_ASSISTANT = json.dumps({
    'reasoning': 'zone_b holds blue_cube, so park blue_cube on free table space first.',
    'goal': [{'object': 'red_cube', 'zone': 'zone_b'}],
    'plan': [
        {'skill': 'detect_objects'},
        {'skill': 'check_zone', 'zone': 'zone_b'},
        {'skill': 'find_free_position', 'name': 'temp_1', 'object': 'blue_cube'},
        {'skill': 'pick', 'object': 'blue_cube'},
        {'skill': 'place', 'object': 'blue_cube', 'target': 'temp_1'},
        {'skill': 'pick', 'object': 'red_cube'},
        {'skill': 'place', 'object': 'red_cube', 'target': 'zone_b'},
        {'skill': 'home'},
    ]})


def extract_json(text: str) -> Optional[dict]:
    """Robustly pull the first JSON object out of an LLM answer."""
    text = re.sub(r'```(?:json)?', '', text).strip()
    try:
        return json.loads(text)
    except Exception:
        pass
    start = text.find('{')
    while start != -1:
        depth = 0
        for i in range(start, len(text)):
            if text[i] == '{':
                depth += 1
            elif text[i] == '}':
                depth -= 1
                if depth == 0:
                    try:
                        return json.loads(text[start:i + 1])
                    except Exception:
                        break
        start = text.find('{', start + 1)
    return None


class LLMPlanner:
    def __init__(self, backend: str = 'openrouter', model: Optional[str] = None,
                 base_url: Optional[str] = None, api_key: Optional[str] = None,
                 temperature: float = 0.0, timeout: float = 60.0,
                 log: Callable[[str], None] = print):
        self.backend = backend
        self.model = model or os.environ.get('LLM_MODEL', 'xiaomi/mimo-v2.6-flash')
        self.base_url = (base_url or os.environ.get('LLM_BASE_URL', 'https://openrouter.ai/api/v1')
                         ).rstrip('/')
        self.api_key = api_key or os.environ.get('OPENROUTER_API_KEY') or \
            os.environ.get('LLM_API_KEY') or os.environ.get('OPENAI_API_KEY')
        self.temperature = temperature
        self.timeout = timeout
        self.log = log
        self.last_reasoning_details = None

        # Nếu chưa có API key và không yêu cầu explicit offline mode, hỏi từ terminal
        if self.backend not in ('offline',) and not self.api_key:
            try:
                print("\n" + "=" * 65)
                print("[LLM SETUP] Chưa tìm thấy OPENROUTER_API_KEY trong môi trường.")
                key_in = input("-> Nhập OpenRouter API Key (hoặc gõ 'offline' để dùng chế độ Offline): ").strip()
                print("=" * 65 + "\n")
                if key_in.lower() == 'offline' or not key_in:
                    self.backend = 'offline'
                    self.log("[LLM] Đã chuyển sang chế độ OFFLINE (Rule-based Mode).")
                else:
                    self.api_key = key_in
                    os.environ['OPENROUTER_API_KEY'] = key_in
                    self.backend = 'openrouter'
                    self.log(f"[LLM] Đã nhận API Key. Khởi tạo OpenRouter model: {self.model}")
            except (EOFError, KeyboardInterrupt):
                self.backend = 'offline'
                self.log("[LLM] Chuyển sang chế độ OFFLINE (Rule-based Mode).")

    # -------------------------------------------------------------- public
    def plan(self, command: str, scene: SceneState,
             feedback: Optional[List[str]] = None,
             previous: Optional[dict] = None) -> Optional[dict]:
        user = f'Scene (from camera):\n{scene.summary_text()}\n\nCommand: {command}'
        if self.backend == 'offline':
            return self._offline_plan(command, scene)
        messages = [
            {'role': 'system', 'content': SYSTEM_PROMPT},
            {'role': 'user', 'content': EXAMPLE_USER},
            {'role': 'assistant', 'content': EXAMPLE_ASSISTANT},
            {'role': 'user', 'content': user},
        ]
        if feedback:
            asst_msg = {'role': 'assistant', 'content': json.dumps(previous or {})}
            if self.last_reasoning_details:
                asst_msg['reasoning_details'] = self.last_reasoning_details
            messages.append(asst_msg)
            messages.append({'role': 'user', 'content':
                             'The plan validator REJECTED your plan:\n- ' + '\n- '.join(feedback)
                             + '\nReturn a corrected plan as ONE JSON object.'})
        return self._chat(messages)

    # ------------------------------------------------------------ backends
    def _chat(self, messages) -> Optional[dict]:
        url = f'{self.base_url}/chat/completions'
        body = {
            'model': self.model,
            'messages': messages,
            'temperature': self.temperature,
            'reasoning': {'enabled': True}
        }
        headers = {
            'Authorization': f'Bearer {self.api_key}',
            'Content-Type': 'application/json',
            'HTTP-Referer': 'https://github.com/ur3_llm_tamp',
            'X-Title': 'UR3 LLM TAMP'
        }
        try:
            r = requests.post(url, headers=headers, data=json.dumps(body), timeout=self.timeout)
        except Exception as e:
            self.log(f'LLM request failed: {e}')
            return None
        if r.status_code != 200:
            self.log(f'LLM HTTP {r.status_code}: {r.text[:300]}')
            return None
        try:
            resp_json = r.json()
            choice_msg = resp_json['choices'][0]['message']
            content = choice_msg.get('content', '')
            self.last_reasoning_details = choice_msg.get('reasoning_details')
        except Exception:
            self.log(f'Unexpected LLM response: {r.text[:300]}')
            return None
        self.log(f'LLM raw answer:\n{content}')
        plan = extract_json(content or '')
        if plan is None:
            self.log('Could not parse JSON from LLM answer.')
        return plan

    def _offline_plan(self, command: str, scene: SceneState) -> Optional[dict]:
        """Rule-based fallback (NOT an LLM): handles 'put/move/place <color> cube in zone <x>'."""
        self.log('[OFFLINE RULE-BASED PLANNER - not an LLM, for testing only]')
        cmd = command.lower()
        pairs = re.findall(r'(red|yellow|blue|green|purple)\s*(?:cube|block)?[^.;]*?'
                           r'(?:in|into|to|on|onto)\s*(?:the\s*)?zone[\s_]*([abc])', cmd)
        if not pairs:
            return {'reasoning': 'command not understood', 'goal': [], 'plan': [{'skill': 'home'}]}
        sim = scene.copy()
        plan = [{'skill': 'detect_objects'}]
        goal = []
        k = 0
        for color, z in pairs:
            obj, zone = f'{color}_cube', f'zone_{z}'
            goal.append({'object': obj, 'zone': zone})
            plan.append({'skill': 'check_zone', 'zone': zone})
            occ = sim.check_zone(zone)
            if occ == obj:
                continue
            if occ is not None:
                k += 1
                plan += [{'skill': 'find_free_position', 'name': f'temp_{k}', 'object': occ},
                         {'skill': 'pick', 'object': occ},
                         {'skill': 'place', 'object': occ, 'target': f'temp_{k}'}]
                sim.zones[zone].occupant = None
            plan += [{'skill': 'pick', 'object': obj},
                     {'skill': 'place', 'object': obj, 'target': zone}]
            for zz in sim.zones.values():
                if zz.occupant == obj:
                    zz.occupant = None
            sim.zones[zone].occupant = obj
        plan.append({'skill': 'home'})
        return {'reasoning': 'rule-based', 'goal': goal, 'plan': plan}

