"""Plan Validator.

Every plan produced by the LLM is checked here BEFORE any motion is executed.
The validator is pure Python (no ROS) and works on a *copy* of the
camera-derived SceneState, symbolically simulating each step.

Checks
------
1. Schema      : JSON object with a non-empty "plan" list (<= MAX_STEPS),
                 each step has a whitelisted "skill" and exactly the
                 required/optional parameters.  Any joint-level content
                 (joints, trajectory, positions, velocities ...) is rejected:
                 the LLM may only choose skills and their symbolic arguments.
2. Symbols     : objects/zones/temp-position names must exist; objects to be
                 picked must currently be visible to the camera.
3. Preconditions (symbolic simulation):
                 - pick   : gripper empty, object visible & reachable,
                            nothing stacked on top (not modelled -> n/a)
                 - place  : holding exactly that object, target zone EMPTY at
                            that moment, or target is a temp position created
                            by find_free_position, target reachable
                 - check_zone / find_object : argument valid
                 - find_free_position : a collision-free spot must exist
                            (computed with the *simulated* state so several
                            temp positions never overlap)
4. Postconditions: gripper must be empty at the end; plan should end with
                 home() (auto-appended with a warning if missing).
5. Goal        : if the LLM states a "goal" (list of {object, zone}), the
                 simulated final state must satisfy it.

The validator also "grounds" the plan: temp positions get concrete (x, y)
coordinates, which the executor then uses.
"""
import copy
import math
from typing import List, Tuple

from .world_model import (KNOWN_OBJECTS, KNOWN_ZONES, ObjectState, SceneState,
                          find_free_position, in_workspace)

MAX_STEPS = 25

# skill -> (required params, optional params)
SKILLS = {
    'detect_objects': (set(), set()),
    'check_zone': ({'zone'}, set()),
    'find_object': ({'object'}, set()),
    'find_free_position': ({'name'}, {'object'}),
    'pick': ({'object'}, set()),
    'place': ({'object', 'target'}, set()),
    'home': (set(), set()),
}

FORBIDDEN_KEYS = {'joints', 'joint', 'joint_positions', 'positions', 'trajectory',
                  'velocities', 'effort', 'q', 'pose', 'xyz', 'x', 'y', 'z'}


class ValidationResult:
    def __init__(self):
        self.errors: List[str] = []
        self.warnings: List[str] = []
        self.plan: List[dict] = []        # grounded plan
        self.final_scene = None

    @property
    def ok(self) -> bool:
        return not self.errors

    def report(self) -> str:
        lines = [f'VALID: {self.ok}']
        lines += [f'  ERROR: {e}' for e in self.errors]
        lines += [f'  WARN : {w}' for w in self.warnings]
        return '\n'.join(lines)


def _fmt_step(i: int, s: dict) -> str:
    args = ', '.join(f'{k}={v}' for k, v in s.items() if k not in ('skill', 'resolved'))
    return f'step {i + 1} {s.get("skill")}({args})'


def validate_plan(plan_json, scene: SceneState, cfg: dict) -> ValidationResult:
    res = ValidationResult()

    # ------------------------------------------------------------ 1. schema
    if not isinstance(plan_json, dict) or not isinstance(plan_json.get('plan'), list):
        res.errors.append('Output must be a JSON object with a "plan" list.')
        return res
    steps = plan_json['plan']
    if not steps:
        res.errors.append('Plan is empty.')
        return res
    if len(steps) > MAX_STEPS:
        res.errors.append(f'Plan has {len(steps)} steps (max {MAX_STEPS}).')
        return res

    clean: List[dict] = []
    for i, s in enumerate(steps):
        if not isinstance(s, dict) or 'skill' not in s:
            res.errors.append(f'step {i + 1}: must be an object with a "skill" field.')
            continue
        skill = s['skill']
        if skill not in SKILLS:
            res.errors.append(f'step {i + 1}: unknown skill "{skill}". Allowed: {sorted(SKILLS)}.')
            continue
        bad = FORBIDDEN_KEYS.intersection(s.keys())
        if bad:
            res.errors.append(f'step {i + 1}: forbidden low-level fields {sorted(bad)} - the '
                              'planner may not output joint values, trajectories or coordinates.')
            continue
        req, opt = SKILLS[skill]
        params = set(s.keys()) - {'skill'}
        missing, extra = req - params, params - req - opt
        if missing:
            res.errors.append(f'{_fmt_step(i, s)}: missing parameter(s) {sorted(missing)}.')
        if extra:
            res.errors.append(f'{_fmt_step(i, s)}: unexpected parameter(s) {sorted(extra)}.')
        if any(not isinstance(s[p], str) for p in params):
            res.errors.append(f'{_fmt_step(i, s)}: parameters must be strings.')
        clean.append({k: (v.strip().lower() if isinstance(v, str) else v) for k, v in s.items()})
    if res.errors:
        return res

    # ------------------------------------------- 2-4. symbolic simulation
    sim = scene.copy()
    radius = cfg['zone_occupancy_radius']
    holding = None
    temps = {}                              # name -> (x, y)
    grounded: List[dict] = []

    for i, s in enumerate(clean):
        skill = s['skill']
        tag = _fmt_step(i, s)
        g = copy.deepcopy(s)

        if skill in ('detect_objects', 'home'):
            pass

        elif skill == 'check_zone':
            if s['zone'] not in KNOWN_ZONES:
                res.errors.append(f'{tag}: unknown zone "{s["zone"]}". Known: {KNOWN_ZONES}.')

        elif skill == 'find_object':
            if s['object'] not in KNOWN_OBJECTS:
                res.errors.append(f'{tag}: unknown object "{s["object"]}".')
            elif s['object'] not in sim.objects and s['object'] != holding:
                res.errors.append(f'{tag}: "{s["object"]}" is not visible to the camera.')

        elif skill == 'find_free_position':
            name = s['name']
            if name in KNOWN_ZONES or name in KNOWN_OBJECTS:
                res.errors.append(f'{tag}: temp position name "{name}" clashes with a zone/object.')
            else:
                mover = s.get('object') or holding
                near = None
                if mover and mover in sim.objects:
                    near = (sim.objects[mover].x, sim.objects[mover].y)
                pos = find_free_position(sim, cfg, moving_object=mover,
                                         reserved=list(temps.values()), prefer_near=near)
                if pos is None:
                    res.errors.append(f'{tag}: no collision-free free position on the table.')
                else:
                    temps[name] = pos
                    g['resolved'] = list(pos)

        elif skill == 'pick':
            obj = s['object']
            if obj not in KNOWN_OBJECTS:
                res.errors.append(f'{tag}: unknown object "{obj}".')
            elif holding is not None:
                res.errors.append(f'{tag}: gripper already holds "{holding}" - place it first.')
            elif obj not in sim.objects:
                res.errors.append(f'{tag}: "{obj}" is not visible to the camera / not on the table.')
            else:
                o = sim.objects[obj]
                ok, why = in_workspace(o.x, o.y, cfg)
                if not ok:
                    res.errors.append(f'{tag}: "{obj}" unreachable: {why}.')
                else:
                    g['resolved'] = [round(o.x, 4), round(o.y, 4), round(o.yaw, 4)]
                    holding = obj
                    del sim.objects[obj]
                    sim.recompute_occupancy(radius)

        elif skill == 'place':
            obj, tgt = s['object'], s['target']
            xy, err = None, None
            if holding != obj:
                err = f'robot is holding "{holding}", not "{obj}".'
            elif tgt in KNOWN_ZONES:
                occ = sim.check_zone(tgt)
                if occ is not None:
                    err = (f'{tgt} is occupied by "{occ}" at this point of the plan - move '
                           f'"{occ}" away (e.g. pick it and place it at a find_free_position '
                           'result) first.')
                else:
                    xy = (sim.zones[tgt].x, sim.zones[tgt].y)
            elif tgt in temps:
                xy = temps[tgt]
            else:
                err = (f'unknown target "{tgt}" (must be a zone or a name created earlier by '
                       'find_free_position).')
            if err is None:
                ok, why = in_workspace(xy[0], xy[1], cfg)
                if not ok:
                    err = f'target unreachable: {why}.'
            if err is not None:
                res.errors.append(f'{tag}: {err}')
            else:
                g['resolved'] = [round(xy[0], 4), round(xy[1], 4)]
                sim.objects[obj] = ObjectState(name=obj, x=xy[0], y=xy[1])
                sim.recompute_occupancy(radius)
                holding = None
        grounded.append(g)

    if res.errors:
        return res

    if holding is not None:
        res.errors.append(f'Plan ends while still holding "{holding}".')
    if not grounded or grounded[-1]['skill'] != 'home':
        res.warnings.append('Plan did not end with home(); home() appended automatically.')
        grounded.append({'skill': 'home'})

    # ------------------------------------------------------------- 5. goal
    for gl in plan_json.get('goal', []) or []:
        if not isinstance(gl, dict):
            continue
        obj, zone = str(gl.get('object', '')).lower(), str(gl.get('zone', '')).lower()
        if obj and zone:
            o = sim.objects.get(obj)
            if o is None or o.zone != zone:
                res.errors.append(f'Goal not achieved by plan: {obj} should end in {zone} '
                                  f'(simulated: {o.zone if o else "missing"}).')

    res.plan = grounded
    res.final_scene = sim
    return res


def plan_to_text(plan: List[dict]) -> str:
    out = []
    for i, s in enumerate(plan):
        args = ', '.join(str(s[k]) for k in ('object', 'zone', 'name', 'target') if k in s)
        extra = f'   -> {s["resolved"]}' if 'resolved' in s else ''
        out.append(f'  {i + 1:2d}. {s["skill"]}({args}){extra}')
    return '\n'.join(out)


def distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])
