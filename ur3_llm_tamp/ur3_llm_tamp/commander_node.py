#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import os
import sys

from ament_index_python.packages import get_package_share_directory

from ur3_llm_tamp.world_model import SceneState, load_scene_config
from ur3_llm_tamp.plan_validator import validate_plan, plan_to_text
from ur3_llm_tamp.llm_planner import LLMPlanner
from ur3_llm_tamp.robot_skills import UR3Skills

class LLMCommanderNode(Node):
    def __init__(self):
        super().__init__('llm_commander_node')

        # Load scene configuration
        pkg_path = get_package_share_directory('ur3_llm_tamp')
        cfg_path = os.path.join(pkg_path, 'config', 'scene.yaml')
        self.cfg = load_scene_config(cfg_path)

        # Declare parameters
        self.declare_parameter('llm_backend', 'openrouter')
        self.declare_parameter('llm_model', 'xiaomi/mimo-v2.6-flash')

        backend = self.get_parameter('llm_backend').value
        model = self.get_parameter('llm_model').value

        api_key = os.environ.get('OPENROUTER_API_KEY') or os.environ.get('LLM_API_KEY') or os.environ.get('OPENAI_API_KEY')
        if backend != 'offline' and not api_key:
            try:
                print("\n" + "=" * 65)
                print("[LLM SETUP] Chưa tìm thấy OPENROUTER_API_KEY trong môi trường.")
                user_key = input("-> Nhập OpenRouter API Key (hoặc gõ 'offline' để chạy offline): ").strip()
                print("=" * 65 + "\n")
                if user_key.lower() == 'offline' or not user_key:
                    backend = 'offline'
                    self.get_logger().info("Đã chuyển sang chế độ OFFLINE (Rule-based Mode).")
                else:
                    api_key = user_key
                    os.environ['OPENROUTER_API_KEY'] = user_key
                    self.get_logger().info(f"Đã nhận API Key. Model: {model}")
            except (EOFError, KeyboardInterrupt):
                backend = 'offline'
                self.get_logger().info("Chuyển sang chế độ OFFLINE.")

        self.get_logger().info(f"Initializing LLMPlanner (backend={backend}, model={model})...")
        self.planner = LLMPlanner(backend=backend, model=model, api_key=api_key, log=self.get_logger().info)

        # Subscribe to /scene_state
        self.latest_scene = None
        self.sub_scene = self.create_subscription(String, '/scene_state', self.scene_callback, 10)

        # Initialize robot skills
        self.skills = UR3Skills(node_name='ur3_commander_skills')
        self.get_logger().info("LLM Commander Node is ready!")

    def scene_callback(self, msg: String):
        try:
            self.latest_scene = SceneState.from_json(msg.data)
        except Exception as e:
            self.get_logger().error(f"Error parsing /scene_state: {e}")

    def wait_for_scene(self, timeout_sec: float = 10.0) -> bool:
        start = time.time()
        self.get_logger().info("Waiting for camera perception (/scene_state)...")
        while time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_scene and len(self.latest_scene.objects) > 0:
                self.get_logger().info("Perception data received successfully.")
                return True
        return False

    def execute_plan(self, validated_plan: list) -> bool:
        print("\n" + "="*25 + " EXECUTING ROBOT PLAN " + "="*25)
        for i, step in enumerate(validated_plan):
            skill = step['skill']
            print(f"\nStep {i+1}/{len(validated_plan)}: Executing skill [{skill}]...")

            if skill == 'detect_objects':
                # Spin once or sleep to refresh scene from camera
                start = time.time()
                while time.time() - start < 1.0:
                    rclpy.spin_once(self, timeout_sec=0.1)
                print("  -> Camera scene refreshed.")

            elif skill == 'check_zone':
                zone_name = step['zone']
                rclpy.spin_once(self, timeout_sec=0.1)
                occ = self.latest_scene.check_zone(zone_name) if self.latest_scene else None
                status = f"OCCUPIED by {occ}" if occ else "EMPTY"
                print(f"  -> Zone {zone_name} is currently: {status}")

            elif skill == 'find_object':
                obj_name = step['object']
                rclpy.spin_once(self, timeout_sec=0.1)
                obj = self.latest_scene.find_object(obj_name) if self.latest_scene else None
                if obj:
                    print(f"  -> Found {obj_name} at ({obj.x:.3f}, {obj.y:.3f})")
                else:
                    print(f"  -> {obj_name} not found by camera!")

            elif skill == 'find_free_position':
                temp_name = step['name']
                pos = step.get('resolved')
                print(f"  -> Free position allocated for '{temp_name}': {pos}")

            elif skill == 'pick':
                obj_name = step['object']
                res = step.get('resolved', [0.3, 0.0, 0.0])
                x, y = res[0], res[1]
                yaw = res[2] if len(res) > 2 else 0.0
                status = self.skills.pick(x, y, obj_name, yaw)
                if status != "SUCCESS":
                    print(f"  -> ERROR: Pick failed for {obj_name}!")
                    return False

            elif skill == 'place':
                tgt_name = step['target']
                res = step.get('resolved', [0.3, 0.0])
                x, y = res[0], res[1]
                status = self.skills.place(x, y, tgt_name)
                if status != "SUCCESS":
                    print(f"  -> ERROR: Place failed for target {tgt_name}!")
                    return False

            elif skill == 'home':
                self.skills.home()

            time.sleep(0.5)

        print("\n" + "="*25 + " PLAN EXECUTION COMPLETED " + "="*25)
        return True

    def process_command(self, user_command: str):
        if not self.wait_for_scene():
            self.get_logger().error("Cannot plan: Camera scene is not available!")
            return

        print("\n" + "="*60)
        print("CURRENT PERCEPTION STATE (FROM CAMERA):")
        print(self.latest_scene.summary_text())
        print("="*60)

        # 1. LLM Planning
        self.get_logger().info(f"Querying LLM Planner for command: '{user_command}'...")
        raw_plan = self.planner.plan(user_command, self.latest_scene)

        if not raw_plan:
            self.get_logger().error("LLM Planner failed to return a plan.")
            return

        print("\n[LLM Generated Plan]:")
        print(json.dumps(raw_plan, indent=2))

        # 2. Plan Validation
        self.get_logger().info("Validating plan with Plan Validator...")
        val_res = validate_plan(raw_plan, self.latest_scene, self.cfg)
        print("\n[Plan Validator Report]:")
        print(val_res.report())

        if not val_res.ok:
            self.get_logger().warn("Initial plan was REJECTED by validator. Requesting replanning with feedback...")
            corrected_plan = self.planner.plan(
                user_command, self.latest_scene,
                feedback=val_res.errors, previous=raw_plan
            )
            if corrected_plan:
                val_res = validate_plan(corrected_plan, self.latest_scene, self.cfg)
                print("\n[Re-planned Validator Report]:")
                print(val_res.report())

        if not val_res.ok:
            self.get_logger().error("Plan validation failed after replanning. Aborting execution.")
            return

        print("\n[Final Grounded Execution Plan]:")
        print(plan_to_text(val_res.plan))

        # 3. Execution
        self.execute_plan(val_res.plan)

def main(args=None):
    rclpy.init(args=args)
    node = LLMCommanderNode()

    # If run in non-interactive / demonstration mode with CLI argument:
    if len(sys.argv) > 1 and not sys.argv[1].startswith('--'):
        cmd = " ".join(sys.argv[1:])
        node.process_command(cmd)
    else:
        # Interactive loop
        print("\n=== UR3 LLM Skill Planner CLI ===")
        print("Example commands:")
        print("  - Put the red cube in Zone B.")
        print("  - Move yellow cube to zone C.")
        print("Type 'exit' or 'q' to quit.\n")
        try:
            while rclpy.ok():
                try:
                    cmd = input("Enter command > ").strip()
                except EOFError:
                    break
                if not cmd:
                    continue
                if cmd.lower() in ['exit', 'quit', 'q']:
                    break
                node.process_command(cmd)
        except KeyboardInterrupt:
            pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

