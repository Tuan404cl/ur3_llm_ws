#!/usr/bin/env python3
"""
Master Autonomous ReAct Orchestrator Node cho UR3 + Gripper + Camera (Bài 03).
Vận hành vòng lặp khép kín:
  [SENSE] -> [REASON] -> [PLAN & VALIDATE] -> [ACT via MoveIt 2] -> [VERIFY] -> [LOOP/DONE]
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import math
import os
import sys
from typing import Dict, Optional, Tuple, List

from ament_index_python.packages import get_package_share_directory

from ur3_llm_tamp.world_model import SceneState, load_scene_config, find_free_position
from ur3_llm_tamp.plan_validator import validate_plan, plan_to_text
from ur3_llm_tamp.llm_planner import LLMPlanner
from ur3_llm_tamp.robot_skills import UR3Skills

class AutonomousOrchestratorNode(Node):
    def __init__(self):
        super().__init__('master_orchestrator_node')

        # 1. Tải cấu hình Scene tĩnh (bàn, workspace, camera extrinsics)
        pkg_path = get_package_share_directory('ur3_llm_tamp')
        cfg_path = os.path.join(pkg_path, 'config', 'scene.yaml')
        self.cfg = load_scene_config(cfg_path)

        # 2. Khởi tạo LLM Planner
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
                    self.get_logger().info("[Orchestrator] Đã chuyển sang chế độ OFFLINE (Rule-based Mode).")
                else:
                    api_key = user_key
                    os.environ['OPENROUTER_API_KEY'] = user_key
                    self.get_logger().info(f"[Orchestrator] Đã nhận API Key. Model: {model}")
            except (EOFError, KeyboardInterrupt):
                backend = 'offline'
                self.get_logger().info("[Orchestrator] Chuyển sang chế độ OFFLINE.")

        self.get_logger().info(f"[Orchestrator] Đang nạp LLM Planner: backend={backend}, model={model}")
        self.planner = LLMPlanner(backend=backend, model=model, api_key=api_key, log=self.get_logger().info)

        # 3. Kết nối Camera Perception
        self.latest_scene: Optional[SceneState] = None
        self.sub_scene = self.create_subscription(
            String, '/scene_state', self.scene_callback, 10
        )

        # 4. Robot Skills (MoveIt 2 IK + Gripper Controller)
        self.skills = UR3Skills(node_name='orchestrator_skills')
        self.get_logger().info("[Orchestrator] Master Controller Node đã sẵn sàng hoạt động!")

    def scene_callback(self, msg: String):
        try:
            self.latest_scene = SceneState.from_json(msg.data)
        except Exception as e:
            self.get_logger().error(f"[Orchestrator] Lỗi parse dữ liệu camera: {e}")

    # =========================================================================
    # STEP 1: SENSE (Lấy dữ liệu thời gian thực từ Camera)
    # =========================================================================
    def sense_environment(self, timeout_sec: float = 6.0) -> Optional[SceneState]:
        """Chờ và cập nhật ảnh trạng thái mới nhất từ Perception Node."""
        self.latest_scene = None
        start = time.time()
        self.get_logger().info("[SENSE] Đang quét trạng thái môi trường từ Camera...")
        while time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_scene and len(self.latest_scene.objects) > 0:
                return self.latest_scene
        self.get_logger().error("[SENSE] Quá thời gian chờ dữ liệu camera!")
        return None

    # =========================================================================
    # STEP 2: REASON & GOAL CHECKING
    # =========================================================================
    def is_goal_accomplished(self, goal_spec: List[Dict[str, str]], current_scene: SceneState) -> Tuple[bool, str]:
        """
        Đối chiếu mục tiêu (Goal) với trạng thái thực tế từ camera.
        goal_spec ví dụ: [{'object': 'red_cube', 'zone': 'zone_b'}]
        """
        for req in goal_spec:
            obj = req['object']
            target_zone = req['zone']
            curr_obj = current_scene.find_object(obj)

            if curr_obj is None:
                return False, f"Vật '{obj}' không tìm thấy trên camera."
            if curr_obj.zone != target_zone:
                return False, f"Vật '{obj}' đang ở '{curr_obj.zone}', chưa vào '{target_zone}'."

            # Kiểm tra khoảng cách vật tới tâm zone
            zone_info = current_scene.zones.get(target_zone)
            if zone_info:
                dist = math.hypot(curr_obj.x - zone_info.x, curr_obj.y - zone_info.y)
                if dist > self.cfg['zone_occupancy_radius']:
                    return False, f"Vật '{obj}' bị lệch khỏi tâm '{target_zone}' (dist={dist:.3f}m)."

        return True, "Toàn bộ mục tiêu đã đạt được hoàn hảo!"

    # =========================================================================
    # STEP 3 & 4: EXECUTE & VERIFICATION
    # =========================================================================
    def execute_atomic_skill(self, step: dict) -> bool:
        """Thực thi một action skill đơn lẻ với exception catching."""
        skill = step['skill']

        if skill == 'home':
            return self.skills.home() == "SUCCESS"

        elif skill == 'pick':
            obj = step['object']
            res = step.get('resolved', [0.3, 0.0, 0.0])
            x, y = res[0], res[1]
            yaw = res[2] if len(res) > 2 else 0.0
            status = self.skills.pick(x, y, obj_name=obj, yaw=yaw)
            return status == "SUCCESS"

        elif skill == 'place':
            tgt = step['target']
            res = step.get('resolved', [0.3, 0.0])
            x, y = res[0], res[1]
            status = self.skills.place(x, y, target_name=tgt)
            return status == "SUCCESS"

        elif skill in ['detect_objects', 'check_zone', 'find_object', 'find_free_position']:
            # Các kỹ năng nhận thức / tính toán đã được ground bởi validator
            time.sleep(0.2)
            return True

        return False

    # =========================================================================
    # AUTONOMOUS REACT PIPELINE (Chạy liên tục cho đến khi hoàn thành)
    # =========================================================================
    def run_react_mission(self, user_command: str, max_iterations: int = 5):
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"[MISSION START] Nhận lệnh: '{user_command}'")
        self.get_logger().info("=" * 60)

        iteration = 0
        goal_spec = []

        while rclpy.ok() and iteration < max_iterations:
            iteration += 1
            print(f"\n{'='*20} REACT ITERATION #{iteration} {'='*20}")

            # ----------------------------------------------------
            # 1. SENSE
            # ----------------------------------------------------
            scene = self.sense_environment()
            if scene is None:
                self.get_logger().error("[SENSE FAILED] Không nhận được camera scene. Thử lại...")
                time.sleep(1.0)
                continue

            print("\n[CAMERA SCENE SUMMARY]:")
            print(scene.summary_text())

            # ----------------------------------------------------
            # 2. REASON: Kiểm tra xem đã hoàn thành chưa?
            # ----------------------------------------------------
            if goal_spec:
                accomplished, reason = self.is_goal_accomplished(goal_spec, scene)
                if accomplished:
                    print(f"\n>>> [MISSION ACCOMPLISHED]: {reason}")
                    self.skills.home()
                    return True
                else:
                    self.get_logger().info(f"[REASON] Mục tiêu chưa hoàn thành: {reason}")

            # ----------------------------------------------------
            # 3. PLAN: Gọi LLM Planner
            # ----------------------------------------------------
            self.get_logger().info(f"[PLAN] Gửi prompt trạng thái tới LLM Planner...")
            raw_plan = self.planner.plan(user_command, scene)
            if not raw_plan:
                self.get_logger().error("[PLAN FAILED] LLM không sinh được plan. Đang chuyển chu kỳ tiếp theo.")
                continue

            if not goal_spec and 'goal' in raw_plan:
                goal_spec = raw_plan['goal']

            # ----------------------------------------------------
            # 4. VALIDATE
            # ----------------------------------------------------
            val_res = validate_plan(raw_plan, scene, self.cfg)
            if not val_res.ok:
                self.get_logger().warn("[VALIDATE REJECTED] Validator từ chối plan. Yêu cầu LLM điều chỉnh...")
                corrected = self.planner.plan(user_command, scene, feedback=val_res.errors, previous=raw_plan)
                if corrected:
                    val_res = validate_plan(corrected, scene, self.cfg)

            if not val_res.ok:
                self.get_logger().error(f"[VALIDATE ERROR] Plan không hợp lệ: {val_res.errors}")
                continue

            print("\n[GROUNDED EXECUTION PLAN]:")
            print(plan_to_text(val_res.plan))

            # ----------------------------------------------------
            # 5. ACT & FEEDBACK VERIFICATION
            # ----------------------------------------------------
            step_success = True
            execution_feedback = []
            for i, step in enumerate(val_res.plan):
                skill_name = step['skill']
                obj_target = step.get('object', step.get('target', ''))
                print(f"--> Thực thi bước {i+1}: {skill_name}({obj_target})")
                ok = self.execute_atomic_skill(step)

                if not ok:
                    err_msg = f"Skill '{skill_name}' failed at step {i+1} during execution."
                    self.get_logger().error(f"[ACT ERROR] {err_msg}")
                    execution_feedback.append(err_msg)
                    step_success = False
                    # Recovery: đưa tay lên cao an toàn về Home để dọn đường và tránh va chạm
                    self.skills.home()
                    break

                # Post-step verification for pick / place
                if skill_name == 'pick':
                    # Kiểm tra xem gripper có đang giữ vật hay bị rơi
                    if hasattr(self.skills, 'vacuum_grasped') and not self.skills.vacuum_grasped and self.skills.vacuum_client.service_is_ready():
                        self.get_logger().warn(f"[SLIP WARNING] Grasp sensor báo chưa kẹp chặt {obj_target}.")

            if not step_success:
                self.get_logger().warn("[RETRY LOOP] Hành động gặp lỗi. Tự động chuyển chu kỳ ReAct tiếp theo để khắc phục.")
                # Feed errors to planner in next iteration
                continue

            # Sau khi thực thi chuỗi skill, ngủ 1s để vật ổn định trên bàn trước khi sense lại
            time.sleep(1.0)
            scene_after = self.sense_environment()
            if scene_after and goal_spec:
                accomplished, reason = self.is_goal_accomplished(goal_spec, scene_after)
                if accomplished:
                    print(f"\n>>> [MISSION SUCCESS]: {reason}")
                    self.skills.home()
                    return True
                else:
                    self.get_logger().warn(f"[VERIFY FAILED] Sau khi hoàn tất plan nhưng camera xác nhận chưa đạt đích: {reason}")

        self.get_logger().error("[MISSION FAILED] Đạt giới hạn số lần ReAct mà chưa đạt đích.")
        self.skills.home()
        return False

def main(args=None):
    rclpy.init(args=args)
    orchestrator = AutonomousOrchestratorNode()

    # Nhận lệnh qua tham số dòng lệnh CLI hoặc chạy demo mặc định
    command = " ".join(sys.argv[1:]) if len(sys.argv) > 1 and not sys.argv[1].startswith('--') else "Put the red cube in Zone B."
    orchestrator.run_react_mission(command)

    orchestrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

