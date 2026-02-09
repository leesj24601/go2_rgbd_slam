#!/usr/bin/env python3
import argparse
import sys
import os
import time
import numpy as np
import torch
import gymnasium as gym

from isaaclab.app import AppLauncher

# 0. Pre-parse --rt argument (before AppLauncher/Hydra)
rt_mode = "true"
argv_copy = sys.argv.copy()
for i, arg in enumerate(argv_copy):
    if arg == "--rt" and i + 1 < len(argv_copy):
        rt_mode = argv_copy[i + 1].lower()
        # Remove --rt and its value from sys.argv so Hydra doesn't see it
        sys.argv = argv_copy[:i] + argv_copy[i + 2 :]
        break
    elif arg.startswith("--rt="):
        rt_mode = arg.split("=", 1)[1].lower()
        sys.argv = argv_copy[:i] + argv_copy[i + 1 :]
        break

# 1. Setup Parser
parser = argparse.ArgumentParser(description="Go2 Simulation matching run_slam style")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments.")
parser.add_argument(
    "--task",
    type=str,
    default="Isaac-Velocity-Rough-Unitree-Go2-Play-v0",
    help="Task name.",
)
parser.add_argument(
    "--use_pretrained_checkpoint",
    action="store_true",
    default=True,
    help="Use checkpoint.",
)

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import cli_args

cli_args.add_rsl_rl_args(parser)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
args_cli.enable_cameras = True
args_cli.rt = rt_mode

# Launch simulation app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# 2. Imports after app launch
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
from isaaclab.devices import Se2Keyboard, Se2KeyboardCfg
from isaaclab_rl.utils.pretrained_checkpoint import get_published_pretrained_checkpoint
from rsl_rl.runners import OnPolicyRunner
from my_slam_env import MySlamEnvCfg
from isaaclab_tasks.utils.hydra import hydra_task_config
import isaaclab_tasks  # noqa

import omni.graph.core as og
from isaacsim.core.utils import extensions

# ROS2 bridge 확장 활성화
extensions.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()


class WasdKeyboard(Se2Keyboard):
    def _create_key_bindings(self):
        self._INPUT_KEY_MAPPING = {
            "W": np.asarray([1.0, 0.0, 0.0]) * self.v_x_sensitivity,
            "S": np.asarray([-1.0, 0.0, 0.0]) * self.v_x_sensitivity,
            "A": np.asarray([0.0, 1.0, 0.0]) * self.v_y_sensitivity,
            "D": np.asarray([0.0, -1.0, 0.0]) * self.v_y_sensitivity,
            "Q": np.asarray([0.0, 0.0, 1.0]) * self.omega_z_sensitivity,
            "E": np.asarray([0.0, 0.0, -1.0]) * self.omega_z_sensitivity,
            "K": np.asarray([0.0, 0.0, 0.0]),
        }


def setup_ros2_camera_graph(camera_prim_path: str):
    """숨겨진 뷰포트에서 렌더 프로덕트 생성 → OmniGraph ROS2 퍼블리시."""
    from omni.kit.viewport.utility import create_viewport_window

    # 숨겨진 뷰포트 생성 (메인 뷰포트에 영향 없음)
    vp_window = create_viewport_window(
        "ROS2_Camera", width=640, height=480, visible=False
    )
    vp_api = vp_window.viewport_api
    vp_api.set_active_camera(camera_prim_path)
    # NOTE: simulation_app.update() 제거 - 시간 불일치 경고 방지
    rp_path = vp_api.get_render_product_path()
    print(f"[INFO] 숨겨진 뷰포트 렌더 프로덕트: {rp_path}")

    keys = og.Controller.Keys
    (ros_camera_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS2_Camera",
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("cameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("cameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "cameraHelperRgb.inputs:execIn"),
                ("OnTick.outputs:tick", "cameraHelperDepth.inputs:execIn"),
                ("OnTick.outputs:tick", "cameraHelperInfo.inputs:execIn"),
            ],
            keys.SET_VALUES: [
                ("cameraHelperRgb.inputs:renderProductPath", rp_path),
                ("cameraHelperRgb.inputs:frameId", "camera_link"),
                ("cameraHelperRgb.inputs:topicName", "camera/color/image_raw"),
                ("cameraHelperRgb.inputs:type", "rgb"),
                ("cameraHelperDepth.inputs:renderProductPath", rp_path),
                ("cameraHelperDepth.inputs:frameId", "camera_link"),
                ("cameraHelperDepth.inputs:topicName", "camera/depth/image_rect_raw"),
                ("cameraHelperDepth.inputs:type", "depth"),
                ("cameraHelperInfo.inputs:renderProductPath", rp_path),
                ("cameraHelperInfo.inputs:frameId", "camera_link"),
                ("cameraHelperInfo.inputs:topicName", "camera/camera_info"),
            ],
        },
    )

    og.Controller.evaluate_sync(ros_camera_graph)
    # NOTE: simulation_app.update() 제거 - 시간 불일치 경고 방지
    print("[INFO] ROS2 카메라 퍼블리셔 OmniGraph 설정 완료")


@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
def main(env_cfg, agent_cfg):
    # 3. Create Environment
    custom_env_cfg = MySlamEnvCfg()
    custom_env_cfg.scene.num_envs = args_cli.num_envs

    env = gym.make(args_cli.task, cfg=custom_env_cfg)
    env = RslRlVecEnvWrapper(env)

    # 4. Load Policy
    task_name = args_cli.task.split(":")[-1]
    train_task_name = task_name.replace("-Play", "")
    resume_path = get_published_pretrained_checkpoint("rsl_rl", train_task_name)

    print(f"[INFO] Loading policy from: {resume_path}")
    runner = OnPolicyRunner(
        env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device
    )
    runner.load(resume_path)
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    # 5. ROS2 OmniGraph 카메라 퍼블리셔 설정
    cam_prim_path = "/World/envs/env_0/Robot/base/front_cam"
    try:
        setup_ros2_camera_graph(cam_prim_path)
    except Exception as e:
        print(f"[WARN] ROS2 bridge 설정 실패: {e}")

    # 6. Reset & Loop
    obs = env.get_observations()
    dt = env.unwrapped.step_dt
    keyboard = WasdKeyboard(
        Se2KeyboardCfg(
            v_x_sensitivity=1.0, v_y_sensitivity=1.0, omega_z_sensitivity=1.5
        )
    )

    while simulation_app.is_running():
        start_time = time.time()
        vel_cmd = keyboard.advance()

        if hasattr(env.unwrapped, "command_manager"):
            cmd_term = env.unwrapped.command_manager.get_term("base_velocity")
            if cmd_term is not None:
                cmd_term.vel_command_b[:] = torch.tensor(
                    vel_cmd, device=env.unwrapped.device, dtype=torch.float32
                )

        with torch.inference_mode():
            actions = policy(obs)
            obs, _, _, _ = env.step(actions)

        if args_cli.rt.lower() in ("true", "1", "yes"):
            sleep_time = dt - (time.time() - start_time)
            if sleep_time > 0:
                time.sleep(sleep_time)

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
