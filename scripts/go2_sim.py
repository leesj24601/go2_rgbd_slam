#!/usr/bin/env python3
import argparse
import sys
import os
import time
import logging
import numpy as np
import torch
import gymnasium as gym

# Isaac Sim 경고 로그 필터링
logging.getLogger("isaacsim").setLevel(logging.ERROR)
logging.getLogger("omni").setLevel(logging.ERROR)

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
    """숨겨진 뷰포트에서 렌더 프로덕트 생성 → OmniGraph ROS2 퍼블리시.

    공식 예제 방식: execution evaluator + SIMULATION pipeline + frameSkipCount
    → evaluate_sync 블로킹 없이 시뮬레이션 스텝과 자동 동기화
    """
    from omni.kit.viewport.utility import create_viewport_window

    # 숨겨진 뷰포트 생성 (메인 뷰포트에 영향 없음) - 320x240 저해상도
    vp_window = create_viewport_window(
        "ROS2_Camera", width=320, height=240, visible=False
    )
    vp_api = vp_window.viewport_api
    vp_api.set_active_camera(camera_prim_path)
    rp_path = vp_api.get_render_product_path()
    print(f"[INFO] 숨겨진 뷰포트 렌더 프로덕트: {rp_path}")

    # frameSkipCount: 퍼블리시Hz = simFPS / (skipCount + 1)
    # 시뮬레이션 ~30fps 기준 → skipCount=2 → ~10Hz 퍼블리시
    FRAME_SKIP = 2

    keys = og.Controller.Keys
    (ros_camera_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS2_Camera",
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("cameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("cameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "cameraHelperRgb.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "cameraHelperDepth.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "cameraHelperInfo.inputs:execIn"),
            ],
            keys.SET_VALUES: [
                ("cameraHelperRgb.inputs:renderProductPath", rp_path),
                ("cameraHelperRgb.inputs:frameId", "camera_optical_frame"),
                ("cameraHelperRgb.inputs:topicName", "camera/color/image_raw"),
                ("cameraHelperRgb.inputs:type", "rgb"),
                ("cameraHelperRgb.inputs:frameSkipCount", FRAME_SKIP),
                ("cameraHelperDepth.inputs:renderProductPath", rp_path),
                ("cameraHelperDepth.inputs:frameId", "camera_optical_frame"),
                ("cameraHelperDepth.inputs:topicName", "camera/depth/image_rect_raw"),
                ("cameraHelperDepth.inputs:type", "depth"),
                ("cameraHelperDepth.inputs:frameSkipCount", FRAME_SKIP),
                ("cameraHelperInfo.inputs:renderProductPath", rp_path),
                ("cameraHelperInfo.inputs:frameId", "camera_optical_frame"),
                ("cameraHelperInfo.inputs:topicName", "camera/camera_info"),
                ("cameraHelperInfo.inputs:frameSkipCount", FRAME_SKIP),
            ],
        },
    )
    print(f"[INFO] ROS2 카메라 퍼블리셔 설정 완료 (320x240, frameSkip={FRAME_SKIP})")

    # /clock 퍼블리시 (use_sim_time 지원)
    (clock_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS2_Clock",
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("readSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("publishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "publishClock.inputs:execIn"),
                ("readSimTime.outputs:simulationTime", "publishClock.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("publishClock.inputs:topicName", "/clock"),
            ],
        },
    )
    print("[INFO] ROS2 /clock 퍼블리셔 설정 완료")


def setup_robot_tf_graph():
    """Go2 base_link TF 퍼블리셔 설정 (odom → base_link)"""
    keys = og.Controller.Keys
    (tf_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS2_RobotTF",
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("readSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("readPrim", "isaacsim.core.nodes.IsaacReadPrimNode"),
                ("publishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "readPrim.inputs:execIn"),
                ("readPrim.outputs:execOut", "publishTF.inputs:execIn"),
                ("readSimTime.outputs:simulationTime", "publishTF.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("readPrim.inputs:primPath", "/World/envs/env_0/Robot/base"),
                ("readPrim.inputs:axis", "X"),
                (
                    "publishTF.inputs:parentFrameId",
                    "camera_link",
                ),  # camera_link 기준 (RTAB-Map odom 기준)
                ("publishTF.inputs:childFrameId", "base_link"),
                ("publishTF.inputs:topicName", "/tf"),
            ],
        },
    )
    print("[INFO] ROS2 robot TF 퍼블리셔 설정 완료 (camera_link → base_link)")


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

    # 5. ROS2 OmniGraph 카메라 퍼블리셔 설정 (SIMULATION 파이프라인 - 자동 실행)
    cam_prim_path = "/World/envs/env_0/Robot/base/front_cam"
    try:
        setup_ros2_camera_graph(cam_prim_path)
    except Exception as e:
        print(f"[WARN] ROS2 bridge 설정 실패: {e}")

    # 5.5 Go2 base_link TF 퍼블리셔 설정
    try:
        setup_robot_tf_graph()
    except Exception as e:
        print(f"[WARN] Robot TF 설정 실패: {e}")

    # 6. Reset & Loop
    obs = env.get_observations()
    dt = env.unwrapped.step_dt
    keyboard = WasdKeyboard(
        Se2KeyboardCfg(
            v_x_sensitivity=1.0, v_y_sensitivity=1.0, omega_z_sensitivity=1.5
        )
    )

    # 명령 manager 미리 캐싱
    cmd_term = None
    if hasattr(env.unwrapped, "command_manager"):
        cmd_term = env.unwrapped.command_manager.get_term("base_velocity")

    while simulation_app.is_running():
        start_time = time.time()
        vel_cmd = keyboard.advance()

        # 명령어 적용
        if cmd_term is not None:
            cmd_term.vel_command_b[0, 0] = vel_cmd[0]
            cmd_term.vel_command_b[0, 1] = vel_cmd[1]
            cmd_term.vel_command_b[0, 2] = vel_cmd[2]

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
