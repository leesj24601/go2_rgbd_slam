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


def setup_odom_graph(chassis_prim_path: str):
    """Odometry + TF (odom → base_link) 퍼블리셔 설정.

    IsaacComputeOdometry가 prim에서 직접 position/orientation/velocity를 읽어
    같은 SIMULATION pipeline tick에서 퍼블리시 → 카메라와 완벽 동기화.
    """
    keys = og.Controller.Keys
    og.Controller.edit(
        {
            "graph_path": "/ROS2_Odom",
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("readSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("computeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("publishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("publishTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ],
            keys.CONNECT: [
                # 실행 흐름: tick → computeOdom → publishOdom, publishTF
                ("OnPlaybackTick.outputs:tick", "computeOdom.inputs:execIn"),
                ("computeOdom.outputs:execOut", "publishOdom.inputs:execIn"),
                ("computeOdom.outputs:execOut", "publishTF.inputs:execIn"),
                # 타임스탬프
                ("readSimTime.outputs:simulationTime", "publishOdom.inputs:timeStamp"),
                ("readSimTime.outputs:simulationTime", "publishTF.inputs:timeStamp"),
                # computeOdom 출력 → publishOdom 입력
                ("computeOdom.outputs:position", "publishOdom.inputs:position"),
                ("computeOdom.outputs:orientation", "publishOdom.inputs:orientation"),
                ("computeOdom.outputs:linearVelocity", "publishOdom.inputs:linearVelocity"),
                ("computeOdom.outputs:angularVelocity", "publishOdom.inputs:angularVelocity"),
                # computeOdom 출력 → publishTF 입력
                ("computeOdom.outputs:position", "publishTF.inputs:translation"),
                ("computeOdom.outputs:orientation", "publishTF.inputs:rotation"),
            ],
            keys.SET_VALUES: [
                # Odometry 메시지 설정
                ("publishOdom.inputs:chassisFrameId", "base_link"),
                ("publishOdom.inputs:odomFrameId", "odom"),
                ("publishOdom.inputs:topicName", "/odom"),
                # TF: odom → base_link
                ("publishTF.inputs:parentFrameId", "odom"),
                ("publishTF.inputs:childFrameId", "base_link"),
                ("publishTF.inputs:topicName", "/tf"),
            ],
        },
    )

    # chassisPrim relationship 설정 (USD API 필요)
    import omni.usd
    from pxr import Sdf

    stage = omni.usd.get_context().get_stage()
    compute_prim = stage.GetPrimAtPath("/ROS2_Odom/computeOdom")
    compute_prim.GetRelationship("inputs:chassisPrim").SetTargets(
        [Sdf.Path(chassis_prim_path)]
    )
    print("[INFO] ROS2 Odometry + TF (odom → base_link) 퍼블리셔 설정 완료 (OmniGraph 동기화)")


def setup_imu_graph():
    """IMU 퍼블리셔 설정 (/imu/data).

    그래프만 생성하고, 실제 IMU 데이터는 메인 루프에서 주입합니다.
    """
    keys = og.Controller.Keys
    og.Controller.edit(
        {
            "graph_path": "/ROS2_IMU",
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("readSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("publishImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "publishImu.inputs:execIn"),
                ("readSimTime.outputs:simulationTime", "publishImu.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("publishImu.inputs:frameId", "base_link"),
                ("publishImu.inputs:topicName", "/imu/data"),
            ],
        },
    )
    print("[INFO] ROS2 IMU 퍼블리셔 설정 완료 (/imu/data)")


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

    # 5.5 Odometry + TF (odom → base_link) 퍼블리셔 설정 (OmniGraph 내 동기화)
    robot_base_prim = "/World/envs/env_0/Robot/base"
    try:
        setup_odom_graph(robot_base_prim)
    except Exception as e:
        print(f"[WARN] Odom 설정 실패: {e}")

    # 5.6 IMU 퍼블리셔 설정
    try:
        setup_imu_graph()
    except Exception as e:
        print(f"[WARN] IMU 설정 실패: {e}")

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

    # IMU OmniGraph 속성 경로 헬퍼 (odom/TF는 OmniGraph 내부에서 자동 동기화)
    def _imu_attr(name):
        return og.Controller.attribute(f"/ROS2_IMU/publishImu.inputs:{name}")

    while simulation_app.is_running():
        start_time = time.time()
        vel_cmd = keyboard.advance()

        # 명령어 적용
        if cmd_term is not None:
            cmd_term.vel_command_b[0, 0] = vel_cmd[0]
            cmd_term.vel_command_b[0, 1] = vel_cmd[1]
            cmd_term.vel_command_b[0, 2] = vel_cmd[2]

        # IMU 데이터 사전 주입 (env.step 내 SIMULATION pipeline에서 퍼블리시됨)
        try:
            imu = env.unwrapped.scene["imu_sensor"]
            imu_ang_vel = imu.data.ang_vel_b[0].cpu().numpy()
            imu_lin_acc = imu.data.lin_acc_b[0].cpu().numpy()
            imu_quat_wxyz = imu.data.quat_w[0].cpu().numpy()
            # Isaac Lab WXYZ → OmniGraph XYZW (IJKR)
            imu_quat_xyzw = [imu_quat_wxyz[1], imu_quat_wxyz[2], imu_quat_wxyz[3], imu_quat_wxyz[0]]

            og.Controller.set(_imu_attr("angularVelocity"), imu_ang_vel.tolist())
            og.Controller.set(_imu_attr("linearAcceleration"), imu_lin_acc.tolist())
            og.Controller.set(_imu_attr("orientation"), imu_quat_xyzw)
        except Exception:
            pass  # 초기 프레임에서 데이터 없을 수 있음

        with torch.inference_mode():
            actions = policy(obs)
            obs, _, _, _ = env.step(actions)
            # env.step() 내부에서 SIMULATION pipeline 실행:
            # - IsaacComputeOdometry가 prim에서 직접 pos/quat/vel 읽기
            # - ROS2PublishOdometry + ROS2PublishRawTransformTree 퍼블리시
            # - 카메라 렌더 + 퍼블리시
            # → 모두 같은 tick에서 실행되어 완벽 동기화

        if args_cli.rt.lower() in ("true", "1", "yes"):
            sleep_time = dt - (time.time() - start_time)
            if sleep_time > 0:
                time.sleep(sleep_time)

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
