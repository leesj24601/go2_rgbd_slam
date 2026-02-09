# my_slam_env.py
from isaaclab.utils import configclass
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.sensors import CameraCfg
import isaaclab.sim as sim_utils
from isaaclab_tasks.manager_based.locomotion.velocity.config.go2.rough_env_cfg import (
    UnitreeGo2RoughEnvCfg,
)


@configclass
class MySlamEnvCfg(UnitreeGo2RoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # 1. 새로 저장한 SLAM 전용 USD 경로 지정
        self.scene.terrain = TerrainImporterCfg(
            prim_path="/World/ground",
            terrain_type="usd",
            usd_path="/home/cvr/Desktop/sj/isaac-project/assets/slam_env.usd",
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="multiply",
                restitution_combine_mode="multiply",
                static_friction=1.0,
                dynamic_friction=1.0,
            ),
        )

        # 2. 센서가 창고 바닥과 그 하위 기둥들을 모두 인식하도록 설정
        if hasattr(self.scene, "height_scanner"):
            # 바구니 통합이 완료되면 이 주소 하나면 충분합니다.
            self.scene.height_scanner.mesh_prim_paths = ["/World/ground"]
            self.scene.height_scanner.debug_vis = False

        # 3. 제어 설정 유지
        if hasattr(self.commands, "base_velocity"):
            self.commands.base_velocity.resampling_time_range = (1.0e9, 1.0e9)
            self.commands.base_velocity.debug_vis = False
            # [중요] Heading command를 꺼야 사용자의 Q/E 회전 명령이 직접 전달됩니다.
            self.commands.base_velocity.heading_command = False

        self.episode_length_s = 1.0e9
        if hasattr(self.curriculum, "terrain_levels"):
            self.curriculum.terrain_levels = None

        # Intel RealSense D435 근사 카메라
        self.scene.front_cam = CameraCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base/front_cam",
            update_period=1 / 30,  # 30fps
            height=480,
            width=640,
            data_types=["rgb", "distance_to_image_plane"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=15.0,
                focus_distance=400.0,
                horizontal_aperture=20.955,
                clipping_range=(0.1, 50.0),
            ),
            offset=CameraCfg.OffsetCfg(
                pos=(0.30, 0.0, 0.05),
                rot=(0.5, -0.5, 0.5, -0.5),
                convention="ros",
            ),
        )
