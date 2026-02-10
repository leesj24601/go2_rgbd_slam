# Implementation Plan: Go2 RTAB-Map SLAM in Isaac Sim

## Goal
Isaac Sim 환경에서 Unitree Go2 로봇에 가상 RealSense 카메라를 장착하고, 키보드로 조작하며 RTAB-Map을 통해 3D/2D 맵을 생성한다.

## 현재 환경
| 항목 | 상태 | 경로/버전 |
|------|------|-----------|
| Isaac Sim | Python Package | Anaconda `lab` env (Python 3.11) |
| IsaacLab | 설치됨 | `/home/cvr/Desktop/sj/IsaacLab/` |
| ROS2 | Humble | `/opt/ros/humble` (시스템 Python 3.10) |
| Go2 USD | 확보됨 | `Nucleus/Isaac/IsaacLab/Robots/Unitree/Go2/go2.usd` |
| RTAB-Map | 설치 예정 | `sudo apt install ros-humble-rtabmap-ros` |

## 프로젝트 구조
```
isaac-project/
├── scripts/
│   ├── go2_sim.py              # [핵심] RL 정책 + 키보드 제어 + ROS2 OmniGraph 카메라 퍼블리시
│   ├── my_slam_env.py          # MySlamEnvCfg (terrain=usd, CameraCfg, 커스텀 Config)
│   ├── deploy_scene_mcp.py     # MCP로 장애물 생성/배치 (port 8766)
│   └── cli_args.py             # RSL-RL CLI 인자 파서
├── assets/
│   └── slam_env.usd            # SLAM 환경 USD (기둥, 상자 등 장애물 포함)
├── config/
│   └── go2_sim.rviz            # RViz2 시각화 설정 (RGB/Depth/PointCloud2)
├── launch/                     # (Phase 4: ORB-SLAM3 launch 파일 예정)
├── .pretrained_checkpoints/
│   └── rsl_rl/Isaac-Velocity-Rough-Unitree-Go2-v0/checkpoint.pt
└── outputs/                    # Hydra 실행 로그 (자동 생성)
```

## 아키텍처: Isaac Lab 기반 채택

| | Isaac Lab (채택) | Standalone |
|---|---|---|
| RL 정책 연동 | 쉬움 (`env.step` → obs 자동) | 어려움 (obs 48개 직접 조립) |
| 환경 로딩 | `TerrainImporterCfg(terrain_type="usd")` | `add_reference` 한 줄 |
| 키보드 제어 | `Se2Keyboard` 서브클래스 | 직접 구현 |
| 카메라 센서 | `CameraCfg`를 `__post_init__`에서 동적 추가 | 직접 prim 생성 |
| ROS2 퍼블리시 | OmniGraph ROS2 Bridge Extension | rclpy 직접 사용 |

**실행 명령어**:
```bash
cd /home/cvr/Desktop/sj/isaac-project
/home/cvr/anaconda3/envs/lab/bin/python scripts/go2_sim.py

# 실시간 렌더링 끄고 빠르게 실행
/home/cvr/anaconda3/envs/lab/bin/python scripts/go2_sim.py --rt false
```

---

## Phase 0: 사전 환경 검증 ✅ 완료
- [x] Isaac Sim Python Package 확인 (conda `lab` env)
- [x] ROS2 Humble 확인 (`/opt/ros/humble`)
- [x] Go2 USD 확보 (Nucleus 서버)
- [x] ORB-SLAM3 + Vocabulary 확인
- [x] 프로젝트 디렉토리 생성

---

## Phase 1: Environment & Robot Control ✅ 완료

**목표**: Go2가 SLAM용 환경에서 RL 정책으로 보행하며 키보드로 이동한다.

### 1-1. SLAM 환경 제작 ✅

MCP `execute_script`로 Isaac Sim에 장애물을 직접 생성한 뒤, USD로 저장하여 `TerrainImporterCfg`로 로드하는 방식을 채택.

**현재 환경 구성** (`deploy_scene_mcp.py`):
- `UsdGeom.Cube` 기둥 12개 + `CollisionAPI` + `PhysxSchema` 적용
- 기존 Box_A, Box_B 등 상자 장애물과 겹침 방지 로직 (`math.sqrt` 거리 계산, 최소 1.4m)
- Z = height / 2 지면 밀착 공식 적용
- `random.seed(42)`로 재현 가능한 배치
- MCP 소켓(port 8766)으로 실행 중인 Isaac Sim에 JSON 명령 전송

**USD 제작 3단계 법칙**:
| 규칙 | 내용 |
|------|------|
| Rule 1 | `UsdGeom.Cube/Cylinder` + `CollisionAPI` + `PhysxSchema` 사용 |
| Rule 2 | 단일 부모 Xform 하위에 모든 물체 묶기 |
| Rule 3 | Z = 높이 / 2 (지면 밀착) |

**Isaac Lab RayCaster 센서 제약사항** (핵심!):
| 제약 | 설명 |
|------|------|
| `mesh_prim_paths` 1개만 허용 | 리스트에 경로 여러 개 넣으면 `NotImplementedError` 발생 |
| Mesh 타입만 인식 | `GroundPlane`, `Cylinder` (Primitive)는 인식 안 됨 |
| 정규표현식 미지원 | `"/World/(ground\|Pillar_.*)"` 같은 Regex 불가 |

**해결책**: 모든 장애물을 `/World/ground` 하위로 이동 → `mesh_prim_paths = ["/World/ground"]` 설정

### 1-2. 환경 로딩 ✅

**시도한 방법들과 결과**:
| 방법 | 코드 | 결과 |
|------|------|------|
| `FixedCuboid` 등 | `omni.isaac.core.objects` | Isaac Lab에서 `ModuleNotFoundError` |
| Reference (Fabric ON) | `prim.GetReferences().AddReference()` | 0 children, 안 보임 |
| Reference (Fabric OFF) | 동일 + `--disable_fabric` | 테스트 필요 |
| `TerrainImporterCfg(terrain_type="usd")` | `my_slam_env.py` | **작동 확인 → 채택** |
| MCP 직접 생성 | `deploy_scene_mcp.py`로 실행 중인 Isaac Sim에 직접 생성 | **작동 확인 (환경 제작용)** |

**최종 채택 방식** (`my_slam_env.py`):
```python
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
```

### 1-3. RL 정책 로딩 ✅

`OnPolicyRunner` + `get_published_pretrained_checkpoint` 조합으로 pretrained 체크포인트 자동 탐색.

```python
train_task_name = task_name.replace("-Play", "")  # Play 접미사 제거
resume_path = get_published_pretrained_checkpoint("rsl_rl", train_task_name)
runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
runner.load(resume_path)
policy = runner.get_inference_policy(device=env.unwrapped.device)
```

로컬 폴백: `.pretrained_checkpoints/rsl_rl/Isaac-Velocity-Rough-Unitree-Go2-v0/checkpoint.pt`

### 1-4. 키보드 제어 ✅

`WasdKeyboard(Se2Keyboard)` — `_create_key_bindings`를 오버라이드하여 WASD 매핑.

| 키 | 동작 | 감도 |
|----|------|------|
| W/S | 전진/후진 | `v_x_sensitivity=1.0` |
| A/D | 좌/우 이동 | `v_y_sensitivity=1.0` |
| Q/E | 좌/우 회전 | `omega_z_sensitivity=1.5` |
| K | 정지 | - |

**키보드 제어 필수 설정** (`my_slam_env.py`에서 적용):
```python
self.commands.base_velocity.resampling_time_range = (1.0e9, 1.0e9)  # 랜덤 명령 비활성화
self.commands.base_velocity.heading_command = False                   # Q/E 직접 회전 제어
self.commands.base_velocity.debug_vis = False
self.episode_length_s = 1.0e9                                        # 에피소드 자동 리셋 방지
self.curriculum.terrain_levels = None                                 # Curriculum 비활성화
```

**속도 명령 주입 방식** (`go2_sim.py` 메인 루프):
```python
vel_cmd = keyboard.advance()
cmd_term = env.unwrapped.command_manager.get_term("base_velocity")
cmd_term.vel_command_b[:] = torch.tensor(vel_cmd, device=env.unwrapped.device, dtype=torch.float32)
```

### 1-5. 실시간 제어 (--rt 플래그) ✅

`--rt` 인자를 AppLauncher/Hydra가 파싱하기 **전에** `sys.argv`에서 수동 추출하는 pre-parse 방식 사용.

```python
# Hydra가 알 수 없는 인자로 에러 내는 것 방지
rt_mode = "true"
for i, arg in enumerate(argv_copy):
    if arg == "--rt" and i + 1 < len(argv_copy):
        rt_mode = argv_copy[i + 1].lower()
        sys.argv = argv_copy[:i] + argv_copy[i + 2:]  # sys.argv에서 제거
        break
```

실시간 동기화: `time.sleep(dt - elapsed)` 로 시뮬레이션 dt에 맞춤. `--rt false`로 비활성화 가능.

### Phase 1 해결된 에러
| 에러 | 원인 | 해결 |
|------|------|------|
| `TerrainImporter has no attribute 'terrain_levels'` | USD terrain 사용 시 curriculum 미호환 | `curriculum.terrain_levels = None` |
| `NoneType has no attribute 'size'` | 동일 원인 | 동일 |
| `ModuleNotFoundError: omni.isaac.core` | Isaac Lab에서 standalone API 미지원 | generic USD API (`UsdGeom`, `UsdPhysics`)로 대체 |
| `Accessed invalid expired Xform prim` | Fabric 모드에서 prim 참조 소실 | `omni.kit.commands.execute("CreatePrim")` 사용 |
| Reference 0 children | Fabric ON 상태에서 USD Reference 미해석 | `disable_fabric=True` + `SetDefaultPrim` |
| `RayCaster only supports one mesh prim` | RayCaster 다중 경로 미지원 | `mesh_prim_paths`에 경로 1개만 사용 |
| `Invalid mesh prim path: /World/ground` | Primitive 타입은 RayCaster 미인식 | USD 내 물체를 Mesh 타입으로 생성 |
| `Prim at path '...' is not valid` | 정규표현식 미지원 | 단일 부모 구조로 해결 |
| Q/E 회전 키 미작동 | `heading_command`가 회전 명령 덮어씀 | `heading_command = False` 설정 |
| 로봇이 기둥 통과 | 충돌체 미적용 | `UsdPhysics.CollisionAPI.Apply(prim)` + `PhysxSchema` |
| 기둥이 땅에 박힘 | 원점이 바닥이 아닌 중심 기준 | `Z = height / 2` 공식 적용 |

---

## Phase 2-3: RealSense Camera + ROS2 Publishing ✅ 완료 (검증 완료)

**목표**: Go2 base에 Intel RealSense D435 카메라를 마운트하고, RGB/Depth/CameraInfo를 ROS2 토픽으로 퍼블리시.

### 아키텍처 결정 (변경됨)

초기에는 `CameraCfg + rclpy` 방식을 계획했으나, conda Python 3.11에서 rclpy import가 불가능한 문제로 **OmniGraph ROS2 Bridge** 방식으로 전환함.

| 방식 | 설명 | 결과 |
|------|------|------|
| CameraCfg + rclpy | 메인 루프에서 rclpy로 직접 퍼블리시 | **불가** — conda 3.11 ↔ rclpy 3.10 버전 불일치 |
| TCP 소켓 브릿지 | 소켓으로 데이터 전송 → 별도 ROS2 노드에서 퍼블리시 | 작동하지만 복잡도 높음 |
| **OmniGraph ROS2 Bridge** | Isaac Sim 내장 Extension으로 직접 ROS2 토픽 퍼블리시 | **채택 — 가장 깔끔** |

**최종 채택**: Isaac Lab `CameraCfg`로 카메라 센서를 scene에 추가 + `isaacsim.ros2.bridge` Extension의 OmniGraph 노드로 ROS2 퍼블리시하는 하이브리드 방식.

### 2-3-1. 카메라 센서 추가 (`my_slam_env.py`) ✅

`MySlamEnvCfg.__post_init__`에서 `self.scene.front_cam`으로 동적 추가.
`InteractiveScene._add_entities_from_cfg()`가 `cfg.__dict__`를 순회하므로 동적 속성도 자동 인식됨.

```python
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
```

**카메라 파라미터 계산**:
- HFOV = 2 * atan(horizontal_aperture / (2 * focal_length)) = 2 * atan(20.955 / 30) ≈ 69.9°
- RealSense D435 color HFOV: 69.4° → 근사 일치
- `clipping_range=(0.1, 50.0)` — 초기 10m에서 50m로 확장 (넓은 환경 대응)

**카메라 마운트 위치**:
- Go2 body 길이 ~0.5m, 높이 ~0.15m
- `pos=(0.30, 0.0, 0.05)`: base 기준 전방 30cm, 높이 5cm (Go2 전면부)
- `rot=(0.5, -0.5, 0.5, -0.5)`: ROS camera convention에서 robot +X 방향 정면을 바라봄

### 2-3-2. ROS2 OmniGraph 퍼블리싱 (`go2_sim.py`) ✅

**구현 방식**: `isaacsim.ros2.bridge` Extension 활성화 → 숨겨진 뷰포트에서 렌더 프로덕트 생성 → OmniGraph 노드로 ROS2 퍼블리시.

**1단계: ROS2 Bridge Extension 활성화**
```python
from isaacsim.core.utils import extensions
extensions.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()
```

**2단계: 숨겨진 뷰포트 생성 + 렌더 프로덕트 획득**
```python
from omni.kit.viewport.utility import create_viewport_window

vp_window = create_viewport_window("ROS2_Camera", width=640, height=480, visible=False)
vp_api = vp_window.viewport_api
vp_api.set_active_camera("/World/envs/env_0/Robot/base/front_cam")
rp_path = vp_api.get_render_product_path()
```
- `visible=False`로 메인 뷰포트에 영향 없이 별도 렌더링
- 카메라 prim path는 Isaac Lab의 env_0 절대 경로 사용: `/World/envs/env_0/Robot/base/front_cam`

**3단계: OmniGraph 노드 생성**
```python
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
```

**OmniGraph 노드 구성**:
| 노드 | 타입 | 역할 |
|------|------|------|
| `OnTick` | `omni.graph.action.OnTick` | 매 프레임 트리거 |
| `cameraHelperRgb` | `isaacsim.ros2.bridge.ROS2CameraHelper` | RGB 이미지 퍼블리시 |
| `cameraHelperDepth` | `isaacsim.ros2.bridge.ROS2CameraHelper` | Depth 이미지 퍼블리시 |
| `cameraHelperInfo` | `isaacsim.ros2.bridge.ROS2CameraInfoHelper` | CameraInfo 퍼블리시 |

**발행 토픽**:
| 토픽 | 타입 | frame_id | 용도 |
|------|------|----------|------|
| `camera/color/image_raw` | `sensor_msgs/Image` | `camera_link` | RGB 영상 |
| `camera/depth/image_rect_raw` | `sensor_msgs/Image` | `camera_link` | Depth 영상 |
| `camera/camera_info` | `sensor_msgs/CameraInfo` | `camera_link` | Intrinsic 파라미터 |

**주의사항**:
- `setup_ros2_camera_graph` 내에서 `simulation_app.update()` 호출 제거함 — 시간 불일치 경고(`Physics dt != Rendering dt`) 방지
- `pipeline_stage`를 `GRAPH_PIPELINE_STAGE_ONDEMAND`로 설정하여 자동 실행 방지, `evaluate_sync`로 초기 1회 평가
- try/except로 감싸서 ROS2 Bridge 실패 시에도 시뮬레이션은 계속 동작

### 2-3-3. 실행 방법

```bash
# 터미널 1: Isaac Sim + 카메라 + ROS2 퍼블리싱
source /opt/ros/humble/setup.bash
cd /home/cvr/Desktop/sj/isaac-project
/home/cvr/anaconda3/envs/lab/bin/python scripts/go2_sim.py

# 터미널 2: 토픽 확인 + RViz2
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic hz /camera/color/image_raw
rviz2 -d config/go2_sim.rviz
```

### Phase 2-3 검증
- [x] CameraCfg로 카메라 센서 scene에 추가
- [x] OmniGraph ROS2 Bridge로 RGB/Depth/CameraInfo 퍼블리시 설정
- [x] 숨겨진 뷰포트로 메인 뷰포트 영향 없이 렌더링
- [x] `source /opt/ros/humble/setup.bash` 후 실행 시 Extension 활성화 확인
- [x] `ros2 topic list` → 3개 토픽 확인 (`camera/color/image_raw`, `camera/depth/image_rect_raw`, `camera/camera_info`)
- [x] RViz2에서 RGB/Depth 영상 표시 확인

### Phase 2-3 해결된 에러
| 에러 | 원인 | 해결 |
|------|------|------|
| rclpy import 불가 | conda Python 3.11 ↔ ROS2 rclpy Python 3.10 버전 불일치 | OmniGraph ROS2 Bridge 방식으로 전환 |
| `Physics dt != Rendering dt` 경고 | OmniGraph 설정 중 `simulation_app.update()` 호출 | setup 함수 내 `update()` 호출 제거 |
| `--enable_cameras` argparse 충돌 | AppLauncher가 자체 추가하는 플래그를 수동 등록 | 파싱 후 `args_cli.enable_cameras = True` 직접 설정 |

### 아키텍처 변경 이력
1. **초기 계획**: CameraCfg + rclpy로 메인 루프에서 직접 퍼블리시
2. **문제 발견**: conda 3.11에서 rclpy(3.10) import 불가
3. **중간 시도**: TCP 소켓 브릿지 (`go2_sim.py` → socket → `ros2_camera_bridge.py`)
4. **최종 채택**: Isaac Sim 내장 OmniGraph ROS2 Bridge Extension — rclpy 불필요, 가장 단순

---

## SLAM 방식 변경: ORB-SLAM3 → RTAB-Map

### 변경 이유

| | RTAB-Map (채택) | ORB-SLAM3 (기각) |
|---|---|---|
| 설치 | `sudo apt install ros-humble-rtabmap-ros` | 소스 빌드 필수, 의존성 복잡 |
| 맵 출력 | 3D 포인트클라우드 + 2D OccupancyGrid + dense map | sparse map 위주 |
| 루프 클로저 | Bag-of-Words 기반 자동 감지 | 지원하지만 제한적 |
| ROS2 통합 | 공식 launch 파일, RViz 플러그인, nav2 연동 | 커뮤니티 래퍼 의존 |
| 멀티센서 | LiDAR + RGBD 퓨전 가능 (추후 확장성) | 비주얼 전용 |
| 참고 사례 | Isaac Sim + RTAB-Map 연동 레포 존재 | Isaac Sim 연동 사례 부족 |

### 토픽 매핑 (현재 발행 토픽 → RTAB-Map 파라미터)

| Isaac Sim 토픽 | RTAB-Map launch 파라미터 |
|----------------|--------------------------|
| `camera/color/image_raw` | `rgb_topic` |
| `camera/depth/image_rect_raw` | `depth_topic` |
| `camera/camera_info` | `camera_info_topic` |

---

## Phase 4: RTAB-Map 설치 및 설정 (예정)

**목표**: RTAB-Map을 RGBD 모드로 설치하고, Isaac Sim 카메라 토픽에 연결하여 단독 실행 테스트.

### 4-1. RTAB-Map 설치

```bash
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-rtabmap-ros
```

확인:
```bash
ros2 pkg list | grep rtabmap
```

### 4-2. Launch 파일 작성 (`launch/go2_rtabmap.launch.py`)

RTAB-Map RGBD 모드 launch 파일 작성. 주요 설정:

- [ ] `rtabmap` 노드 설정 (RGBD 모드, 루프 클로저 활성화)
- [ ] `rgbd_odometry` 노드 설정 (비주얼 오도메트리)
- [ ] 토픽 remapping (Isaac Sim 카메라 토픽 → RTAB-Map 입력)
- [ ] `frame_id` 설정 (`camera_link` — Isaac Sim OmniGraph와 일치)
- [ ] `approx_sync` 활성화 (RGB와 Depth 타임스탬프 근사 동기화)

**예상 launch 파라미터**:
```python
# RTAB-Map RGBD launch 주요 파라미터
parameters=[{
    'frame_id': 'camera_link',
    'subscribe_depth': True,
    'approx_sync': True,
    'Vis/MinInliers': '10',           # 최소 inlier 수
    'RGBD/OptimizeMaxError': '0',     # 최적화 에러 필터 비활성화
}],
remappings=[
    ('rgb/image', 'camera/color/image_raw'),
    ('depth/image', 'camera/depth/image_rect_raw'),
    ('rgb/camera_info', 'camera/camera_info'),
],
```

### 4-3. TF 설정

RTAB-Map이 동작하려면 최소한의 TF tree가 필요:
- [ ] `odom` → `camera_link` TF 퍼블리시 방법 결정
  - 방법 A: RTAB-Map의 `rgbd_odometry` 노드가 자체 생성
  - 방법 B: Isaac Sim에서 OmniGraph TF Publisher로 퍼블리시
- [ ] `map` → `odom` TF는 RTAB-Map이 자동 생성

### 4-4. 단독 실행 테스트

- [ ] RTAB-Map 노드 단독 실행 (크래시 없는지 확인)
- [ ] `ros2 topic list` → RTAB-Map 출력 토픽 확인
- [ ] `rgbd_odometry` 노드가 오도메트리 계산하는지 확인

---

## Phase 5: Full Integration & Verification (예정)

**목표**: Isaac Sim + RTAB-Map + RViz2 전체 시스템을 통합 실행하여 3D 맵 생성.

### 실행 순서

```bash
# 터미널 1: Isaac Sim (환경 + 로봇 + 카메라 + ROS2 Bridge)
source /opt/ros/humble/setup.bash
cd /home/cvr/Desktop/sj/isaac-project
/home/cvr/anaconda3/envs/lab/bin/python scripts/go2_sim.py

# 터미널 2: RTAB-Map (SLAM + 비주얼 오도메트리)
source /opt/ros/humble/setup.bash
ros2 launch /home/cvr/Desktop/sj/isaac-project/launch/go2_rtabmap.launch.py

# 터미널 3: RViz2 (시각화)
source /opt/ros/humble/setup.bash
rviz2 -d config/go2_sim.rviz
```

### 검증 항목

- [ ] RTAB-Map이 Isaac Sim 카메라 토픽 구독 확인
- [ ] `rgbd_odometry`가 오도메트리 출력하는지 확인 (`/odom` 토픽)
- [ ] Go2 키보드 조작으로 환경 탐색
- [ ] RTAB-Map Feature 추출 동작 확인
- [ ] RViz2에서 3D Point Cloud Map 생성 확인
- [ ] RViz2에서 2D OccupancyGrid 맵 확인
- [ ] 루프 클로저 감지 동작 확인 (같은 장소 재방문 시)
- [ ] Trajectory 경로 시각화 확인

### RViz2 시각화 설정 업데이트

`config/go2_sim.rviz`에 추가할 Display:
- [ ] `MapCloud` — RTAB-Map 3D 포인트클라우드 (`/rtabmap/mapData`)
- [ ] `Map` — 2D OccupancyGrid (`/rtabmap/map` 또는 `/map`)
- [ ] `Odometry` — 로봇 궤적 (`/odom`)
- [ ] `TF` — TF tree 시각화

---

## 트러블슈팅

### Isaac Sim
| 이슈 | 해결 |
|------|------|
| ROS2 Bridge Extension 미활성화 | `source /opt/ros/humble/setup.bash` 후 실행 필수 |
| 토픽 발행 안됨 | OmniGraph `OnTick → execIn` 연결 확인 |
| Depth 전부 0 | Camera Helper type=`depth`, `clipping_range` 확인 |
| `Physics dt != Rendering dt` 경고 | OmniGraph 설정 중 `simulation_app.update()` 제거 |

### Python 환경
| 이슈 | 해결 |
|------|------|
| rclpy import 불가 | conda 3.11에서 불가 → OmniGraph Bridge 사용 |
| Hydra가 `--rt` 플래그 에러 | AppLauncher 파싱 전 `sys.argv`에서 수동 추출 |

### RTAB-Map (예상)
| 이슈 | 해결 |
|------|------|
| Feature 부족 / 매칭 실패 | `Vis/MinInliers` 낮추기, 환경에 텍스처 추가 |
| RGB-Depth 동기화 실패 | `approx_sync:=true`, `approx_sync_max_interval:=0.05` |
| QoS 불일치 | `ros2 topic info -v`로 QoS 확인 후 맞춤 |
| TF tree 누락 | `odom` → `camera_link` TF 퍼블리시 확인, `rgbd_odometry` 사용 |
| 맵이 생성 안 됨 | `rtabmap` 로그에서 수신 토픽 확인, `rqt_graph`로 연결 확인 |
| OccupancyGrid 비어있음 | `Grid/FromDepth:=true` 파라미터 확인 |
| 메모리 과다 사용 | `Mem/STMSize` 파라미터로 단기 메모리 크기 제한 |

### MCP 연결
| 이슈 | 해결 |
|------|------|
| `Connection refused` | Isaac Sim 미실행 또는 Extension 미로드 |
| `EOF` 에러 | 대용량 스크립트 → `deploy_scene_mcp.py`로 터미널에서 실행 |
| 8766 포트 확인 | `ss -tlnp \| grep 8766` |
