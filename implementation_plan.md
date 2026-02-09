# Implementation Plan: Go2 ORB-SLAM3 in Isaac Sim

## Goal
Isaac Sim 환경에서 Unitree Go2 로봇에 가상 Realsense 카메라를 장착하고, 키보드로 조작하며 ORB-SLAM3를 통해 맵을 생성한다.

## 현재 환경
| 항목 | 상태 | 경로/버전 |
|------|------|-----------|
| Isaac Sim | Python Package | Anaconda `lab` env |
| IsaacLab | 설치됨 | `/home/cvr/Desktop/sj/IsaacLab/` |
| ROS2 | Humble | `/opt/ros/humble` |
| Go2 USD | 확보됨 | `Nucleus/Isaac/IsaacLab/Robots/Unitree/Go2/go2.usd` |
| ros2_orb_slam3 | 빌드됨 | `/home/cvr/Desktop/sj/ros2_orb_slam3/` |
| ORBvoc.txt | 확인됨 | `.../Vocabulary/ORBvoc.txt.bin` |
| Isaac Sim MCP | 동작 중 | port 8766 (`isaac-sim-mcp` 스킬) |

## 프로젝트 구조
```
isaac-project/
├── scripts/
│   ├── go2_sim.py              # [핵심] RL 정책 + 키보드 제어 + 환경 로드
│   ├── my_slam_env.py          # MySlamEnvCfg (terrain=usd, 커스텀 Config)
│   ├── deploy_scene_mcp.py     # MCP로 장애물 생성/배치
│   ├── my_warehouse_env.py     # 창고 환경 Config (참고용)
│   └── cli_args.py             # RSL-RL CLI 인자 파서
├── assets/
│   └── slam_env.usd            # SLAM 환경 USD
├── .agent/workflows/
│   ├── isaac-sim-mcp.md        # MCP 연결 워크플로우
│   └── usd-builder.md          # USD 환경 제작 가이드
├── config/                     # (Phase 4: ORB-SLAM3 yaml 예정)
├── launch/                     # (Phase 4: launch 파일 예정)
└── .pretrained_checkpoints/
    └── rsl_rl/.../checkpoint.pt
```

## 아키텍처: Isaac Lab 기반 채택

| | Isaac Lab (채택) | Standalone |
|---|---|---|
| RL 정책 연동 | 쉬움 (`env.step` → obs 자동) | 어려움 (obs 48개 직접 조립) |
| 환경 로딩 | Reference/Sublayer/TerrainImporter | `add_reference` 한 줄 |
| 키보드 제어 | `Se2Keyboard` 서브클래스 | 직접 구현 |
| 핵심 스크립트 | `go2_sim.py` | - |

**실행 명령어**:
```bash
cd /home/cvr/Desktop/sj/isaac-project
/home/cvr/anaconda3/envs/lab/bin/python scripts/go2_sim.py
```

---

## Phase 0: 사전 환경 검증 ✅ 완료
- [x] Isaac Sim Python Package 확인
- [x] ROS2 Humble 확인
- [x] Go2 USD 확보
- [x] ORB-SLAM3 + Vocabulary 확인
- [x] 프로젝트 디렉토리 생성

---

## Phase 1: Environment & Robot Control 🔧 진행 중

**목표**: Go2가 SLAM용 환경에서 RL 정책으로 보행하며 키보드로 이동한다.

### 1-1. SLAM 환경 제작 ✅

MCP `execute_script`로 Isaac Sim에 장애물을 직접 생성.

**현재 환경 구성** (`deploy_scene_mcp.py`):
- 실린더 기둥 12개 (`UsdGeom.Cube` + `CollisionAPI` + `PhysxSchema`)
- 기존 Box_A, Box_B 등 상자 장애물과 겹침 방지 로직
- Z = height / 2 지면 밀착 공식 적용

**USD 제작 3단계 법칙** (`usd-builder` 스킬로 정리):
| 규칙 | 내용 |
|------|------|
| Rule 1 | `UsdGeom.Cube/Cylinder` + `CollisionAPI` 사용 |
| Rule 2 | 단일 부모 Xform 하위에 모든 물체 묶기 |
| Rule 3 | Z = 높이 / 2 (지면 밀착) |

**Isaac Lab RayCaster 센서 제약사항** (핵심!):
| 제약 | 설명 |
|------|------|
| `mesh_prim_paths` 1개만 허용 | 리스트에 경로 여러 개 넣으면 `NotImplementedError` 발생 |
| Mesh 타입만 인식 | `GroundPlane`, `Cylinder` (Primitive)는 인식 안 됨 |
| 정규표현식 미지원 | `"/World/(ground\|Pillar_.*)"` 같은 Regex 불가 |

**해결책**: 모든 장애물을 `/World/ground` 하위로 이동 → `mesh_prim_paths = ["/World/ground"]` 설정

### 1-2. 환경 로딩 🔧

`go2_rl_play.py`에서 SLAM 환경을 stage에 로드하는 방법.

**시도한 방법들**:
| 방법 | 코드 | 결과 |
|------|------|------|
| `FixedCuboid` 등 | `omni.isaac.core.objects` | Isaac Lab에서 `ModuleNotFoundError` |
| Reference (Fabric ON) | `prim.GetReferences().AddReference()` | 0 children, 안 보임 |
| Reference (Fabric OFF) | 동일 + `--disable_fabric` | 테스트 필요 |
| `TerrainImporterCfg(terrain_type="usd")` | `my_slam_env.py` | **작동 확인** |
| MCP 직접 생성 (현재) | `deploy_scene_mcp.py`로 실행 중인 Isaac Sim에 직접 생성 | **작동 확인** |

**현재 실사용 방식 2가지**:

1. **MySlamEnvCfg** (`my_slam_env.py`) - Isaac Lab 정식 경로:
```python
self.scene.terrain = TerrainImporterCfg(
    terrain_type="usd",
    usd_path="assets/slam_env.usd",
)
```

2. **MCP 직접 생성** (`deploy_scene_mcp.py`) - 실행 중인 Isaac Sim에 바로 추가:
```python
send_isaac("execute_script", {"code": "UsdGeom.Cube.Define(stage, path)..."})
```

### 1-3. RL 정책 로딩 ✅

| 로딩 방식 | 조건 | 코드 |
|-----------|------|------|
| JIT | `.pt` + `exported` 경로 | `torch.jit.load()` |
| Full checkpoint | OnPolicyRunner | `runner.load()` → `get_inference_policy()` |

체크포인트 탐색 순서: `--checkpoint` → pretrained (Nucleus) → 로컬 폴백 2곳

### 1-4. 키보드 제어 ✅

`WasdKeyboard(Se2Keyboard)`:

| 키 | 동작 | 감도 |
|----|------|------|
| W/S | 전진/후진 | 1.0 |
| A/D | 좌/우 이동 | 0.8 |
| Q/E | 회전 | 1.0 |
| K | 정지 | - |

**키보드 제어 필수 설정**:
```python
resampling_time_range = (1.0e9, 1.0e9)  # 랜덤 명령 비활성화
heading_command = False                   # 직접 회전 제어
episode_length_s = 1.0e9                  # 에피소드 리셋 방지
```

### 1-5. Terrain / Curriculum 호환성 ✅

plane terrain 사용 시:
```python
env_cfg.scene.terrain.terrain_type = "plane"
env_cfg.scene.terrain.terrain_generator = None
env_cfg.curriculum.terrain_levels = None   # 필수 (안 하면 AttributeError)
```

### Phase 1 검증
- [x] RL 정책 로딩 성공
- [x] 키보드 WASD + QE 보행/회전 동작
- [x] 에피소드 자동 리셋 없음
- [x] SLAM 환경 생성 (기둥, 벽, 상자)
- [x] 충돌체 적용 (`CollisionAPI` + `PhysxSchema`)
- [x] MCP로 환경 직접 배치 동작 확인
- [ ] **환경 + Go2가 동시에 화면에 표시되는지 최종 확인**

### Phase 1 해결된 에러
| 에러 | 해결 |
|------|------|
| `TerrainImporter has no attribute 'terrain_levels'` | `curriculum.terrain_levels = None` |
| `NoneType has no attribute 'size'` | 동일 |
| `ModuleNotFoundError: omni.isaac.core` | generic USD API로 대체 |
| `Accessed invalid expired Xform prim` | `omni.kit.commands.execute("CreatePrim")` |
| Reference 0 children | `disable_fabric=True` + `SetDefaultPrim` |
| `RayCaster only supports one mesh prim` | `mesh_prim_paths`에 경로 1개만 사용 |
| `Invalid mesh prim path: /World/ground` | USD 내 물체를 Mesh 타입으로 생성 |
| `Prim at path '...' is not valid` | Regex 미지원 → 단일 부모 구조로 해결 |
| Q/E 회전 키 미작동 | `heading_command = False` 설정 필수 |
| 로봇이 기둥 통과 | `UsdPhysics.CollisionAPI.Apply(prim)` 적용 |
| 기둥이 땅에 박힘 | `Z = height / 2` 공식 적용 |

---

## Phase 2-3: RealSense Camera + ROS2 Publishing 🔧 진행 중

**목표**: Go2 base에 Intel RealSense D435 카메라를 마운트하고, RGB/Depth 이미지를 ROS2 토픽으로 퍼블리시하여 RViz2에서 시각화한다.

### 아키텍처 결정

| 방식 | 설명 | 채택 |
|------|------|------|
| Isaac Lab CameraCfg + rclpy | Scene config에 카메라 추가, 메인 루프에서 rclpy로 퍼블리시 | **채택** |
| OmniGraph ROS2 Bridge | OmniGraph Camera Helper 노드 사용 | 미채택 (불필요한 복잡도) |

**채택 이유**: Isaac Lab 환경(`gym.make`)을 이미 사용 중이므로, `CameraCfg`로 센서를 scene에 추가하고 `env.unwrapped.scene["front_cam"]`으로 데이터에 접근하는 것이 가장 자연스러움.

### 핵심 발견사항

- `InteractiveScene._add_entities_from_cfg()`가 `self.cfg.__dict__`를 순회하므로, `__post_init__`에서 동적으로 추가한 scene 속성(카메라 등)도 자동으로 인식됨
- Isaac Lab 카메라 데모: `/home/cvr/Desktop/sj/IsaacLab/scripts/demos/sensors/cameras.py`
- Go2 로봇 body 이름: `base` (prim path: `{ENV_REGEX_NS}/Robot/base`)
- `--enable_cameras` 플래그 필수 (AppLauncher 인자에 이미 포함)

### 2-3-1. 카메라 센서 추가 (`my_slam_env.py`)

`MySlamEnvCfg.__post_init__`에 `CameraCfg` 추가:

```python
from isaaclab.sensors import CameraCfg
import isaaclab.sim as sim_utils

# Intel RealSense D435 스펙 근사
self.scene.front_cam = CameraCfg(
    prim_path="{ENV_REGEX_NS}/Robot/base/front_cam",
    update_period=1/30,                     # 30fps
    height=480,
    width=640,
    data_types=["rgb", "distance_to_image_plane"],
    spawn=sim_utils.PinholeCameraCfg(
        focal_length=15.0,                  # ~69° HFOV (D435 color 근사)
        focus_distance=400.0,
        horizontal_aperture=20.955,
        clipping_range=(0.1, 10.0),         # depth 범위: 0.1~10m
    ),
    offset=CameraCfg.OffsetCfg(
        pos=(0.30, 0.0, 0.05),             # Go2 앞면, base에서 +30cm 전방, +5cm 위
        rot=(0.5, -0.5, 0.5, -0.5),        # ROS convention: +X 방향 정면
        convention="ros",
    ),
)
```

**카메라 파라미터 계산**:
- HFOV = 2 × atan(horizontal_aperture / (2 × focal_length)) = 2 × atan(20.955 / 30) ≈ 69.9°
- RealSense D435 color HFOV: 69.4° → 근사 일치
- depth clipping: 0.1~10m (실내 환경에 적합)

**카메라 마운트 위치**:
- Go2 body 길이 ~0.5m, 높이 ~0.15m
- `pos=(0.30, 0.0, 0.05)`: base 기준 전방 30cm, 높이 5cm (Go2 전면부)
- `rot=(0.5, -0.5, 0.5, -0.5)`: ROS camera convention → robot +X 방향 정면

### 2-3-2. ROS2 퍼블리싱 (`go2_sim.py`)

**수정 사항**:

1. `--enable_cameras` 플래그 기본 활성화
2. `rclpy` 초기화 + Publisher 생성 (env 생성 후)
3. 메인 루프에서 카메라 데이터 읽기 + ROS2 Image 메시지 퍼블리시
4. 종료 시 `rclpy.shutdown()`

**발행 토픽**:

| 토픽 | 타입 | 인코딩 | 용도 |
|------|------|--------|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | `rgb8` | RGB 영상 |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | `32FC1` | Depth 영상 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | - | Intrinsic 파라미터 |

**데이터 접근 방식**:
```python
cam = env.unwrapped.scene["front_cam"]
rgb = cam.data.output["rgb"][0, ..., :3].cpu().numpy()              # (480, 640, 3) uint8
depth = cam.data.output["distance_to_image_plane"][0].cpu().numpy() # (480, 640) float32
intrinsics = cam.data.intrinsic_matrices[0].cpu().numpy()           # (3, 3) float32
```

**이미지 메시지 생성** (cv_bridge 의존성 없이 수동 생성):
```python
from sensor_msgs.msg import Image
msg = Image()
msg.header.stamp = node.get_clock().now().to_msg()
msg.header.frame_id = "camera_link"
msg.height, msg.width = 480, 640
msg.encoding = "rgb8"       # depth: "32FC1"
msg.step = 640 * 3          # depth: 640 * 4
msg.data = numpy_array.tobytes()
```

### 2-3-3. 실행 방법

```bash
# 터미널 1: Isaac Sim + 카메라 + ROS2 퍼블리싱
source /opt/ros/humble/setup.bash
cd /home/cvr/Desktop/sj/isaac-project
python scripts/go2_sim.py --enable_cameras

# 터미널 2: 토픽 확인 + RViz2
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic hz /camera/color/image_raw
rviz2  # → Add → By topic → /camera/color/image_raw
```

### Phase 2-3 검증
- [ ] `ros2 topic list` → 3개 토픽 확인
- [ ] `ros2 topic hz /camera/color/image_raw` → ~30Hz
- [ ] RViz2 Image display → RGB 영상 표시
- [ ] RViz2 Image display → Depth 영상 표시
- [ ] Go2 키보드 이동 시 카메라 영상 실시간 변화 확인

---

## Phase 4: ORB-SLAM3 Config (예정)

- [ ] `config/orb_slam3_rgbd.yaml` 작성
- [ ] `launch/go2_slam.launch.py` 작성
- [ ] 토픽 remapping

---

## Phase 5: Full Integration (예정)

실행 순서: Isaac Sim → ORB-SLAM3 → RViz2

---

## 트러블슈팅

### Isaac Sim
| 이슈 | 해결 |
|------|------|
| ROS2 Bridge 미표시 | `source /opt/ros/humble/setup.bash` 후 실행 |
| 토픽 발행 안됨 | OmniGraph `execIn` 연결 확인 |
| Depth 전부 0 | Camera Helper type=`depth`, clipping range 확인 |

### ORB-SLAM3
| 이슈 | 해결 |
|------|------|
| Feature 0개 | 텍스처 추가, `iniThFAST` 낮추기 |
| QoS 불일치 | `ros2 topic info -v`로 확인 |
| Intrinsics 불일치 | Isaac Sim에서 정확한 값 추출 |

### MCP 연결
| 이슈 | 해결 |
|------|------|
| `Connection refused` | Isaac Sim 미실행 또는 Extension 미로드 → `/isaac-sim-mcp` 워크플로우로 재시작 |
| `EOF` 에러 | 대용량 스크립트 전송 시 타임아웃 → `deploy_scene_mcp.py` 스크립트로 터미널 실행 |
| 8766 포트 확인 | `ss -tlnp \| grep 8766` |

### 관련 스킬
| 스킬 | 용도 |
|------|------|
| `isaac-sim-mcp` | MCP 소켓 연결, execute_script |
| `usd-builder` | USD 환경 제작 3단계 법칙 |
