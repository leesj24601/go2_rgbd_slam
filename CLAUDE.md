# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 외부 참조 규칙
- 코드/API 참고 시 `/home/cvr/Desktop/sj/IsaacLab/` 폴더를 우선 탐색한다.
- 추가 정보가 필요하면 공개된 웹 문서(API docs, GitHub 등)를 참고한다.
- PC 내 다른 폴더 탐색이 필요한 경우 반드시 사용자에게 경로를 확인받는다.

## 프로젝트 개요

Isaac Sim 환경에서 Unitree Go2 로봇에 가상 RealSense 카메라를 장착하고, 키보드로 조작하며 ORB-SLAM3를 통해 맵을 생성하는 프로젝트.

## 실행 명령어

```bash
# 메인 시뮬레이션 (conda lab 환경 필수)
cd /home/cvr/Desktop/sj/isaac-project
/home/cvr/anaconda3/envs/lab/bin/python scripts/go2_sim.py

# 실시간 렌더링 비활성화 (빠른 시뮬레이션)
/home/cvr/anaconda3/envs/lab/bin/python scripts/go2_sim.py --rt false

# ROS2 토픽 확인 (시스템 Python 사용)
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic hz /camera/color/image_raw

# RViz2로 카메라 시각화
rviz2 -d config/go2_sim.rviz

# MCP를 통한 장애물 배치 (Isaac Sim 실행 중에만)
python scripts/deploy_scene_mcp.py
```

## 아키텍처

### 듀얼 Python 환경
- **conda `lab` (Python 3.11)**: Isaac Lab/Sim 실행 전용. `go2_sim.py` 등 시뮬레이션 스크립트 실행.
- **시스템 Python 3.10**: ROS2 Humble 전용. `rclpy`는 conda 환경에서 import 불가.
- **Isaac Lab ↔ ROS2 통신**: OmniGraph ROS2 Bridge Extension을 사용하여 Isaac Sim 내부에서 직접 ROS2 토픽 퍼블리시.

### 핵심 파이프라인

```
go2_sim.py (Python 3.11, Isaac Lab)
  ├─ MySlamEnvCfg로 환경 구성 (USD 지형 + 카메라)
  ├─ RL 정책 로딩 (rsl_rl OnPolicyRunner)
  ├─ WasdKeyboard로 속도 명령 주입
  ├─ OmniGraph ROS2 Bridge로 카메라 데이터 퍼블리시
  └─ 시뮬레이션 루프: keyboard → policy(obs) → env.step
```

### 주요 파일

| 파일 | 역할 |
|------|------|
| `scripts/go2_sim.py` | 메인 시뮬레이션. RL 정책 추론 + WASD 키보드 제어 + ROS2 카메라 OmniGraph 설정 |
| `scripts/my_slam_env.py` | `MySlamEnvCfg`: USD 지형 로딩, 카메라 센서, 커리큘럼/명령 설정 |
| `scripts/cli_args.py` | RSL-RL CLI argument 파서 유틸리티 |
| `scripts/deploy_scene_mcp.py` | MCP 소켓(port 8766)으로 Isaac Sim에 장애물 동적 배치 |
| `assets/slam_env.usd` | SLAM 환경 USD 파일 |
| `config/go2_sim.rviz` | RViz2 시각화 설정 |

## 중요 제약사항

### AppLauncher 관련
- `--enable_cameras` 플래그는 AppLauncher가 자체 추가하므로 argparse에 수동 등록하면 충돌 발생
- `args_cli.enable_cameras = True`를 파싱 후 직접 설정하는 방식 사용

### Isaac Lab import 순서
- `AppLauncher` 생성/실행 **이후에만** Isaac Lab 모듈(`isaaclab_rl`, `isaaclab_tasks`, `omni.*` 등) import 가능. 순서 어기면 crash.

### CameraCfg 동적 추가
- `MySlamEnvCfg.__post_init__`에서 `self.scene.front_cam = CameraCfg(...)`로 추가
- `InteractiveScene`이 `cfg.__dict__`를 순회하므로 동적 속성도 자동 인식됨

### RayCaster 센서 제약
- `mesh_prim_paths`에 경로 1개만 허용 (복수 경로 → `NotImplementedError`)
- Mesh 타입만 인식 (`GroundPlane`, Primitive Cylinder 인식 안 됨)
- 정규표현식 미지원 → 모든 장애물을 단일 부모(`/World/ground`) 하위로 구성

### 키보드 제어 필수 설정
- `resampling_time_range = (1.0e9, 1.0e9)` — 랜덤 명령 자동 재생성 비활성화
- `heading_command = False` — Q/E 직접 회전 제어 활성화
- `episode_length_s = 1.0e9` — 에피소드 자동 리셋 방지

### ROS2 토픽
| 토픽 | 타입 | 용도 |
|------|------|------|
| `camera/color/image_raw` | `sensor_msgs/Image` (rgb8) | RGB 영상 |
| `camera/depth/image_rect_raw` | `sensor_msgs/Image` (32FC1) | Depth 영상 |
| `camera/camera_info` | `sensor_msgs/CameraInfo` | 카메라 intrinsics |

## USD 환경 제작 규칙
1. `UsdGeom.Cube/Cylinder` + `CollisionAPI` + `PhysxSchema` 사용
2. 단일 부모 Xform 하위에 모든 물체 묶기 (RayCaster 호환)
3. Z = 높이 / 2 (지면 밀착 공식)
