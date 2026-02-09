---
name: usd-builder
description: "Isaac Sim용 USD 환경(맵) 제작 가이드. 절대 실패하지 않는 3단계 법칙. Use when: usd, 환경 제작, 맵 만들기, slam 환경, isaac sim 환경, 장애물, obstacle, scene build"
---

# USD 환경 제작 스킬 (Isaac Sim)

Isaac Sim MCP(`execute_script`)를 통해 SLAM/로봇용 환경(벽, 장애물, 기둥 등)을 USD로 제작하고 저장하는 가이드.

## 절대 실패하지 않는 3단계 법칙

### Rule 1: Mesh 타입만 사용하라

Isaac Sim에서 물체를 만들 때 **반드시 `UsdGeom.Mesh` 계열**을 사용한다.

- GUI: `Create -> Mesh -> Cube / Cylinder / Sphere` (신규)
- API: `UsdGeom.Cube`, `UsdGeom.Cylinder`, `UsdGeom.Mesh` 등 **UsdGeom 계열**
- `omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type="Cube")`

**사용 금지 (Isaac Lab/Standalone에서 로드 시 문제 발생 가능)**:
- `FixedCuboid`, `FixedCylinder` 등 `omni.isaac.core.objects` 클래스
- `Create -> Physics -> ...` 메뉴

**이유**: `omni.isaac.core.objects`는 내부적으로 Physics Schema를 자동 적용하는데, 이것이 Isaac Lab의 ManagerBasedRLEnv와 충돌하거나 Fabric 모드에서 무시될 수 있다.

#### Mesh로 물체 만드는 코드 패턴

```python
import omni.usd
from pxr import UsdGeom, UsdPhysics, Gf, Sdf

stage = omni.usd.get_context().get_stage()

# 박스 (벽, 장애물)
def create_box(path, position, size, color=None):
    """Mesh 기반 박스 생성. size=[width, depth, height]"""
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    xform.AddScaleOp().Set(Gf.Vec3d(*size))

    cube = UsdGeom.Cube.Define(stage, f"{path}/Mesh")
    cube.CreateSizeAttr(1.0)  # unit cube, scale로 크기 조절

    # 충돌체 (물리 반응 필요 시)
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    if color:
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])

# 실린더 (기둥)
def create_cylinder(path, position, radius, height, color=None):
    """Mesh 기반 실린더 생성."""
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))

    cyl = UsdGeom.Cylinder.Define(stage, f"{path}/Mesh")
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(height)
    cyl.CreateAxisAttr("Z")

    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())

    if color:
        cyl.CreateDisplayColorAttr([Gf.Vec3f(*color)])
```

### Rule 2: 단일 바구니 구조 (가장 중요)

모든 장애물은 **반드시 하나의 부모 Xform** 아래에 묶는다.

```
/World
└── Environment          <-- 단일 바구니 (DefaultPrim 후보)
    ├── ground           <-- 바닥
    ├── Pillar_1
    ├── Pillar_2
    ├── Wall_N
    ├── Wall_S
    ├── Wall_E
    ├── Wall_W
    └── Box_1
```

**왜 중요한가?**
- USD를 Reference로 불러올 때 `DefaultPrim` 하나만 가져온다
- 물체가 `/World` 바로 아래 흩어져 있으면 Reference 로드 시 일부만 가져오거나 아예 빈 결과가 된다
- 하나의 Xform 안에 묶으면 그 Xform을 DefaultPrim으로 지정하여 **한 번에 전부** 가져올 수 있다

```python
# 부모 Xform 생성
omni.kit.commands.execute("CreatePrim", prim_path="/World/Environment", prim_type="Xform")

# 모든 물체를 /World/Environment 하위에 생성
create_box("/World/Environment/Wall_N", ...)
create_box("/World/Environment/Wall_S", ...)
create_cylinder("/World/Environment/Pillar_1", ...)
```

#### DefaultPrim 설정 (USD 저장 전 필수)

```python
env_prim = stage.GetPrimAtPath("/World/Environment")
stage.SetDefaultPrim(env_prim)
omni.usd.get_context().save_as_stage("/path/to/scene.usd")
```

### Rule 3: 지면 밀착 Z-offset 공식

물체가 바닥에 반쯤 파묻히는 것을 방지하려면:

```
Z 위치 = 물체의 실제 높이 / 2
```

| 물체 | 높이(h) | Z 위치 |
|------|---------|--------|
| 2m 벽 | 2.0 | 1.0 |
| 1.5m 기둥 | 1.5 | 0.75 |
| 0.6m 박스 | 0.6 | 0.3 |

**예외**: 물체의 Pivot(기준점)이 바닥면에 있다면 Z = 0. 확신이 없으면 Isaac Sim GUI에서 물체를 바닥에 놓아보고 Translate Z 값을 확인한다.

```python
h = 2.0  # 벽 높이
create_box("/World/Environment/Wall_N",
    position=[0, 6.0, h/2],   # Z = h/2 로 지면 밀착
    size=[12.0, 0.15, h],
)
```

## 전체 환경 제작 템플릿

아래는 MCP `execute_script`로 보내는 전체 코드 템플릿이다.

```python
import omni.usd
import omni.kit.commands
from pxr import UsdGeom, UsdPhysics, UsdLux, Gf, Sdf
import math, random

random.seed(42)

# ── 0. 새 스테이지 ──
omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()

# ── 1. 부모 Xform (단일 바구니) ──
omni.kit.commands.execute("CreatePrim", prim_path="/World", prim_type="Xform")
omni.kit.commands.execute("CreatePrim", prim_path="/World/Environment", prim_type="Xform")

# ── 2. 바닥 (Mesh 기반) ──
ground_xform = UsdGeom.Xform.Define(stage, "/World/Environment/ground")
ground_cube = UsdGeom.Cube.Define(stage, "/World/Environment/ground/Mesh")
ground_cube.CreateSizeAttr(1.0)
ground_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
ground_xform.AddScaleOp().Set(Gf.Vec3d(50, 50, 0.1))
ground_cube.CreateDisplayColorAttr([Gf.Vec3f(0.45, 0.45, 0.45)])
UsdPhysics.CollisionAPI.Apply(ground_cube.GetPrim())

# ── 3. 조명 ──
sun = UsdLux.DistantLight.Define(stage, "/World/Environment/Sun")
sun.CreateIntensityAttr(2500)
sun.CreateAngleAttr(1.0)
xf = UsdGeom.Xformable(sun.GetPrim())
xf.AddRotateXYZOp().Set(Gf.Vec3f(-45, 30, 0))

dome = UsdLux.DomeLight.Define(stage, "/World/Environment/DomeLight")
dome.CreateIntensityAttr(300)

# ── 4. 벽/장애물 생성 (Rule 1: Mesh, Rule 3: Z=h/2) ──
def create_box(name, pos, size, color):
    path = f"/World/Environment/{name}"
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddScaleOp().Set(Gf.Vec3d(*size))
    cube = UsdGeom.Cube.Define(stage, f"{path}/Mesh")
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

def create_cylinder(name, pos, radius, height, color):
    path = f"/World/Environment/{name}"
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    cyl = UsdGeom.Cylinder.Define(stage, f"{path}/Mesh")
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(height)
    cyl.CreateAxisAttr("Z")
    cyl.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())

# 외곽 벽 (12m x 12m)
WALL_H = 2.0
create_box("Wall_N", [0, 6.0, WALL_H/2],  [12.15, 0.15, WALL_H], [0.7, 0.3, 0.2])
create_box("Wall_S", [0, -6.0, WALL_H/2], [12.15, 0.15, WALL_H], [0.7, 0.3, 0.2])
create_box("Wall_E", [6.0, 0, WALL_H/2],  [0.15, 12.15, WALL_H], [0.2, 0.4, 0.7])
create_box("Wall_W", [-6.0, 0, WALL_H/2], [0.15, 12.15, WALL_H], [0.2, 0.4, 0.7])

# 랜덤 실린더 장애물
for i in range(12):
    angle = random.uniform(0, 2 * math.pi)
    dist = random.uniform(2.0, 5.5)
    x, y = dist * math.cos(angle), dist * math.sin(angle)
    r = random.uniform(0.15, 0.35)
    h = random.uniform(0.8, 2.5)
    create_cylinder(f"Pillar_{i}", [x, y, h/2], r, h,
                    [random.random(), random.random(), random.random()])

# 랜덤 박스 장애물
for i in range(8):
    angle = random.uniform(0, 2 * math.pi)
    dist = random.uniform(1.8, 5.0)
    x, y = dist * math.cos(angle), dist * math.sin(angle)
    sx, sy = random.uniform(0.3, 0.8), random.uniform(0.3, 0.8)
    sz = random.uniform(0.5, 1.8)
    create_box(f"Box_{i}", [x, y, sz/2], [sx, sy, sz],
               [random.random(), random.random(), random.random()])

# ── 5. DefaultPrim 설정 + 저장 ──
env_prim = stage.GetPrimAtPath("/World/Environment")
stage.SetDefaultPrim(env_prim)

save_path = "/home/cvr/Desktop/sj/isaac-project/assets/slam_env.usd"
import os
os.makedirs(os.path.dirname(save_path), exist_ok=True)
omni.usd.get_context().save_as_stage(save_path)
print(f"[MCP] Saved: {save_path}")
```

## 로드 방법 (go2_rl_play.py 등에서)

### 방법 1: Reference (DefaultPrim 필수)

```python
import omni.usd
stage = omni.usd.get_context().get_stage()
prim = stage.DefinePrim("/World/SlamEnv", "Xform")
prim.GetReferences().AddReference(scene_path)
```

- DefaultPrim이 설정되어 있어야 함
- `/World/SlamEnv/ground`, `/World/SlamEnv/Wall_N` 등으로 매핑됨
- `--disable_fabric` 필수 (Isaac Lab 환경)

### 방법 2: Sublayer (namespace 동일하게 합성)

```python
root_layer = stage.GetRootLayer()
root_layer.subLayerPaths.append(scene_path)
```

- USD의 prim 경로가 그대로 stage에 합쳐짐
- `/World/Environment/Wall_N` 이 현재 stage의 같은 경로에 나타남
- DefaultPrim 설정 불필요 (있어도 무방)

### Isaac Lab에서 로드 시 주의사항

| 항목 | 설정 |
|------|------|
| `--disable_fabric` | `True` (필수! Fabric 모드에서 USD 변경 반영 안 됨) |
| 로드 타이밍 | `gym.make()` 전에 로드 (물리 초기화 전) |
| terrain_type | `"plane"` (SLAM 환경은 평면 위에 장애물 배치) |
| terrain_generator | `None` |
| curriculum.terrain_levels | `None` (plane에서는 terrain_levels 접근 시 에러) |

## MCP로 보내는 방법

### 헬퍼 스크립트 사용

```bash
python3 /home/cvr/Desktop/sj/isaac-project/scripts/deploy_scene_mcp.py
```

## 흔한 실수와 해결

| 실수 | 증상 | 해결 |
|------|------|------|
| `FixedCuboid` 사용 | 로드 시 물체가 안 보임 | `UsdGeom.Cube` + `CollisionAPI` 사용 |
| 물체가 `/World` 바로 아래 흩어짐 | Reference 로드 시 0 children | 단일 Xform(`/World/Environment`) 하위로 묶기 |
| DefaultPrim 미설정 | Reference 로드 시 빈 결과 | `stage.SetDefaultPrim(env_prim)` |
| Z=0에 물체 배치 | 바닥에 반쯤 파묻힘 | Z = height / 2 |
| Fabric 모드 ON | 런타임 USD 변경 무시 | `--disable_fabric` |
| `gym.make()` 후 USD 로드 | 물리 엔진이 새 물체 무시 | `gym.make()` **전에** 로드 |

## 관련 스킬

- `isaac-sim-mcp`: MCP 소켓 연결, 서버 관리, 명령어 목록
