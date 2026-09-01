# PX4 Boat (USV) 개발 계획

## 목차
1. [현재 구현 분석 - 쌍동선(Catamaran)](#1-현재-구현-분석---쌍동선catamaran)
2. [일반 보트 개발 계획 - 단일 선체 + 프로펠러 + 러더](#2-일반-보트-개발-계획---단일-선체--프로펠러--러더)
  - **현재 계획 대상: Gazebo Classic** (`gazebo-classic_boat_mono`)
3. [마일스톤별 구현 로드맵](#3-마일스톤별-구현-로드맵)
4. [Gz (New Gazebo) 포팅 계획](#4-gz-new-gazebo-포팅-계획)

---

## 1. 현재 구현 분석 - 쌍동선(Catamaran)

### 1.1 관련 파일 목록

| 파일 | 역할 |
|------|------|
| `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/boat/boat.sdf.jinja` | Gazebo 시뮬레이션 모델 (SDF) |
| `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/boat/meshes/` | 3D 메시 파일 (body.dae, propeller.dae, engine.dae, battery.dae) |
| `ROMFS/px4fmu_common/init.d-posix/airframes/1070_gazebo-classic_boat` | SITL용 airframe 파라미터 설정 |
| `ROMFS/px4fmu_common/init.d/rc.boat_defaults` | 공통 보트 기본 파라미터 |
| `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` (line 66) | airframe 빌드 등록 |

### 1.2 Gazebo SDF 모델 구조

#### 물리 특성

| 항목 | 값 |
|------|-----|
| 전체 질량 | 227 kg |
| 선체 형태 | 쌍동선 (좌/우 부체 각각 길이 4m, 반지름 0.2m 원통) |
| 선체 폭 (boatWidth) | 2.4 m (Y축 ±1.03m) |
| 선체 길이 (boatLength) | 4.9 m |
| 흘수 반경 (hullRadius) | 0.213 m |
| 수밀도 | 997.8 kg/m³ |

#### 링크 구성

```
base_link
├── left_propeller_link   (pose: x=-2.65, y=+1.03, z=-0.19)
│   └── joint: left_engine_propeller_joint (revolute, X축)
├── right_propeller_link  (pose: x=-2.65, y=-1.03, z=-0.19)
│   └── joint: right_engine_propeller_joint (revolute, X축)
├── boat/imu_link
└── gps::link (fixed)
```

#### Gazebo 플러그인

| 플러그인 | 역할 |
|---------|------|
| `libgazebo_motor_model.so` (×2) | 좌/우 프로펠러 모터 시뮬레이션 |
| `libgazebo_usv_dynamics_plugin.so` | 수상 선박 6DOF 수력학적 동역학 |
| `libgazebo_mavlink_interface.so` | PX4↔Gazebo MAVLink 연결 |
| `libgazebo_imu_plugin.so` | IMU 센서 |
| `libgazebo_magnetometer_plugin.so` | 자력계 |
| `libgazebo_barometer_plugin.so` | 기압계 |

#### 수력학 파라미터 (USV Dynamics)

| 파라미터 | 값 | 의미 |
|---------|----|------|
| `xU` | 51.3 | 전진 선형 저항 |
| `xUU` | 72.4 | 전진 이차 저항 |
| `yV` | 102.6 | 횡방향 선형 저항 |
| `zW` | 500.0 | 수직 복원력 |
| `nR` | 400.0 | 요(yaw) 선형 저항 |

#### MAVLink 제어 채널

| 채널 | 입력 인덱스 | 대상 조인트 | 제어 방식 |
|------|------------|------------|---------|
| `left_rotor` | 0 | `left_engine_propeller_joint` | velocity |
| `right_rotor` | 1 | `right_engine_propeller_joint` | velocity |

### 1.3 Airframe 설정 (`1070_gazebo-classic_boat`)

#### Control Allocator 설정

```sh
CA_AIRFRAME 9          # Custom (레거시 방식)
CA_ROTOR_COUNT 2
CA_ROTOR0_PX -2, CA_ROTOR0_PY -1   # 좌현 프로펠러 위치
CA_ROTOR1_PX -2, CA_ROTOR1_PY +1   # 우현 프로펠러 위치
CA_ROTOR0_AX 1, CA_ROTOR1_AX 1     # 모터 추력 방향: +X (전진)
CA_ROTOR0_KM 0, CA_ROTOR1_KM 0     # 토크 계수 0 (수중 프로펠러)
CA_R_REV 3             # 모터 0, 1 모두 역방향 가능 (비트마스크)
PWM_MAIN_FUNC1 101     # 출력 ch1 → 모터 1 (좌)
PWM_MAIN_FUNC2 102     # 출력 ch2 → 모터 2 (우)
```

#### 방향 제어 원리 (차동 추력)

```
직진:    좌 모터 = 우 모터  (동일 속도)
우회전:  좌 모터 > 우 모터
좌회전:  좌 모터 < 우 모터
제자리 회전: 한쪽 역방향 가능
```

#### 레거시 파라미터 (GND_*)

```sh
GND_L1_DIST, GND_L1_PERIOD    # L1 경로 추종 파라미터
GND_SP_CTRL_MODE 1             # 속도 제어 모드
GND_SPEED_P/I/D                # 속도 PID
GND_THR_CRUISE 0.85            # 순항 스로틀
GND_MAX_ANG 0.6                # 최대 조향각
GND_WHEEL_BASE 2               # 바퀴간 거리 (쌍동선 폭 대응)
```

### 1.4 ⚠️ 현재 구현의 문제점

1. **제어 모듈 없음**: `rc.boat_defaults`에서 `set VEHICLE_TYPE rover`로 설정하지만,
  `ROMFS/px4fmu_common/init.d/rc.rover_apps` 파일이 존재하지 않아 **고수준 자율 제어 모듈이 시작되지 않음**.

2. **GND_ 파라미터 레거시**: `GND_SP_CTRL`, `GND_L1_*` 등을 사용하던 `gnd_control` 모듈이
  PX4 메인 브랜치에서 제거됨. 해당 파라미터들은 현재 어떤 모듈도 소비하지 않는 dead code.

3. **자율 비행 불가**: Manual/Stabilized 조작은 가능하지만 Mission, Auto 모드에서
  경로 추종 불가.

### 1.5 쌍동선 제어 장단점 요약

| | 장점 | 단점 |
|-|------|------|
| 제어 | 제자리 회전 가능, 저속에서도 방향 제어 | 좌우 모터 속도 차이 비선형성, 에너지 비효율 |
| 구조 | 안정성 높음 (복원력 큼) | 복잡한 2채널 제어 |
| 시뮬 | 모터 모델 2개 독립 동작 | 러더 없어 단순하지만 실제 해상 표적 추적에는 부적합 |

---

## 2. 일반 보트 개발 계획 - 단일 선체 + 프로펠러 + 러더

### 2.1 목표 형상

```
    [선수(bow)]
        ↑
   ┌────────────┐
   │            │  ← 단일 선체 (monohull)
   │  [GPS/IMU] │
   │            │
   └─────┬──────┘
         │ [프로펠러 shaft]
        [●] ← 프로펠러 (선미)
         │
        [|] ← 러더 (프로펠러 후방, 회전 servo)
```

**제어 방식**: 스로틀(프로펠러 RPM) + 러더 각도(서보)
→ PX4의 **Ackermann Rover** 구조와 동일한 제어 토폴로지

### 2.2 아키텍처 비교

| 항목 | 쌍동선 (현재) | 일반 보트 (목표) |
|------|-------------|---------------|
| CA_AIRFRAME | 9 (Custom) | 5 (Rover Ackermann) |
| VEHICLE_TYPE | rover (레거시) | rover_ackermann |
| 추진 | 모터 2개 (차동) | 모터 1개 (프로펠러) |
| 조향 | 모터 속도 차이 | 러더 서보 |
| 저속 조향 | 가능 | **불가** (러더는 속도 의존적) |
| 제자리 회전 | 가능 | **불가** |
| 제어 모듈 | 없음 (레거시) | rover_ackermann |
| PWM 출력 | ch1=모터L, ch2=모터R | ch1=모터, SV1=러더 |

### 2.3 개발 항목 상세

---

#### Phase 1: Gazebo-Classic SDF 모델 신규 제작

**파일**: `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/boat_mono/`

##### 1-1. 선체 (Hull) 형상

단일 선체를 base_link에 구성:

```xml
<link name="base_link">
 <!-- 선체: 길이 4.5m, 폭 0.8m, 흘수 0.3m 선박 -->
 <collision name="hull_collision">
   <pose>0 0 0.1 0 0 0</pose>
   <geometry>
     <box><size>4.5 0.8 0.6</size></box>
   </geometry>
 </collision>
 <!-- 질량: 150~200 kg (쌍동선 227kg보다 가벼움) -->
 <inertial>
   <mass>150</mass>
   <inertia>
     <ixx>60</ixx>   <!-- 롤: 좁은 선폭 -->
     <iyy>300</iyy>  <!-- 피치: 긴 선체 -->
     <izz>320</izz>  <!-- 요: 긴 선체 -->
   </inertia>
 </inertial>
</link>
```

##### 1-2. 프로펠러 (Propeller) - revolute joint

```xml
<link name="propeller_link">
 <pose>-2.0 0 -0.25 0 1.5708 0</pose>  <!-- 선미 중앙 수선 아래 -->
</link>
<joint name="propeller_joint" type="revolute">
 <child>propeller_link</child>
 <parent>base_link</parent>
 <axis><xyz>1 0 0</xyz></axis>  <!-- X축 회전 -->
</joint>
```

##### 1-3. 러더 (Rudder) - revolute joint (핵심 신규 요소)

```xml
<link name="rudder_link">
 <pose>-2.3 0 -0.2 0 0 0</pose>  <!-- 프로펠러 바로 뒤 -->
 <inertial>
   <mass>0.5</mass>
   <inertia><ixx>0.001</ixx><iyy>0.002</iyy><izz>0.001</izz></inertia>
 </inertial>
 <collision name="rudder_collision">
   <geometry>
     <box><size>0.05 0.3 0.4</size></box>  <!-- 폭×높이 러더판 -->
   </geometry>
 </collision>
</link>
<joint name="rudder_joint" type="revolute">
 <child>rudder_link</child>
 <parent>base_link</parent>
 <axis>
   <xyz>0 0 1</xyz>                     <!-- Z축 회전 (좌우 꺾임) -->
   <limit><lower>-0.52</lower><upper>0.52</upper></limit>  <!-- ±30° -->
   <dynamics><damping>0.5</damping></dynamics>
 </axis>
</joint>
```

##### 1-4. USV Dynamics 플러그인 파라미터 (단일선체 기준 조정)

쌍동선 대비 변경 필요 항목:

| 파라미터 | 쌍동선 값 | 단일선체 값 (제안) | 이유 |
|---------|---------|--------------|------|
| `hullRadius` | 0.213 | 0.35 | 더 깊은 흘수 |
| `boatWidth` | 2.4 | 0.8 | 단일 선체 폭 |
| `boatLength` | 4.9 | 4.5 | 선체 길이 |
| `yV` | 102.6 | 180.0 | 횡방향 저항 증가 (좁은 선폭) |
| `nR` | 400.0 | 350.0 | 요 저항 조정 |
| `length_n` | 2 | 1 | 선체 개수 1개 |

##### 1-5. MAVLink 제어 채널 (쌍동선과 가장 큰 차이)

```xml
<control_channels>
 <!-- ch0: 모터 (프로펠러 RPM) -->
 <channel name="propeller">
   <input_index>0</input_index>
   <input_scaling>100</input_scaling>
   <joint_control_type>velocity</joint_control_type>
   <joint_name>propeller_joint</joint_name>
 </channel>
 <!-- ch1: 러더 서보 (각도 제어) -->
 <channel name="rudder">
   <input_index>1</input_index>
   <input_scaling>0.52</input_scaling>      <!-- 최대 러더각 0.52 rad -->
   <zero_position_disarmed>0</zero_position_disarmed>
   <zero_position_armed>0</zero_position_armed>
   <joint_control_type>position</joint_control_type>  <!-- velocity 아닌 position -->
   <joint_name>rudder_joint</joint_name>
 </channel>
</control_channels>
```

##### 1-6. 신규 추가: 러더 유체역학 플러그인 (선택 사항)

`libgazebo_usv_dynamics_plugin.so`는 러더 힘을 별도로 모델링하지 않음.
러더 효과를 현실적으로 구현하려면 다음 중 하나:

- **옵션 A**: PID 기반 간이 러더 플러그인 신규 작성 (C++ Gazebo plugin)
 - 속도에 비례한 횡력 생성: `F_rudder = 0.5 * ρ * v² * A * CL * sin(δ)`
- **옵션 B**: USV dynamics 플러그인의 `nR` 파라미터만으로 근사 (간단, 정확도 낮음)
- **옵션 C**: Gazebo 기본 물리 엔진에 러더 링크 면적 충분히 설정하여 항력으로 조향 근사

초기 개발에는 **옵션 B → C 순서**로 진행하고, 정확도 필요 시 옵션 A 구현.

---

#### Phase 2: Airframe 설정 파일 신규 작성

**파일**: `ROMFS/px4fmu_common/init.d-posix/airframes/1071_gazebo-classic_boat_mono`

```sh
#!/bin/sh
# @name Boat Mono (Single Hull USV)
# @type Rover
# @class Rover

. ${R}etc/init.d/rc.rover_ackermann_defaults

# VEHICLE_TYPE=rover_ackermann → rc.rover_ackermann_apps 실행 → rover_ackermann start

param set-default MAV_TYPE 11   # MAV_TYPE_SURFACE_BOAT

# Pure Pursuit 경로 추종
param set-default PP_LOOKAHD_GAIN 1.5
param set-default PP_LOOKAHD_MAX 15
param set-default PP_LOOKAHD_MIN 2

# Ackermann(러더) 파라미터
param set-default RA_WHEEL_BASE 2.0        # 프로펠러-러더 간 거리 (steering arm)
param set-default RA_MAX_STR_ANG 0.52      # 최대 러더각 30° (rad)
param set-default RA_STR_RATE_LIM 30       # 러더 슬루율 (deg/s) - 수중 서보 특성
param set-default RA_ACC_RAD_GAIN 2
param set-default RA_ACC_RAD_MAX 5

# 속도/스로틀 제어
param set-default RO_MAX_THR_SPEED 3.0     # 최대 속도 (m/s)
param set-default RO_SPEED_LIM 2.5
param set-default RO_SPEED_P 1.2
param set-default RO_SPEED_I 0.02
param set-default RO_ACCEL_LIM 1.0         # 보트는 관성이 크므로 제한
param set-default RO_DECEL_LIM 1.5
param set-default RO_JERK_LIM 3

# Yaw rate 제어 (러더 경유)
param set-default RO_YAW_RATE_P 0.3
param set-default RO_YAW_RATE_I 0.01
param set-default RO_YAW_RATE_LIM 60       # 보트는 yaw rate 제한 (deg/s)
param set-default RO_YAW_P 2.5
param set-default RO_YAW_EXPO 0.6

# 웨이포인트 수락 반경
param set-default NAV_ACC_RAD 1.0          # 수상: 1m (쌍동선 0.5m보다 여유)

# Control Allocator: Ackermann
param set-default CA_AIRFRAME 5            # Rover (Ackermann)
param set-default CA_R_REV 1              # 프로펠러 역추진 가능
param set-default CA_SV_CS_COUNT 1        # 조향 서보 1개 (러더)

# PWM 출력 매핑
param set-default PWM_MAIN_FUNC1 101       # ch1 → 모터 (프로펠러)
param set-default PWM_MAIN_FUNC2 201       # ch2 → 서보 1 (러더)
```

**CMakeLists.txt 등록 필요**:
```cmake
# ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
1071_gazebo-classic_boat_mono
```

---

#### Phase 3: 제어 모듈 적용 및 파라미터 튜닝

##### 3-1. rover_ackermann 모듈 재사용 가능성 검토

`src/modules/rover_ackermann/` 모듈은 다음 구조를 가짐:

```
rover_ackermann
├── AckermannActControl    ← 모터(스로틀) + 서보(러더) 출력
├── AckermannRateControl   ← Yaw rate PID
├── AckermannAttControl    ← Heading 제어
├── AckermannSpeedControl  ← 속도 PID
├── AckermannPosControl    ← 위치/경로 추종 (Pure Pursuit)
└── AckermannDriveModes    ← Manual/Auto/Offboard 모드
```

일반 보트의 프로펠러+러더 토폴로지는 **Ackermann 구조와 동일**하므로 모듈 재사용 가능.

단, 다음 특성 차이로 인한 파라미터 조정 필수:

| 차이점 | 지상 Ackermann | 수상 보트 | 대응 방법 |
|--------|--------------|---------|---------|
| 조향 응답 | 즉각적 (바퀴) | 지연 있음 (러더+관성) | `RA_STR_RATE_LIM` 낮춤 |
| 저속 조향 | 가능 | **불가** (최소 속도 필요) | `RO_SPEED_LIM` 하한 설정 |
| 외력 | 마찰 지배 | 조류/바람/파도 | `RO_SPEED_I` 증가 |
| 가속/감속 | 빠름 | 느림 (관성 큼) | `RO_ACCEL_LIM`, `RO_DECEL_LIM` 낮춤 |
| 요 관성 | 작음 | 큼 | `RO_YAW_RATE_P` 낮춤 |

##### 3-2. 저속 최소 속도 처리 (중요)

러더는 유속에 비례하여 횡력을 생성하므로, 정지 상태에서 조향 불가능.
이를 위해 Auto 모드 진입 시 최소 전진 속도 보장 로직 검토:

- `RO_SPEED_LIM` (하한값) 설정으로 일부 완화 가능
- 또는 rover_ackermann 모듈 내 `AckermannPosControl`에서 heading error에
 비례하여 minimum throttle 적용하는 로직 추가 필요 (커스텀 수정)

---

#### Phase 4: 오픈소스 3D 모델 적용 (옵션 A 구체 절차)

##### 4-0. 모델 요구사항 (10m 보트 기준)

| 항목 | 목표값 |
|------|--------|
| 전장 | 10 m |
| 폭 | 3.0~3.5 m |
| 파일 형식 | `.dae` (Collada) — Gazebo-Classic 지원 형식 |
| 좌표 기준 | +X = 선수, +Z = 상방 |
| 단위 | 미터(m) |

---

##### 4-1. 소스 선택: VRX noetic-devel (Gazebo Classic 전용 브랜치)

VRX(Virtual RobotX) 저장소의 `noetic-devel` 브랜치는 Gazebo Classic용으로 검증된
선박 `.dae` 메시를 포함한다. 메인 브랜치(`jazzy`)는 신형 Gazebo 전용이므로 반드시
`noetic-devel`을 사용한다.

```bash
# PX4-Autopilot 외부 임시 디렉토리에 클론
cd ~
git clone --depth=1 --branch noetic-devel https://github.com/osrf/vrx.git vrx_noetic

# 포함된 선박 메시 확인
find vrx_noetic/vrx_gazebo/models -name "*.dae" | sort
```

주요 메시 파일 위치:

```
vrx_noetic/vrx_gazebo/models/
├── wamv_base/meshes/
│   ├── WAM-V-Base.dae       ← 선체 구조물 (중앙 데크)
│   ├── Propeller.dae        ← 프로펠러 (재사용 가능)
│   └── Thruster_Arm.dae     ← 엔진 마운트
├── wamv_cylinder_hull/meshes/
│   └── Hull.dae             ← 원통형 부체 (쌍동선용)
└── wamv_pinger_sensor/meshes/
   └── ...
```

> WAM-V는 쌍동선 형태이므로 선체 메시보다 **Propeller.dae**를 프로펠러 파트로 재사용하고,
> 선체(hull)는 아래 4-2에서 별도로 확보한다.

---

##### 4-2. 단일선체 메시 확보: asv_wave_sim (Gazebo Classic 단일선체 USV)

`asv_wave_sim`은 Gazebo Classic 호환 단일선체 USV 모델을 포함한다.

```bash
cd ~
git clone --depth=1 https://github.com/srmainwaring/asv_wave_sim.git

# 단일선체 보트 메시 확인
find asv_wave_sim -name "*.dae" -o -name "*.stl" | sort
```

포함 모델 경로:

```
asv_wave_sim/asv_wave_sim_gazebo/models/
├── asv_wave_sim_boat/
│   └── meshes/
│       └── boat.dae    ← 단일선체 USV 메시 (직접 사용 가능)
```

---

##### 4-3. 메시 파일 복사 및 배치

PX4 sitl_gazebo-classic의 `boat_mono` 모델 디렉토리에 메시 파일을 복사한다.

```bash
MODEL_DIR="Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/boat_mono"

# 모델 디렉토리 생성
mkdir -p ${MODEL_DIR}/meshes

# 단일선체: asv_wave_sim에서 hull 메시 복사
cp ~/asv_wave_sim/asv_wave_sim_gazebo/models/asv_wave_sim_boat/meshes/boat.dae \
  ${MODEL_DIR}/meshes/hull.dae

# 프로펠러: VRX에서 복사 (또는 기존 PX4 boat 모델에서 복사)
cp ~/vrx_noetic/vrx_gazebo/models/wamv_base/meshes/Propeller.dae \
  ${MODEL_DIR}/meshes/propeller.dae

# 러더: 기존 PX4 boat 모델에는 별도 러더 메시 없음 → 프로펠러 메시 재활용하거나
#        초기에는 cylinder primitive로 대체 (4-4 참조)
```

asv_wave_sim에 boat.dae가 없는 경우, 기존 쌍동선 모델의 엔진 메시를 임시 사용:

```bash
# fallback: 기존 boat 모델 메시 활용
cp Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/boat/meshes/engine.dae \
  ${MODEL_DIR}/meshes/hull.dae
```

---

##### 4-4. 메시 스케일 및 좌표 검증

복사한 메시가 10m 기준에 맞는지 `assimp`로 확인한다.

```bash
# assimp 설치 (macOS)
brew install assimp

# 메시 정보 확인 (크기, 폴리곤 수, 좌표계)
assimp info ${MODEL_DIR}/meshes/hull.dae
```

출력에서 확인할 항목:

```
Bounding box: X=[-5.0, 5.0]  → 전장 10m (X축), 선수가 +X 방향인지 확인
            Y=[-1.5, 1.5]  → 선폭 3m
            Z=[-0.5, 0.8]  → 흘수/건현 높이
```

전장이 10m가 아닌 경우 SDF `<scale>` 태그로 보정 (4-5 참조).

---

##### 4-5. SDF에서 메시 참조 및 스케일 적용

`boat_mono.sdf.jinja` 내 메시 참조:

```xml
<link name="base_link">
 <!-- visual: 오픈소스 메시 사용 -->
 <visual name="hull_visual">
   <pose>0 0 0 0 0 0</pose>
   <geometry>
     <mesh>
       <!-- 메시 전장이 5m인 경우 scale을 2로 설정하여 10m로 조정 -->
       <scale>2 2 2</scale>
       <uri>model://boat_mono/meshes/hull.dae</uri>
     </mesh>
   </geometry>
 </visual>

 <!-- collision: 시뮬레이션 성능을 위해 primitive 사용 -->
 <collision name="hull_collision">
   <pose>0 0 0 0 0 0</pose>
   <geometry>
     <box><size>9.0 2.8 1.2</size></box>
   </geometry>
 </collision>
</link>

<link name="propeller_link">
 <visual name="propeller_visual">
   <pose>0 0 0 0 -1.5708 0</pose>
   <geometry>
     <mesh>
       <scale>1.5 1.5 1.5</scale>
       <uri>model://boat_mono/meshes/propeller.dae</uri>
     </mesh>
   </geometry>
 </visual>
</link>

<!-- 러더: 초기에는 cylinder primitive, 추후 메시로 교체 -->
<link name="rudder_link">
 <visual name="rudder_visual">
   <geometry>
     <box><size>0.05 0.4 0.5</size></box>
   </geometry>
 </visual>
</link>
```

---

##### 4-6. 10m 보트 물리 파라미터

```xml
<inertial>
 <mass>4000</mass>
 <inertia>
   <ixx>2000</ixx>
   <ixy>0</ixy><ixz>0</ixz>
   <iyy>35000</iyy>
   <iyz>0</iyz>
   <izz>37000</izz>
 </inertia>
</inertial>
```

USV Dynamics 플러그인 파라미터:

```xml
<plugin name='usv_dynamics' filename='libgazebo_usv_dynamics_plugin.so'>
 <bodyName>base_link</bodyName>
 <waterLevel>0</waterLevel>
 <waterDensity>997.8</waterDensity>
 <xDotU>200.0</xDotU>
 <yDotV>400.0</yDotV>
 <nDotR>100.0</nDotR>
 <xU>200.0</xU>
 <xUU>500.0</xUU>
 <yV>600.0</yV>
 <yVV>0.0</yVV>
 <zW>8000.0</zW>
 <kP>200.0</kP>
 <mQ>200.0</mQ>
 <nR>2000.0</nR>
 <nRR>0.0</nRR>
 <hullRadius>0.5</hullRadius>
 <boatWidth>3.2</boatWidth>
 <boatLength>10.0</boatLength>
 <length_n>1</length_n>
</plugin>
```

---

##### 4-7. model.config 파일

```xml
<?xml version="1.0"?>
<model>
 <name>Boat Mono (10m Single Hull USV)</name>
 <version>1.0</version>
 <sdf version="1.6">boat_mono.sdf.jinja</sdf>
 <description>
   10m single-hull USV with stern propeller and rudder.
   Hull mesh from asv_wave_sim (Apache-2.0).
   Based on rover_ackermann control (CA_AIRFRAME=5).
 </description>
</model>
```

---

##### 4-8. 전체 작업 요약 (순서대로)

```bash
# 1. 오픈소스 저장소 클론 (1회만)
git clone --depth=1 --branch noetic-devel https://github.com/osrf/vrx.git ~/vrx_noetic
git clone --depth=1 https://github.com/srmainwaring/asv_wave_sim.git ~/asv_wave_sim

# 2. boat_mono 모델 디렉토리 생성
mkdir -p Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/boat_mono/meshes

# 3. 메시 복사
cp ~/asv_wave_sim/asv_wave_sim_gazebo/models/asv_wave_sim_boat/meshes/boat.dae \
  Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/boat_mono/meshes/hull.dae
cp ~/vrx_noetic/vrx_gazebo/models/wamv_base/meshes/Propeller.dae \
  Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/boat_mono/meshes/propeller.dae

# 4. 메시 크기 확인
assimp info Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/boat_mono/meshes/hull.dae

# 5. model.config + boat_mono.sdf.jinja 작성 (Phase 1~3 내용 기반)
# 6. CMakeLists.txt 등록 (Phase 5 참조)
# 7. 실행 테스트
make px4_sitl gazebo-classic_boat_mono
```

---

#### Phase 5: 빌드 시스템 등록

##### 5-1. Gazebo-Classic 모델 등록

```cmake
# Tools/simulation/gazebo-classic/sitl_gazebo-classic/CMakeLists.txt
# (또는 해당 models 등록 파일)
boat_mono 모델 디렉토리 추가
```

##### 5-2. PX4 Makefile target 확인

`make px4_sitl gazebo-classic_boat_mono` 형식의 make target은
airframe 번호(1071)와 `PX4_SIM_MODEL=boat_mono` 환경변수로 자동 매핑.
별도 Makefile 수정 없이 airframe 파일에 아래 추가:

```sh
PX4_SIMULATOR=${PX4_SIMULATOR:=gazebo-classic}
PX4_GZ_WORLD=${PX4_GZ_WORLD:=empty}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=boat_mono}
```

---

## 3. 마일스톤별 구현 로드맵

### M1: Gazebo 모델 기본 동작 확인 (1~2주)

- [ ] `models/boat_mono/` 디렉토리 생성
- [ ] `boat_mono.sdf.jinja` 작성 (단일 선체, 프로펠러, 러더)
- [ ] Primitive geometry만으로 시각화 (메시 없이)
- [ ] `libgazebo_usv_dynamics_plugin.so` 파라미터 단일선체 기준으로 조정
- [ ] MAVLink 채널 0=모터(velocity), 1=러더(position) 설정
- [ ] `make px4_sitl gazebo-classic_boat_mono` 실행 및 Manual 모드 조작 확인

**성공 기준**: QGC에서 보트 연결, 러더 움직임 확인, 수면에 떠있음 확인

### M2: Airframe 및 제어 모듈 연결 (1주)

- [ ] `1071_gazebo-classic_boat_mono` airframe 파일 작성
- [ ] `CMakeLists.txt`에 등록
- [ ] `VEHICLE_TYPE=rover_ackermann` 설정으로 `rover_ackermann` 모듈 자동 시작 확인
- [ ] Stabilized 모드에서 스로틀/러더 수동 제어 확인
- [ ] `CA_AIRFRAME 5` 기반 control allocation 동작 확인

**성공 기준**: Stabilized 모드에서 프로펠러 RPM 및 러더 각도 정상 응답

### M3: 속도 제어 및 방향 제어 튜닝 (2~3주)

- [ ] `RO_SPEED_*` 파라미터 튜닝 (속도 PID)
- [ ] `RO_YAW_RATE_*` 파라미터 튜닝 (yaw rate PID)
- [ ] `RA_*` 파라미터 튜닝 (러더 응답 특성)
- [ ] 최소 속도 이하 조향 불가 문제 처리 방안 결정
- [ ] Position 모드 (Hold) 동작 확인

**성공 기준**: Position 모드에서 목표 속도 추종, 방위각 유지

### M4: 자율 미션 (Mission Mode) 확인 (1~2주)

- [ ] `PP_LOOKAHD_*` Pure Pursuit 파라미터 튜닝
- [ ] QGC에서 웨이포인트 미션 업로드 후 자율 항법 실행
- [ ] `NAV_ACC_RAD` 기반 웨이포인트 전환 동작 확인
- [ ] U-turn 기동 (최소 선회 반경 기반) 검증
- [ ] Return to Launch (RTL) 동작 확인

**성공 기준**: 3개 이상 웨이포인트 미션 자율 완주

### M5: 물리 모델 정확도 개선 (선택)

- [ ] 러더 수력학 플러그인 작성 (`libgazebo_rudder_plugin.so`)
 - 유속·러더각 기반 횡력 모델: `F = 0.5 * ρ * v² * A * CL * sin(δ)`
 - 저속에서 조향 불가 현상 자동 반영
- [ ] 단일선체 정확한 3D 메시 (.dae) 제작 및 교체
- [ ] 파도/조류 외란 하 미션 성능 검증

### M6: 실기체 포팅 (선택)

- [ ] 실제 보트 하드웨어 선정 및 Pixhawk 장착
- [ ] 러더 서보 PWM 캘리브레이션 (`PWM_MAIN_FUNC2` 출력 확인)
- [ ] ESC (모터 컨트롤러) 설정
- [ ] GPS/IMU 마운팅 및 EKF2 튜닝
- [ ] 실수조(pool) 또는 해상 시험

---

## 4. Gz (New Gazebo) 포팅 계획

> 현재 계획(섹션 2~3)은 **Gazebo Classic** 기반이다.
> 이 섹션은 이후 **Gz (Gazebo Sim, 구 Ignition Gazebo)** 로 포팅할 때 필요한 작업을 정리한다.

### 4.1 Gazebo Classic vs Gz 핵심 차이

| 항목 | Gazebo Classic | Gz (New Gazebo) |
|------|---------------|----------------|
| 명령어 | `make px4_sitl gazebo-classic_*` | `make px4_sitl gz_*` |
| 시뮬레이터 파라미터 | `PX4_SIMULATOR=gazebo-classic` | `PX4_SIMULATOR=gz` |
| PX4 연결 방식 | `libgazebo_mavlink_interface.so` (MAVLink TCP) | `gz_bridge` 모듈 (Gz Transport) |
| 모터 출력 토픽 | `control_channels` 내 joint 직접 제어 | `/model/{name}/command/motor_speed` |
| 서보 출력 토픽 | `control_channels` 내 joint position | `/model/{name}/servo_0`, `servo_1`, ... |
| 모터 파라미터 | `PWM_MAIN_FUNC*` | `SIM_GZ_WH_FUNC*` (wheel) |
| 서보 파라미터 | `PWM_MAIN_FUNC*` | `SIM_GZ_SV_FUNC*`, `SIM_GZ_SV_MAXA*` |
| 수상 동역학 플러그인 | `libgazebo_usv_dynamics_plugin.so` | **없음** (별도 개발 또는 외부 플러그인) |
| SDF 플러그인 네임스페이스 | `libgazebo_*.so` | `gz-sim-*-system` |
| 모델 경로 | `sitl_gazebo-classic/models/` | `PX4_GZ_MODEL_PATH` 환경변수 |
| IMU 토픽 | `/world/.../imu_sensor/imu` | `/world/{world}/model/{name}/link/base_link/sensor/imu_sensor/imu` |

---

### 4.2 포팅 필요 작업 목록

#### 작업 1: Airframe 파일 신규 작성
