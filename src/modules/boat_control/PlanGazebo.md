# Gazebo Boat SITL 구현 계획

## 1. 목표

`boat_control` 모듈을 Gazebo(gz) 기반 SITL에서 검증할 수 있는 환경을 구축한다.

기존 Isaac Sim 기반 `px4_sitl_boat` 타겟과 병행하여 Gazebo 네이티브 bridge를 사용하는
`gz_boat` 타겟을 추가한다. 최종 목표 명령은 다음이다.

```sh
make px4_sitl gz_boat
```

---

## 2. Isaac SITL vs Gazebo SITL 비교

| 항목 | Isaac SITL (`px4_sitl_boat`) | Gazebo SITL (`gz_boat`) |
|---|---|---|
| 시뮬레이터 | NVIDIA Isaac Sim (USD 기반) | Gazebo Harmonic (SDF 기반) |
| 브릿지 모듈 | `simulator_mavlink` (HIL MAVLink) | `gz_bridge` (gz-transport) |
| PX4_SIMULATOR | `isaac` | `gz` |
| 물리 | Isaac physics engine | ODE |
| 센서 | Isaac sensor bridge → HIL_SENSOR | Gazebo sensor plugins → gz-transport |
| actuator 경로 | HIL_ACTUATOR_CONTROLS | gz-transport Actuators/servo topics |
| 수상 물리 | USD hydrodynamics (Isaac) | gz-sim-buoyancy + gz-sim-hydrodynamics |
| 세팅 부담 | Isaac Sim 설치 필요 | Gazebo Harmonic만 필요 |

Gazebo SITL은 Isaac Sim 없이 로컬에서 빠르게 기본 제어 루프와 waypoint 추종을
검증하는 데 적합하다.

---

## 3. 시스템 아키텍처

```text
make px4_sitl gz_boat
        |
        v
PX4_SIM_MODEL=gz_boat
        |
        v
rcS
        |
        v
airframe 51001_gz_boat
        |
        +--> rc.boat_defaults          (VEHICLE_TYPE=boat, MAV_TYPE=11)
        +--> PX4_SIMULATOR=gz
        +--> PX4_GZ_WORLD=boat         (water world)
        +--> PX4_SIM_MODEL=boat        (gz_prefix stripped by px4-rc.gzsim)
        +--> SIM_GZ_EN=1
        +--> SIM_GZ_WH_FUNC1=101       (motor[0] → propeller joint velocity)
        +--> SIM_GZ_SV_FUNC1=201       (servo[0] → rudder joint position)
        |
        v
px4-rc.simulator
        |
        +--> PX4_SIMULATOR=gz → px4-rc.gzsim
        |
        v
px4-rc.gzsim
        |
        +--> gz sim 실행: Tools/simulation/gz_custom/worlds/boat.sdf
        +--> gz_bridge spawn: Tools/simulation/gz_custom/models/boat/model.sdf
        +--> gz_bridge start -w boat -n boat_0
        |
        v
rc.vehicle_setup → rc.boat_apps
        |
        +--> boat_control start
        +--> land_detector start rover
```

### Actuator Contract

```text
PX4 내부 → gz_bridge 경로:

actuator_motors.control[0]   = signed_thrust [-1, 1]
  → GZMixingInterfaceWheel → gz-transport /model/boat_0/command/motor_speed
  → SDF: JointController (propeller_joint, velocity)
  → gz-sim-thruster-system 또는 힘 직접 인가로 추력 생성

actuator_servos.control[0]   = steering [-1, 1]
  → GZMixingInterfaceServo → gz-transport /model/boat_0/servo_0
  → SDF: JointPositionController (rudder_joint, position)
  → 선체 후방 rudder 각도 변환
```

Wheel 인터페이스 (`SIM_GZ_WH_*`)를 propeller에 사용하는 이유:
- ESC 인터페이스 (`SIM_GZ_EC_*`)는 단방향(0–1000, 중립=0)이므로 역추진 표현 불가
- Wheel 인터페이스는 중립=100, min=0, max=200 구간을 사용하여 양방향 회전을 표현 가능
- `signed_thrust < 0`이면 역추진, `signed_thrust > 0`이면 전진 추진으로 자연스럽게 매핑

---

## 4. 신규/수정 파일 목록

```
Tools/simulation/gz_custom/worlds/boat.sdf   (신규) 수면 세계
Tools/simulation/gz_custom/models/boat/      (신규) 보트 모델
    model.config
    model.sdf
ROMFS/px4fmu_common/init.d-posix/airframes/
    51001_gz_boat                            (신규) Gazebo 보트 airframe
ROMFS/px4fmu_common/init.d-posix/airframes/
    CMakeLists.txt                           (수정) 51001_gz_boat 등록
boards/px4/sitl/boat.px4board               (수정) GZ 모듈 활성화
```

`Tools/simulation/gz`는 `PX4/PX4-gazebo-models` submodule이므로, 메인
PX4-Autopilot 브랜치에서 직접 추적해야 하는 boat 전용 임시 자산은
`Tools/simulation/gz_custom` 아래에 둔다. `px4-rc.gzsim`과
`gz_bridge/CMakeLists.txt`는 기본 submodule 자산과 custom 자산을 함께 찾도록
확장한다.

---

## 5. 항목별 구현 상세

### 5.1 Gazebo World: `boat.sdf`

파일 위치: `Tools/simulation/gz_custom/worlds/boat.sdf`

참고: `rover.sdf`를 기반으로 ground_plane 대신 water surface를 추가한다.

필수 world-level 플러그인:

```xml
<!-- 부력: 수면 아래 volume에 비례한 부력 적용 -->
<plugin filename="gz-sim-buoyancy-system"
        name="gz::sim::systems::Buoyancy">
  <graded_buoyancy>
    <default_fluid_density>1000</default_fluid_density>
    <fluid_level>0</fluid_level>  <!-- z=0 이 수면 -->
  </graded_buoyancy>
</plugin>

<!-- 표준 플러그인 세트 (rover.sdf 와 동일) -->
<plugin filename="gz-sim-physics-system" .../>
<plugin filename="gz-sim-scene-broadcaster-system" .../>
<plugin filename="gz-sim-navsat-system" .../>
<plugin filename="gz-sim-imu-system" .../>
<plugin filename="gz-sim-air-pressure-system" .../>
<plugin filename="gz-sim-sensors-system" .../>
```

water surface 시각:
- `ground_plane`의 plane geometry를 유지하되 재질을 반투명 파란색으로 변경
- z=0 에 배치하여 Buoyancy의 `fluid_level`과 일치시킨다

자기장 및 중력은 `rover.sdf`와 동일한 값을 사용한다.

### 5.2 Gazebo Boat Model: `boat/model.sdf`

파일 위치: `Tools/simulation/gz_custom/models/boat/model.sdf`

#### 5.2.1 링크 구조

```
base_link (선체)
  └── propeller_link (프로펠러)
        propeller_joint (revolute, 전후축 회전)
  └── rudder_link (방향타)
        rudder_joint (revolute, 수직축 회전)
```

#### 5.2.2 `base_link` 설정

| 항목 | 값 | 비고 |
|---|---|---|
| mass | 20 kg | 초기값, 실선 파라미터로 추후 조정 |
| 관성 | ixx=0.5, iyy=3.0, izz=3.5 | 선체 형상 기준 추정값 |
| collision | box 2.0 × 0.8 × 0.4 m | 초기 단순 형상 |
| visual | box 또는 mesh | 초기에는 box |
| pose z | 0.05 m | 수면 위 약간 떠있는 초기 위치 |

부력 volume 태그: `<enable_buoyancy>1</enable_buoyancy>` 또는
collision geometry가 buoyancy 플러그인에 의해 자동 처리됨
(Gazebo Buoyancy 플러그인은 collision geometry를 volume으로 사용)

#### 5.2.3 센서 (rover_ackermann 기준과 동일)

```xml
<!-- IMU: 250 Hz -->
<sensor name="imu_sensor" type="imu">
  <update_rate>250</update_rate>
  <!-- Gaussian noise (IIM42653 기준) -->
</sensor>

<!-- NavSat: 30 Hz -->
<sensor name="navsat_sensor" type="navsat">
  <update_rate>30</update_rate>
</sensor>

<!-- Air Pressure (baro): 50 Hz -->
<sensor name="air_pressure_sensor" type="air_pressure">
  <update_rate>50</update_rate>
</sensor>

<!-- Magnetometer: 100 Hz -->
<sensor name="magnetometer_sensor" type="magnetometer">
  <update_rate>100</update_rate>
</sensor>
```

#### 5.2.4 Propeller Link 및 Joint

```xml
<link name="propeller_link">
  <pose relative_to="base_link">-1.0 0 -0.15 0 0 0</pose>
  <inertial>
    <mass>0.1</mass>
    <!-- 경량 프로펠러 관성 -->
  </inertial>
  <visual name="propeller_visual">
    <geometry><cylinder><radius>0.1</radius><length>0.02</length></cylinder></geometry>
  </visual>
</link>

<joint name="propeller_joint" type="revolute">
  <parent>base_link</parent>
  <child>propeller_link</child>
  <axis>
    <xyz>1 0 0</xyz>  <!-- 선수미 축(X) 회전 -->
    <limit><lower>-1e+16</lower><upper>1e+16</upper></limit>
  </axis>
</joint>
```

#### 5.2.5 Rudder Link 및 Joint

```xml
<link name="rudder_link">
  <pose relative_to="base_link">-0.95 0 -0.2 0 0 0</pose>
  <inertial><mass>0.05</mass></inertial>
  <visual name="rudder_visual">
    <geometry><box><size>0.15 0.02 0.2</size></box></geometry>
  </visual>
</link>

<joint name="rudder_joint" type="revolute">
  <parent>base_link</parent>
  <child>rudder_link</child>
  <axis>
    <xyz>0 0 1</xyz>  <!-- 수직축(Z) 회전 -->
    <limit>
      <lower>-0.5236</lower>  <!-- -30도 -->
      <upper>0.5236</upper>   <!-- +30도 -->
    </limit>
  </axis>
</joint>
```

#### 5.2.6 SDF 플러그인

```xml
<!-- Propeller: wheel 인터페이스 → propeller_joint 속도 제어 -->
<plugin filename="gz-sim-joint-controller-system"
        name="gz::sim::systems::JointController">
  <joint_name>propeller_joint</joint_name>
  <sub_topic>command/motor_speed</sub_topic>
  <control_type>velocity</control_type>
  <use_actuator_msg>true</use_actuator_msg>
  <actuator_number>0</actuator_number>
</plugin>

<!-- Thruster: 프로펠러 회전 속도 → 추력 변환 -->
<plugin filename="gz-sim-thruster-system"
        name="gz::sim::systems::Thruster">
  <joint_name>propeller_joint</joint_name>
  <thrust_coefficient>0.02</thrust_coefficient>   <!-- 초기값, 캘리브레이션 필요 -->
  <fluid_density>1000</fluid_density>
  <propeller_diameter>0.2</propeller_diameter>
  <velocity_control>true</velocity_control>
</plugin>

<!-- Rudder: servo 인터페이스 → rudder_joint 위치 제어 -->
<plugin filename="gz-sim-joint-position-controller-system"
        name="gz::sim::systems::JointPositionController">
  <joint_name>rudder_joint</joint_name>
  <sub_topic>servo_0</sub_topic>
  <p_gain>50</p_gain>
  <i_gain>1</i_gain>
  <d_gain>0</d_gain>
</plugin>

<!-- Hydrodynamics: 수중 선형/각도 감쇠 -->
<plugin filename="gz-sim-hydrodynamics-system"
        name="gz::sim::systems::Hydrodynamics">
  <link_name>base_link</link_name>
  <!-- Added mass (diagonal only, 초기 추정값) -->
  <xDotU>-0.1</xDotU>
  <yDotV>-1.5</yDotV>
  <nDotR>-0.1</nDotR>
  <!-- Linear drag -->
  <xU>-2.0</xU>
  <yV>-10.0</yV>
  <nR>-2.0</nR>
  <!-- Quadratic drag -->
  <xUU>-2.0</xUU>
  <yVV>-10.0</yVV>
  <nRR>-2.0</nRR>
</plugin>

<!-- Joint state publisher -->
<plugin filename="gz-sim-joint-state-publisher-system"
        name="gz::sim::systems::JointStatePublisher">
  <joint_name>propeller_joint</joint_name>
  <joint_name>rudder_joint</joint_name>
</plugin>
```

> **주의**: `gz-sim-thruster-system`이 gz_bridge의 `command/motor_speed`와 충돌하지 않도록
> `velocity_control=true`를 설정한다. `JointController`가 회전속도를 제어하고
> `Thruster` 플러그인이 해당 속도를 추력으로 변환하는 방식이다.
> 만약 충돌 문제가 발생하면 `Thruster` 플러그인만 사용하고 `JointController`를 제거하는
> 단순화된 접근을 시도한다.

#### 5.2.7 추력 모델 대안 (Thruster 플러그인 사용 불가 시)

gz-sim-thruster-system이 wheel velocity와 충돌하거나 사용 불가능한 경우:
- `JointController`로 propeller_joint 회전속도만 제어
- 추력은 `gz-sim-lift-drag-system`으로 프로펠러 날개에서 lift/drag 계산
- 또는 `ApplyLinkWrench` 를 통한 별도 힘 인가 bridge 스크립트 사용

### 5.3 Airframe: `51001_gz_boat`

파일 위치: `ROMFS/px4fmu_common/init.d-posix/airframes/51001_gz_boat`

```sh
#!/bin/sh
# @name Gazebo Boat
# @type Boat
# @class Rover

. ${R}etc/init.d/rc.boat_defaults

PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
PX4_GZ_WORLD=${PX4_GZ_WORLD:=boat}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=boat}

param set-default SIM_GZ_EN 1

# Propeller: wheel interface (bi-directional)
# 중립=100, 역추진=0~99, 전진=101~200
param set-default SIM_GZ_WH_FUNC1 101      # motor[0] = signed_thrust
param set-default SIM_GZ_WH_MIN1 0
param set-default SIM_GZ_WH_MAX1 200
param set-default SIM_GZ_WH_DIS1 100       # disarmed = 중립

# Rudder: servo interface
param set-default SIM_GZ_SV_FUNC1 201      # servo[0] = steering
param set-default SIM_GZ_SV_MAXA1 30       # +30도 (rad 변환은 gz bridge 내부)
param set-default SIM_GZ_SV_MINA1 -30      # -30도
param set-default SIM_GZ_SV_REV 0

# Control allocation
param set-default CA_AIRFRAME 9
param set-default CA_ROTOR_COUNT 1
param set-default CA_ROTOR0_AX 1
param set-default CA_ROTOR0_AZ 0
param set-default CA_ROTOR0_KM 0
param set-default CA_ROTOR0_PX -1.0
param set-default CA_ROTOR0_PY 0
param set-default CA_ROTOR0_PZ -0.15
param set-default CA_R_REV 1
param set-default CA_SV_CS_COUNT 1

# Navigation
param set-default NAV_ACC_RAD 2

# Boat control parameters (rc.boat_defaults에서 상속 후 SITL 전용 override)
param set-default BOAT_LOOKAHD 5.0
param set-default BOAT_START_DIST 20.0
```

### 5.4 CMakeLists.txt 수정

파일: `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`

```cmake
# 기존 1070, 1071 아래에 추가
list(APPEND config_module_list
    ...
    51001_gz_boat
    ...
)
```

### 5.5 board 파일 수정

파일: `boards/px4/sitl/boat.px4board`

현재 `boat.px4board`는 Gazebo bridge 모듈이 비활성화되어 있다.
`gz_boat` 타겟을 위해 Gazebo 모듈을 활성화한다.

```kconfig
# 기존 설정 유지
CONFIG_MODULES_BOAT_CONTROL=y
CONFIG_DRIVERS_ROBOCLAW=n

# Gazebo 지원 추가
CONFIG_MODULES_SIMULATION_GZ_BRIDGE=y
CONFIG_MODULES_SIMULATION_GZ_MSGS=y
CONFIG_MODULES_SIMULATION_GZ_PLUGINS=y
CONFIG_COMMON_SIMULATION=y
```

> Isaac 전용 빌드(`px4_sitl_boat`)와 Gazebo 빌드(`gz_boat`)가 같은
> board target을 공유하므로, 위 모듈 추가가 두 빌드 모두에 영향을 준다.
> Isaac 빌드에서는 Gazebo 모듈이 포함되어도 `PX4_SIMULATOR=isaac` 조건 하에
> 실행 시 gz 모듈이 시작되지 않으므로 런타임 충돌은 없다.

---

## 6. init.d 흐름 검증

### 6.1 `px4-rc.simulator` 분기

현재 코드:
```sh
if [ "$PX4_SIMULATOR" = "gz" ] || [ "$(param show -q SIM_GZ_EN)" = "1" ]; then
    . px4-rc.gzsim   # ← gz_boat 는 이 경로를 탄다
```

`51001_gz_boat`에서 `PX4_SIMULATOR=gz`를 설정하므로 별도 수정 불필요.

### 6.2 `px4-rc.gzsim` 모델 spawn 흐름

```sh
# PX4_SIM_MODEL = gz_boat → gz_ prefix 제거 → MODEL_NAME = boat
MODEL_NAME="${PX4_SIM_MODEL#*gz_}"    # → "boat"
MODEL_NAME_INSTANCE="boat_0"

# Tools/simulation/gz_custom/models/boat/model.sdf 사용
gz_bridge start -w boat -n boat_0
```

`Tools/simulation/gz_custom/models/boat/` 디렉토리가 존재해야 한다.

### 6.3 `rc.vehicle_setup` → `rc.boat_apps`

`VEHICLE_TYPE=boat` → `rc.boat_apps` 실행
`boat_control start` / `land_detector start rover` 동작 유지.

---

## 7. 구현 단계 (Phase)

### Phase 1: 빌드 환경 구성

1. `boards/px4/sitl/boat.px4board`에 GZ 모듈 활성화
2. `make px4_sitl_boat` 빌드 성공 확인 (기존 Isaac 빌드 깨지지 않는지)
3. `ROMFS/.../airframes/51001_gz_boat` 생성 (최소 내용)
4. `CMakeLists.txt`에 `51001_gz_boat` 등록
5. `make px4_sitl gz_boat` 빌드 성공 확인

**검증**: 빌드 오류 없음

### Phase 2: 최소 Gazebo 모델 생성

1. `Tools/simulation/gz_custom/worlds/boat.sdf` 생성
   - `rover.sdf` 기반으로 복사 후 Buoyancy 플러그인 추가
   - ground_plane 재질을 파란색 반투명 water surface로 변경
2. `Tools/simulation/gz_custom/models/boat/model.config` 생성
3. `Tools/simulation/gz_custom/models/boat/model.sdf` 생성
   - 단순 box 선체 (mesh 없음)
   - 센서 4종 포함
   - propeller_joint + rudder_joint 포함
   - JointController + JointPositionController 플러그인 포함
   - Buoyancy volume 설정

**검증**: `make px4_sitl gz_boat` 실행 후 Gazebo GUI에서 선체가 수면에 뜨는지 확인

### Phase 3: Actuator 연결 확인

1. QGC 또는 commander로 arm
2. MANUAL 모드에서 조종기 입력 → 프로펠러/방향타 Gazebo 시각 반응 확인
3. `gz topic -e -t /model/boat_0/command/motor_speed`로 propeller velocity 확인
4. `gz topic -e -t /model/boat_0/servo_0`로 rudder angle 확인
5. actuator_motors, actuator_servos uORB topic 로그 확인

**검증**: 조종기 스틱 → Gazebo 모델 응답 (선체 이동/회전)

### Phase 4: 추력 및 유체역학 파라미터 튜닝

1. Thruster `thrust_coefficient`, `propeller_diameter` 조정
   - 목표: 최대 thrust command 시 ~2–3 m/s 전진 속도
2. Hydrodynamics 감쇠 계수 조정
   - 목표: 급가속/감속 시 자연스러운 응답
3. 직선 구간 MANUAL 모드 주행으로 속도/방향 응답 검증

**검증**: 최대 조종기 입력 → 기대 속도 범위 달성

### Phase 5: BoatPosControl AUTO 모드 검증

1. QGC에서 waypoint 2–3개 미션 생성
2. AUTO_MISSION 모드 arm 후 실행
3. `boat_control` 로그에서 LOS error, steering, thrust setpoint 확인
4. 선체 waypoint 도달 및 다음 waypoint 이동 확인
5. 미션 완료 후 auto-disarm 동작 확인

**검증**: 설정한 waypoint를 순서대로 통과 (NAV_ACC_RAD 이내 도달 판정)

### Phase 6: mesh 및 물 시각 개선 (선택적)

1. 간단한 선체 mesh `.dae` 제작 또는 기존 오픈소스 선박 mesh 사용
2. Gazebo Ocean 또는 WaveSim 플러그인 추가 (파도 시뮬레이션)
3. 실제 해안 환경 world SDF (만약 지형 데이터 있는 경우)

---

## 8. 주요 파라미터 요약

| 파라미터 | 값 | 설명 |
|---|---|---|
| `SIM_GZ_EN` | 1 | Gazebo bridge 활성화 |
| `SIM_GZ_WH_FUNC1` | 101 | motor[0] → propeller (wheel interface) |
| `SIM_GZ_WH_MIN1` | 0 | 역추진 최대 |
| `SIM_GZ_WH_MAX1` | 200 | 전진 최대 |
| `SIM_GZ_WH_DIS1` | 100 | 중립 (disarmed) |
| `SIM_GZ_SV_FUNC1` | 201 | servo[0] → rudder |
| `SIM_GZ_SV_MAXA1` | 30 | rudder 최대 각도 (deg) |
| `SIM_GZ_SV_MINA1` | -30 | rudder 최소 각도 (deg) |
| `CA_AIRFRAME` | 9 | boat airframe type |
| `NAV_ACC_RAD` | 2 | waypoint acceptance radius (m) |
| `BOAT_LOOKAHD` | 5.0 | LOS lookahead distance (m) |
| `BOAT_START_DIST` | 20.0 | start point fallback distance (m) |

---

## 9. 알려진 위험 및 대응

| 위험 | 대응 |
|---|---|
| `gz-sim-thruster-system`이 `JointController`와 충돌 | `Thruster` 플러그인만 사용하고 `JointController` 제거 |
| 부력 설정 미흡으로 선체 침몰 | collision geometry z offset 조정, fluid_density/level 재확인 |
| `SIM_GZ_WH_*` 역방향 매핑 미지원 | `signed_thrust`를 항상 양수로 제한하고 역추진은 Phase 4 이후 처리 |
| `rc.boat_apps`에서 `boat_control`이 Gazebo 입력을 못 받음 | `SIM_GZ_EN=1` 파라미터 확인, gz_bridge 로그 확인 |
| 기존 Isaac `px4_sitl_boat` 빌드 깨짐 | board 파일 수정 전 Isaac 빌드 성공 여부 먼저 기록 |

---

## 10. 참고 파일

| 파일 | 역할 |
|---|---|
| `Tools/simulation/gz/models/rover_ackermann/model.sdf` | Gazebo 지상차량 SDF 참고 |
| `Tools/simulation/gz/worlds/rover.sdf` | Gazebo 지면 세계 SDF 참고 |
| `Tools/simulation/gz/worlds/underwater.sdf` | 수중/부력 플러그인 구성 참고 |
| `Tools/simulation/gz/models/uuv_bluerov2_heavy/model.sdf` | UUV 센서 및 hydrodynamics 참고 |
| `ROMFS/.../airframes/51000_gz_rover_ackermann` | gz_rover airframe 참고 |
| `src/modules/simulation/gz_bridge/module.yaml` | SIM_GZ_WH/SV/EC 파라미터 정의 |
| `src/modules/simulation/gz_bridge/GZMixingInterfaceWheel.cpp` | wheel actuator 동작 구현 |
| `src/modules/boat_control/Plan.md` | boat_control 제어 알고리즘 상세 |
| `BoatSITL_Plan.md` | Isaac 기반 SITL 계획 (참고) |
