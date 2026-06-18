# Boat SITL 구현 계획

## 목표

`boards/px4/fmu-v6x/boat.px4board`의 boat 전용 모듈 구성을 참고하여 PX4 POSIX SITL에서 boat 전용 빌드 타겟을 만들고, NVIDIA Isaac Sim 기반 boat 시뮬레이션 환경과 MAVLink로 연동한다.

최종 목표는 다음 명령 흐름으로 PX4 SITL을 실행하고 Isaac Sim boat 시뮬레이터와 연결하는 것이다.

```sh
make px4_sitl_boat
python3 Tools/simulation/isaac/bridge/isaac_boat_mavlink_bridge.py --host 127.0.0.1 --port 4560
cd build/px4_sitl_boat
PX4_SIM_MODEL=isaac_boat PX4_SIMULATOR=isaac bin/px4 -s etc/init.d-posix/rcS .
```

실제 실행 명령은 구현 중 PX4 기존 SITL target 규칙에 맞춰 조정한다.

## 전체 통신 아키텍처

PX4와 Isaac Sim 사이의 simulator link는 기존 PX4 SITL의 `simulator_mavlink` 경로를 사용한다. 기본 구성에서는 Isaac bridge가 TCP server로 `127.0.0.1:4560`에서 대기하고, PX4가 client로 접속한다.

```text
Host PC / Localhost
================================================================================

  +-----------------------------+                 +-----------------------------+
  | NVIDIA Isaac Sim            |                 | PX4 SITL                    |
  |                             |                 | px4_sitl_boat               |
  |  +-----------------------+  |                 | PX4_SIM_MODEL=isaac_boat    |
  |  | Boat simulation scene |  |                 | PX4_SIMULATOR=isaac         |
  |  | - hull dynamics       |  |                 |                             |
  |  | - water environment   |  |                 |  +-----------------------+  |
  |  | - thrusters           |  |                 |  | simulator_mavlink     |  |
  |  | - sensor models       |  |                 |  | TCP client            |  |
  |  +-----------+-----------+  |                 |  | remote 127.0.0.1:4560 |  |
  |              |              |                 |  +-----------+-----------+  |
  |              | Isaac API    |                             |
  |              v              |                             |
  |  +-----------------------+  |  MAVLink over TCP           |
  |  | Isaac MAVLink bridge  |<-+-----------------------------+
  |  | TCP server/listener   |    127.0.0.1:4560
  |  | --host 127.0.0.1     |                               |
  |  | --port 4560          |                               |
  |  +-----------+-----------+                               |
  |              ^                                           |
  |              |                                           v
  |  PX4 -> Isaac Sim messages                 +-----------------------------+
  |  - HIL_ACTUATOR_CONTROLS                   | PX4 internal uORB/modules   |
  |    controls[0]: right thruster             |                             |
  |    controls[1]: left thruster              | sensors / estimator:        |
  |    mode: armed/disarmed flag               | - sensors                   |
  |                                            | - ekf2                      |
  |  Isaac Sim -> PX4 messages                 |                             |
  |  - HEARTBEAT                               | vehicle control:            |
  |  - HIL_SENSOR                              | - commander                 |
  |    accel, gyro, mag, baro                  | - navigator                 |
  |  - HIL_GPS                                 | - boat_control              |
  |    lat, lon, alt, NED velocity, COG        | - control_allocator         |
  |                                            | - pwm_out_sim               |
  +--------------------------------------------+-----------------------------+


PX4 external MAVLink links, same as existing SITL
--------------------------------------------------------------------------------

  +----------------------+        UDP localhost        +------------------------+
  | QGroundControl / GCS |<--------------------------->| PX4 mavlink GCS link   |
  |                      |                             | local: 18570+instance  |
  |                      |                             | remote: 14550 default  |
  +----------------------+                             +------------------------+

  +----------------------+        UDP localhost        +------------------------+
  | Offboard/API client  |<--------------------------->| PX4 mavlink onboard    |
  | MAVSDK/ROS/etc.      |                             | local: 14580+instance  |
  |                      |                             | remote:14540+instance  |
  +----------------------+                             +------------------------+

  +----------------------+        UDP localhost        +------------------------+
  | Payload client       |<--------------------------->| PX4 payload link       |
  |                      |                             | local: 14280+instance  |
  |                      |                             | remote:14030+instance  |
  +----------------------+                             +------------------------+

  +----------------------+        UDP localhost        +------------------------+
  | Gimbal client        |<--------------------------->| PX4 gimbal link        |
  |                      |                             | local: 13030+instance  |
  |                      |                             | remote:13280+instance  |
  +----------------------+                             +------------------------+
```

주소와 포트 결정 규칙:

- 기본 simulator link: Isaac bridge listens on `127.0.0.1:4560`, PX4 connects to it.
- simulator TCP port: `4560 + px4_instance`
- `PX4_SIM_HOSTNAME`이 설정되면 PX4는 해당 hostname의 simulator TCP port로 접속한다.
- `PX4_SIM_HOST_ADDR`가 설정되면 PX4는 해당 IP address의 simulator TCP port로 접속한다.
- 둘 다 없으면 `localhost`를 사용한다.
- GCS/Offboard/Payload/Gimbal MAVLink UDP 포트는 기존 PX4 SITL과 동일하게 `px4_instance` offset을 적용한다.

메시지 책임:

- Isaac bridge는 Isaac Sim의 pose/velocity/sensor 값을 MAVLink HIL 메시지로 변환한다.
- PX4 `simulator_mavlink`는 `HIL_SENSOR`, `HIL_GPS`를 uORB sensor topic으로 반영한다.
- PX4 control pipeline은 `boat_control`과 `control_allocator`를 통해 actuator output을 만든다.
- PX4 `simulator_mavlink`는 actuator output을 `HIL_ACTUATOR_CONTROLS`로 Isaac bridge에 보낸다.
- Isaac bridge는 `HIL_ACTUATOR_CONTROLS`를 Isaac Sim boat thruster 명령으로 변환한다.

## 현재 참고 사항

- Hardware boat 설정: `boards/px4/fmu-v6x/boat.px4board`
  - 비행체용 제어 모듈을 비활성화한다.
  - `CONFIG_MODULES_BOAT_CONTROL=y`를 활성화한다.
  - `CONFIG_DRIVERS_ROBOCLAW=y`를 활성화한다.
  - 단, POSIX SITL에서는 Roboclaw hardware serial driver가 필요 없고 macOS에서 일부 baud 상수 차이로 빌드가 깨질 수 있으므로 `px4_sitl_boat`에서는 비활성화한다.
- POSIX SITL 기본 설정: `boards/px4/sitl/default.px4board`
  - `CONFIG_PLATFORM_POSIX=y`
  - `CONFIG_COMMON_SIMULATION=y`
  - `CONFIG_MODULES_MAVLINK=y`
  - `CONFIG_MODULES_SIMULATION_GZ_BRIDGE=y`
  - `CONFIG_MODULES_SIMULATION_GZ_PLUGINS=y`
- 기존 boat airframe: `ROMFS/px4fmu_common/init.d-posix/airframes/1070_gazebo-classic_boat`
  - `rc.boat_defaults`를 사용한다.
  - `CA_AIRFRAME=9`, rotor 2개, `PWM_MAIN_FUNC1=101`, `PWM_MAIN_FUNC2=102` mapping을 사용한다.
- 기존 MAVLink simulator bridge: `ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlinksim`
  - simulator TCP port는 `4560 + px4_instance`이다.
  - `PX4_SIM_HOSTNAME` 또는 `PX4_SIM_HOST_ADDR`가 있으면 해당 host로 연결한다.
  - 값이 없으면 localhost로 연결한다.
- 기존 SITL MAVLink link: `ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink`
  - GCS local UDP port: `18570 + px4_instance`
  - Offboard local UDP port: `14580 + px4_instance`
  - Offboard remote UDP port: `14540 + px4_instance`
  - Payload local/remote UDP port: `14280/14030 + px4_instance`
  - Gimbal local/remote UDP port: `13030/13280 + px4_instance`

## 구현 범위

1. Boat 전용 SITL build target 추가
2. Isaac Sim boat airframe 추가
3. Isaac Sim과 PX4 SITL 간 MAVLink HIL bridge 정의
4. 기존 드론용 SITL과 동일한 네트워크 인터페이스와 포트 정책 적용
5. 빌드 및 런타임 검증 절차 작성

## 1. Boat 전용 SITL build target

### 1.1 `boards/px4/sitl/boat.px4board` 추가

`boards/px4/sitl/default.px4board`를 base로 merge되는 label board를 추가한다. PX4 빌드 시스템은 `boards/*/*/*.px4board` 파일을 target으로 변환하므로, `boards/px4/sitl/boat.px4board`를 만들면 `px4_sitl_boat` target을 사용할 수 있다.

초기 구성 방향:

```text
CONFIG_MODULES_AIRSPEED_SELECTOR=n
CONFIG_MODULES_FLIGHT_MODE_MANAGER=n
CONFIG_MODULES_FW_ATT_CONTROL=n
CONFIG_MODULES_FW_AUTOTUNE_ATTITUDE_CONTROL=n
CONFIG_MODULES_FW_MODE_MANAGER=n
CONFIG_MODULES_FW_LATERAL_LONGITUDINAL_CONTROL=n
CONFIG_MODULES_FW_RATE_CONTROL=n
CONFIG_MODULES_MC_ATT_CONTROL=n
CONFIG_MODULES_MC_AUTOTUNE_ATTITUDE_CONTROL=n
CONFIG_MODULES_MC_HOVER_THRUST_ESTIMATOR=n
CONFIG_MODULES_MC_POS_CONTROL=n
CONFIG_MODULES_MC_RATE_CONTROL=n
CONFIG_MODULES_VTOL_ATT_CONTROL=n
CONFIG_MODULES_ROVER_ACKERMANN=n
CONFIG_MODULES_ROVER_DIFFERENTIAL=n
CONFIG_MODULES_ROVER_MECANUM=n
CONFIG_MODULES_UUV_ATT_CONTROL=n
CONFIG_MODULES_UUV_POS_CONTROL=n
CONFIG_DRIVERS_ROBOCLAW=n
CONFIG_MODULES_BOAT_CONTROL=y
```

단, SITL 실행에 필요한 공통 모듈은 유지한다.

- `CONFIG_MODULES_COMMANDER=y`
- `CONFIG_MODULES_CONTROL_ALLOCATOR=y`
- `CONFIG_MODULES_DATAMAN=y`
- `CONFIG_MODULES_EKF2=y`
- `CONFIG_MODULES_EVENTS=y`
- `CONFIG_MODULES_LOGGER=y`
- `CONFIG_MODULES_MANUAL_CONTROL=y`
- `CONFIG_MODULES_MAVLINK=y`
- `CONFIG_MODULES_NAVIGATOR=y`
- `CONFIG_MODULES_RC_UPDATE=y`
- `CONFIG_MODULES_SENSORS=y`
- `CONFIG_COMMON_SIMULATION=y`

### 1.2 빌드 확인

```sh
make px4_sitl_boat
```

확인 항목:

- `boat_control`이 빌드에 포함되는지 확인한다.
- `mavlink`, `simulator_mavlink`, `ekf2`, `sensors`, `commander`, `navigator`, `control_allocator`가 누락되지 않는지 확인한다.
- 불필요한 MC/FW/VTOL/Rover/UUV controller가 제외되는지 확인한다.

## 2. Isaac Sim boat airframe 추가

### 2.1 신규 airframe 파일

다음 파일을 추가한다.

```text
ROMFS/px4fmu_common/init.d-posix/airframes/1071_isaac_boat
```

초기 구성 방향:

```sh
#!/bin/sh
#
# @name Isaac Sim Boat
# @type Boat
#

. ${R}etc/init.d/rc.boat_defaults

PX4_SIMULATOR=${PX4_SIMULATOR:=isaac}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=isaac_boat}

param set-default MAV_TYPE 11
param set-default CA_AIRFRAME 9

param set-default CA_ROTOR_COUNT 2
param set-default CA_ROTOR0_AX 1
param set-default CA_ROTOR0_AZ 0
param set-default CA_ROTOR0_KM 0
param set-default CA_ROTOR0_PX -2
param set-default CA_ROTOR0_PY -1
param set-default CA_ROTOR1_AX 1
param set-default CA_ROTOR1_AZ 0
param set-default CA_ROTOR1_KM 0
param set-default CA_ROTOR1_PX -2
param set-default CA_ROTOR1_PY 1
param set-default CA_R_REV 3

param set-default PWM_MAIN_FUNC1 101
param set-default PWM_MAIN_FUNC2 102
```

기존 `1070_gazebo-classic_boat` 값을 우선 재사용하고, Isaac Sim boat 모델의 추진기 방향과 좌표계가 확정되면 `CA_ROTOR*` 값을 보정한다.

### 2.2 airframe 등록 확인

`ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`에 신규 airframe이 자동 포함되는지 확인한다. 자동 glob이 아니라 명시 목록이면 `1071_isaac_boat`를 추가한다.

### 2.3 실행 모델 선택

PX4 init script는 `PX4_SIM_MODEL`과 airframe 파일명을 매칭한다. 따라서 아래 값으로 실행되게 한다.

```sh
PX4_SIM_MODEL=isaac_boat
```

## 3. Isaac Sim 연동 방식

### 3.1 기본 통신 방향

PX4의 기존 `simulator_mavlink` module을 우선 재사용한다.

- Isaac Sim 또는 별도 bridge process가 PX4 simulator MAVLink endpoint에 연결한다.
- Isaac Sim에서 센서 데이터를 MAVLink HIL message로 PX4에 전송한다.
- PX4는 actuator output을 `HIL_ACTUATOR_CONTROLS`로 Isaac Sim에 전송한다.
- Isaac Sim은 이 actuator command를 boat 추진기 명령으로 변환한다.

### 3.2 MAVLink message mapping

Isaac Sim에서 PX4로 보낼 message:

- `HIL_SENSOR`
  - IMU accel/gyro
  - magnetometer
  - barometer
  - fields_updated bitmask 설정
- `HIL_GPS`
  - latitude, longitude, altitude
  - NED velocity
  - heading/course over ground
  - fix type, eph, epv, satellites
- 필요 시 `ODOMETRY` 또는 `VISION_POSITION_ESTIMATE`
  - Isaac Sim ground truth를 EKF2 보조 입력으로 사용할 때 검토한다.
- `HEARTBEAT`
  - simulator component heartbeat

PX4에서 Isaac Sim으로 받을 message:

- `HIL_ACTUATOR_CONTROLS`
  - `controls[0]`, `controls[1]`을 boat 좌/우 추진기 또는 steering/throttle mapping으로 변환한다.
  - `mode`의 armed flag를 확인하여 disarmed 상태에서는 추진기 출력을 0으로 둔다.

### 3.3 좌표계 변환

Isaac Sim과 PX4 사이의 좌표계를 명확히 고정한다.

- PX4 local frame: NED
- Isaac Sim world frame: Isaac stage 설정을 확인한 뒤 ENU 또는 USD stage axis 기준에서 NED로 변환한다.
- attitude quaternion은 PX4가 기대하는 body frame과 부호 convention을 검증한다.
- GPS 기준점은 시뮬레이션 world origin과 `lat/lon/alt` reference를 매핑한다.

### 3.4 lockstep

초기 구현은 기존 SITL의 lockstep 동작을 따른다.

- `HIL_SENSOR`를 주기적으로 보내 PX4 time progression이 안정적인지 확인한다.
- actuator update와 physics step을 동기화할 수 있으면 lockstep을 유지한다.
- Isaac Sim bridge가 lockstep을 안정적으로 맞추기 어렵다면 `boards/px4/sitl/nolockstep.px4board` 구성을 참고해 별도 `boat_nolockstep` target을 검토한다.

## 4. 네트워크 인터페이스와 포트 정책

기존 드론 SITL과 동일한 정책을 기본값으로 사용한다.

### 4.1 Simulator MAVLink endpoint

`px4-rc.mavlinksim` 기준:

```text
TCP simulator port = 4560 + px4_instance
```

기본 instance 0:

```text
TCP 4560
```

host 선택:

- `PX4_SIM_HOSTNAME`이 있으면 hostname으로 연결
- 없고 `PX4_SIM_HOST_ADDR`가 있으면 IP address로 연결
- 둘 다 없으면 localhost 사용

Isaac Sim bridge는 다음 두 방식 중 하나를 선택한다.

- PX4가 `simulator_mavlink start -c 4560`로 TCP client 접속을 시도하고 Isaac Sim bridge가 server로 대기
- PX4가 `PX4_SIM_HOST_ADDR` 또는 `PX4_SIM_HOSTNAME` 대상으로 client 접속하고 Isaac Sim bridge가 server로 대기

초기 구현은 기존 기본값과 맞춰 Isaac Sim bridge가 localhost TCP 4560에서 listen하도록 한다.

### 4.2 일반 MAVLink links

`px4-rc.mavlink` 기준:

| 용도 | Local port | Remote port |
| --- | ---: | ---: |
| GCS | `18570 + px4_instance` | QGC discovery 또는 target 설정 |
| Offboard/API | `14580 + px4_instance` | `14540 + px4_instance` |
| Payload | `14280 + px4_instance` | `14030 + px4_instance` |
| Gimbal | `13030 + px4_instance` | `13280 + px4_instance` |

기본 instance 0:

| 용도 | Local port | Remote port |
| --- | ---: | ---: |
| GCS | `18570` | - |
| Offboard/API | `14580` | `14540` |
| Payload | `14280` | `14030` |
| Gimbal | `13030` | `13280` |

### 4.3 멀티 instance

`px4_instance`에 따라 port offset을 적용한다.

- simulator: `4560 + px4_instance`
- GCS: `18570 + px4_instance`
- Offboard local: `14580 + px4_instance`
- Offboard remote: `14540 + px4_instance`

10개 초과 instance에서는 기존 script처럼 offboard remote port overlap 정책을 확인한다.

## 5. Isaac Sim bridge 구현 계획

### 5.1 위치

PX4 repo 내부에 둘 경우 후보:

```text
Tools/simulation/isaac/
```

구성 예:

```text
Tools/simulation/isaac/
  README.md
  bridge/
    isaac_boat_mavlink_bridge.py
  worlds/
    boat_environment.usd
  models/
    boat/
```

Isaac Sim extension으로 구현하는 경우에는 PX4 repo에는 실행 wrapper와 문서만 두고, extension repo 경로를 README에 명시한다.

### 5.2 bridge 역할

- Isaac Sim physics step callback에서 boat pose, velocity, angular velocity를 읽는다.
- IMU/GPS/mag/baro 값을 생성한다.
- MAVLink `HIL_SENSOR`, `HIL_GPS`를 주기적으로 보낸다.
- PX4의 `HIL_ACTUATOR_CONTROLS`를 수신한다.
- actuator command를 Isaac Sim boat joint/thruster command로 변환한다.
- 통신 끊김, disarmed, failsafe 상태에서는 actuator command를 0으로 clamp한다.

### 5.3 sensor update rate 초기값

초기값:

- IMU/HIL_SENSOR: 250 Hz
- GPS/HIL_GPS: 5 Hz 또는 10 Hz
- actuator receive loop: PX4 publish rate 기준으로 즉시 반영

성능 문제가 있으면 Isaac Sim physics rate와 PX4 lockstep 동작을 기준으로 조정한다.

## 6. Isaac Sim 구현 계획

이 섹션은 PX4 repo 쪽 변경이 아니라 Isaac Sim scene/extension에서 구현해야 하는 항목이다. PX4 쪽 bridge script는 smoke test와 protocol reference로 사용하고, 실제 Isaac Sim 연동에서는 아래 기능을 Isaac Sim physics callback 또는 extension 내부에 연결한다.

### 6.1 Isaac Sim extension 구조

Isaac Sim 쪽 구현은 별도 extension으로 구성하는 것을 우선한다.

```text
isaac_boat_px4_bridge/
  extension.toml
  python/
    isaac_boat_px4_bridge/
      __init__.py
      extension.py
      px4_mavlink_bridge.py
      boat_state.py
      sensor_model.py
      frame_transform.py
      thruster_controller.py
  data/
    worlds/
      boat_environment.usd
    robots/
      boat.usd
    config/
      boat_px4.yaml
```

역할 분리:

- `extension.py`: Isaac Sim UI, lifecycle, physics callback 등록/해제
- `px4_mavlink_bridge.py`: TCP server, MAVLink encode/decode, PX4 connection state 관리
- `boat_state.py`: USD prim에서 pose, velocity, angular velocity 읽기
- `sensor_model.py`: IMU/GPS/mag/baro 값 생성 및 noise/bias 적용
- `frame_transform.py`: Isaac world frame과 PX4 NED/body frame 변환
- `thruster_controller.py`: PX4 actuator command를 Isaac articulation/joint/force command로 적용
- `boat_px4.yaml`: port, origin, sensor rate, thruster mapping, scale, noise 설정

### 6.2 Isaac Sim scene 구성

필수 scene 구성:

- Boat USD model
  - rigid body 또는 articulation root 설정
  - mass, inertia, center of mass 설정
  - visual mesh와 collision mesh 분리
  - 좌/우 thruster 위치와 방향을 명확히 표시
- Water environment
  - 초기 단계에서는 평면 수면 + 단순 drag/buoyancy 모델로 시작
  - 이후 파도, 조류, 바람 disturbance를 단계적으로 추가
- World origin
  - Isaac world origin을 PX4 local NED origin으로 매핑
  - GPS 기준점 `lat0/lon0/alt0`를 config로 고정
- Lighting/camera
  - 디버깅용 chase camera, top-down camera
  - boat heading, waypoint, velocity vector를 볼 수 있는 debug overlay 검토

초기 world config 예:

```yaml
world:
  usd: data/worlds/boat_environment.usd
  physics_dt: 0.004
  render_dt: 0.0166667
  gravity_m_s2: 9.80665

origin:
  lat_deg: 47.397742
  lon_deg: 8.545594
  alt_m: 488.0
```

### 6.3 Boat physics 구현

초기 구현은 PX4 연동 검증이 목적이므로 단순 모델부터 시작한다.

1. 기본 rigid body dynamics
   - Isaac PhysX rigid body를 사용한다.
   - boat body x축을 forward로 정의한다.
   - 좌/우 thruster force를 body frame 기준 forward 방향으로 적용한다.
2. 수상 운동 damping
   - surge/sway/yaw damping을 추가한다.
   - roll/pitch는 과도하게 흔들리지 않도록 damping 또는 constraint를 적용한다.
3. buoyancy
   - 초기에는 z position을 수면 근처로 유지하는 단순 buoyancy force를 적용한다.
   - 이후 hull shape 기반 buoyancy sampling으로 확장한다.
4. disturbance
   - 조류 current: world frame velocity bias
   - 바람 wind: lateral force/yaw moment
   - wave: vertical force/roll/pitch disturbance

초기 thrust mapping:

```text
PX4 HIL_ACTUATOR_CONTROLS.controls[0] -> right thruster normalized [-1, 1]
PX4 HIL_ACTUATOR_CONTROLS.controls[1] -> left thruster normalized [-1, 1]

right_force_N = controls[0] * max_thrust_N
left_force_N  = controls[1] * max_thrust_N
```

주의:

- PX4 airframe의 `CA_R_REV=3`이 좌/우 motor reversible을 의미하므로 Isaac thruster도 reverse force를 허용한다.
- 좌/우 thruster 순서는 `1071_isaac_boat`의 actuator mapping과 실제 boat USD의 위치가 일치해야 한다.
- PX4 disarmed 상태에서는 force를 0으로 clamp한다.

### 6.4 Sensor model 구현

Isaac Sim에서 PX4로 보내야 하는 최소 sensor는 IMU와 GPS이다. EKF 안정화를 위해 mag/baro도 함께 보낸다.

IMU:

- source: boat body pose/velocity/angular velocity
- output: `HIL_SENSOR`
- rate: 250 Hz
- fields:
  - `xacc/yacc/zacc`: body frame acceleration, m/s^2
  - `xgyro/ygyro/zgyro`: body frame angular rate, rad/s
  - `xmag/ymag/zmag`: body frame magnetic field, gauss 또는 PX4 simulator convention에 맞춘 값
  - `abs_pressure`: hPa
  - `pressure_alt`: m
  - `temperature`: degC
- 초기에는 noise 없이 deterministic 값으로 시작한다.
- 이후 bias, random walk, Gaussian noise, update jitter를 추가한다.

GPS:

- source: world pose + NED origin
- output: `HIL_GPS`
- rate: 5 Hz 또는 10 Hz
- fields:
  - `lat/lon/alt`: origin 기준 local offset을 WGS84로 변환
  - `vn/ve/vd`: NED velocity cm/s
  - `vel`: horizontal/3D speed cm/s
  - `cog`: course over ground centidegree
  - `fix_type`: 3 이상
  - `eph/epv`: 초기 1.0 m 수준
  - `satellites_visible`: 초기 12

Baro:

- source: altitude
- output: `HIL_SENSOR.abs_pressure`, `HIL_SENSOR.pressure_alt`
- 수상 주행에서는 altitude 변화가 작으므로 baro stale이 발생하지 않도록 update bit와 rate를 유지한다.

Mag:

- source: world magnetic field + boat attitude
- output: `HIL_SENSOR.xmag/ymag/zmag`
- 초기에는 고정 magnetic vector를 body frame으로 회전한다.

### 6.5 Frame transform 구현

PX4와 Isaac Sim 사이에서 가장 먼저 고정해야 하는 계약이다.

PX4 convention:

- local position: NED
- body frame: FRD
  - x: forward
  - y: right
  - z: down
- attitude: PX4가 기대하는 body-to-world convention을 `simulator_mavlink` 입력 기준으로 검증한다.

Isaac/USD convention 확인 항목:

- stage up axis: 보통 Z-up
- world frame: 보통 x/y horizontal, z up
- boat model forward axis: USD asset에 따라 x-forward 또는 y-forward일 수 있음

변환 계획:

```text
Isaac world position     -> PX4 local NED position
Isaac world velocity     -> PX4 NED velocity
Isaac angular velocity   -> PX4 body FRD angular rate
Isaac orientation        -> PX4 roll/pitch/yaw 또는 sensor body frame 변환
```

검증 방법:

- boat가 Isaac +X 방향으로 움직일 때 PX4 NED north/east 중 어느 축이 증가하는지 기록한다.
- yaw 0, 90, 180 deg에서 GPS COG와 PX4 heading이 일치하는지 확인한다.
- 정지 상태에서 accelerometer z축이 PX4 expectation과 일치하는지 확인한다.

### 6.6 MAVLink bridge runtime 구현

Isaac extension 안에서는 현재 PX4 repo의 smoke-test bridge를 다음과 같이 확장한다.

- TCP server
  - 기본 `127.0.0.1:4560`
  - `px4_instance`별 port offset 지원
  - PX4 disconnect 시 재-listen
- receive loop
  - `HIL_ACTUATOR_CONTROLS` 최신값 저장
  - armed flag 해석
  - timeout 시 actuator 0 처리
- send loop
  - physics callback 또는 timer에서 `HIL_SENSOR` 송신
  - GPS timer에서 `HIL_GPS` 송신
  - 1 Hz `HEARTBEAT` 송신
- thread model
  - socket receive는 background thread 또는 non-blocking poll
  - Isaac physics state 접근은 main/physics thread에서만 수행
  - thread 간 공유 데이터는 lock 또는 atomic snapshot으로 보호

### 6.7 Isaac UI 및 config

Extension UI에 최소 제어를 둔다.

- PX4 host bind address
- simulator TCP port
- connect/listen 상태
- sensor rates
- origin lat/lon/alt
- max thrust
- actuator direction invert checkbox
- start/stop bridge button
- current actuator command display
- last sensor send time, packet counters, packet loss/debug counters

Config file 예:

```yaml
mavlink:
  bind_host: 127.0.0.1
  simulator_port: 4560
  system_id: 1
  component_id: 250

boat:
  prim_path: /World/Boat
  right_thruster:
    control_index: 0
    max_thrust_N: 120.0
    position_body_m: [-2.0, 1.0, 0.0]
    direction_body: [1.0, 0.0, 0.0]
  left_thruster:
    control_index: 1
    max_thrust_N: 120.0
    position_body_m: [-2.0, -1.0, 0.0]
    direction_body: [1.0, 0.0, 0.0]

sensors:
  imu_rate_hz: 250
  gps_rate_hz: 5
  baro_rate_hz: 50
  mag_rate_hz: 50
```

### 6.8 Isaac Sim 검증 계획

단계별 검증:

1. Scene-only 검증
   - Isaac Sim에서 boat USD가 로드된다.
   - physics simulation을 시작해도 boat가 폭발하거나 침몰하지 않는다.
   - 좌/우 thruster에 수동 force를 주면 전진/회전한다.
2. Bridge-only 검증
   - PX4 없이 bridge TCP server가 시작된다.
   - dummy PX4 client 또는 현재 smoke-test로 MAVLink encode/decode가 동작한다.
3. PX4 연결 검증
   - PX4가 TCP 4560에 접속한다.
   - Isaac extension에서 actuator command 수신 counter가 증가한다.
   - PX4 `mavlink status`에서 simulator link가 connected로 보인다.
4. Sensor fusion 검증
   - PX4 `listener sensor_gps` update 확인
   - PX4 `ekf2 status`에서 GPS/IMU fusion 확인
   - `vehicle_local_position`이 Isaac pose와 같은 방향으로 변한다.
5. Actuator 검증
   - disarmed 상태에서 force가 0이다.
   - armed/manual throttle에서 boat가 전진한다.
   - yaw/steering command에서 좌우 thrust 차이가 발생한다.
6. Mission 검증
   - QGC waypoint mission upload
   - `boat_control`이 waypoint를 추종한다.
   - 경로 이탈, acceptance radius, overshoot를 로그로 확인한다.

### 6.9 Isaac Sim 산출물

완료 시 필요한 산출물:

- Isaac Sim extension 또는 standalone app
- `boat_environment.usd`
- `boat.usd` 또는 기존 boat asset reference
- `boat_px4.yaml`
- 실행 README
- 좌표계/actuator mapping 문서
- smoke test 절차
- PX4 ulog + Isaac trajectory 비교 예시

## 7. 검증 절차

### 7.1 빌드 검증

```sh
make px4_sitl_boat
```

확인:

- 빌드 성공
- `build/px4_sitl_boat/bin/px4` 생성
- `boat_control` module 포함

### 7.2 PX4 단독 기동 검증

```sh
PX4_SIM_MODEL=isaac_boat PX4_SIMULATOR=isaac ./build/px4_sitl_boat/bin/px4
```

확인:

- `SYS_AUTOSTART`가 `1071`로 설정되는지 확인
- `boat_control status`
- `mavlink status`
- `listener actuator_outputs_sim`
- `listener sensor_gps`
- `listener vehicle_odometry`

### 7.3 포트 검증

```sh
lsof -iTCP:4560 -sTCP:LISTEN
lsof -iUDP:18570
lsof -iUDP:14580
```

확인:

- simulator MAVLink TCP 4560 사용 여부
- GCS/Offboard UDP port가 기존 SITL 정책과 같은지 확인

### 7.4 Isaac Sim 연동 검증

1. PX4 SITL 실행
2. Isaac Sim boat environment 실행
3. Isaac bridge가 TCP 4560에 연결되는지 확인
4. PX4에서 sensor topic update 확인
5. arm 전 actuator가 0인지 확인
6. arm 후 manual 또는 mission command에서 actuator output이 Isaac Sim boat 움직임으로 반영되는지 확인
7. waypoint mission에서 boat가 경로를 추종하는지 확인

### 7.5 로그 검증

- `mavlink status`: packet loss, connection status
- `ekf2 status`: GPS/IMU fusion 상태
- `commander status`: arming/failsafe 상태
- `logger`: ulog 저장 후 `vehicle_local_position`, `vehicle_attitude`, `actuator_outputs`, `vehicle_command` 확인

## 8. 작업 순서

1. `boards/px4/sitl/boat.px4board` 추가
2. `make px4_sitl_boat` 빌드 확인
3. `1071_isaac_boat` airframe 추가
4. airframe 등록 및 `PX4_SIM_MODEL=isaac_boat` 매칭 확인
5. `PX4_SIMULATOR=isaac`일 때 `px4-rc.mavlinksim`을 사용하도록 init flow 확인
6. Isaac Sim MAVLink bridge 최소 구현
7. HIL_SENSOR/HIL_GPS 송신 확인
8. HIL_ACTUATOR_CONTROLS 수신 및 boat actuator mapping 확인
9. GCS/Offboard UDP 포트가 기존 SITL과 동일한지 확인
10. mission waypoint 기반 boat 주행 검증

Isaac Sim 쪽 작업 순서:

1. Isaac Sim extension skeleton 생성
2. boat USD model과 water world scene 구성
3. config 기반 origin/thruster/sensor 설정 로딩
4. TCP MAVLink server를 Isaac extension lifecycle에 연결
5. Isaac physics callback에서 boat state 읽기
6. frame transform과 GPS origin 변환 구현
7. `HIL_SENSOR`, `HIL_GPS`, `HEARTBEAT` 송신 구현
8. `HIL_ACTUATOR_CONTROLS` 수신 및 armed flag 처리
9. 좌/우 thruster force 적용
10. sensor fusion, manual control, mission waypoint 순서로 통합 검증

## 9. 미정 및 확인 필요 사항

- Isaac Sim boat model의 추진기 배치와 좌표계
- `HIL_ACTUATOR_CONTROLS.controls[]`를 좌/우 추진기로 직접 쓸지, steering/throttle로 변환할지 여부
- Isaac Sim bridge를 PX4 repo 내부 script로 둘지, Isaac extension repo로 분리할지 여부
- lockstep 유지 가능 여부
- GPS origin과 world origin 기준점
- boat water dynamics 모델 범위
- QGC 연결 포트를 기본 discovery로 둘지 명시 target으로 설정할지 여부
- Isaac Sim extension 배포 위치와 repo ownership
- Isaac Sim 버전 및 required extension dependency 목록
- water dynamics를 단순 force 모델로 유지할지 PhysX/외부 hydrodynamics 모델로 확장할지 여부

## 10. 완료 기준

- `make px4_sitl_boat`가 성공한다.
- `PX4_SIM_MODEL=isaac_boat`로 PX4 SITL이 boat airframe을 선택한다.
- Isaac Sim bridge가 MAVLink로 PX4에 연결된다.
- PX4가 Isaac Sim 센서 데이터를 받아 EKF2 position/attitude를 추정한다.
- PX4 actuator output이 Isaac Sim boat 추진기 명령으로 반영된다.
- 기존 SITL과 동일한 MAVLink network port 정책을 유지한다.
- manual 또는 mission mode에서 boat가 시뮬레이션 환경 안에서 정상 이동한다.
- Isaac Sim extension에서 scene load, bridge start/stop, actuator display, packet counters를 확인할 수 있다.
- Isaac trajectory와 PX4 ulog의 position/heading/actuator 값이 같은 방향과 scale로 일치한다.
