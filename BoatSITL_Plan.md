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
  |  | - engine + clutch     |  |                 |  | simulator_mavlink     |  |
  |  | - steering            |  |                 |  | TCP client            |  |
  |  | - sensor models       |  |                 |  | remote 127.0.0.1:4560 |  |
  |  +-----------+-----------+  |                 |  +-----------+-----------+  |
  |              |              |                             |
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
  |    controls[0]: steering                   |                             |
  |    controls[1]: signed thrust              | sensors / estimator:        |
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
- PX4 control pipeline은 `boat_control`과 `control_allocator`를 통해 steering과 signed thrust output을 만든다.
- PX4 `simulator_mavlink`는 actuator output을 `HIL_ACTUATOR_CONTROLS`로 Isaac bridge에 보낸다.
- Isaac bridge는 `HIL_ACTUATOR_CONTROLS`를 Isaac Sim boat steering, engine throttle, clutch state 명령으로 변환한다.

실제 보트와 SITL의 output interface는 다음처럼 분리한다.

```text
Primary real boat interface:
  MAVLink SERVO_OUTPUT_RAW

Debug/backup interface:
  pwm_out physical pins

SITL interface:
  HIL_ACTUATOR_CONTROLS
```

실제 Pixhawk firmware에서는 PX4 내부에 boat 전용 drivetrain driver를 두지 않는다. `boat_control`과 output mapping이 만든 steering/signed thrust를 `SERVO_OUTPUT_RAW`로 외부 MAVLink boat driver에 전달하고, 외부 driver가 engine, clutch, steering을 제어한다. `pwm_out` physical pins는 같은 mapping을 bench에서 확인하거나 backup wiring을 시험하는 debug 경로로 둔다.

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
  - 이는 과거 two-thruster 참고값이다. 새 Isaac boat contract는 1 engine + 1 clutch + 1 steering이며, SITL output order는 `HIL_ACTUATOR_CONTROLS.controls[0]=steering`, `controls[1]=signed_thrust`로 둔다.
  - 실제 보트 primary interface는 `SERVO_OUTPUT_RAW.servo1_raw=steering`, `SERVO_OUTPUT_RAW.servo2_raw=signed_thrust`로 둔다.
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

1. Boat 전용 SITL build target 추가 및 구성 확인
2. Isaac Sim boat airframe 추가 및 actuator contract 확인
3. Isaac Sim과 PX4 SITL 간 MAVLink HIL bridge 정의
4. 기존 드론용 SITL과 동일한 네트워크 인터페이스와 포트 정책 적용
5. 빌드 및 런타임 검증 절차 작성

## PX4쪽 구현 계획: rover SITL 기준 업데이트

`gz_rover_ackermann` 분석 기준으로 보면 PX4쪽 구현의 책임은 simulator physics를 직접 구현하는 것이 아니라, airframe/init flow, vehicle module, sensor input contract, actuator output contract를 명확히 만드는 것이다.

rover Gazebo SITL의 PX4쪽 구조:

```text
make px4_sitl gz_rover_ackermann
        |
        v
PX4_SIM_MODEL=gz_rover_ackermann
        |
        v
rcS
        |
        v
airframe 51000_gz_rover_ackermann
        |
        +--> rc.rover_ackermann_defaults
        +--> VEHICLE_TYPE=rover_ackermann
        +--> PX4_SIMULATOR=gz
        +--> PX4_GZ_WORLD=rover
        +--> PX4_SIM_MODEL=rover_ackermann
        +--> actuator mapping: SIM_GZ_WH_* / SIM_GZ_SV_*
        |
        v
px4-rc.simulator
        |
        v
px4-rc.gzsim
        |
        v
rc.vehicle_setup
        |
        v
rc.rover_ackermann_apps
        |
        +--> rover_ackermann start
        +--> land_detector start rover
```

Isaac boat SITL의 PX4쪽 목표 구조:

```text
make px4_sitl_boat
        |
        v
PX4_SIM_MODEL=isaac_boat
PX4_SIMULATOR=isaac
        |
        v
rcS
        |
        v
airframe 1071_isaac_boat
        |
        +--> rc.boat_defaults
        +--> VEHICLE_TYPE=boat
        +--> PX4_SIMULATOR=isaac
        +--> PX4_SIM_MODEL=isaac_boat
        +--> actuator mapping: PWM_MAIN_FUNC* / CA_ROTOR*
        |
        v
px4-rc.simulator
        |
        v
px4-rc.mavlinksim
        |
        v
rc.vehicle_setup
        |
        v
rc.boat_apps
        |
        +--> boat_control start
        +--> land_detector start rover
```

rover와 boat의 대응 관계:

| 역할 | rover Gazebo SITL | Isaac boat SITL |
| --- | --- | --- |
| build target | `gz_rover_ackermann` | `px4_sitl_boat` |
| airframe | `51000_gz_rover_ackermann` | `1071_isaac_boat` |
| defaults | `rc.rover_ackermann_defaults` | `rc.boat_defaults` |
| vehicle type | `rover_ackermann` | `boat` |
| vehicle app | `rover_ackermann start` | `boat_control start` |
| simulator launcher | `px4-rc.gzsim` | `px4-rc.mavlinksim` |
| actuator contract | `SIM_GZ_WH_*`, `SIM_GZ_SV_*` | steering + signed thrust, `HIL_ACTUATOR_CONTROLS.controls[]` |
| simulator side | Gazebo SDF + `gz_bridge` | Isaac USD/stage + MAVLink bridge |

PX4쪽에서 구현/검증해야 할 항목은 다음이다.

1. `boards/px4/sitl/boat.px4board`
   - `boat_control`, `mavlink`, `simulator_mavlink`, `sensors`, `ekf2`, `commander`, `navigator`, `control_allocator`, `pwm_out_sim`이 포함되어야 한다.
   - MC/FW/VTOL/Rover/UUV 전용 controller와 Gazebo bridge는 boat SITL 목적상 제외한다.

2. `ROMFS/px4fmu_common/init.d-posix/airframes/1071_isaac_boat`
   - `rc.boat_defaults`를 source해야 한다.
   - `PX4_SIMULATOR=isaac`, `PX4_SIM_MODEL=isaac_boat` contract를 둔다.
   - 1 engine, 1 clutch, 1 steering actuator contract를 명시한다.
   - PX4 내부 contract는 `actuator_servos.control[0] = steering`, `actuator_motors.control[0] = signed_thrust`이다.
   - 외부 simulator/HIL contract는 `controls[0] = steering`, `controls[1] = signed_thrust`로 둔다.

3. `ROMFS/px4fmu_common/init.d/rc.boat_defaults`
   - `set VEHICLE_TYPE boat`가 설정되어야 한다.
   - `MAV_TYPE=11` surface boat 값을 유지한다.
   - boat mission/waypoint 추종에 필요한 기본 navigation radius 등을 설정한다.

4. `ROMFS/px4fmu_common/init.d/rc.boat_apps`
   - `boat_control start`가 실행되어야 한다.
   - rover/boat 모두 지상 차량 취급이므로 `land_detector start rover`를 유지한다.

5. `ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator`
   - `PX4_SIMULATOR=isaac`은 `gz`, `sihsim`, `jmavsim` 조건에 걸리지 않아야 한다.
   - 최종적으로 `else` 경로에서 `px4-rc.mavlinksim`을 실행해야 한다.

6. `ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlinksim`
   - `simulator_mavlink start -c 4560` 기본 흐름을 사용한다.
   - `PX4_SIM_HOSTNAME`, `PX4_SIM_HOST_ADDR`, `px4_instance` port offset 동작을 유지한다.

7. PX4 sensor input contract
   - Isaac bridge가 보내는 `HIL_SENSOR`, `HIL_GPS`가 PX4 내부 sensor/uORB 흐름으로 정상 반영되어야 한다.
   - `sensors`, `ekf2`, `commander`가 이 입력으로 arming과 estimator 상태를 정상 판단해야 한다.

8. PX4 actuator output contract
   - `boat_control`과 control allocation 결과가 simulator output으로 전달되어야 한다.
   - `simulator_mavlink`가 이 출력을 `HIL_ACTUATOR_CONTROLS`로 송신해야 한다.
   - `controls[0]`, `controls[1]`의 steering/signed thrust 순서를 airframe comment, output mapping, Isaac bridge, USD model에서 동일하게 맞춰야 한다.
   - 실제 보트 firmware에서는 동일한 output mapping을 `SERVO_OUTPUT_RAW.servo1_raw/servo2_raw`와 debug `pwm_out` CH1/CH2로 확인할 수 있어야 한다.

현재 repo 기준으로는 `boards/px4/sitl/boat.px4board`, `1071_isaac_boat`, `rc.boat_apps`, `rc.boat_defaults`가 이미 존재한다. 따라서 다음 작업은 파일 생성보다 “rover SITL과 같은 init contract가 실제 실행에서 성립하는지”를 검증하고, actuator index mapping 불일치를 제거하는 것이다.

## 통합 분석 요약

이 섹션은 기존 통합 메모에 정리되어 있던 `rover_sitl_gz.md`, `px4_sitl.md`, `gazebo_sitl_comm.md`, `boat_rover.md`의 핵심을 Boat SITL 계획 관점에서 정리한다.

### A. `gz_rover_ackermann` Reference Lessons

`gz_rover_ackermann`은 Gazebo GZ native bridge를 사용하는 rover SITL reference이다.

```text
make px4_sitl gz_rover_ackermann
        |
        v
PX4_SIM_MODEL=gz_rover_ackermann
        |
        v
rcS
        |
        v
airframe 51000_gz_rover_ackermann
        |
        +--> rc.rover_ackermann_defaults
        +--> VEHICLE_TYPE=rover_ackermann
        +--> PX4_SIMULATOR=gz
        +--> PX4_GZ_WORLD=rover
        +--> PX4_SIM_MODEL=rover_ackermann
        +--> SIM_GZ_EN=1
        +--> SIM_GZ_WH_* wheel output mapping
        +--> SIM_GZ_SV_* servo output mapping
        |
        v
px4-rc.simulator
        |
        v
px4-rc.gzsim
        |
        +--> start world: Tools/simulation/gz/worlds/rover.sdf
        +--> spawn model: Tools/simulation/gz/models/rover_ackermann/model.sdf
        +--> gz_bridge start -w rover -n rover_ackermann_0
        |
        v
rc.vehicle_setup
        |
        v
rc.rover_ackermann_apps
        |
        +--> rover_ackermann start
        +--> land_detector start rover
```

중요한 점은 `PX4_SIM_MODEL`이 두 단계 의미를 가진다는 것이다.

```text
make/CMake target:
  PX4_SIM_MODEL=gz_rover_ackermann
  -> rcS가 51000_gz_rover_ackermann airframe을 찾기 위한 이름

airframe 내부:
  PX4_SIM_MODEL=rover_ackermann
  -> px4-rc.gzsim이 Gazebo model directory를 찾기 위한 이름
```

Rover actuator path:

```text
rover_ackermann
  -> actuator_motors.control[0]
  -> actuator_servos.control[0]
  -> GZMixingInterfaceWheel / GZMixingInterfaceServo
  -> Gazebo wheel / steering topics
```

Rover Gazebo actuator mapping:

| Purpose | Parameter | Meaning |
| --- | --- | --- |
| Wheel function | `SIM_GZ_WH_FUNC1=101` | Motor1 |
| Wheel output range | `SIM_GZ_WH_MIN1=70`, `SIM_GZ_WH_MAX1=130`, `SIM_GZ_WH_DIS1=100` | wheel velocity offset around 100 |
| Steering function | `SIM_GZ_SV_FUNC1=201` | Servo1 |
| Steering range | `SIM_GZ_SV_MINA1=-30`, `SIM_GZ_SV_MAXA1=30` | steering angle deg |
| Steering reverse | `SIM_GZ_SV_REV=1` | reverse servo direction |
| Ackermann geometry | `RA_WHEEL_BASE=0.5`, `RA_MAX_STR_ANG=0.5236` | wheelbase and max steering |

Rover sensor topic reference:

| Data | Gazebo topic | PX4 uORB |
| --- | --- | --- |
| Clock | `/world/rover/clock` | PX4 simulation clock |
| Pose | `/world/rover/pose/info` | `vehicle_*_groundtruth` |
| IMU | `/world/rover/model/rover_ackermann_0/link/base_link/sensor/imu_sensor/imu` | `sensor_accel`, `sensor_gyro` |
| Magnetometer | `/world/rover/model/rover_ackermann_0/link/base_link/sensor/magnetometer_sensor/magnetometer` | `sensor_mag` |
| GPS/NavSat | `/world/rover/model/rover_ackermann_0/link/base_link/sensor/navsat_sensor/navsat` | `sensor_gps` |
| Barometer | `/world/rover/model/rover_ackermann_0/link/base_link/sensor/air_pressure_sensor/air_pressure` | `sensor_baro` |

Boat에 적용할 교훈:

```text
controller should not know simulator details
airframe defines actuator contract
bridge maps uORB/output to simulator endpoints
simulator model must match names, axes, units, and actuator order
```

### B. Gazebo GZ Bridge vs MAVLink Simulator Path

PX4 simulator 통신 경로는 크게 두 종류로 볼 수 있다.

| Path | Typical target | PX4 module | Transport |
| --- | --- | --- | --- |
| Gazebo GZ | `make px4_sitl gz_rover_ackermann` | `gz_bridge` | Gazebo Transport to uORB |
| MAVLink simulator | `PX4_SIMULATOR=isaac` | `simulator_mavlink` | MAVLink HIL messages |

Gazebo GZ path:

```text
Gazebo Transport
  -> gz_bridge
  -> uORB sensor topics
  -> PX4 controllers
  -> actuator_motors/servos
  -> GZMixingInterface*
  -> Gazebo actuator topics
```

MAVLink simulator path:

```text
External simulator bridge
  -> HEARTBEAT
  -> HIL_SENSOR
  -> HIL_GPS
  -> optional HIL_STATE_QUATERNION / ODOMETRY
  -> simulator_mavlink
  -> uORB sensor topics
  -> PX4 controllers
  -> actuator_outputs_sim
  -> HIL_ACTUATOR_CONTROLS
  -> external simulator bridge
```

Isaac boat는 Isaac/Omniverse dependency를 PX4 binary에 link하지 않기 위해 MAVLink simulator path로 시작한다.

### C. Coordinate And Time Contracts

Gazebo GZ는 ENU/FLU convention을 사용하고 `gz_bridge`가 PX4 NED/FRD로 변환한다.

```text
Gazebo world frame: ENU
PX4 local frame:   NED

Gazebo body frame: FLU
PX4 body frame:    FRD
```

Isaac bridge도 같은 수준의 명시적 변환 contract가 필요하다.

```text
Isaac world axis      -> PX4 local NED
Isaac boat body axis  -> PX4 body FRD
Isaac angular rates   -> PX4 body rates
Isaac acceleration    -> HIL_SENSOR expected body acceleration
Isaac world origin    -> HIL_GPS lat/lon/alt origin
```

Time sync reference:

```text
Gazebo GZ:
  /world/<world>/clock
  -> GZBridge::clockCallback()
  -> px4_clock_settime()

MAVLink simulator:
  HIL_SENSOR.time_usec
  -> simulator_mavlink
  -> lockstep clock progression
```

Isaac 초기 구현은 non-lockstep 또는 soft sync로 시작하고, `HIL_SENSOR.time_usec` monotonic 증가와 안정적인 sensor rate를 먼저 보장한다. 이후 필요하면 actuator command가 Isaac physics step을 trigger하고, step 결과가 새 `HIL_SENSOR` sample을 만드는 lockstep형 구조를 검토한다.

### D. `boat_control` vs `rover_ackermann`

두 모듈은 PX4 ground vehicle 구조를 공유한다.

```text
Mode layer
  -> setpoint topics
  -> controller layer
  -> actuator topic layer
```

공통점:

- `vehicle_status.nav_state`와 `vehicle_control_mode`로 mode별 동작을 dispatch한다.
- `rover_position_setpoint`, `rover_speed_setpoint`, `rover_attitude_setpoint`, `rover_rate_setpoint`, `rover_steering_setpoint`, `rover_throttle_setpoint`를 사용한다.
- simulator-specific command가 아니라 actuator topic을 publish한다.

차이점:

```text
rover_ackermann:
  position
    -> speed
    -> attitude
    -> rate
    -> steering
    -> actuator output

boat_control:
  position
    -> speed
    -> attitude/yaw-rate
    -> direct steering
    -> signed thrust
    -> actuator output
```

Ackermann rover에는 다음 kinematic relation이 유효하다.

```text
yaw_rate ~= velocity * tan(steering_angle) / wheel_base
```

Boat yaw response는 hull, rudder, prop wash, water flow, speed, drag, wave, current, actuator placement에 의존한다. 따라서 `boat_control`은 Ackermann의 rate-to-steering model을 그대로 복사하지 않고, 초기에는 yaw/yaw-rate setpoint를 `BoatAttControl`에서 steering으로 직접 변환한다. 더 넓은 speed envelope 또는 yaw-rate tracking 성능이 필요해지면 Ackermann kinematics가 아니라 boat-specific yaw-rate controller를 추가한다.

### E. Boat Output Flow Summary

```text
vehicle state / mission / manual input
        |
        v
boat_control
        |
        +--> BoatAutoMode / BoatManualMode / BoatOffboardMode
        +--> BoatPosControl
        +--> BoatSpeedControl
        +--> BoatAttControl
        +--> BoatActControl
                 |
                 v
        uORB actuator topics
                 |
                 +--> actuator_servos.control[0] = steering [-1, 1]
                 +--> actuator_motors.control[0] = signed_thrust [-1, 1]
                 |
                 v
        control allocation / output mapping
                 |
                 +--> SITL: actuator_outputs_sim -> HIL_ACTUATOR_CONTROLS
                 |          controls[0] = steering
                 |          controls[1] = signed_thrust
                 |
                 +--> real: actuator_outputs -> SERVO_OUTPUT_RAW
                 |          servo1_raw = steering PWM-equivalent
                 |          servo2_raw = signed_thrust PWM-equivalent
                 |
                 +--> debug/backup: pwm_out physical pins
                            CH1 = steering
                            CH2 = signed_thrust
```

Drivetrain conversion은 PX4 controller/control allocation output이 아니다. Isaac bridge 또는 외부 MAVLink boat driver가 다음을 담당한다.

- clutch deadband
- clutch hysteresis
- shift delay
- engage delay
- throttle cut before gear change
- thrust ramp limit
- failsafe neutral/zero behavior
- reverse inhibit if future speed safety requires it

### F. Key Files

PX4 startup:

```text
ROMFS/px4fmu_common/init.d-posix/rcS
ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator
ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlinksim
ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim
ROMFS/px4fmu_common/init.d/rc.vehicle_setup
```

Boat:

```text
boards/px4/sitl/boat.px4board
boards/px4/fmu-v6x/boat.px4board
ROMFS/px4fmu_common/init.d-posix/airframes/1071_isaac_boat
ROMFS/px4fmu_common/init.d/rc.boat_defaults
ROMFS/px4fmu_common/init.d/rc.boat_apps
src/modules/boat_control
src/modules/boat_control/Plan.md
src/modules/boat_control/Controller.md
```

Rover/Gazebo reference:

```text
ROMFS/px4fmu_common/init.d-posix/airframes/51000_gz_rover_ackermann
ROMFS/px4fmu_common/init.d/rc.rover_ackermann_defaults
ROMFS/px4fmu_common/init.d/rc.rover_ackermann_apps
Tools/simulation/gz/worlds/rover.sdf
Tools/simulation/gz/models/rover_ackermann/model.sdf
src/modules/simulation/gz_bridge/GZBridge.cpp
src/modules/simulation/gz_bridge/GZMixingInterfaceWheel.cpp
src/modules/simulation/gz_bridge/GZMixingInterfaceServo.cpp
```

MAVLink simulator:

```text
src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp
Tools/simulation/isaac/bridge/isaac_boat_mavlink_bridge.py
```

## 1. Boat 전용 SITL build target

### 1.1 `boards/px4/sitl/boat.px4board` 구성 확인

`boards/px4/sitl/default.px4board`를 base로 merge되는 label board를 사용한다. PX4 빌드 시스템은 `boards/*/*/*.px4board` 파일을 target으로 변환하므로, `boards/px4/sitl/boat.px4board`가 있으면 `px4_sitl_boat` target을 사용할 수 있다.

초기 구성 방향:

```text
CONFIG_MODULES_AIRSPEED_SELECTOR=n
CONFIG_MODULES_FLIGHT_MODE_MANAGER=n
CONFIG_MODULES_FW_ATT_CONTROL=n
CONFIG_MODULES_FW_AUTOTUNE_ATTITUDE_CONTROL=n
CONFIG_MODULES_FW_MODE_MANAGER=n
CONFIG_MODULES_FW_LATERAL_LONGITUDINAL_CONTROL=n
CONFIG_MODULES_FW_RATE_CONTROL=n
CONFIG_FIGURE_OF_EIGHT=n
CONFIG_MODULES_MC_ATT_CONTROL=n
CONFIG_MODULES_MC_AUTOTUNE_ATTITUDE_CONTROL=n
CONFIG_MODULES_MC_HOVER_THRUST_ESTIMATOR=n
CONFIG_MODULES_MC_POS_CONTROL=n
CONFIG_MODULES_MC_RATE_CONTROL=n
CONFIG_MODULES_ROVER_ACKERMANN=n
CONFIG_MODULES_ROVER_DIFFERENTIAL=n
CONFIG_MODULES_ROVER_MECANUM=n
CONFIG_MODULES_SPACECRAFT=n
CONFIG_MODULES_UUV_ATT_CONTROL=n
CONFIG_MODULES_UUV_POS_CONTROL=n
CONFIG_MODULES_VTOL_ATT_CONTROL=n
CONFIG_MODULES_SIMULATION_GZ_BRIDGE=n
CONFIG_MODULES_SIMULATION_GZ_MSGS=n
CONFIG_MODULES_SIMULATION_GZ_PLUGINS=n
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
- Isaac boat는 Gazebo native bridge를 쓰지 않으므로 `gz_bridge` 관련 module이 boat SITL target에서 제외되는지 확인한다.

## 2. Isaac Sim boat airframe 추가

### 2.1 airframe 파일 구성

다음 airframe 파일을 사용한다.

```text
ROMFS/px4fmu_common/init.d-posix/airframes/1071_isaac_boat
```

수정 목표 구성 방향:

```sh
#!/bin/sh
#
# @name Isaac Sim Boat
# @type Boat
#

. ${R}etc/init.d/rc.boat_defaults

PX4_SIMULATOR=${PX4_SIMULATOR:=isaac}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=isaac_boat}

param set-default CA_AIRFRAME 9

# Boat actuator contract:
# - PX4 internal:
#   actuator_servos.control[0] = steering
#   actuator_motors.control[0] = signed_thrust
# - SITL/HIL order:
#   controls[0] = steering
#   controls[1] = signed_thrust
# - Real boat primary interface:
#   SERVO_OUTPUT_RAW.servo1_raw = steering PWM-equivalent
#   SERVO_OUTPUT_RAW.servo2_raw = signed_thrust PWM-equivalent
# - Debug/backup physical pins:
#   PWM CH1 = steering
#   PWM CH2 = signed_thrust
# - signed_thrust sign is converted to clutch state by Isaac bridge or external MAVLink boat driver.
param set-default CA_ROTOR_COUNT 1
param set-default CA_ROTOR0_AX 1
param set-default CA_ROTOR0_AZ 0
param set-default CA_ROTOR0_KM 0
param set-default CA_ROTOR0_PX -3.7
param set-default CA_ROTOR0_PY 0
param set-default CA_ROTOR0_PZ -0.15
param set-default CA_R_REV 1
param set-default CA_SV_CS_COUNT 1

param set-default PWM_MAIN_FUNC1 201  # Servo1: steering
param set-default PWM_MAIN_FUNC2 101  # Motor1: signed thrust
param set-default PWM_MAIN_MIN1 1000
param set-default PWM_MAIN_TRIM1 1500
param set-default PWM_MAIN_MAX1 2000
param set-default PWM_MAIN_MIN2 1000
param set-default PWM_MAIN_TRIM2 1500
param set-default PWM_MAIN_MAX2 2000
```

현재 repo의 `1071_isaac_boat`는 1 engine + 1 clutch + 1 steering contract로 수정되었다. PX4 내부에는 clutch output을 만들지 않고, steering과 signed thrust만 output function에 배치한다.

```text
PX4 internal:
  actuator_servos.control[0] = steering
  actuator_motors.control[0] = signed_thrust

SITL / simulator order:
  HIL_ACTUATOR_CONTROLS.controls[0] = steering
  HIL_ACTUATOR_CONTROLS.controls[1] = signed_thrust

Real boat primary MAVLink order:
  SERVO_OUTPUT_RAW.servo1_raw = steering PWM-equivalent
  SERVO_OUTPUT_RAW.servo2_raw = signed_thrust PWM-equivalent

Debug/backup PWM order:
  PWM CH1 = steering
  PWM CH2 = signed_thrust
```

`signed_thrust`는 Isaac bridge 또는 외부 MAVLink boat driver에서 다음처럼 drivetrain 명령으로 변환한다.

```text
signed_thrust > deadband:
  clutch = forward
  throttle = abs(signed_thrust)

signed_thrust < -deadband:
  clutch = reverse
  throttle = abs(signed_thrust)

abs(signed_thrust) <= deadband:
  clutch = neutral
  throttle = 0
```

clutch는 control allocation의 독립 output으로 두지 않는다. clutch는 signed thrust를 실제 engine system에 적용하기 위한 하위 drivetrain state이며, PX4 내부 driver가 아니라 Isaac bridge 또는 외부 MAVLink boat driver가 처리한다.

### 2.2 airframe 등록 확인

`ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`에 신규 airframe이 자동 포함되는지 확인한다. 자동 glob이 아니라 명시 목록이면 `1071_isaac_boat`를 추가한다.

### 2.3 실행 모델 선택

PX4 init script는 `PX4_SIM_MODEL`과 airframe 파일명을 매칭한다. 따라서 아래 값으로 실행되게 한다.

```sh
PX4_SIM_MODEL=isaac_boat
```

`rcS`는 이 값을 이용해 `1071_isaac_boat`를 찾고, airframe 내부의 `rc.boat_defaults`가 `VEHICLE_TYPE=boat`를 설정한다. 이후 `rc.vehicle_setup`에서 `rc.boat_apps`가 선택되어 `boat_control start`가 실행되어야 한다.

## 3. Isaac Sim 연동 방식

### 3.1 기본 통신 방향

PX4의 기존 `simulator_mavlink` module을 우선 재사용한다.

- Isaac Sim 또는 별도 bridge process가 TCP server/listener로 대기하고, PX4 `simulator_mavlink`가 TCP client로 접속한다.
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
  - 기본 mapping은 `controls[0] = steering`, `controls[1] = signed_thrust`로 둔다.
  - 이 순서는 `1071_isaac_boat`, output mapping, Isaac bridge, USD model과 반드시 일치해야 한다.
  - signed thrust의 부호는 clutch forward/neutral/reverse로 변환하고, magnitude는 engine throttle로 변환한다.
  - `mode`의 armed flag를 확인하여 disarmed 상태에서는 추진기 출력을 0으로 둔다.

실제 보트에서 외부 driver가 받을 MAVLink message:

- `SERVO_OUTPUT_RAW`
  - `servo1_raw = steering PWM-equivalent`
  - `servo2_raw = signed_thrust PWM-equivalent`
  - `servo2_raw`는 1500 us를 neutral 기준으로 하여 1000 us 방향을 reverse, 2000 us 방향을 forward로 해석한다.
  - 외부 driver는 `SERVO_OUTPUT_RAW` timeout 시 engine throttle 0, clutch neutral을 강제한다.

실제 보트에서 debug/backup으로 확인할 physical output:

- `pwm_out`
  - `PWM CH1 = steering`
  - `PWM CH2 = signed_thrust`
  - `SERVO_OUTPUT_RAW.servo1_raw/servo2_raw`와 같은 output mapping을 공유해야 한다.

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
- `drivetrain_controller.py`: PX4 actuator command를 Isaac steering, engine force, clutch state로 적용
- `boat_px4.yaml`: port, origin, sensor rate, steering/signed thrust mapping, scale, noise 설정

### 6.2 Isaac Sim scene 구성

필수 scene 구성:

- Boat USD model
  - rigid body 또는 articulation root 설정
  - mass, inertia, center of mass 설정
  - visual mesh와 collision mesh 분리
  - steering/rudder actuator 위치와 축을 명확히 표시
  - engine thrust application point와 방향을 명확히 표시
  - clutch state는 별도 control allocation output이 아니라 signed thrust sign에서 파생되는 drivetrain state로 시작
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
   - signed thrust magnitude를 body frame 기준 forward/reverse engine force로 적용한다.
   - steering command는 rudder/steering yaw moment 또는 rudder angle model로 적용한다.
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

초기 actuator mapping:

```text
PX4 HIL_ACTUATOR_CONTROLS.controls[0] -> steering normalized [-1, 1]
PX4 HIL_ACTUATOR_CONTROLS.controls[1] -> signed_thrust normalized [-1, 1]

steering_cmd = controls[0]
signed_thrust = controls[1]

if signed_thrust > deadband:
  clutch = forward
  engine_force_N = abs(signed_thrust) * max_forward_thrust_N
elif signed_thrust < -deadband:
  clutch = reverse
  engine_force_N = -abs(signed_thrust) * max_reverse_thrust_N
else:
  clutch = neutral
  engine_force_N = 0
```

주의:

- PX4 airframe의 `CA_R_REV=1`은 Motor1 signed thrust가 reversible임을 의미한다.
- `controls[0]`, `controls[1]` 순서는 `1071_isaac_boat`의 output mapping과 Isaac bridge, boat USD가 일치해야 한다.
- clutch forward/reverse 전환에는 deadband, hysteresis, shift delay, throttle cut을 둔다.
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
- max forward/reverse thrust
- steering scale
- clutch deadband/hysteresis/shift delay
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
  steering:
    control_index: 0
    max_rudder_angle_deg: 30.0
    invert: false
  engine:
    control_index: 1
    max_forward_thrust_N: 120.0
    max_reverse_thrust_N: 80.0
    position_body_m: [-2.0, 0.0, 0.0]
    direction_body: [1.0, 0.0, 0.0]
    deadband: 0.03
    shift_delay_s: 0.5
    engage_delay_s: 0.2

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
   - steering과 signed thrust에 수동 명령을 주면 전진/후진/회전한다.
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
   - armed/manual throttle에서 boat가 전진/후진한다.
   - steering command에서 보트 yaw 방향이 올바르게 변한다.
   - signed thrust 부호 전환 시 clutch state가 forward/neutral/reverse 순서로 안전하게 바뀐다.
   - 실제 보트의 `SERVO_OUTPUT_RAW.servo1_raw/servo2_raw` contract와 같은 steering/signed thrust 해석을 사용한다.
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
- `rc.boat_defaults`에 의해 `VEHICLE_TYPE=boat`가 설정되는지 확인
- `rc.vehicle_setup`이 `rc.boat_apps`를 선택하는지 확인
- `boat_control status`
- `simulator_mavlink status`
- `mavlink status`
- `listener actuator_outputs_sim`
- `listener actuator_motors`
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
3. Isaac bridge가 TCP 4560에서 listen하고, PX4 `simulator_mavlink`가 client로 접속하는지 확인
4. PX4에서 sensor topic update 확인
5. arm 전 actuator가 0인지 확인
6. arm 후 manual 또는 mission command에서 actuator output이 Isaac Sim boat 움직임으로 반영되는지 확인
7. waypoint mission에서 boat가 경로를 추종하는지 확인

### 7.5 실제 보트 MAVLink output 검증

실제 Pixhawk firmware 또는 bench 환경에서는 SITL의 `HIL_ACTUATOR_CONTROLS` 대신 `SERVO_OUTPUT_RAW`를 확인한다.

확인 항목:

- `SERVO_OUTPUT_RAW.servo1_raw`가 steering 명령에 따라 1500 us 기준 좌/우로 변한다.
- `SERVO_OUTPUT_RAW.servo2_raw`가 signed thrust 명령에 따라 1500 us 기준 forward/reverse 방향으로 변한다.
- `pwm_out` debug pin CH1/CH2가 `SERVO_OUTPUT_RAW.servo1_raw/servo2_raw`와 같은 방향과 scale로 움직인다.
- 외부 MAVLink boat driver가 `servo2_raw` deadband 안에서는 clutch neutral, throttle 0을 만든다.
- 외부 MAVLink boat driver가 `SERVO_OUTPUT_RAW` timeout 시 throttle 0, clutch neutral을 강제한다.

### 7.6 로그 검증

- `mavlink status`: packet loss, connection status
- `ekf2 status`: GPS/IMU fusion 상태
- `commander status`: arming/failsafe 상태
- `logger`: ulog 저장 후 `vehicle_local_position`, `vehicle_attitude`, `actuator_outputs`, `vehicle_command` 확인
- 실제 firmware debug 시 QGC MAVLink Inspector에서 `SERVO_OUTPUT_RAW`와 필요 시 `ACTUATOR_OUTPUT_STATUS` 확인

## 8. 작업 순서

PX4쪽 작업 순서:

1. `boards/px4/sitl/boat.px4board`가 `px4_sitl_boat` target을 생성하는지 확인
2. `make px4_sitl_boat` 빌드 확인
3. `1071_isaac_boat` airframe이 `PX4_SIM_MODEL=isaac_boat`로 선택되는지 확인
4. `rc.boat_defaults`가 `VEHICLE_TYPE=boat`, `MAV_TYPE=11`을 설정하는지 확인
5. `PX4_SIMULATOR=isaac`일 때 `px4-rc.simulator -> px4-rc.mavlinksim` 흐름을 타는지 확인
6. `rc.vehicle_setup -> rc.boat_apps -> boat_control start` 흐름을 확인
7. `pwm_out_sim`, `control_allocator`, `simulator_mavlink`가 함께 동작하는지 확인
8. `PWM_MAIN_FUNC1/2`, `Servo1/Motor1`, `HIL_ACTUATOR_CONTROLS.controls[0/1]` mapping을 steering/signed thrust 순서로 통일
9. HIL_SENSOR/HIL_GPS 수신 후 `sensor_gps`, estimator, commander 상태 확인
10. HIL_ACTUATOR_CONTROLS 송신 및 boat actuator mapping 확인
11. GCS/Offboard UDP 포트가 기존 SITL과 동일한지 확인
12. mission waypoint 기반 boat 주행 검증
13. 실제 firmware에서는 같은 mapping이 `SERVO_OUTPUT_RAW.servo1_raw/servo2_raw`와 debug `pwm_out` CH1/CH2로 확인되는지 검증

Isaac Sim 쪽 작업 순서:

1. Isaac Sim extension skeleton 생성
2. boat USD model과 water world scene 구성
3. config 기반 origin/steering/engine/sensor 설정 로딩
4. TCP MAVLink server를 Isaac extension lifecycle에 연결
5. Isaac physics callback에서 boat state 읽기
6. frame transform과 GPS origin 변환 구현
7. `HIL_SENSOR`, `HIL_GPS`, `HEARTBEAT` 송신 구현
8. `HIL_ACTUATOR_CONTROLS` 수신 및 armed flag 처리
9. steering command와 signed thrust 기반 engine/clutch force 적용
10. sensor fusion, manual control, mission waypoint 순서로 통합 검증

## 9. 미정 및 확인 필요 사항

- Isaac Sim boat model의 steering 축, engine force application point, 좌표계
- `HIL_ACTUATOR_CONTROLS.controls[]`는 `controls[0]=steering`, `controls[1]=signed_thrust`로 확정
- 실제 보트 primary interface는 `SERVO_OUTPUT_RAW.servo1_raw=steering`, `SERVO_OUTPUT_RAW.servo2_raw=signed_thrust`로 확정
- `pwm_out` physical pins는 primary 제어가 아니라 debug/backup interface로 사용
- signed thrust를 clutch forward/neutral/reverse와 engine throttle magnitude로 변환하는 deadband/hysteresis/shift delay 값
- 외부 MAVLink boat driver의 timeout, arming/failsafe 처리 방식
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
- Isaac Sim bridge가 MAVLink TCP server/listener로 실행되고 PX4 `simulator_mavlink`가 client로 접속한다.
- PX4가 Isaac Sim 센서 데이터를 받아 EKF2 position/attitude를 추정한다.
- PX4 actuator output이 Isaac Sim boat 추진기 명령으로 반영된다.
- 실제 firmware에서는 같은 output mapping이 `SERVO_OUTPUT_RAW`와 debug `pwm_out`에서 확인된다.
- 기존 SITL과 동일한 MAVLink network port 정책을 유지한다.
- manual 또는 mission mode에서 boat가 시뮬레이션 환경 안에서 정상 이동한다.
- Isaac Sim extension에서 scene load, bridge start/stop, actuator display, packet counters를 확인할 수 있다.
- Isaac trajectory와 PX4 ulog의 position/heading/actuator 값이 같은 방향과 scale로 일치한다.
