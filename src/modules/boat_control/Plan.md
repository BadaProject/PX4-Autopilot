# PX4 Boat Control Implementation Plan

## 1. Objective

`/Users/jeyong/projects/kaist/bada2_ws/src/control/src/aura_wpt_model_carrot_pid_thrust.cpp`에서 이미 모터 동작이 확인된 boat 제어 알고리즘을 PX4 모듈로 이식한다.

단, waypoint 관리 방식은 기존 ROS2 코드처럼 boat controller 내부에서 waypoint 배열과 mission index를 직접 관리하지 않는다. PX4 표준 구조를 따른다.

- Waypoint, mission progress, reached 판단은 `navigator`가 담당한다.
- `boat_control`은 `rover_ackermann`처럼 navigator가 만든 setpoint를 받아 제어 setpoint와 actuator 출력을 만든다.
- Waypoint 관련 제어 구현은 `AckermannPosControl` 형태를 기준으로 `BoatPosControl`에 배치한다.

## 2. Direction Change From ROS2 Algorithm

### 2.1 Reuse

기존 ROS2 알고리즘에서 다음 제어 핵심만 이식한다.

- 현재 위치와 waypoint line 사이의 carrot/LOS 기하 계산
- LOS error 기반 steering PD 제어
- 속도에 따른 steering gain scheduling
- waypoint cruising speed 기반 thrust feedforward
- steering/thrust slew-rate limit
- disarm/reset 시 내부 상태 초기화

### 2.2 Do Not Reuse

아래 기능은 PX4 navigator가 담당하므로 `boat_control`에 구현하지 않는다.

- `/bada/waypoints` 구독
- lat/lon waypoint 배열 저장
- UTM waypoint 배열 생성
- `carrot_waypoints` vector 생성
- `current_mission_index`, `mission_item_reached`, `total_mission_number` 관리
- AUTO_MISSION 완료 판단
- waypoint 도달 판단
- 시작 waypoint를 임의로 생성하는 legacy 동작

기존 ROS 코드의 `carrot_waypoints[0]` 관련 호환 동작도 구현하지 않는다. PX4에서는 `position_setpoint_triplet.previous/current/next`와 `rover_position_setpoint.start_ned/position_ned`를 사용한다.

## 3. PX4 Waypoint Flow

Boat waypoint 흐름은 `rover_ackermann`과 동일한 계층을 따른다.

```text
navigator
  -> position_setpoint_triplet
  -> BoatAutoMode
  -> rover_position_setpoint
  -> BoatPosControl
  -> rover_speed_setpoint + rover_attitude_setpoint 또는 rover_steering_setpoint
  -> BoatSpeedControl / BoatAttControl
  -> BoatActControl
  -> actuator_motors + actuator_servos
```

핵심 원칙:

- `navigator`가 mission waypoint를 관리한다.
- `BoatAutoMode`는 `position_setpoint_triplet`을 local NED setpoint로 변환하는 얇은 adapter 역할만 한다.
- `BoatPosControl`은 `AckermannPosControl`처럼 `_start_ned`, `_target_waypoint_ned`, `_curr_pos_ned`, `_vehicle_yaw`, `_cruising_speed`, `_acceptance_radius`만 보고 제어 setpoint를 만든다.
- `BoatPosControl` 내부에는 waypoint list나 mission index가 없어야 한다.

## 4. PX4 Manual Control Flow

Boat manual 계열 모드는 `rover_ackermann/AckermannManualMode`와 동일하게 `manual_control_setpoint`를 입력으로 사용한다.

```text
manual_control_setpoint
  -> BoatManualMode
  -> rover_throttle_setpoint / rover_steering_setpoint
  -> BoatActControl
  -> actuator_motors + actuator_servos
```

또는 보조 제어가 켜진 모드에서는 아래 흐름을 사용한다.

```text
manual_control_setpoint
  -> BoatManualMode
  -> rover_speed_setpoint / rover_rate_setpoint / rover_attitude_setpoint / rover_position_setpoint
  -> BoatSpeedControl / BoatAttControl / BoatPosControl
  -> BoatActControl
```

지원할 nav_state:

- `NAVIGATION_STATE_MANUAL`: 조종기 roll을 steering, throttle을 thrust setpoint로 직접 변환한다.
- `NAVIGATION_STATE_ACRO`: throttle은 직접 사용하고 roll은 yaw-rate setpoint로 변환한다.
- `NAVIGATION_STATE_STAB`: roll 입력이 있으면 yaw-rate control, roll 입력이 없고 throttle이 있으면 현재 heading hold를 수행한다.
- `NAVIGATION_STATE_POSCTL`: throttle은 speed setpoint로 변환하고, roll 입력이 있으면 yaw-rate control, roll 입력이 없으면 현재 진행 방향을 유지하는 course-control target waypoint를 생성한다.

MANUAL 모드는 actuator에 가장 가까운 직접 조종 경로로 둔다. POSCTL/STAB/ACRO는 `rover_ackermann`처럼 조종기 입력을 setpoint로 변환한 뒤 하위 controller가 따르도록 한다.

## 5. Reference Algorithm Mapping

### 5.1 Input Mapping

| 기존 ROS2 algorithm data | PX4 source/target |
| --- | --- |
| estimated `x, y` | `vehicle_local_position.x/y` |
| estimated yaw `psi` | `vehicle_attitude` quaternion to yaw |
| surge speed `u` | local velocity를 body frame으로 변환한 `speed_body_x` |
| yaw rate `r` | `vehicle_angular_velocity.xyz[2]` |
| waypoint lat/lon list | 사용하지 않음, `navigator`가 관리 |
| current mission index | 사용하지 않음, `navigator`가 관리 |
| waypoint speed | `position_setpoint_triplet.current.cruising_speed` |
| RC/조종기 입력 | `manual_control_setpoint` |
| armed/mode | `vehicle_control_mode`, `vehicle_status.nav_state` |
| steering PWM | `actuator_servos.control[0]` normalized |
| thrust PWM | `actuator_motors.control[0]` normalized |

### 5.2 Position Control Geometry

기존 ROS 코드의 `carrot_waypoints[k] -> carrot_waypoints[k + 1]` 선분은 PX4에서는 다음으로 대체한다.

```text
start = rover_position_setpoint.start_ned
target = rover_position_setpoint.position_ned
current = vehicle_local_position.xy
```

첫 번째 waypoint처럼 `previous waypoint`가 없어서 `start`가 invalid인 경우에는 carrot 선분 방향을 정의하기 위한 임시 start point를 만든다. 이 임시 start point는 waypoint list나 mission index를 boat controller가 관리한다는 의미가 아니며, 오직 첫 구간의 `start_ned` fallback이다.

```text
if start is invalid and target/current are valid:
    direction = normalize(target - current)
    start = current - BOAT_START_DIST * direction
```

`BOAT_START_DIST`는 사용자 설정 파라미터로 두며 default는 `20.0 m`로 한다. 단, `target - current` 거리가 너무 짧아 direction을 안정적으로 만들 수 없으면 `start = current`를 사용한다. 두 번째 waypoint부터는 navigator가 제공하는 previous waypoint를 사용하므로 임시 start point를 다시 만들지 않는다.

기존 slope 기반 projection은 수직 선분에서 발산할 수 있으므로 PX4 구현에서는 vector projection을 사용한다.

```text
line = target - start
line_len_sq = dot(line, line)
t = dot(current - start, line) / line_len_sq
projection = start + constrain(t, 0, 1) * line
line_heading = atan2(line.y, line.x)
carrot = projection + BOAT_LOOKAHD * normalize(line)
LOS = wrap_pi(atan2(carrot.y - current.y, carrot.x - current.x) - vehicle_yaw)
```

주의:

- PX4 local position은 NED 좌표계이다. `x`는 North, `y`는 East이다.
- `atan2(y, x)` 사용 방향과 yaw 부호가 기존 ROS UTM 좌표계와 동일한지 log로 검증한다.
- `line_len_sq`가 너무 작으면 target bearing만 사용하거나 stop setpoint를 낸다.

### 5.3 Steering Control

기존 steering PD 형태는 유지한다.

```text
steer_input = -Kp * LOS - Kd * wrap_pi(LOS - prev_LOS) * 0.1 / dt
steer_input = constrain(steer_input, -max_steer, max_steer)
steer = slew_limit(steer_input, last_steering, max_steer_diff)
normalized_steering = steer / max_steer
```

`dt`는 고정 `0.333 s`가 아니라 `hrt_absolute_time()` 차이를 사용한다.

### 5.4 Speed And Thrust

속도 setpoint는 PX4 mission item의 cruising speed를 사용한다.

```text
desired_velocity = rover_position_setpoint.cruising_speed
```

기존 ROS 코드의 `speed_ms / 10` 스케일은 waypoint 입력 변환 로직이므로 boat controller에는 넣지 않는다. 필요하면 mission speed 입력 단계 또는 별도 파라미터로 다룬다.

초기 thrust feedforward는 기존 검증값을 유지한다.

```text
proposed_thrust = BOAT_THR_FF * desired_velocity
thrust = slew_limit(proposed_thrust, last_thrust, BOAT_THR_RATE)
remap_thrust = sqrt(thrust / BOAT_THR_SCALE)
remap_thrust = constrain(remap_thrust, 0, BOAT_THR_MAX)
normalized_throttle = remap_thrust / BOAT_THR_MAX
```

기존 ROS 코드는 PWM을 직접 만들지만 PX4에서는 normalized actuator setpoint를 publish하고 PWM 변환은 mixer/control allocation에 맡긴다.

### 5.5 Manual Input Mapping

조종기 입력 mapping은 `AckermannManualMode`와 동일하게 시작한다.

```text
manual_control_setpoint.roll     -> steering 또는 yaw-rate command
manual_control_setpoint.throttle -> throttle 또는 speed command
```

MANUAL:

```text
rover_steering_setpoint.normalized_steering_setpoint = roll
rover_throttle_setpoint.throttle_body_x = throttle
```

ACRO/STAB/POSCTL yaw-rate command:

```text
yaw_rate_setpoint = sign(throttle) * BOAT_Y_RATE_LIM * superexpo(deadzone(roll))
```

POSCTL speed command:

```text
speed_setpoint = interpolate(throttle, -1, 1, -BOAT_SPEED_LIM, BOAT_SPEED_LIM)
```

POSCTL course hold:

```text
if roll is centered and speed_setpoint is nonzero:
    course_direction = current heading unit vector
    start = current position when course hold starts
    target = start + sign(speed_setpoint) * (distance_along_course + BOAT_LOOKAHD) * course_direction
    publish rover_position_setpoint(start, target, speed_setpoint)
```

## 6. Module Structure

`rover_ackermann`과 유사하게 아래 구조로 구현한다.

```text
src/modules/boat_control/
  CMakeLists.txt
  Kconfig
  module.yaml
  BoatControl.cpp
  BoatControl.hpp
  BoatActControl/
    BoatActControl.cpp
    BoatActControl.hpp
    CMakeLists.txt
  BoatAttControl/
    BoatAttControl.cpp
    BoatAttControl.hpp
    CMakeLists.txt
  BoatPosControl/
    BoatPosControl.cpp
    BoatPosControl.hpp
    CMakeLists.txt
  BoatSpeedControl/
    BoatSpeedControl.cpp
    BoatSpeedControl.hpp
    CMakeLists.txt
  BoatDriveModes/
    CMakeLists.txt
    BoatAutoMode/
      BoatAutoMode.cpp
      BoatAutoMode.hpp
      CMakeLists.txt
    BoatManualMode/
      BoatManualMode.cpp
      BoatManualMode.hpp
      CMakeLists.txt
    BoatOffboardMode/
      BoatOffboardMode.cpp
      BoatOffboardMode.hpp
      CMakeLists.txt
```

초기 구현 범위는 AUTO_MISSION과 조종기 기반 MANUAL/POSCTL/STAB/ACRO를 포함한다. Offboard는 `rover_ackermann`의 모듈 구조와 빌드 경로를 맞추는 최소 구현으로 시작한다.

## 7. Controller Responsibility Split

### 7.1 `BoatControl`

- `RoverAckermann`과 같은 메인 워크큐 모듈로 작성한다.
- `vehicle_control_mode`, `vehicle_status`, `parameter_update`를 구독한다.
- Armed + sanity check 통과 시 setpoint 생성과 controller update를 실행한다.
- Disarm 또는 mode 변경 시 모든 controller 상태와 slew-rate 상태를 reset하고 actuator를 정지한다.
- `generateSetpoints()`에서 nav_state별로 아래를 호출한다.
  - AUTO_MISSION/AUTO_LOITER/AUTO_RTL: `BoatAutoMode::autoControl()`
  - MANUAL: `BoatManualMode::manual()`
  - ACRO: `BoatManualMode::acro()`
  - STAB: `BoatManualMode::stab()`
  - POSCTL: `BoatManualMode::position()`

### 7.2 `BoatAutoMode`

- `AckermannAutoMode`와 같은 adapter 형태로 작성한다.
- `position_setpoint_triplet`과 `vehicle_local_position`을 구독한다.
- `MapProjection`을 사용해 previous/current/next global setpoint를 local NED로 변환한다.
- `rover_position_setpoint`를 publish한다.

`BoatAutoMode`가 해야 할 일:

- `rover_position_setpoint.position_ned = current waypoint`
- `rover_position_setpoint.start_ned = previous waypoint`
- previous waypoint가 invalid인 첫 구간에서는 `current position - BOAT_START_DIST * normalize(current waypoint - current position)`을 임시 `start_ned`로 사용
- `rover_position_setpoint.cruising_speed = current.cruising_speed` 또는 boat speed limit
- `rover_position_setpoint.arrival_speed` 설정
- `position_controller_status.acceptance_radius` publish

`BoatAutoMode`가 하지 말아야 할 일:

- waypoint list 저장
- mission index 저장
- waypoint reached 판단
- 임의 carrot waypoint 배열 생성
- 첫 구간 이후에도 임시 start point를 계속 재생성하는 동작

### 7.3 `BoatManualMode`

`AckermannManualMode`와 같은 조종기 입력 adapter로 작성한다.

구독:

- `manual_control_setpoint`
- `vehicle_attitude`
- `vehicle_local_position`

publish:

- `rover_throttle_setpoint`
- `rover_steering_setpoint`
- `rover_rate_setpoint`
- `rover_attitude_setpoint`
- `rover_speed_setpoint`
- `rover_position_setpoint`

모드별 동작:

- `manual()`: roll/throttle을 normalized steering/throttle로 직접 publish한다.
- `acro()`: throttle은 직접 publish하고 roll은 yaw-rate setpoint로 publish한다.
- `stab()`: roll 입력이 있거나 throttle이 0이면 yaw-rate setpoint를 publish하고 heading setpoint는 invalid로 둔다. roll 입력이 없고 throttle이 있으면 현재 yaw를 heading setpoint로 유지한다.
- `position()`: throttle을 speed setpoint로 변환한다. roll 입력이 있거나 speed가 0이면 speed/yaw-rate setpoint를 publish하고 position setpoint는 invalid로 둔다. roll 입력이 없고 speed가 있으면 현재 heading 방향으로 가상 target waypoint를 만들어 `rover_position_setpoint`를 publish한다.

reset 시 `_stab_yaw_setpoint`, `_pos_ctl_course_direction`, `_pos_ctl_start_position_ned`, `_curr_pos_ned`를 invalid로 초기화한다.

### 7.4 `BoatPosControl`

`AckermannPosControl` 구현 방식을 기준으로 한다.

입력:

- `rover_position_setpoint`
- `position_controller_status`
- `vehicle_local_position`
- `vehicle_attitude`

출력:

- `rover_speed_setpoint`
- `rover_attitude_setpoint` 또는 `rover_steering_setpoint`

초기 구현:

- `BoatPosControl`이 carrot/LOS 계산 후 `rover_attitude_setpoint.yaw_setpoint`와 `rover_speed_setpoint.speed_body_x`를 publish한다.
- `BoatAttControl`이 yaw error를 steering으로 변환한다.
- `BoatPosControl`은 direct `rover_steering_setpoint`를 publish하지 않는다.

정지 조건:

- target waypoint가 finite가 아니면 출력하지 않는다.
- target까지 거리가 acceptance radius 이하이고 arrival speed가 0이면 speed setpoint를 0으로 publish한다.
- stopped 상태에서 local position reset counter가 바뀌면 target을 current position으로 재설정한다. 이 동작은 `AckermannPosControl`을 따른다.

### 7.5 `BoatSpeedControl`

- `rover_speed_setpoint`를 받아 throttle setpoint를 만든다.
- 초기에는 기존 `BOAT_THR_FF`, `BOAT_THR_SCALE`, `BOAT_THR_RATE` 기반 feedforward를 사용한다.
- 추후 `Kup/Kui/Kud` 기반 speed feedback을 옵션으로 추가한다.
- measured speed는 `vehicle_local_position` velocity를 body frame으로 변환해 사용한다.
- MANUAL 모드에서 `rover_throttle_setpoint`가 직접 publish되는 경우에는 speed controller가 개입하지 않는다.

### 7.6 `BoatAttControl`

- `rover_attitude_setpoint.yaw_setpoint`와 현재 yaw를 받아 steering setpoint를 만든다.
- 기존 ROS steering PD를 이 계층에 둘 경우 `LOS` 대신 `wrap_pi(yaw_setpoint - vehicle_yaw)`를 error로 사용한다.
- STAB heading hold와 POSCTL course hold에서 생성된 yaw/position setpoint를 처리한다.
- ACRO/POSCTL roll 입력에서 생성된 yaw-rate setpoint도 `BoatAttControl`에서 steering setpoint로 직접 변환한다.

### 7.7 `BoatActControl`

- `rover_throttle_setpoint`와 `rover_steering_setpoint`를 받아 actuator topic으로 publish한다.
- steering은 normalized `[-1, 1]`, throttle은 normalized `[0, 1]`로 제한한다.
- PWM 변환은 PX4 mixer/control allocation에 맡긴다.
- disarm/reset 시 motor와 steering 모두 0으로 publish한다.

## 8. Parameters

`module.yaml`에는 boat 전용 파라미터를 추가한다. 이름은 `BOAT_` prefix를 사용한다.

필수 파라미터 초안:

- `BOAT_ACC_RAD`: waypoint acceptance radius, default `8.0 m`
- `BOAT_SPEED_LIM`: boat speed limit, default mission speed fallback
- `BOAT_LOOKAHD`: carrot lookahead distance, default `50.0 m`
- `BOAT_START_DIST`: first waypoint temporary start distance, default `20.0 m`
- `BOAT_STR_P`: LOS/yaw steering P gain, default `500.0`
- `BOAT_STR_D`: LOS/yaw steering D gain, default `1000.0`
- `BOAT_STR_MAX`: steering command limit, default `150.0`
- `BOAT_STR_RATE`: steering command slew limit, default `24.5`
- `BOAT_Y_RATE_LIM`: manual yaw-rate limit, default tuned by bench test
- `BOAT_Y_STICK_DZ`: manual roll stick deadzone, default `0.05`
- `BOAT_Y_EXPO`: manual yaw input expo, default `0.0`
- `BOAT_Y_SUPEXPO`: manual yaw input superexpo, default `0.0`
- `BOAT_THR_MAX`: remapped thrust maximum, default `48.0`
- `BOAT_THR_RATE`: thrust slew limit, default `0.01`
- `BOAT_THR_FF`: velocity to thrust feedforward gain, default `0.21`
- `BOAT_THR_SCALE`: thrust remap denominator, default `0.00058466`
- `BOAT_DEBUG`: publish/log debug data, default `0`

Waypoint list, mission index, legacy start waypoint 관련 파라미터는 만들지 않는다.

## 9. Build Integration

1. `src/modules/boat_control/CMakeLists.txt` 작성
2. `src/modules/boat_control/Kconfig` 작성
3. `src/modules/boat_control/module.yaml` 작성
4. 상위 빌드 설정에서 `boat_control` 모듈이 포함되는지 확인
5. `px4_add_module(MODULE modules__boat_control MAIN boat_control ...)` 형태로 등록
6. 초기 `DEPENDS` 후보:

```text
px4_work_queue
rover_control
```

`pure_pursuit`는 `AckermannPosControl`의 기존 pure pursuit를 재사용하기로 결정할 때만 추가한다. 초기 구현은 기존 boat carrot/LOS 수식을 `BoatPosControl`에 직접 구현한다.

## 10. Implementation Order

1. `RoverAckermann` 구조를 참고하여 `BoatControl` 메인 모듈 skeleton을 만든다.
2. `BoatControl::generateSetpoints()`에 AUTO/OFFBOARD/MANUAL/ACRO/STAB/POSCTL nav_state 분기를 만든다.
3. `AckermannManualMode`를 참고하여 `BoatManualMode`를 작성하고 조종기 입력을 setpoint로 변환한다.
4. `AckermannAutoMode`를 참고하여 `BoatAutoMode`를 작성하고, `position_setpoint_triplet -> rover_position_setpoint` 변환만 구현한다.
5. 첫 구간에서 previous waypoint가 invalid이면 `BOAT_START_DIST` 기반 임시 `start_ned` fallback을 생성한다.
6. `AckermannPosControl`을 참고하여 `BoatPosControl` 구독/상태/update 구조를 만든다.
7. `BoatPosControl`에 `start_ned -> target_waypoint_ned` 선분 기반 carrot/LOS 계산을 구현한다.
8. `BoatPosControl`에서 `rover_speed_setpoint`와 yaw 또는 steering setpoint를 publish한다.
9. `BoatSpeedControl`에 기존 thrust feedforward, remap, slew limit을 구현한다.
10. `BoatAttControl`에 기존 steering PD와 yaw-rate to steering 변환을 구현한다.
11. `BoatRateControl`은 초기 구현 범위에서 제외한다.
12. `BoatActControl`에서 normalized steering/throttle을 actuator topic으로 publish한다.
13. `module.yaml` 파라미터를 추가하고 controller마다 `ModuleParams`로 연결한다.
14. disarm, mode change, local position reset, mission setpoint update, manual stick input 조건에서 reset/stop 동작을 검증한다.

## 11. Verification Plan

### 11.1 Unit-level check

- `start_ned`, `target_waypoint_ned`, `curr_pos_ned`로 carrot point와 LOS가 finite인지 확인
- 수평/수직/대각 waypoint 선분에서 vector projection이 정상인지 확인
- `start == target` 또는 매우 짧은 선분에서 안전하게 stop 또는 target bearing으로 fallback하는지 확인
- yaw wrap boundary `-pi/pi` 근처에서 steering derivative가 튀지 않는지 확인
- manual roll/throttle 입력이 각 모드에서 기대하는 setpoint topic으로 변환되는지 확인
- POSCTL에서 roll centered + throttle nonzero일 때 course-control `rover_position_setpoint`가 finite인지 확인
- POSCTL에서 roll input이 들어오면 position setpoint가 invalid 처리되는지 확인
- steering normalized 출력이 항상 `[-1, 1]`인지 확인
- throttle normalized 출력이 항상 `[0, 1]`인지 확인

### 11.2 PX4 runtime check

- `boat_control start`로 모듈이 시작되는지 확인
- disarmed 상태에서 actuator 출력이 0인지 확인
- AUTO_MISSION 외 nav_state에서 mission setpoint 기반 출력이 생성되지 않는지 확인
- navigator가 `position_setpoint_triplet`을 갱신하면 `rover_position_setpoint`와 `BoatPosControl` target이 갱신되는지 확인
- MANUAL에서 roll/throttle stick이 actuator output으로 반영되는지 확인
- ACRO에서 roll stick이 yaw-rate setpoint로 반영되는지 확인
- STAB에서 roll stick release 시 heading hold가 유지되는지 확인
- POSCTL에서 throttle stick이 speed setpoint로 반영되고 roll centered 상태에서 course hold target이 생성되는지 확인
- mission 완료 또는 disarm 시 motor 정지 명령이 나가는지 확인

### 11.3 Field/bench comparison

- 같은 `start/current/target` 조건에서 기존 ROS 노드의 `LOS`, `steer`, `remap_thrust`, `desired_velocity`와 PX4 로그를 비교한다.
- 차이가 나면 먼저 좌표계와 yaw 부호를 확인한다.
- actuator 방향이 반대이면 mixer 또는 actuator parameter에서 보정하고 알고리즘 부호는 최대한 유지한다.
- `BOAT_LOOKAHD`, `BOAT_STR_P`, `BOAT_STR_D`, `BOAT_THR_FF`, `BOAT_THR_RATE` 순서로 튜닝한다.
- 조종기 bench test는 propeller/추진기 안전 상태에서 MANUAL, STAB, POSCTL 순서로 진행한다.

## 12. Open Decisions

- `BoatAttControl`이 yaw setpoint와 yaw-rate setpoint를 모두 steering으로 변환한다. 필요 시 추후 `BoatRateControl`로 분리한다.
- PX4 airframe/mixer에서 boat servo와 motor가 어떤 actuator index를 사용할지 확인해야 한다.
- 기존 ROS 코드의 yaw 변환 `psi = -msg->data[2] + pi/2`가 PX4 NED yaw와 어떤 관계인지 log로 검증해야 한다.
