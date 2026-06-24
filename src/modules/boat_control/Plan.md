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

결정된 actuator contract:

```text
PX4 internal:
  actuator_servos.control[0] = steering [-1, 1]
  actuator_motors.control[0] = signed_thrust [-1, 1]

External simulator / output driver order:
  output/control[0] = steering
  output/control[1] = signed_thrust

Drivetrain driver:
  sign(signed_thrust) -> clutch forward / neutral / reverse
  abs(signed_thrust)  -> engine throttle magnitude
```

`clutch`는 `boat_control`이나 control allocation이 직접 만드는 독립 제어축이 아니다. SITL에서는 Isaac bridge가, 실제 Pixhawk firmware에서는 외부 MAVLink boat driver가 `SERVO_OUTPUT_RAW`를 받아 `signed_thrust`를 clutch state와 throttle magnitude로 변환한다.

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
| steering command | `actuator_servos.control[0]` normalized `[-1, 1]` |
| signed thrust command | `actuator_motors.control[0]` normalized `[-1, 1]` |

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
signed_thrust = remap_thrust / BOAT_THR_MAX
```

기존 ROS 코드는 PWM을 직접 만들지만 PX4에서는 normalized actuator setpoint를 publish하고 PWM 변환은 mixer/control allocation/output driver에 맡긴다. 엔진 throttle과 clutch state는 `signed_thrust`를 받은 bridge/driver에서 결정한다.

### 5.5 Manual Input Mapping

조종기 입력 mapping은 `AckermannManualMode`와 동일하게 시작한다.

```text
manual_control_setpoint.roll     -> steering 또는 yaw-rate command
manual_control_setpoint.throttle -> signed thrust 또는 speed command
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

현재 code path의 `BoatManualMode`, `BoatSpeedControl`, `BoatActControl`은 signed thrust contract에 맞게 수정되었다. manual direct thrust와 actuator motor output은 `[-1, 1]`, POSCTL speed setpoint는 `[-BOAT_SPEED_LIM, BOAT_SPEED_LIM]` 범위를 사용한다.

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

- `manual()`: roll/throttle을 normalized steering/signed thrust로 직접 publish한다.
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

- `rover_speed_setpoint`를 받아 signed thrust setpoint를 만든다.
- 초기에는 기존 `BOAT_THR_FF`, `BOAT_THR_SCALE`, `BOAT_THR_RATE` 기반 feedforward를 사용한다.
- 현재 구현은 `BOAT_SPD_EN`으로 feedforward-only와 measured-speed feedback PID 경로를 선택한다.
- 현재 구현의 `BOAT_THR_RATE`는 update 1회당 raw thrust 변화량 제한이다. `dt` 기반 speed ramp는 `BOAT_SPD_ACC_UP/DN`, 최종 engine soft-start는 `BOAT_ENG_RAMP_UP/DN`으로 처리한다.
- `BOAT_SPD_P/I/D` 기반 speed feedback은 옵션으로 구현되어 있으며 기본값은 disabled이다.
- measured speed는 `vehicle_local_position` velocity를 body frame으로 변환해 사용한다.
- MANUAL 모드에서 `rover_throttle_setpoint`가 직접 publish되는 경우에는 speed controller가 개입하지 않는다.

#### 7.5.1 Closed-loop speed control implementation plan

폐루프 속도 제어는 기존 feedforward 경로를 유지하면서 measured speed feedback을 더하는 형태로 구현한다.

입력:

- `rover_speed_setpoint.speed_body_x`
- `vehicle_local_position.vx/vy/vz`
- `vehicle_attitude`

상태:

- body frame surge speed `measured_speed_body_x`
- speed error integral
- previous speed error
- last update timestamp
- last thrust command

제어식:

```text
speed_error = speed_setpoint - measured_speed_body_x
feedforward_thrust = BOAT_THR_FF * speed_setpoint
feedback_thrust = BOAT_SPD_P * speed_error
                + BOAT_SPD_I * speed_error_integral
                + BOAT_SPD_D * (speed_error - prev_speed_error) / dt
raw_thrust = feedforward_thrust + feedback_thrust
thrust = slew_limit(raw_thrust, last_thrust, BOAT_THR_RATE)
signed_thrust = remap_and_constrain(thrust, -1, 1)
```

구현 세부:

- `dt`는 `hrt_absolute_time()` 차이로 계산하고, 첫 update 또는 비정상 `dt`에서는 D항을 0으로 둔다.
- `speed_setpoint == 0`이면 integral과 derivative 상태를 reset하고 signed thrust 0을 publish한다.
- integral은 `BOAT_SPD_IMAX`로 제한한다.
- signed thrust가 `[-1, 1]` 제한에 걸리고 error가 같은 방향으로 integral을 더 키우는 경우 anti-windup을 적용한다.
- 후진은 signed thrust 음수로 표현한다. 실제 clutch reverse 전환은 외부 MAVLink boat driver가 deadband/hysteresis/shift delay를 적용해 처리한다.
- `rover_speed_status.pid_throttle_body_x_integral`에는 실제 integral 기여분 또는 integral 상태를 publish한다.
- `BOAT_SPD_EN`이 0이면 기존 feedforward-only 동작을 유지한다.

구현 순서:

1. `module.yaml`에 `BOAT_SPD_EN`, `BOAT_SPD_P`, `BOAT_SPD_I`, `BOAT_SPD_D`, `BOAT_SPD_IMAX`를 추가한다.
2. `BoatSpeedControl.hpp`에 PID 상태와 새 파라미터를 추가한다.
3. `BoatSpeedControl::reset()`에서 PID 상태와 timestamp를 초기화한다.
4. `BoatSpeedControl::updateSpeedControl()`에서 feedforward-only와 closed-loop 경로를 `BOAT_SPD_EN`으로 분기한다.
5. measured speed invalid 또는 local position velocity invalid 상태에서는 feedback을 사용하지 않고 feedforward-only로 degrade한다.
6. `rover_speed_status`에 measured speed, adjusted setpoint, integral 상태가 항상 publish되는지 확인한다.

#### 7.5.2 Diesel engine soft-start and speed ramp plan

Boat 추진기는 디젤 엔진을 사용하므로 정지 또는 저속 상태에서 speed/throttle 명령이 step으로 들어가면 안 된다. 특히 AUTO/POSCTL/OFFBOARD에서 mission speed가 갑자기 들어오는 경우에도 실제 controller 내부 목표 속도와 throttle 명령은 천천히 상승해야 한다.

현재 구현 상태:

- `BoatSpeedControl`은 `_last_thrust`와 `BOAT_THR_RATE`로 thrust command 변화량을 제한한다.
- 하지만 `BOAT_THR_RATE`는 초당 변화율이 아니라 controller update 1회당 변화량으로 적용된다.
- `rover_speed_setpoint.speed_body_x`는 그대로 `speed_setpoint`가 되며, 별도의 speed setpoint ramp가 없다.
- MANUAL 모드는 `rover_throttle_setpoint`를 직접 publish하므로 `BoatSpeedControl`의 thrust slew limit을 거치지 않는다.

구현 방향은 두 단계 제한을 함께 둔다.

1. Speed setpoint ramp:

```text
requested_speed_setpoint = constrain(rover_speed_setpoint.speed_body_x,
                                      -BOAT_SPEED_LIM,
                                      BOAT_SPEED_LIM)

requested_direction = sign(requested_speed_setpoint)
requested_speed_abs = abs(requested_speed_setpoint)

if BOAT_RAMP_SPD > 0 and startup_ramp_active:
    target_speed_abs = min(requested_speed_abs, BOAT_RAMP_SPD)
else:
    target_speed_abs = requested_speed_abs

target_speed_setpoint = requested_direction * target_speed_abs

speed_rate_limit = target_speed_setpoint > adjusted_speed_setpoint ? BOAT_SPD_ACC_UP : BOAT_SPD_ACC_DN
adjusted_speed_setpoint = slew_limit_by_dt(
    target_speed_setpoint,
    adjusted_speed_setpoint,
    speed_rate_limit * dt)
```

2. Signed thrust / engine ramp:

```text
raw_signed_thrust = speed_controller_output
thrust_rate_limit = abs(raw_signed_thrust) > abs(last_signed_thrust) ? BOAT_ENG_RAMP_UP : BOAT_ENG_RAMP_DN
limited_signed_thrust = slew_limit_by_dt(raw_signed_thrust, last_signed_thrust, thrust_rate_limit * dt)
```

설계 원칙:

- `BOAT_SPD_ACC_UP`은 초기 속도 상승을 제한하는 핵심 파라미터이며 단위는 `m/s^2`로 둔다.
- `BOAT_RAMP_SPD`는 diesel startup ramp target speed이다. `BOAT_RAMP_TARGET_SPEED` 개념의 PX4 파라미터명이며, PX4 파라미터 이름 길이 제한을 고려해 짧은 이름을 사용한다.
- `BOAT_RAMP_SPD > 0`이면 정지 또는 저속에서 출발할 때 controller는 mission/POSCTL/OFFBOARD 요청 속도가 더 크더라도 먼저 `BOAT_RAMP_SPD`까지만 천천히 올린다.
- 후진 요청은 음수 speed setpoint와 음수 signed thrust로 표현한다. startup ramp는 속도 크기에는 적용하지만 부호는 유지한다.
- `adjusted_speed_setpoint` 또는 measured speed가 `BOAT_RAMP_SPD` 근처에 도달하고 `BOAT_RAMP_HOLD` 시간이 지나면 `startup_ramp_active`를 해제하고 원래 요청 속도까지 계속 ramp한다.
- 요청 속도가 `BOAT_RAMP_SPD`보다 낮으면 별도 startup phase 없이 요청 속도까지만 ramp한다.
- `BOAT_RAMP_SPD <= 0`이면 특정 속도까지 먼저 올리는 startup target 기능을 비활성화하고 일반 speed ramp만 사용한다.
- `BOAT_ENG_RAMP_UP`은 normalized signed thrust magnitude의 초당 증가량으로 둔다.
- 감속은 안전 정지 요구가 있으므로 상승보다 빠르게 허용한다. `BOAT_SPD_ACC_DN`, `BOAT_ENG_RAMP_DN`을 별도로 둔다.
- `adjusted_speed_setpoint`를 feedforward와 feedback PID 모두의 목표값으로 사용한다.
- PID integral은 `adjusted_speed_setpoint` 기준 error로 적분한다. 원본 target speed 기준으로 적분하면 ramp 중 windup이 발생할 수 있다.
- 정지 상태에서 speed setpoint로 처음 전환될 때 `_adjusted_speed_setpoint`, `_last_signed_thrust`, `_last_thrust`, ramp timestamp를 0에서 시작한다.
- disarm, mode change, speed setpoint 0, failsafe stop에서는 ramp 상태와 PID integral을 reset한다.
- measured speed가 이미 adjusted setpoint보다 높은 경우에는 ramp-up을 기다리지 말고 feedback이 throttle을 줄일 수 있게 한다.
- `dt`가 비정상적으로 크면 ramp jump를 막기 위해 `dt`를 `BOAT_CTRL_DT_MAX` 또는 내부 상한으로 제한한다.

MANUAL 모드 처리:

- 디젤 엔진 보호를 위해 최종 signed thrust magnitude ramp는 `BoatActControl`에도 두는 방안을 우선 검토한다.
- 이렇게 하면 MANUAL direct signed thrust, ACRO direct signed thrust, speed controller 출력이 모두 같은 engine ramp limiter를 통과한다.
- 단, emergency stop/disarm/failsafe에서는 ramp down을 기다리지 않고 즉시 signed thrust 0을 publish한다.

구현 순서:

1. `BoatSpeedControl`에 `_adjusted_speed_setpoint`, `_timestamp`, `_last_thrust` 기반 speed/thrust ramp를 추가한다.
2. `BoatSpeedControl`에 `_startup_ramp_active`, `_startup_ramp_reached_time` 상태를 추가한다.
3. `BOAT_RAMP_SPD`가 설정되어 있으면 정지/저속 출발 시 `abs(requested_speed_setpoint)` 대신 `min(abs(requested_speed_setpoint), BOAT_RAMP_SPD)`를 1차 목표 크기로 사용하고 원래 부호를 유지한다.
4. `BOAT_RAMP_SPD` 도달 후 `BOAT_RAMP_HOLD`가 지나면 startup ramp를 종료하고 원래 요청 속도까지 ramp한다.
5. `BoatSpeedControl`의 feedforward/PID 입력을 raw speed setpoint가 아니라 ramp된 `adjusted_speed_setpoint`로 변경한다.
6. `BoatActControl`에 최종 signed thrust 증가율 limiter를 추가해 MANUAL direct thrust에도 diesel soft-start를 적용한다.
7. disarm/mode change/stop/failsafe 경로에서 speed ramp와 engine ramp 상태가 reset되는지 확인한다.
8. `rover_speed_status.adjusted_speed_body_x_setpoint`에 ramp 이후 목표 속도를 publish해 로그에서 ramp 동작을 확인할 수 있게 한다.

### 7.6 `BoatAttControl`

- `rover_attitude_setpoint.yaw_setpoint`와 현재 yaw를 받아 steering setpoint를 만든다.
- 기존 ROS steering PD를 이 계층에 둘 경우 `LOS` 대신 `wrap_pi(yaw_setpoint - vehicle_yaw)`를 error로 사용한다.
- STAB heading hold와 POSCTL course hold에서 생성된 yaw/position setpoint를 처리한다.
- ACRO/POSCTL roll 입력에서 생성된 yaw-rate setpoint도 `BoatAttControl`에서 steering setpoint로 직접 변환한다.

### 7.7 `BoatActControl`

- `rover_throttle_setpoint`와 `rover_steering_setpoint`를 받아 actuator topic으로 publish한다.
- steering은 normalized `[-1, 1]`, signed thrust는 normalized `[-1, 1]`로 제한한다.
- `BoatActControl`은 `actuator_motors.control[0]`를 `[-1, 1]` signed thrust로 publish한다.
- `BoatManualMode`와 `BoatSpeedControl`의 signed thrust/signed speed path도 함께 수정되어 `BoatActControl`만 단독으로 signed contract를 맞추는 상태가 아니다.
- PWM 변환은 PX4 mixer/control allocation에 맡긴다.
- disarm/reset 시 motor와 steering 모두 0으로 publish한다.
- clutch는 여기서 직접 publish하지 않는다. 실제 clutch forward/neutral/reverse는 signed thrust를 받은 외부 MAVLink boat driver 또는 SITL bridge가 결정한다.

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
- `BOAT_THR_MAX`: remapped thrust maximum before signed thrust normalization, default `48.0`
- `BOAT_THR_RATE`: thrust slew limit, default `0.01`
- `BOAT_THR_FF`: velocity to thrust feedforward gain, default `0.21`
- `BOAT_THR_SCALE`: thrust remap denominator, default `0.00058466`
- `BOAT_SPD_EN`: closed-loop speed feedback enable, default `0`
- `BOAT_SPD_P`: speed feedback P gain, default `0.0`
- `BOAT_SPD_I`: speed feedback I gain, default `0.0`
- `BOAT_SPD_D`: speed feedback D gain, default `0.0`
- `BOAT_SPD_IMAX`: speed feedback integral limit, default `0.2`
- `BOAT_RAMP_SPD`: diesel startup ramp target speed, default `0.0 m/s` disabled
- `BOAT_RAMP_HOLD`: hold time after reaching startup ramp target speed, default `2.0 s`
- `BOAT_SPD_ACC_UP`: speed setpoint ramp-up limit for diesel soft-start, default `0.2 m/s^2`
- `BOAT_SPD_ACC_DN`: speed setpoint ramp-down limit, default `0.5 m/s^2`
- `BOAT_ENG_RAMP_UP`: normalized signed thrust magnitude ramp-up limit, default `0.05 / s`
- `BOAT_ENG_RAMP_DN`: normalized signed thrust magnitude ramp-down limit, default `0.20 / s`
- `BOAT_DEBUG`: publish/log debug data, default `0`

### 8.1 Parameter summary table

이 표는 현재 `module.yaml`에 등록된 파라미터와 구현 상태를 함께 정리한다.

| Parameter | Full name / meaning | 용도 | 구현 상태 |
| --- | --- | --- | --- |
| `BOAT_ACC_RAD` | Boat acceptance radius | waypoint acceptance radius. AUTO에서 `position_controller_status.acceptance_radius`로 publish하고, position controller 정지 조건에 사용한다. | 구현됨. `module.yaml` 등록, `BoatAutoMode`, `BoatPosControl`에서 사용. |
| `BOAT_SPEED_LIM` | Boat speed limit | mission cruising speed fallback과 manual POSCTL throttle-to-speed 변환의 상한. speed controller 입력 제한에도 사용한다. | 구현됨. `module.yaml` 등록, `BoatAutoMode`, `BoatManualMode`, `BoatPosControl`, `BoatSpeedControl`에서 사용. |
| `BOAT_LOOKAHD` | Boat carrot lookahead distance | carrot/LOS target point를 선분 projection 앞쪽에 생성하는 거리. POSCTL course hold target 생성에도 사용한다. | 구현됨. `module.yaml` 등록, `BoatPosControl`, `BoatManualMode`에서 사용. |
| `BOAT_START_DIST` | Boat first waypoint temporary start distance | navigator가 previous waypoint를 제공하지 않는 첫 mission 구간에서 임시 `start_ned`를 current position 뒤쪽에 생성하는 거리. | 구현됨. `module.yaml` 등록, `BoatAutoMode`에서 사용. |
| `BOAT_STR_P` | Boat steering proportional gain | yaw/LOS error를 steering command로 바꾸는 P gain. | 구현됨. `module.yaml` 등록, `BoatAttControl`에서 사용. |
| `BOAT_STR_D` | Boat steering derivative gain | yaw/LOS error derivative에 대한 D gain. | 구현됨. `module.yaml` 등록, `BoatAttControl`에서 사용. |
| `BOAT_STR_MAX` | Boat steering command maximum | steering command saturation 및 normalized steering 변환 기준값. | 구현됨. `module.yaml` 등록, `BoatAttControl`에서 사용. |
| `BOAT_STR_RATE` | Boat steering command slew limit | steering command 변화량 제한. 현재는 update 1회당 변화량 제한이다. | 구현됨. `module.yaml` 등록, `BoatAttControl`에서 사용. |
| `BOAT_Y_RATE_LIM` | Boat manual yaw-rate limit | ACRO/STAB/POSCTL manual roll 입력을 yaw-rate setpoint 또는 normalized steering으로 변환할 때의 최대 yaw rate. | 구현됨. `module.yaml` 등록, `BoatManualMode`, `BoatAttControl`에서 사용. |
| `BOAT_Y_STICK_DZ` | Boat yaw stick deadzone | manual roll stick deadzone. | 구현됨. `module.yaml` 등록, `BoatManualMode`에서 사용. |
| `BOAT_Y_EXPO` | Boat yaw stick expo | manual yaw input expo shaping. | 구현됨. `module.yaml` 등록, `BoatManualMode`에서 사용. |
| `BOAT_Y_SUPEXPO` | Boat yaw stick superexpo | manual yaw input superexpo shaping. | 구현됨. `module.yaml` 등록, `BoatManualMode`에서 사용. |
| `BOAT_THR_MAX` | Boat remapped thrust maximum | remapped thrust를 normalized signed thrust `[-1, 1]`로 변환하는 상한. | 구현됨. `module.yaml` 등록, `BoatSpeedControl`에서 signed remap에 사용. |
| `BOAT_THR_RATE` | Boat thrust slew limit | thrust command 변화량 제한. 현재는 update 1회당 thrust delta 제한이며 diesel soft-start용 초당 ramp는 아니다. | 구현됨. `module.yaml` 등록, `BoatSpeedControl`에서 raw thrust 변화 제한에 사용. 초당 diesel ramp는 `BOAT_SPD_ACC_*`, `BOAT_ENG_RAMP_*`가 담당. |
| `BOAT_THR_FF` | Boat velocity-to-thrust feedforward gain | speed setpoint를 thrust feedforward로 변환하는 gain. | 구현됨. `module.yaml` 등록, `BoatSpeedControl`에서 사용. |
| `BOAT_THR_SCALE` | Boat thrust remap denominator | thrust를 `sqrt(thrust / scale)` 형태로 remap하는 denominator. | 구현됨. `module.yaml` 등록, `BoatSpeedControl`에서 사용. |
| `BOAT_DEBUG` | Boat debug logging enable | boat control debug logging 또는 extra status 출력을 켜기 위한 flag. | 등록됨, 미사용. `module.yaml`에는 있으나 현재 code path에서는 사용하지 않음. |
| `BOAT_SPD_EN` | Boat speed feedback enable | feedforward-only와 measured-speed feedback PID 경로를 선택한다. | 구현됨. `module.yaml` 등록, `BoatSpeedControl`에서 feedforward-only/PID 분기. |
| `BOAT_SPD_P` | Boat speed feedback P gain | `adjusted_speed_setpoint - measured_speed_body_x` error의 P feedback. | 구현됨. `BoatSpeedControl` closed-loop speed PID에 사용. |
| `BOAT_SPD_I` | Boat speed feedback I gain | speed error integral feedback. | 구현됨. integral clamp와 saturation anti-windup 포함. |
| `BOAT_SPD_D` | Boat speed feedback D gain | speed error derivative feedback. | 구현됨. `dt` 기반 derivative, first update D항 0 처리. |
| `BOAT_SPD_IMAX` | Boat speed integral maximum | speed PID integral clamp. | 구현됨. speed PID integral 제한에 사용. |
| `BOAT_RAMP_SPD` | Boat diesel startup ramp target speed | 정지/저속 출발 시 요청 속도가 더 크더라도 먼저 이 속도까지 천천히 상승시키는 startup target speed. `BOAT_RAMP_TARGET_SPEED` 개념의 PX4 길이 제한 대응 이름. | 구현됨. `BoatSpeedControl` startup ramp state에 사용. |
| `BOAT_RAMP_HOLD` | Boat diesel startup ramp hold time | `BOAT_RAMP_SPD` 도달 후 원래 요청 속도로 넘어가기 전에 유지할 시간. | 구현됨. `_startup_ramp_reached_time` 기반 hold logic에 사용. |
| `BOAT_SPD_ACC_UP` | Boat speed setpoint ramp-up acceleration | `adjusted_speed_setpoint`가 목표 속도까지 증가할 때의 최대 기울기, 단위 `m/s^2`. | 구현됨. signed speed setpoint magnitude ramp-up에 사용. |
| `BOAT_SPD_ACC_DN` | Boat speed setpoint ramp-down acceleration | speed setpoint 감소 시 최대 기울기. 상승보다 빠른 감속을 허용한다. | 구현됨. signed speed setpoint magnitude ramp-down에 사용. |
| `BOAT_ENG_RAMP_UP` | Boat engine thrust ramp-up rate | 최종 normalized signed thrust magnitude의 초당 증가율 제한. MANUAL direct thrust도 보호하기 위해 actuator 직전 적용을 검토한다. | 구현됨. `BoatActControl` final signed thrust ramp limiter에 사용. |
| `BOAT_ENG_RAMP_DN` | Boat engine thrust ramp-down rate | 최종 normalized signed thrust magnitude의 초당 감소율 제한. disarm/failsafe stop은 우회해 즉시 0으로 내려야 한다. | 구현됨. `BoatActControl` final signed thrust ramp limiter에 사용. |

`BOAT_CTRL_DT_MAX`는 현재 정식 파라미터가 아니라 ramp jump 방지를 위한 내부 `dt` 상한 또는 추후 파라미터 후보로만 언급되어 있다.

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
10. `BoatSpeedControl`에 `BOAT_SPD_EN`으로 선택 가능한 measured-speed feedback PID를 추가한다.
11. `BoatSpeedControl`에 `BOAT_RAMP_SPD`까지 먼저 천천히 올리는 diesel startup target speed ramp를 추가한다.
12. `BoatActControl`에 final signed thrust ramp limiter를 추가해 MANUAL direct thrust도 보호한다.
13. `BoatAttControl`에 기존 steering PD와 yaw-rate to steering 변환을 구현한다.
14. `BoatRateControl`은 초기 구현 범위에서 제외한다.
15. `BoatActControl`에서 normalized steering/signed thrust를 actuator topic으로 publish한다.
16. `module.yaml` 파라미터를 추가하고 controller마다 `ModuleParams`로 연결한다.
17. disarm, mode change, local position reset, mission setpoint update, manual stick input 조건에서 reset/stop 동작을 검증한다.

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
- signed thrust normalized 출력이 항상 `[-1, 1]`인지 확인
- `BOAT_SPD_EN=0`에서 기존 feedforward-only throttle 출력이 유지되는지 확인
- `BOAT_SPD_EN=1`에서 speed error가 양수이면 throttle이 증가하고 음수이면 감소하는지 확인
- speed setpoint가 0이 되거나 reset이 호출되면 speed PID integral이 초기화되는지 확인
- signed thrust saturation 중 integral windup이 제한되는지 확인
- 정지 상태에서 speed setpoint가 step으로 들어와도 `adjusted_speed_body_x_setpoint`가 `BOAT_SPD_ACC_UP` 이하의 기울기로 증가하는지 확인
- 요청 speed setpoint가 `BOAT_RAMP_SPD`보다 크면 `adjusted_speed_body_x_setpoint`가 먼저 `BOAT_RAMP_SPD`까지 ramp되고 `BOAT_RAMP_HOLD` 이후 원래 요청 속도까지 계속 ramp되는지 확인
- 요청 speed setpoint가 `BOAT_RAMP_SPD`보다 작으면 `BOAT_RAMP_SPD`까지 과도하게 올라가지 않고 요청 속도에서 멈추는지 확인
- signed thrust command가 step으로 들어와도 최종 actuator signed thrust magnitude가 `BOAT_ENG_RAMP_UP` 이하의 기울기로 증가하는지 확인
- MANUAL direct thrust에서도 final signed thrust ramp limiter가 적용되는지 확인
- disarm/failsafe/emergency stop에서는 thrust ramp를 우회하고 즉시 signed thrust 0이 publish되는지 확인
- signed thrust 부호가 bridge/driver에서 clutch forward/neutral/reverse로 변환되는지 확인

### 11.2 PX4 runtime check

- `boat_control start`로 모듈이 시작되는지 확인
- disarmed 상태에서 actuator 출력이 0인지 확인
- AUTO_MISSION 외 nav_state에서 mission setpoint 기반 출력이 생성되지 않는지 확인
- navigator가 `position_setpoint_triplet`을 갱신하면 `rover_position_setpoint`와 `BoatPosControl` target이 갱신되는지 확인
- MANUAL에서 roll/throttle stick이 actuator output으로 반영되는지 확인
- ACRO에서 roll stick이 yaw-rate setpoint로 반영되는지 확인
- STAB에서 roll stick release 시 heading hold가 유지되는지 확인
- POSCTL에서 throttle stick이 signed speed setpoint로 반영되고 roll centered 상태에서 course hold target이 생성되는지 확인
- `rover_speed_status.measured_speed_body_x`, `adjusted_speed_body_x_setpoint`, `pid_throttle_body_x_integral`이 closed-loop tuning에 필요한 값으로 publish되는지 확인
- AUTO/POSCTL/OFFBOARD에서 목표 속도가 갑자기 바뀌어도 로그상 speed setpoint와 signed thrust가 diesel soft-start 제한을 따라 천천히 상승하는지 확인
- mission 완료 또는 disarm 시 motor 정지 명령이 나가는지 확인

### 11.3 Field/bench comparison

- 같은 `start/current/target` 조건에서 기존 ROS 노드의 `LOS`, `steer`, `remap_thrust`, `desired_velocity`와 PX4 로그를 비교한다.
- 차이가 나면 먼저 좌표계와 yaw 부호를 확인한다.
- actuator 방향이 반대이면 mixer, actuator parameter, bridge/driver mapping에서 보정하고 알고리즘 부호는 최대한 유지한다.
- `BOAT_LOOKAHD`, `BOAT_STR_P`, `BOAT_STR_D`, `BOAT_SPD_ACC_UP`, `BOAT_ENG_RAMP_UP`, `BOAT_THR_FF`, `BOAT_THR_RATE` 순서로 튜닝한다.
- 조종기 bench test는 propeller/추진기 안전 상태에서 MANUAL, STAB, POSCTL 순서로 진행한다.

## 12. Resolved Contract And Remaining Checks

- `BoatAttControl`이 yaw setpoint와 yaw-rate setpoint를 모두 steering으로 변환한다. 필요 시 추후 `BoatRateControl`로 분리한다.
- actuator contract는 `actuator_servos.control[0] = steering`, `actuator_motors.control[0] = signed_thrust`로 결정한다. 외부 simulator/output order는 `output/control[0] = steering`, `output/control[1] = signed_thrust`로 맞춘다.
- `BoatManualMode`, `BoatSpeedControl`, `BoatActControl`의 signed thrust path는 구현 완료되었다.
- 실제 Pixhawk firmware에서는 외부 MAVLink boat driver가 `SERVO_OUTPUT_RAW`를 받아 signed thrust를 engine throttle과 clutch forward/neutral/reverse로 변환한다.
- 기존 ROS 코드의 yaw 변환 `psi = -msg->data[2] + pi/2`가 PX4 NED yaw와 어떤 관계인지 log로 검증해야 한다.

## 13. Required Code Changes

이 섹션은 현재 코드 기준으로 signed thrust contract와 diesel soft-start를 완성하기 위해 실제 수정해야 할 항목을 파일별로 정리한다.

상태:

- 완료: `BoatManualMode`, `BoatSpeedControl`, `BoatActControl`, `module.yaml`의 boat_control 내부 구현.
- 완료: `make px4_sitl_boat -j2` 빌드 검증.
- boat_control 범위 밖: Isaac bridge 또는 외부 MAVLink boat driver의 clutch state machine 구현.

### 13.1 Signed Thrust Contract

목표:

```text
actuator_servos.control[0] = steering [-1, 1]
actuator_motors.control[0] = signed_thrust [-1, 1]
```

수정 완료 대상:

- `BoatDriveModes/BoatManualMode/BoatManualMode.cpp`
  - 완료: `manual()`, `acro()`, `stab()`에서 `manual_control_setpoint.throttle`을 `[-1, 1]` signed thrust로 제한한다.
  - 완료: `rover_throttle_setpoint.throttle_body_x`는 throttle magnitude가 아니라 signed thrust command로 취급한다.
  - 완료: yaw-rate command에서 signed throttle의 부호를 사용한다.
  - 완료: `position()`에서 POSCTL speed setpoint를 `[-BOAT_SPEED_LIM, BOAT_SPEED_LIM]`로 생성한다.

- `BoatSpeedControl/BoatSpeedControl.cpp`
  - 완료: `rover_speed_setpoint.speed_body_x`를 `[-BOAT_SPEED_LIM, BOAT_SPEED_LIM]`로 제한한다.
  - 완료: feedforward thrust 계산은 부호를 보존한다.

```text
speed_sign = sign(speed_setpoint)
thrust_abs = BOAT_THR_FF * abs(speed_setpoint)
remapped_abs = sqrt(thrust_abs / BOAT_THR_SCALE)
signed_thrust = speed_sign * constrain(remapped_abs / BOAT_THR_MAX, 0, 1)
```

  - 완료: measured speed feedback에서 `measured_speed_body_x`와 `speed_setpoint`의 부호를 모두 유지한다.
  - 완료: `speed_setpoint == 0`에서는 integral, derivative, ramp state를 reset하고 signed thrust 0을 publish한다.

- `BoatActControl/BoatActControl.cpp`
  - 완료: `actuator_motors.control[0]`는 `[-1.f, 1.f]` signed thrust로 제한한다.
  - 완료: 내부 변수명을 `_signed_thrust_setpoint`로 변경했다. uORB field 이름 `throttle_body_x`는 유지한다.
  - 완료: `actuator_motors.reversible_flags`는 `CA_R_REV`를 반영한다.
  - 완료: disarm/reset/failsafe stop에서는 ramp를 우회하고 즉시 `control[0] = 0`을 publish한다.

### 13.2 Diesel Soft-start And Ramp

목표:

```text
requested speed step
  -> adjusted speed setpoint ramp
  -> signed thrust command
  -> final signed thrust magnitude ramp
  -> actuator_motors.control[0]
```

구현 완료 대상:

- `module.yaml`
  - 아래 파라미터를 추가했다.

```text
BOAT_SPD_EN
BOAT_SPD_P
BOAT_SPD_I
BOAT_SPD_D
BOAT_SPD_IMAX
BOAT_RAMP_SPD
BOAT_RAMP_HOLD
BOAT_SPD_ACC_UP
BOAT_SPD_ACC_DN
BOAT_ENG_RAMP_UP
BOAT_ENG_RAMP_DN
```

- `BoatSpeedControl/BoatSpeedControl.hpp`
  - speed PID 상태를 추가했다.

```text
_speed_error_integral
_previous_speed_error
_timestamp
```

  - speed ramp 상태를 추가했다.

```text
_adjusted_speed_setpoint
_startup_ramp_active
_startup_ramp_reached_time
```

- `BoatSpeedControl/BoatSpeedControl.cpp`
  - 완료: `BOAT_SPD_EN == 0`이면 현재 feedforward-only 경로를 유지한다.
  - 완료: `BOAT_SPD_EN == 1`이면 feedforward에 measured speed PID feedback을 더한다.
  - 완료: `adjusted_speed_setpoint`를 raw speed setpoint 대신 feedforward/PID의 목표값으로 사용한다.
  - 완료: `BOAT_RAMP_SPD > 0`이면 출발 시 `abs(requested_speed_setpoint)`를 먼저 `BOAT_RAMP_SPD`까지 제한하고, `BOAT_RAMP_HOLD` 이후 원래 요청 속도로 ramp한다.
  - 완료: 후진 요청은 negative speed setpoint로 유지하되 ramp 계산은 magnitude 기준으로 적용한다.
  - 완료: `rover_speed_status.adjusted_speed_body_x_setpoint`와 `pid_throttle_body_x_integral`을 로그 튜닝에 쓸 수 있게 채운다.

- `BoatActControl/BoatActControl.hpp/.cpp`
  - 완료: final signed thrust magnitude ramp limiter를 추가한다.
  - 완료: MANUAL direct thrust, ACRO direct thrust, speed controller 출력이 모두 이 limiter를 통과하게 한다.
  - 완료: `BOAT_ENG_RAMP_UP`은 magnitude 증가율, `BOAT_ENG_RAMP_DN`은 magnitude 감소율로 사용한다.
  - 완료: 부호 변경 시에는 drivetrain layer가 clutch neutral/shift delay를 처리하므로, `boat_control`은 signed thrust만 연속적으로 제한한다.

### 13.3 Drivetrain Boundary

`boat_control`은 clutch를 publish하지 않는다.

```text
boat_control:
  publish steering + signed_thrust

SITL Isaac bridge or external MAVLink boat driver:
  signed_thrust > deadband  -> clutch forward, throttle abs(signed_thrust)
  signed_thrust < -deadband -> clutch reverse, throttle abs(signed_thrust)
  near zero                 -> clutch neutral, throttle 0
```

따라서 아래 항목은 `boat_control` 내부에 구현하지 않는다.

- clutch state machine
- shift delay
- engage delay
- engine throttle PWM mapping
- clutch servo/GPIO/CAN command

다만 `BoatActControl`의 final signed thrust ramp는 engine 보호를 위해 controller boundary 안에 둔다.

### 13.4 Verification Items For This Change

구현 후 최소 확인 항목:

- MANUAL throttle negative 입력이 `actuator_motors.control[0] < 0`으로 publish되는지 확인한다.
- POSCTL throttle centered에서 speed setpoint가 0이고 motor output이 0인지 확인한다.
- POSCTL throttle negative에서 `rover_speed_setpoint.speed_body_x < 0`이 publish되는지 확인한다.
- AUTO mission speed가 양수일 때 기존 전진 동작이 유지되는지 확인한다.
- `BOAT_SPD_EN=0`에서 기존 feedforward-only 동작이 보존되는지 확인한다.
- `BOAT_SPD_EN=1`에서 measured speed feedback이 signed thrust를 올바른 방향으로 보정하는지 확인한다.
- step thrust 입력 시 `BOAT_ENG_RAMP_UP`보다 빠르게 magnitude가 증가하지 않는지 확인한다.
- disarm/failsafe/reset 시 ramp와 관계없이 signed thrust가 즉시 0이 되는지 확인한다.
- Isaac bridge 또는 외부 MAVLink boat driver에서 signed thrust 부호가 clutch forward/neutral/reverse로 변환되는지 확인한다.

## 14. Real Boat Output Interface Plan

이 섹션은 실제 보트와 SITL에서 `boat_control` 출력이 어떤 interface로 전달되는지 정리한다.

결정된 방향:

```text
Primary real boat interface:
  MAVLink SERVO_OUTPUT_RAW

Debug/backup interface:
  pwm_out physical pins

SITL interface:
  HIL_ACTUATOR_CONTROLS
```

PX4 내부에 별도 `boat_drivetrain` driver를 두지 않는다. clutch, engine throttle, gear shift 보호 로직은 외부 MAVLink boat driver 또는 SITL bridge가 담당한다.

### 14.1 Interface Contract

`boat_control`은 계속 steering과 signed thrust만 publish한다.

```text
boat_control
  -> actuator_servos.control[0] = steering [-1, 1]
  -> actuator_motors.control[0] = signed_thrust [-1, 1]
```

PX4 output mapping 이후 외부로 나가는 contract는 channel 2개로 고정한다.

```text
Output channel 1:
  steering

Output channel 2:
  signed_thrust
```

실제 외부 boat driver는 `signed_thrust`를 해석한다.

```text
signed_thrust > deadband:
  clutch = forward
  engine_throttle = abs(signed_thrust)

signed_thrust < -deadband:
  clutch = reverse
  engine_throttle = abs(signed_thrust)

abs(signed_thrust) <= deadband:
  clutch = neutral
  engine_throttle = 0
```

### 14.2 Real Firmware Path

실제 Pixhawk firmware에서 primary path:

```text
boat_control
        |
        v
actuator_servos / actuator_motors
        |
        v
control_allocator / output function mapping
        |
        v
actuator_outputs
        |
        v
MAVLink SERVO_OUTPUT_RAW
        |
        v
external MAVLink boat driver
        +--> steering actuator
        +--> engine throttle
        +--> clutch forward / neutral / reverse
```

`SERVO_OUTPUT_RAW`를 primary interface로 쓰는 이유:

- 실제 PX4 firmware에서 기본 MAVLink stream으로 제공된다.
- 외부 driver가 PWM pin을 직접 읽지 않고도 PWM-equivalent 값을 받을 수 있다.
- `pwm_out` physical pins와 같은 output mapping을 공유하므로 bench debug가 쉽다.
- PX4 내부에 보트 전용 output driver를 추가하지 않아도 된다.

권장 MAVLink field mapping:

```text
SERVO_OUTPUT_RAW.servo1_raw = steering PWM-equivalent
SERVO_OUTPUT_RAW.servo2_raw = signed_thrust PWM-equivalent
```

PWM-equivalent 해석 예:

```text
steering:
  1000 us = left
  1500 us = center
  2000 us = right

signed_thrust:
  1000 us = reverse max
  1500 us = neutral
  2000 us = forward max
```

외부 driver는 `servo2_raw`를 다시 normalized signed thrust로 변환한 뒤 clutch state machine을 실행한다.

### 14.3 Debug And Backup PWM Path

`pwm_out`은 실제 제어 primary가 아니라 debug/backup interface로 둔다.

```text
boat_control
        |
        v
actuator_servos / actuator_motors
        |
        v
control_allocator / output function mapping
        |
        +--> MAVLink SERVO_OUTPUT_RAW -> external boat driver
        |
        +--> pwm_out physical pins     -> oscilloscope / servo tester / backup wiring
```

권장 debug PWM mapping:

```text
PWM CH1 = steering
PWM CH2 = signed_thrust
```

사용 목적:

- MAVLink로 외부 driver가 받은 값과 실제 PWM pin 값을 비교한다.
- bench에서 oscilloscope 또는 servo tester로 output mapping을 확인한다.
- QGC actuator test와 실제 output 방향을 검증한다.
- 외부 MAVLink boat driver 장애 시 backup wiring 가능성을 실험한다.

### 14.4 SITL Path

SITL/Isaac Sim에서는 `HIL_ACTUATOR_CONTROLS`를 사용한다.

```text
boat_control
        |
        v
actuator_servos / actuator_motors
        |
        v
pwm_out_sim / actuator_outputs_sim
        |
        v
MAVLink HIL_ACTUATOR_CONTROLS
        |
        v
Isaac Sim bridge
        +--> steering
        +--> signed_thrust
        +--> simulated engine/clutch behavior
```

권장 SITL field mapping:

```text
HIL_ACTUATOR_CONTROLS.controls[0] = steering [-1, 1]
HIL_ACTUATOR_CONTROLS.controls[1] = signed_thrust [-1, 1]
```

Isaac Sim bridge는 실제 외부 boat driver와 같은 signed thrust 해석 규칙을 사용한다.

### 14.5 External Boat Driver Responsibilities

외부 MAVLink boat driver가 담당한다.

- `SERVO_OUTPUT_RAW` 수신
- `servo1_raw`를 steering command로 변환
- `servo2_raw`를 signed thrust로 변환
- signed thrust deadband/hysteresis 처리
- clutch forward / neutral / reverse state machine 처리
- direction change 시 throttle cut, neutral wait, engage delay 적용
- reverse inhibit 필요 시 적용
- MAVLink timeout 시 throttle 0 + clutch neutral 강제
- arming/failsafe 상태를 필요하면 `HEARTBEAT`, `SYS_STATUS`, `EXTENDED_SYS_STATE` 등과 함께 확인

외부 driver safety 예:

```text
if SERVO_OUTPUT_RAW timeout > 0.2~0.5 s:
  engine_throttle = 0
  clutch = neutral
  steering = center or last-safe
```

### 14.6 PX4 Configuration Requirements

PX4 쪽 요구 사항:

- output function mapping에서 channel 1을 steering으로 설정한다.
- output function mapping에서 channel 2를 signed thrust로 설정한다.
- channel 2는 reverse/neutral/forward를 표현할 수 있도록 center-based signed output으로 설정한다.
- 실제 PWM debug를 위해 channel 1/2의 min/trim/max를 1000/1500/2000 us 기준으로 맞춘다.
- MAVLink에서 `SERVO_OUTPUT_RAW_0` stream이 충분한 rate로 나가게 한다.
- POSIX SITL의 GCS link는 기존 `px4-rc.mavlink`에서 `SERVO_OUTPUT_RAW_0` 50 Hz stream을 이미 설정한다.
- 실제 firmware의 외부 boat driver link는 연결 port/baud/role이 정해진 뒤 같은 stream을 요구 rate로 설정한다.

예:

```sh
mavlink stream -s SERVO_OUTPUT_RAW_0 -r 20
```

더 빠른 외부 driver update가 필요하면:

```sh
mavlink stream -s SERVO_OUTPUT_RAW_0 -r 50
```

추가 확인용 stream:

```sh
mavlink stream -s ACTUATOR_OUTPUT_STATUS -r 20
```

`ACTUATOR_OUTPUT_STATUS`는 primary control interface가 아니라 PX4 내부 actuator output 배열을 확인하기 위한 debug stream으로 본다.

### 14.7 Removed PX4 Boat Drivetrain Driver

이전 계획의 PX4 내부 `boat_drivetrain` driver는 사용하지 않는다.

삭제 대상:

- `src/drivers/boat_drivetrain/`
- `msg/BoatDrivetrainStatus.msg`
- `msg/CMakeLists.txt`의 `BoatDrivetrainStatus.msg`
- `boards/px4/*/boat.px4board`의 `CONFIG_DRIVERS_BOAT_DRIVETRAIN`
- `rc.boat_apps`의 `boat_drivetrain start`

삭제 이유:

- 실제 primary interface가 MAVLink `SERVO_OUTPUT_RAW`로 결정되었다.
- `pwm_out`은 debug/backup 역할로 충분하다.
- clutch state machine은 PX4 내부 output driver보다 실제 engine/clutch hardware를 아는 외부 boat driver에 두는 편이 안전하다.
- SITL도 `HIL_ACTUATOR_CONTROLS`로 같은 steering/signed thrust contract를 재사용할 수 있다.

### 14.8 Implementation Checklist

- [x] `boat_control` signed thrust contract 구현.
- [x] `actuator_servos.control[0] = steering` 유지.
- [x] `actuator_motors.control[0] = signed_thrust` 유지.
- [x] PX4 내부 `boat_drivetrain` driver 삭제.
- [x] `BoatDrivetrainStatus.msg` 삭제.
- [x] `rc.boat_apps`에서 `boat_drivetrain start` 삭제.
- [x] boat board config에서 `CONFIG_DRIVERS_BOAT_DRIVETRAIN` 삭제.
- [x] `1071_isaac_boat` output function mapping에서 CH1 steering, CH2 signed thrust 설정.
- [x] POSIX SITL `1071_isaac_boat`에서는 `pwm_out_sim`이 제공하는 `PWM_MAIN_FUNC1/2`만 설정하고, 존재하지 않는 `PWM_MAIN_MIN/TRIM/MAX*` 설정은 제거.
- [x] Isaac Sim bridge에서 `HIL_ACTUATOR_CONTROLS.controls[0/1]`를 steering/signed thrust로 처리.
- [x] Isaac Sim bridge에서 signed thrust deadband 기반 clutch forward/neutral/reverse, throttle magnitude 변환 구현.
- [x] POSIX SITL GCS MAVLink link는 `px4-rc.mavlink`에서 `SERVO_OUTPUT_RAW_0` 50 Hz stream을 사용.
- [ ] 실제 firmware 외부 boat driver가 붙는 MAVLink link에서 `SERVO_OUTPUT_RAW_0` stream rate를 요구 rate로 설정.
- [ ] 외부 MAVLink boat driver에서 `SERVO_OUTPUT_RAW.servo1_raw/servo2_raw` 처리 구현.
- [ ] 외부 driver에서 clutch state machine, timeout, reverse inhibit 검증.
- [ ] 실제 hardware 또는 SITL MAVLink Inspector에서 channel 2 signed output이 1000/1500/2000 us로 보존되는지 확인.
- [ ] `pwm_out` physical pins로 MAVLink 값과 PWM pin 값이 일치하는지 bench 검증.

### 14.9 Verification Checklist

PX4 firmware 확인:

- `SERVO_OUTPUT_RAW.servo1_raw`가 steering 입력에 따라 변하는지 확인한다.
- `SERVO_OUTPUT_RAW.servo2_raw`가 signed thrust 입력에 따라 1500 us 기준 양/음 방향으로 변하는지 확인한다.
- throttle neutral에서 `servo2_raw`가 trim 근처로 유지되는지 확인한다.
- disarm/failsafe에서 `servo2_raw`가 neutral 또는 disarmed-safe 값으로 가는지 확인한다.
- debug PWM CH1/CH2가 `SERVO_OUTPUT_RAW`와 같은 방향으로 움직이는지 확인한다.

SITL 확인:

- `HIL_ACTUATOR_CONTROLS.controls[0]`가 steering `[-1, 1]`로 전달되는지 확인한다.
- `HIL_ACTUATOR_CONTROLS.controls[1]`가 signed thrust `[-1, 1]`로 전달되는지 확인한다.
- Isaac Sim bridge가 signed thrust 부호를 forward/neutral/reverse로 해석하는지 확인한다.

외부 driver 확인:

- `servo2_raw > 1500 + deadband`에서 clutch forward, throttle magnitude가 생성되는지 확인한다.
- `servo2_raw < 1500 - deadband`에서 clutch reverse, throttle magnitude가 생성되는지 확인한다.
- `servo2_raw`가 deadband 내부이면 clutch neutral, throttle 0이 되는지 확인한다.
- forward/reverse 전환 시 throttle cut과 neutral delay가 적용되는지 확인한다.
- MAVLink timeout 시 throttle 0 + clutch neutral이 즉시 적용되는지 확인한다.

## 15. Current Open Issues From Implementation Review

2026-06-24 코드 확인 기준으로 `boat_control` 핵심 구조와 signed thrust path, diesel ramp, Isaac SITL bridge 연동은 대부분 구현되어 있다. 다만 Plan 전체 완료로 보기 전에 아래 항목을 해결하거나 검증해야 한다.

### 15.1 POSCTL Reverse Course Hold Signed Speed Loss

상태: 수정됨. SITL/bench 검증 필요.

문제:

- `BoatManualMode::position()`은 POSCTL throttle negative 입력을 `[-BOAT_SPEED_LIM, BOAT_SPEED_LIM]` signed speed로 변환한다.
- roll centered + speed nonzero 조건에서는 `matrix::sign(speed_setpoint)`를 사용해 현재 heading의 반대 방향 target waypoint를 생성하고, `rover_position_setpoint.cruising_speed`에 negative speed를 publish한다.
- 기존 구현에서는 `BoatPosControl::updatePosControl()`이 `_cruising_speed`를 `[0, BOAT_SPEED_LIM]` 범위로 constrain했다.
- 결과적으로 POSCTL 후진 course hold에서 target은 뒤쪽에 생성되지만 `rover_speed_setpoint.speed_body_x`는 0 이상으로 바뀌어 후진 thrust가 생성되지 않았다.
- 현재 구현은 `BoatPosControl`의 speed constrain 범위를 `[-BOAT_SPEED_LIM, BOAT_SPEED_LIM]`로 바꿔 signed speed를 보존한다.

영향:

- 코드상으로는 Plan의 “POSCTL throttle negative에서 `rover_speed_setpoint.speed_body_x < 0`이 publish되는지 확인” 항목을 만족하도록 수정되었다.
- 실제 SITL/bench에서 `BoatSpeedControl`, `BoatActControl`, output mapping까지 음수 명령이 유지되는지 확인해야 한다.

권장 수정 방향:

- 완료: `BoatPosControl`에서 speed constrain 범위를 `[-BOAT_SPEED_LIM, BOAT_SPEED_LIM]`로 바꿔 signed speed를 보존한다.
- 후진 course hold yaw policy를 명확히 정한다.
- 현재 `BoatManualMode` 구조는 후진 target waypoint를 현재 heading 반대 방향에 두고, `BoatPosControl`은 negative speed일 때 `bearing_setpoint + pi`를 yaw setpoint로 사용한다.
- 이 정책은 bow heading을 유지한 채 뒤로 물러나는 동작을 의도한다. 현장 운용에서 “후진 방향을 향해 선회”가 더 자연스럽다면 `BoatManualMode`와 `BoatPosControl`을 함께 조정한다.

완료 기준:

- POSCTL throttle negative + roll centered에서 `rover_position_setpoint.cruising_speed < 0`이 유지된다.
- `BoatPosControl` 이후 `rover_speed_setpoint.speed_body_x < 0`이 publish된다.
- `BoatSpeedControl` 이후 `rover_throttle_setpoint.throttle_body_x < 0`이 publish된다.
- `BoatActControl` 이후 `actuator_motors.control[0] < 0`이 publish된다.
- 후진 course hold 중 yaw setpoint 방향이 선택한 정책과 일관된다.

### 15.2 Failsafe Or Unsupported Nav State Stop Behavior

상태: 수정됨. SITL/bench 검증 필요.

문제:

- 기존 구현에서 `BoatControl::generateSetpoints()`는 미지원 nav_state default 경로에서 새 setpoint를 publish하지 않았다.
- armed 상태이고 sanity check가 통과하면 controller update는 계속 실행될 수 있었다.
- `BoatActControl`은 기존 finite throttle/steering setpoint가 남아 있으면 actuator topic을 계속 publish할 수 있었다.
- 현재 구현은 `generateSetpoints()`가 지원 nav_state 여부를 bool로 반환하고, 미지원 nav_state에서는 `reset()`과 `stopVehicle()`을 호출한 뒤 controller update를 건너뛴다.

영향:

- Plan의 “disarm/failsafe/emergency stop에서는 ramp down을 기다리지 않고 즉시 signed thrust 0” 요구가 미지원 nav_state 경로에서도 코드상 적용된다.
- 실제 failsafe nav_state 전이와 output topic을 SITL/bench에서 확인해야 한다.

권장 수정 방향:

- 완료: `BoatControl`에서 지원하지 않는 nav_state를 명시적으로 감지해 `reset()`과 `stopVehicle()`을 호출한다.
- 또는 `BoatActControl`에 setpoint timeout을 추가해 일정 시간 새 setpoint가 없으면 motor/steering을 0으로 publish한다.

완료 기준:

- disarm뿐 아니라 failsafe/unsupported nav_state에서도 `actuator_motors.control[0] = 0`이 즉시 publish된다.
- final signed thrust ramp limiter가 stop path를 지연하지 않는다.

### 15.3 Remaining External Driver And Hardware Verification

상태: Plan checklist에 미완료로 남아 있음.

남은 항목:

- 실제 firmware 외부 boat driver가 붙는 MAVLink link에서 `SERVO_OUTPUT_RAW_0` stream rate를 요구 rate로 설정한다.
- 외부 MAVLink boat driver에서 `SERVO_OUTPUT_RAW.servo1_raw/servo2_raw` 처리 구현을 확인한다.
- 외부 driver에서 clutch state machine, timeout, reverse inhibit를 검증한다.
- 실제 hardware 또는 SITL MAVLink Inspector에서 channel 2 signed output이 1000/1500/2000 us로 보존되는지 확인한다.
- `pwm_out` physical pins로 MAVLink 값과 PWM pin 값이 일치하는지 bench 검증한다.

비고:

- 위 항목은 `boat_control` 내부 구현 범위 밖이지만, Plan 전체 완료 판단에는 포함된다.
- Isaac Sim bridge의 `HIL_ACTUATOR_CONTROLS.controls[0] = steering`, `controls[1] = signed_thrust` 처리와 deadband 기반 clutch/throttle 변환은 코드상 존재한다.
