# Boat Control Controller Notes

이 문서는 `boat_control` 모듈의 제어기 구조, 특히 `BoatAttControl`이 attitude/rate 입력을 바로 steering으로 변환하는 이유와 이 방식의 적용 가능 범위, 한계, 향후 개선 방향을 정리한다.

## 1. Current Controller Pipeline

`boat_control`은 PX4 rover 계열 setpoint 토픽을 재사용한다. 상위 mode layer는 flight mode에 따라 setpoint를 만들고, controller layer는 control flag에 따라 필요한 제어기를 실행한다.

```text
BoatControl
  -> BoatAutoMode / BoatManualMode / BoatOffboardMode
  -> BoatPosControl
  -> BoatSpeedControl
  -> BoatAttControl
  -> BoatActControl
```

토픽 흐름은 다음과 같다.

```text
AUTO/OFFBOARD position
        |
        v
rover_position_setpoint
        |
        v
BoatPosControl
        |
        +--> rover_speed_setpoint ----+
        |                             v
        |                      BoatSpeedControl
        |                             |
        |                             v
        |                      rover_throttle_setpoint
        |
        +--> rover_attitude_setpoint -+
                                      v
                              BoatAttControl
                                      |
                              rover_steering_setpoint
                                      |
                                      v
                              BoatActControl
                                      |
                       +--------------+---------------+
                       v                              v
                actuator_motors                actuator_servos
```

결정된 actuator contract는 다음과 같다.

```text
PX4 internal uORB:
  actuator_servos.control[0] = steering [-1, 1]
  actuator_motors.control[0] = signed_thrust [-1, 1]

External simulator / output order:
  output/control[0] = steering
  output/control[1] = signed_thrust
```

`signed_thrust`의 부호는 drivetrain layer에서 clutch 상태로 변환한다.

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

따라서 `boat_control`은 clutch를 직접 제어하지 않는다. `boat_control`은 steering과 signed thrust만 만들고, SITL에서는 Isaac bridge가, 실제 Pixhawk firmware에서는 boat drivetrain/output driver가 signed thrust를 `throttle + clutch`로 변환한다.

output driver까지의 전체 flow는 다음과 같다.

```text
vehicle state / mission / manual input
        |
        v
boat_control
        |
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
                 +--> output/control[0] = steering
                 +--> output/control[1] = signed_thrust
                 |
                 v
        SITL Isaac bridge or Pixhawk boat drivetrain driver
                 |
                 +--> steering actuator
                 +--> engine throttle = abs(signed_thrust)
                 +--> clutch = forward / neutral / reverse
```

drivetrain driver는 clutch 보호를 위한 deadband, hysteresis, shift delay, throttle cut, ramp limit을 담당한다. 예를 들어 전진에서 후진으로 바뀔 때는 throttle을 먼저 0으로 내리고, clutch neutral 상태와 shift delay를 거친 뒤 reverse clutch를 체결하고 다시 throttle을 올려야 한다.

수동/보조 모드에서는 `BoatManualMode`가 `rover_rate_setpoint` 또는 `rover_attitude_setpoint`를 직접 만들 수 있다.

```text
MANUAL:
  manual_control_setpoint
    -> rover_throttle_setpoint
    -> rover_steering_setpoint

ACRO/STAB/POSCTL:
  manual_control_setpoint
    -> rover_rate_setpoint 또는 rover_attitude_setpoint
    -> BoatAttControl
    -> rover_steering_setpoint
```

## 2. Why BoatAttControl Directly Generates Steering

`rover_ackermann`은 다음과 같이 attitude, rate, steering 계층을 분리한다.

```text
rover_attitude_setpoint
  -> AckermannAttControl
  -> rover_rate_setpoint
  -> AckermannRateControl
  -> rover_steering_setpoint
```

Ackermann rover에서는 이 구조가 자연스럽다. 바퀴 차량은 속도, 휠베이스, 조향각, yaw rate 사이에 비교적 명확한 운동학 관계가 있다.

```text
yaw_rate ~= velocity * tan(steering_angle) / wheel_base
```

따라서 `RA_WHEEL_BASE`, `RA_MAX_STR_ANG`, `RO_YAW_RATE_LIM` 같은 파라미터를 사용해 현재 속도에서 가능한 yaw rate를 제한하고, yaw-rate feedback을 통해 steering을 보정할 수 있다.

보트는 이와 다르다. 보트의 회두 응답은 선체 형상, rudder/servo 효율, 추진기 위치, 속도, 유체 저항, 조류, 바람, 파도, slip 등의 영향을 받는다. Ackermann 차량처럼 `wheel_base`와 `steering_angle`로 yaw rate를 안정적으로 계산하기 어렵다.

그래서 현재 `boat_control`은 다음처럼 단순화된 구조를 사용한다.

```text
rover_attitude_setpoint 또는 rover_rate_setpoint
        |
        v
BoatAttControl
        |
        v
rover_steering_setpoint
```

이 구조의 의도는 다음과 같다.

- Ackermann 운동학 공식을 보트에 억지로 적용하지 않는다.
- yaw error 또는 yaw-rate command를 보트용 steering 명령으로 직접 변환한다.
- 검증된 legacy boat controller의 carrot/LOS steering PD 개념을 PX4 module 구조 안에 유지한다.
- `BOAT_STR_*`, `BOAT_Y_RATE_LIM` 파라미터로 실제 선체 응답에 맞게 현장 튜닝한다.

## 3. BoatAttControl Behavior

`BoatAttControl`은 두 종류의 입력을 처리한다.

### 3.1 Heading/Yaw Setpoint

`rover_attitude_setpoint.yaw_setpoint`이 finite이면 현재 yaw와의 오차를 계산하고 steering PD를 적용한다.

```text
heading_error = wrap_pi(yaw_setpoint - vehicle_yaw)

steering = -BOAT_STR_P * heading_error
           -BOAT_STR_D * d(heading_error)

steering = constrain(steering, -BOAT_STR_MAX, BOAT_STR_MAX)
steering = slew_limit(steering, last_steering, BOAT_STR_RATE)

normalized_steering = steering / BOAT_STR_MAX
```

이 경로는 AUTO/OFFBOARD position 추종이나 STAB/POSCTL heading/course hold에서 주로 사용된다.

### 3.2 Yaw-Rate Setpoint

`rover_rate_setpoint.yaw_rate_setpoint`이 finite이면 명령 yaw rate를 `BOAT_Y_RATE_LIM` 기준으로 정규화해 steering으로 변환한다.

```text
normalized_steering =
  constrain(yaw_rate_setpoint / BOAT_Y_RATE_LIM, -1, 1)
```

이 경로는 ACRO, STAB에서 stick roll 입력이 있을 때, POSCTL에서 stick roll 입력이 있을 때 주로 사용된다.

현재 구현은 `vehicle_angular_velocity`를 구독하지만 yaw-rate feedback loop를 강하게 닫지는 않는다. 즉, yaw-rate command가 실제 yaw-rate로 얼마나 잘 추종되는지를 PID로 보정하는 구조는 아직 아니다.

## 4. Is This Sufficient For Steering Control?

가능하다. 다만 "충분함"은 목표 성능과 운항 환경에 따라 달라진다.

현재 방식은 다음 조건에서 실용적으로 충분할 수 있다.

- 운항 속도 범위가 비교적 좁다.
- 선체 응답이 느리고 안정적이다.
- 강한 조류, 바람, 파도 환경이 아니다.
- waypoint 추종 정밀도가 센티미터급 또는 매우 고성능일 필요는 없다.
- steering actuator 입력 대비 yaw 응답이 현장 튜닝으로 예측 가능하다.
- `BOAT_STR_P`, `BOAT_STR_D`, `BOAT_STR_RATE`, `BOAT_STR_MAX`, `BOAT_Y_RATE_LIM`으로 overshoot와 hunting을 억제할 수 있다.

이 구조는 복잡한 모델 없이 실제 보트에서 빠르게 검증하고 튜닝하기 좋은 시작점이다. 특히 보트의 정확한 hydrodynamic model이 없거나, 센서/액추에이터 특성이 아직 충분히 식별되지 않은 단계에서는 단순 PD/direct steering 방식이 더 안전한 출발점일 수 있다.

## 5. Known Limitations

다음 상황에서는 현재 구조가 부족할 수 있다.

- 속도에 따라 회두 응답이 크게 달라진다.
- 저속에서는 조타가 거의 먹지 않고, 고속에서는 같은 steering 명령이 과도한 yaw rate를 만든다.
- waypoint 주변에서 S-turn, hunting, overshoot가 반복된다.
- 조류/바람 외란 때문에 heading hold가 계속 밀린다.
- yaw-rate command를 줬을 때 실제 yaw-rate 추종성이 낮다.
- rudder각, 추력, 속도, yaw-rate 사이의 관계를 명시적으로 반영해야 한다.
- 더 높은 경로 추종 정밀도나 넓은 속도 envelope가 필요하다.

특히 현재 yaw-rate 경로는 다음과 같은 open-loop 성격이 강하다.

```text
yaw_rate_setpoint
  -> normalized steering
  -> actual yaw_rate
```

실제 yaw-rate를 feedback으로 비교해서 steering을 보정하는 계층이 없으므로, 외란이나 속도 변화에 따라 yaw-rate 응답이 달라질 수 있다.

## 6. Difference From Ackermann Rate Controller

Ackermann의 rate controller는 보트에 그대로 가져오면 안 된다. Ackermann rate-to-steering 변환은 차량 운동학을 사용한다.

```text
steering_angle = atan(yaw_rate * wheel_base / speed)
normalized_steering = steering_angle / max_steering_angle
```

보트에는 `wheel_base`나 tire steering angle에 해당하는 기하학 모델이 없다. rudder 방식의 보트라 해도 yaw moment는 rudder angle만이 아니라 유속, prop wash, 선체 slip, lateral drag, actuator 위치에 의존한다. differential thrust 방식이면 steering의 의미도 완전히 달라진다.

따라서 보트에 rate 계층을 추가한다면 Ackermann 공식을 복사하는 것이 아니라 보트용 yaw-rate controller로 설계해야 한다.

## 7. Recommended Future Improvement

정밀 yaw-rate 추종이나 넓은 속도 범위가 필요해지면 다음 구조로 확장하는 것이 좋다.

```text
rover_attitude_setpoint
        |
        v
BoatAttControl
  yaw error -> desired yaw-rate
        |
        v
rover_rate_setpoint
        |
        v
BoatRateControl
  yaw-rate feedback -> steering command
        |
        v
rover_steering_setpoint
        |
        v
BoatActControl
```

이때 `BoatRateControl`은 Ackermann 운동학 대신 아래 중 하나를 사용할 수 있다.

### 7.1 Simple Yaw-Rate PID

가장 단순한 확장이다.

```text
yaw_rate_error = yaw_rate_setpoint - vehicle_yaw_rate
steering = Kp * yaw_rate_error + Ki * integral + Kd * derivative
steering = constrain_and_slew_limit(steering)
```

장점은 구현이 쉽고 yaw-rate feedback을 닫을 수 있다는 점이다. 단점은 속도별 조타 효율 차이를 별도로 보정하지 않으면 튜닝 범위가 좁을 수 있다는 점이다.

### 7.2 Speed-Based Gain Scheduling

속도에 따라 steering gain을 바꾼다.

```text
effective_gain = f(speed_body_x)
steering = effective_gain * yaw_rate_controller_output
```

예를 들면 저속에서는 gain을 키우고, 고속에서는 gain을 낮춰 overshoot를 줄일 수 있다. 실제 로그 기반으로 속도 구간별 gain table 또는 간단한 함수로 시작할 수 있다.

### 7.3 Rudder/Thrust Effectiveness Model

실험 데이터가 충분하면 rudder angle, throttle, speed, yaw-rate 사이의 경험 모델을 만들 수 있다.

```text
yaw_rate ~= f(speed, throttle, rudder_command)
rudder_command ~= inverse_f(desired_yaw_rate, speed, throttle)
```

이 방식은 정확도가 좋아질 수 있지만, 보트/추진기/러더 구성마다 모델 식별이 필요하다.

## 8. Practical Tuning Guidance

현재 구조를 사용할 때는 다음 순서로 튜닝한다.

1. `BOAT_STR_MAX`를 실제 actuator가 안전하게 낼 수 있는 최대 steering command에 맞춘다.
2. `BOAT_STR_RATE`를 너무 빠른 조타 변화가 생기지 않도록 제한한다.
3. 낮은 속도에서 `BOAT_STR_P`를 올려 heading error가 줄어드는지 확인한다.
4. overshoot나 좌우 진동이 있으면 `BOAT_STR_D`와 `BOAT_STR_RATE`로 감쇠시킨다.
5. ACRO/STAB stick yaw 반응이 너무 민감하면 `BOAT_Y_RATE_LIM`, `BOAT_Y_STICK_DZ`, `BOAT_Y_EXPO`, `BOAT_Y_SUPEXPO`를 조정한다.
6. waypoint 추종 중 S-turn이 크면 `BOAT_LOOKAHD`를 키우거나 `BOAT_STR_P`를 낮춘다.
7. target line으로 너무 늦게 수렴하면 `BOAT_LOOKAHD`를 줄이거나 `BOAT_STR_P`를 올린다.

권장 로그 확인 항목:

```text
vehicle_attitude yaw
vehicle_angular_velocity yaw rate
vehicle_local_position x/y, vx/vy
rover_attitude_setpoint yaw_setpoint
rover_rate_setpoint yaw_rate_setpoint
rover_steering_setpoint normalized_steering_setpoint
rover_speed_setpoint speed_body_x
rover_throttle_setpoint throttle_body_x
actuator_motors control[0] signed_thrust
actuator_servos control[0] steering
```

## 9. Summary

현재 `boat_control`의 steering 제어는 다음 판단에 기반한다.

```text
Ackermann rover:
  vehicle kinematics are well-defined
  -> attitude -> rate -> steering cascade is appropriate

Boat:
  hydrodynamics are nonlinear and vehicle-specific
  -> direct yaw/yaw-rate to steering is a practical first controller
```

따라서 현재 방식으로도 일반적인 저속/중속 보트의 heading hold와 waypoint 추종은 가능하다. 다만 실제 yaw-rate를 폐루프로 추종하는 구조가 아니므로, 더 넓은 속도 범위, 강한 외란, 고정밀 경로 추종이 필요하면 `BoatRateControl` 또는 speed-based gain scheduling을 추가하는 방향으로 확장하는 것이 적절하다.
