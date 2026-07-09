# Boat Flight Modes

이 문서는 현재 `boat_control` 모듈이 실제로 처리하는 boat flight mode와, PX4 공통
navigation state에는 있으나 boat에서 제공하지 않는 mode를 정리한다.

## 제공하는 Flight Mode

현재 `src/modules/boat_control/BoatControl.cpp`의 `generateSetpoints()`에서 처리하는
boat mode는 다음과 같다.

| Flight mode | PX4 navigation state | 조종 입력 처리 | 제어 특징 | 필요한 주요 입력/상태 |
| --- | --- | --- | --- | --- |
| Manual | `NAVIGATION_STATE_MANUAL` | Roll stick은 steering setpoint로, throttle stick은 signed thrust로 직접 변환한다. | 가장 직접적인 수동 조종 모드다. 속도 유지, yaw-rate 제어, heading 유지, position 제어를 수행하지 않는다. | Manual control input |
| Acro | `NAVIGATION_STATE_ACRO` | Throttle stick은 signed thrust로 직접 변환하고, roll stick은 yaw-rate setpoint로 변환한다. | 조종자는 선회율을 명령하고 `BoatAttControl`이 yaw-rate setpoint를 steering setpoint로 변환한다. | Manual control input, angular velocity |
| Stabilized | `NAVIGATION_STATE_STAB` | Throttle stick은 signed thrust로 직접 변환한다. Roll 입력이 있으면 yaw-rate setpoint를 만들고, roll 입력이 없고 throttle이 있으면 현재 yaw를 heading setpoint로 유지한다. | 방향 안정화가 들어간 수동 모드다. 조종자가 선회하지 않을 때 현재 heading을 유지한다. | Manual control input, attitude |
| Position | `NAVIGATION_STATE_POSCTL` | Throttle stick은 speed setpoint로 변환한다. Roll 입력이 있거나 speed가 0이면 speed + yaw-rate setpoint를 만들고, roll 입력이 없고 speed가 있으면 현재 heading 방향의 가상 waypoint를 만든다. | 수동 입력을 기반으로 속도 제어와 course/position 제어를 수행한다. 조종자가 roll을 놓으면 현재 진행 방향으로 직선 course를 유지한다. | Manual control input, attitude, local position, local velocity |
| Mission | `NAVIGATION_STATE_AUTO_MISSION` | Navigator가 생성한 mission position setpoint를 사용한다. | Mission waypoint를 따라가도록 position, speed, attitude/rate, actuator control을 수행한다. | Mission, global/local position, attitude |
| Loiter | `NAVIGATION_STATE_AUTO_LOITER` | Navigator가 생성한 loiter setpoint를 사용한다. | 지정 위치 주변에서 loiter 동작을 수행한다. boat에서는 auto mode path를 통해 position/speed/yaw 제어로 처리된다. | Global/local position, attitude |
| RTL | `NAVIGATION_STATE_AUTO_RTL` | Navigator가 생성한 RTL setpoint를 사용한다. | Home position으로 복귀하는 auto mode다. | Home position, global/local position, attitude |
| Offboard | `NAVIGATION_STATE_OFFBOARD` | 외부 companion/GCS가 제공하는 offboard setpoint를 사용한다. | 외부 시스템이 위치, 속도, 자세, rate 또는 actuator 계층의 setpoint를 제공하는 모드다. | Valid offboard signal and selected offboard setpoints |

## 제공하지 않는 Flight Mode

아래 mode들은 PX4의 공통 `vehicle_status` navigation state에는 존재하지만,
현재 `boat_control`의 `generateSetpoints()`에서 처리하지 않는다. 따라서 boat에서 해당
nav state로 들어가면 boat setpoint가 생성되지 않으며, `BoatControl`은 fail path에서
vehicle stop 처리를 수행한다.

| Flight mode | PX4 navigation state | Boat 미지원 이유 |
| --- | --- | --- |
| Altitude Control | `NAVIGATION_STATE_ALTCTL` | 수면 차량인 boat에는 altitude/climb-rate 제어가 의미 없고, `boat_control`에서 setpoint 생성 경로가 없다. |
| Altitude Cruise | `NAVIGATION_STATE_ALTITUDE_CRUISE` | altitude 기반 cruise mode이며 boat setpoint 생성 경로가 없다. |
| Position Slow | `NAVIGATION_STATE_POSITION_SLOW` | PX4 공통 mode에는 있으나 `boat_control`은 `NAVIGATION_STATE_POSCTL`만 position manual mode로 처리한다. |
| Descend | `NAVIGATION_STATE_DESCEND` | 하강 모드이며 boat setpoint 생성 경로가 없다. |
| Termination | `NAVIGATION_STATE_TERMINATION` | 일반 flight control mode가 아니라 termination 상태다. |
| Auto Takeoff | `NAVIGATION_STATE_AUTO_TAKEOFF` | 이륙 개념이 boat에 맞지 않고 boat auto mode에서 처리하지 않는다. |
| Auto Land | `NAVIGATION_STATE_AUTO_LAND` | 착륙 개념이 boat에 맞지 않고 boat auto mode에서 처리하지 않는다. |
| Follow Target | `NAVIGATION_STATE_AUTO_FOLLOW_TARGET` | boat-specific follow-target setpoint 생성 경로가 없다. |
| Precision Land | `NAVIGATION_STATE_AUTO_PRECLAND` | precision landing 모드이며 boat setpoint 생성 경로가 없다. |
| Orbit | `NAVIGATION_STATE_ORBIT` | orbit mode는 PX4 공통 auto mode지만 현재 boat auto path에서 처리하지 않는다. |
| VTOL Takeoff | `NAVIGATION_STATE_AUTO_VTOL_TAKEOFF` | VTOL 전용 mode이며 boat에 해당하지 않는다. |
| External Modes | `NAVIGATION_STATE_EXTERNAL1` ~ `NAVIGATION_STATE_EXTERNAL8` | 외부 mode executor가 별도로 등록/구현해야 하며, 기본 `boat_control` 처리 대상이 아니다. |

## Manual 계열 Mode 비교

Manual, Acro, Stabilized, Position은 모두 조종자 stick 입력을 사용하지만, 자동제어가
개입하는 계층이 다르다.

| Mode | Throttle stick | Roll stick | 자동제어 개입 수준 | 조종감 요약 |
| --- | --- | --- | --- | --- |
| Manual | Signed thrust 직접 명령 | Steering 직접 명령 | 없음 | 조향과 추진을 조종자가 직접 제어한다. |
| Acro | Signed thrust 직접 명령 | Yaw-rate 명령 | Rate control | 조향각이 아니라 선회율을 명령한다. |
| Stabilized | Signed thrust 직접 명령 | Yaw-rate 명령 또는 heading hold | Rate/heading control | roll을 놓으면 현재 heading을 유지한다. |
| Position | Speed 명령 | Yaw-rate 명령 또는 course hold | Position, speed, heading/rate control | throttle은 속도 명령이 되고, roll을 놓으면 현재 방향으로 직선 주행을 유지한다. |

## 구현 기준

- Boat mode dispatch: `src/modules/boat_control/BoatControl.cpp`
- Manual/Acro/Stabilized/Position setpoint generation:
  `src/modules/boat_control/BoatDriveModes/BoatManualMode/BoatManualMode.cpp`
- Heading/yaw-rate to steering conversion:
  `src/modules/boat_control/BoatAttControl/BoatAttControl.cpp`
- Speed setpoint to signed thrust conversion:
  `src/modules/boat_control/BoatSpeedControl/BoatSpeedControl.cpp`
- Position setpoint to speed/heading setpoint conversion:
  `src/modules/boat_control/BoatPosControl/BoatPosControl.cpp`
