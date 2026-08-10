# Gazebo Boat Fix Plan

## 0. 재시작 체크포인트

이 섹션은 다음 작업을 다시 시작할 때 가장 먼저 확인할 요약이다. 아래 내용이 현재까지의 실험 결론과 바로 이어서 해야 할 일이다.

### 0.1 현재 확인된 상태

- 로컬 BadaSim 경로는 `/home/jeyong/projects/bada/BadaSim`이다. 사용자가 처음 언급한 `/home/jeyong/projects/bada/badasim`은 현재 파일시스템에는 없고, 실제 경로는 대문자 `BadaSim`이다.
- Aura visual asset은 `/home/jeyong/projects/bada/BadaSim/asset/Aura_low.usdc`에서 가져왔고, Gazebo 모델은 `Tools/simulation/gz_custom/models/boat/meshes/Aura_low.obj`를 visual mesh로 사용한다.
- 현재 `boat`는 무입력 상태에서 바로 침몰하거나 뒤집히지 않고 물 위에 떠 있는 baseline까지는 도달했다.
- 현재 world는 `BuoyancySystem`을 `basic_buoyancy=1`로 사용한다. `graded_buoyancy`는 rectangular hull에서 pitch torque가 커져 선미/선수 방향 뒤집힘을 유발했으므로 꺼 두었다.
- 현재 SDF 물리 선체는 Aura 실제 형상이 아니라 `3.7 x 0.50 x 0.90 m` 단일 rectangular collision이다.
- 현재 inertial은 `mass=850 kg`, `CoM z=-0.30`, `ixx=1100`, `iyy=5200`, `izz=4200`이다. `ixx`는 SDF inertia 유효성 조건 때문에 BadaSim 값 `900`에서 올렸다.
- 현재 visual mesh는 `pose 0 0 1.78 0 0 pi`로 놓아 AuraHull이 물리 box 위에 보이도록 맞췄다.
- 현재 추진 SDF는 rear propeller 2개를 갖지만, 두 propeller plugin 모두 `actuator_number=0`을 읽는다. 이는 조향 검증이 아니라, `Motor1` signed thrust 하나로 직진 추진이 되는지 확인하기 위한 임시 설정이다.
- 현재 `51001_gz_boat`는 Gazebo wheel output을 사용한다. wheel bridge는 최종 output에서 `100`을 빼서 `/model/boat_0/command/motor_speed`를 publish하므로 `SIM_GZ_WH_MIN=0`, `SIM_GZ_WH_MAX=200`, `SIM_GZ_WH_DIS=100`이 맞다.

### 0.2 현재 확인된 제어 메시지 경로

QGC virtual joystick에서 throttle을 올리면 PX4가 우선 받는 MAVLink 메시지는 `MANUAL_CONTROL`이다.

```text
QGC virtual joystick
-> MAVLink MANUAL_CONTROL.z
-> PX4 manual_control_setpoint.throttle
-> boat_control
-> actuator_motors.control[0]
-> Gazebo wheel bridge
-> /model/boat_0/command/motor_speed
-> GenericMotorModel actuator_number 0
```

PX4 내부 변환은 다음과 같다.

```text
MANUAL_CONTROL.z = 0     -> manual_control_setpoint.throttle = -1.0
MANUAL_CONTROL.z = 500   -> manual_control_setpoint.throttle =  0.0
MANUAL_CONTROL.z = 1000  -> manual_control_setpoint.throttle =  1.0
```

BadaSim이 실제 제어 입력으로 직접 수신하는 메시지는 QGC의 `MANUAL_CONTROL`이 아니라 PX4 simulator link의 `HIL_ACTUATOR_CONTROLS`이다.

```text
PX4 simulator_mavlink TCP 4560
-> BadaSim PX4SimulatorBridge
-> HIL_ACTUATOR_CONTROLS
-> PX4ActuatorMapper
-> ActuatorCommand
-> Aura dynamics
```

BadaSim의 최신 actuator contract는 다음과 같다.

```text
HIL_ACTUATOR_CONTROLS.mode        -> armed/disarmed 판단
HIL_ACTUATOR_CONTROLS.controls[0] -> steering / rudder
HIL_ACTUATOR_CONTROLS.controls[1] -> signed_thrust
```

관련 BadaSim 파일:

- `/home/jeyong/projects/bada/BadaSim/sim/vehicles/usv/px4_simulator_bridge.py`
- `/home/jeyong/projects/bada/BadaSim/sim/vehicles/usv/px4_actuator_mapping.py`
- `/home/jeyong/projects/bada/BadaSim/config/simulation_config.yaml`
- `/home/jeyong/projects/bada/BadaSim/PX4_LinkPlan.md`

따라서 Gazebo `boat`의 최종 목표는 BadaSim처럼 `single_engine_rudder` 계약으로 가는 것이 맞다. 다만 현재 Gazebo 1단계는 부유 안정성과 throttle 전달을 분리 확인하기 위해 조향 없는 직진 추진 baseline으로 둔다.

### 0.3 지금 바로 해야 할 확인 순서

현재 문제는 "Manual 모드에서 virtual joystick을 움직여도 boat가 움직이지 않는다"이다. 다음 순서로 어디서 끊기는지 확인한다.

1. PX4가 armed/manual 상태인지 확인한다.

```sh
commander status
commander mode manual
commander arm
```

arm이 preflight check로 막히는 순수 SITL 실험에서는 임시로 다음을 사용할 수 있다.

```sh
commander arm -f
```

2. QGC joystick 입력이 PX4에 들어오는지 확인한다.

```sh
listener manual_control_setpoint
```

throttle 스틱을 중앙보다 위로 올릴 때 다음이 보여야 한다.

```text
valid: True
throttle: 0.0보다 큰 값
data_source: MAVLINK 계열
```

3. `boat_control`이 actuator로 변환하는지 확인한다.

```sh
boat_control status
listener rover_throttle_setpoint
listener actuator_motors
```

기대값:

```text
rover_throttle_setpoint.throttle_body_x > 0
actuator_motors.control[0] > 0
```

4. Gazebo bridge가 motor speed topic을 publish하는지 확인한다.

```sh
gz topic -e -t /model/boat_0/command/motor_speed
```

기대값:

```text
velocity[0] > 0
```

현재 SDF에서는 좌우 propeller가 모두 `actuator_number=0`을 읽으므로 `velocity[0]`만 살아도 양쪽 propeller가 같은 명령을 받는다.

5. parameter가 수정 전 값으로 남아 있지 않은지 확인한다.

```sh
param show SIM_GZ_WH_FUNC1
param show SIM_GZ_WH_MIN1
param show SIM_GZ_WH_MAX1
param show SIM_GZ_WH_DIS1
```

기대값:

```text
SIM_GZ_WH_FUNC1 = 101
SIM_GZ_WH_MIN1 = 0
SIM_GZ_WH_MAX1 = 200
SIM_GZ_WH_DIS1 = 100
```

airframe 파일 수정 후에는 기존 SITL parameter가 남을 수 있으므로, 의심되면 `build/px4_sitl_default/rootfs`의 param state를 삭제하거나 `SYS_AUTOCONFIG`로 재초기화한 뒤 다시 실행한다.

### 0.4 다음 구현 TODO

가장 가까운 TODO는 다음 순서다.

1. 위 계측 순서로 joystick throttle이 `manual_control_setpoint`까지 들어오는지 확인한다.
2. `actuator_motors.control[0]`까지 값이 나오는데 `/model/boat_0/command/motor_speed`가 0이면 `SIM_GZ_WH_*` mapping과 `gz_bridge` wheel output path를 수정한다.
3. `/model/boat_0/command/motor_speed`가 양수인데 boat가 움직이지 않으면 `GenericMotorModel` thrust polynomial, propeller pose/axis, force 방향을 수정한다.
4. 직진 추진이 확인되면 임시 "두 propeller 모두 actuator 0" 설정을 끝내고 BadaSim 계약에 맞는 `single_engine_rudder` 구현으로 전환한다.
5. `single_engine_rudder` 전환 시 Gazebo에서는 `actuator_motors.control[0] = signed_thrust`, `actuator_servos.control[0] = steering`을 사용한다. BadaSim의 HIL 계약 `controls[0]=steering`, `controls[1]=signed_thrust`와 PX4 내부 uORB 계약이 서로 다르므로 이 차이를 명시적으로 변환해야 한다.
6. steering/rudder force는 generic `LiftDrag`보다 전용 plugin 또는 단순 custom force 계산으로 구현한다.
7. 이후에만 trapezoid hull 또는 sample point buoyancy로 확장한다.

## 1. 현재 문제 정의

PX4 Gazebo Harmonic용 `boat` 모델은 AuraHull 기반 visual mesh를 사용하도록 구성되었지만,
동역학 안정성은 아직 확보되지 않았다. 현재 관찰된 증상은 다음과 같다.

- 시뮬레이션 시작 직후 선체가 물 위에 안정적으로 떠 있지 못하고 가라앉는다.
- 선수 또는 선미가 급격히 내려가며 pitch 방향으로 고꾸라진다.
- 선미가 내려간 뒤 선체가 위로 솟거나, 선미 방향으로 계속 회전한다.
- 보트 좌표축, visual mesh pose, collision/buoyancy geometry, 추진축 사이의 기준이 완전히 정렬되었는지 확신하기 어렵다.
- Gazebo Harmonic의 generic buoyancy system에 큰 box collision을 넣어 부력을 만들고 있으나, 선체 실제 흘수와 복원 모멘트를 안정적으로 표현하지 못한다.

현재 설정의 핵심 한계는 부력을 collision volume에 의존한다는 점이다. AuraHull은 복잡한 선형을 가진 선체인데, 이를 몇 개의 box collision으로 근사하면 물에 잠기는 순간 중심 부력과 중심 질량의 상대 위치가 급격히 바뀐다. 이 때문에 pitch/roll torque가 과도하게 발생하고, 작은 pose 오차가 침몰 또는 회전 발산으로 이어진다.

또 하나의 문제는 추진/제어 계약의 불일치다. 현재 Harmonic 모델은 single propeller + rudder 구조에 가깝지만, 기존 Gazebo Classic boat airframe은 좌우 2개 모터의 differential thrust 구조였다. 반면 BadaSim의 Aura 설정은 최신 PX4 boat contract에 맞춰 steering + signed thrust, 즉 single engine + rudder 구조를 기준으로 정리되어 있다. 따라서 단순 SDF 수정만으로는 모델, PX4 actuator mapping, 동역학 모델이 서로 같은 가정을 공유하지 못한다.

## 2. 기존 Gazebo Classic 구현의 해결 방식

참고 대상은 PX4 Gazebo Classic PR 409이다.

- PR: https://github.com/PX4/PX4-SITL_gazebo-classic/pull/409
- boat SDF: https://github.com/PX4/PX4-SITL_gazebo-classic/blob/58615bfbe0365ee0247239d45e1d7c235ec320ad/models/boat/boat.sdf
- USV dynamics plugin: https://github.com/PX4/PX4-SITL_gazebo-classic/blob/58615bfbe0365ee0247239d45e1d7c235ec320ad/src/gazebo_usv_dynamics_plugin.cpp

Classic 구현은 Gazebo 기본 buoyancy에 선체 collision을 맡기는 방식이 아니었다. VRX의 WAM-V 모델에서 가져온 `usv_dynamics` 플러그인을 PX4 SITL에 맞게 수정했고, 다음과 같은 방식으로 안정성을 확보했다.

1. 선체 link 하나에 수동역학 힘을 직접 적용했다.
2. surge, sway, yaw 방향의 added mass와 damping을 별도로 계산했다.
3. heave, roll, pitch, yaw damping도 계수로 넣어 발산을 줄였다.
4. 부력은 선체 길이 방향 sample point를 나누어 계산했다.
5. 각 sample point에서 수면과 선체 위치의 차이를 계산하고, 잠긴 깊이에 해당하는 부력만 위쪽 force로 적용했다.
6. 부력 force는 선체 중심이 아니라 해당 sample point 위치에 적용하여 자연스러운 pitch/roll 복원 모멘트를 만들었다.
7. 추진은 좌우 propeller 2개를 사용하고, PX4 airframe도 `CA_ROTOR_COUNT=2` differential thrust에 맞췄다.

즉, Classic의 핵심은 "좋은 collision shape를 찾아서 buoyancy system에 맡긴 것"이 아니라 "보트 전용 dynamics plugin이 부력과 유체 저항을 통합적으로 책임진 것"이다.

Classic boat의 주요 수치도 이 방향을 보여준다.

- 질량: `227 kg`
- 좌우 hull 폭 기준: `boatWidth 2.4`
- 길이 기준: `boatLength 4.9`
- 선체 반경 근사: `hullRadius 0.213`
- 길이 방향 buoyancy sample: `length_n 2`
- 추진: 좌우 propeller 2개, motor 0/1

PX4 쪽 classic airframe도 같은 전제를 가진다.

- 파일: `ROMFS/px4fmu_common/init.d-posix/airframes/1070_gazebo-classic_boat`
- `CA_AIRFRAME=9`
- `CA_ROTOR_COUNT=2`
- 좌우 rotor 위치: `PX=-2`, `PY=-1/+1`
- main output 1/2를 motor 1/2에 연결

## 3. BadaSim/Aura에서 가져올 기준

현재 로컬 BadaSim 경로는 다음과 같다.

- `/home/jeyong/projects/bada/BadaSim`
- Aura USD asset: `/home/jeyong/projects/bada/BadaSim/asset/Aura_low.usdc`
- 원본 STEP: `/home/jeyong/projects/AuraHull.stp`
- 설정 파일: `/home/jeyong/projects/bada/BadaSim/config/simulation_config.yaml`
- Aura 동역학 코드: `/home/jeyong/projects/bada/BadaSim/sim/vehicles/usv/aura_model.py`

BadaSim Aura 설정에서 Gazebo 구현에 반영할 주요 값은 다음과 같다.

| 항목 | 값 |
| --- | ---: |
| length | `8.688735647623002 m` |
| beam | `2.6219928146972334 m` |
| height | `1.8679026917573774 m` |
| draft | `0.45 m` |
| visual waterline z | `1.72 m` |
| mass | `850 kg` |
| Ixx | `900 kg*m^2` |
| Iyy | `5200 kg*m^2` |
| Izz | `4200 kg*m^2` |
| center of buoyancy | `[0, 0, -0.15]` |
| water density | `1025 kg/m^3` |
| propulsion layout | `single_engine_rudder` |
| max engine thrust | `500 N` |
| max rudder angle | `30 deg` |
| stern offset | `-3.7 m` |
| reverse limit | `0.3` |

BadaSim의 3DOF hydrodynamics 계수는 다음과 같다.

| 계수 | 값 |
| --- | ---: |
| `x_u_dot` | `-120` |
| `y_v_dot` | `-650` |
| `n_r_dot` | `-900` |
| `x_u` | `-80` |
| `x_uu` | `-180` |
| `y_v` | `-350` |
| `y_vv` | `-900` |
| `n_r` | `-450` |
| `n_rr` | `-1100` |

이 값들은 Gazebo Harmonic의 초기 튜닝값으로 사용하되, 최종 검증은 무입력 부유 시험, 직진 step, 선회 step으로 다시 맞춰야 한다.

## 4. 제안 해결책

현재 우선순위는 최종적으로 정확한 AuraHull 유체역학을 재현하는 것이 아니라, Gazebo Harmonic에서 보트가 안정적으로 뜨고 PX4 제어 입력에 일관되게 반응하는 기준 모델을 만드는 것이다. 따라서 구현은 다음 두 단계로 먼저 진행한다.

1. `simple rectangular hull`: 직육면체 물리 선체로 부유/축/추진/제어 계약을 안정화한다.
2. `trapezoid or sampled hull`: 아래가 좁아지는 보트형 단면 또는 sample 기반 부력으로 실제 선체 특성에 한 단계 접근한다.

직육면체 hull은 선수파, 활주, 속도별 저항 변화, roll 복원력, 파도/접안/고속 선회 정확도에서는 부족하다. 그러나 이 한계 때문에 오히려 1차 디버깅에는 적합하다. 복잡한 선형 효과를 걷어내고, 좌표축, 질량, 관성, 수면 높이, actuator 방향, Gazebo force 적용 방향을 먼저 검증할 수 있기 때문이다.

### 4.1 1단계 목표: Simple Rectangular Hull 기준 모델

1단계에서는 Aura visual mesh는 유지하되, 물리 모델은 단순한 직육면체 hull 하나로 둔다.

목표:

- 보트가 시작 직후 침몰하지 않는다.
- 무입력 상태에서 60초 이상 roll/pitch가 발산하지 않는다.
- `base_link +X`가 선수 방향인지 확인한다.
- `base_link +Z`와 Gazebo world Z 방향 관계를 확인한다.
- PX4 actuator command가 기대한 추진 방향과 yaw 방향을 만든다.
- 이후 trapezoid/sample hull로 바꾸기 전의 안정적인 baseline을 만든다.

1단계에서는 실제 선체 형상 재현보다 "잘 뜨는 단순 물리 모델"을 우선한다.

### 4.2 1단계 SDF 구성

`Tools/simulation/gz_custom/models/boat/model.sdf`는 다음 원칙으로 정리한다.

- `aura_hull_visual`은 유지한다.
- collision 이름에서 `buoyancy` 의미를 제거하고 `rectangular_hull_collision`로 바꾼다.
- 물리 collision은 하나의 box로 시작한다.
- box 크기는 Aura/BadaSim 치수에서 시작한다.
- world의 generic buoyancy는 일단 사용할 수 있지만, 적용 대상을 명확히 하기 위해 collision을 하나로 단순화한다.
- 기존 여러 buoyancy box, rudder lift-drag, hydrodynamics plugin은 1차 안정화 중에는 제거하거나 비활성화한다.

초기 box 치수:

| 항목 | 값 | 설명 |
| --- | ---: | --- |
| length | `8.6887 m` | Aura 전체 길이 |
| beam | `2.6220 m` | Aura 폭 |
| draft target | `0.45 m` | 목표 흘수 |
| mass | `850 kg` | BadaSim 기준 질량 |
| water density | `1025 kg/m^3` | 해수 밀도 |

단순 직육면체 전체 폭/길이를 그대로 쓰면 필요한 잠김 높이는 다음과 같이 매우 작아진다.

```text
required_displacement_volume = mass / water_density
                             = 850 / 1025
                             = 0.829 m^3

submerged_height = required_displacement_volume / (length * beam)
                 = 0.829 / (8.6887 * 2.6220)
                 = 약 0.036 m
```

즉 `8.6887 x 2.6220` 전체 바닥 면적의 직육면체를 그대로 쓰면 3.6 cm만 잠겨도 부력이 맞는다. 이는 실제 draft `0.45 m`와 맞지 않고, 작은 z 오차에도 부력 변화가 급격해진다. 따라서 1단계 물리 box는 전체 선체 외형이 아니라 "유효 배수 면적"을 가진 단순 box로 잡는 것이 낫다.

권장 초기 collision box:

```xml
<collision name="rectangular_hull_collision">
  <pose>0 0 -0.225 0 0 0</pose>
  <geometry>
    <box>
      <size>3.7 0.50 0.90</size>
    </box>
  </geometry>
</collision>
```

이 box는 `3.7 * 0.50 * 0.45 = 0.8325 m^3`의 잠김 부피를 만들어 `850 kg` 보트에 가까운 정지 부력을 낸다. 실제 외형과는 다르지만, Gazebo generic buoyancy로 정지 부유 평형을 맞추기 쉽다.

초기 inertial 값:

```xml
<inertial>
  <pose>0 0 -0.10 0 0 0</pose>
  <mass>850.0</mass>
  <inertia>
    <ixx>900.0</ixx>
    <ixy>0</ixy>
    <ixz>0</ixz>
    <iyy>5200.0</iyy>
    <iyz>0</iyz>
    <izz>4200.0</izz>
  </inertia>
</inertial>
```

중심 질량은 처음에는 `z=-0.10` 정도로 낮게 둔다. 그래도 roll/pitch가 불안정하면 `z=-0.30`까지 낮춰 복원성을 확보한다. 단, 최종 Aura 물성으로 갈 때는 이 값을 다시 검토한다.

### 4.3 1단계 world 설정

`Tools/simulation/gz_custom/worlds/boat.sdf`는 다음처럼 단순화한다.

- `BuoyancySystem`은 먼저 `basic_buoyancy`로 둔다.
- `graded_buoyancy`는 rectangular hull이 무입력에서 뒤집히지 않는 것을 확인한 뒤 다시 도입한다.
- 해수면은 `z=0`으로 고정한다.
- 물 표면 visual plane도 `z=0`으로 둔다.
- `max_step_size`는 `0.002` 또는 `0.004`로 유지한다.
- generic hydrodynamics system은 model에서 제거하여 force source를 줄인다.

초기 검증 중에는 수면 절단에 의한 pitch torque를 만들지 않는다. 현재 증상처럼 선미가 가라앉으며 뒤집히는 경우에는 rectangular box의 graded buoyancy가 과도한 복원/반발 토크를 만들 수 있으므로, 1단계에서는 질량 보상형 basic buoyancy와 낮은 중심 질량으로 roll/pitch 안정성부터 확인한다.

### 4.4 1단계 추진/제어 구조

1단계에서는 추진 구조를 더 단순하게 둔다.

현재 임시 구현 상태:

- rear propeller 2개는 유지한다.
- 두 propeller plugin 모두 `actuator_number=0`을 읽는다.
- PX4 `boat_control`이 publish하는 `actuator_motors.control[0]` 하나로 양쪽 propeller를 동시에 구동한다.
- 이 설정은 조향/선회 검증용이 아니라 virtual joystick throttle이 실제 Gazebo thrust로 이어지는지 확인하기 위한 직진 baseline이다.
- QGC virtual joystick에서 좌우 스틱만 움직이면 현재 단계에서는 움직이지 않는 것이 정상이다. rudder와 differential thrust가 아직 검증 경로에 들어와 있지 않기 때문이다.
- throttle 스틱을 중앙보다 위로 올렸을 때만 전진 추진을 기대한다.

추천안:

- single propeller + rudder를 유지하지 않는다.
- Classic처럼 좌우 2개 propeller differential thrust로 시작한다.
- rudder와 `LiftDrag`는 제거한다.

이유:

- 정지 부유 안정화 중 rudder side force가 roll/pitch 문제를 만들 가능성을 없앤다.
- PX4 classic boat airframe과 구조가 같아 검증하기 쉽다.
- 좌우 thrust 부호만 확인하면 yaw 방향을 명확히 잡을 수 있다.

SDF 배치:

```xml
<link name="left_propeller_link">
  <pose relative_to="base_link">-3.7 0.9 -0.15 0 1.570796 0</pose>
</link>

<link name="right_propeller_link">
  <pose relative_to="base_link">-3.7 -0.9 -0.15 0 1.570796 0</pose>
</link>
```

PX4 airframe 방향:

```sh
param set-default CA_AIRFRAME 9
param set-default CA_ROTOR_COUNT 2
param set-default CA_ROTOR0_AX 1
param set-default CA_ROTOR0_PX -3.7
param set-default CA_ROTOR0_PY -0.9
param set-default CA_ROTOR1_AX 1
param set-default CA_ROTOR1_PX -3.7
param set-default CA_ROTOR1_PY 0.9
```

좌우 부호는 Gazebo에서 실제 yaw 방향을 보고 확정한다.

단, 위 추천안의 최종 differential thrust로 바로 가지 않고, 현재는 한 단계 더 단순한 "두 propeller 모두 Motor1" 설정을 먼저 둔다. 이유는 PX4 최신 `boat_control`이 manual 모드에서 `actuator_motors.control[0]`와 `actuator_servos.control[0]`를 직접 내보내며, `actuator_motors.control[1]`을 항상 생성하는 구조가 아니기 때문이다. 직진 추진이 확인된 뒤 다음 중 하나를 선택한다.

1. BadaSim 최종 계약에 맞춰 `single_engine_rudder`로 전환한다.
2. Classic식 `twin motor differential thrust`를 유지하려면 PX4 쪽에서 좌우 motor output을 생성하는 별도 allocation/control path를 명확히 만든다.

### 4.5 1단계 완료 기준

1단계는 다음을 만족하면 완료로 본다.

- `make px4_sitl gz_boat` 실행 시 시작 직후 침몰하지 않는다.
- 무입력 상태에서 60초 이상 z, roll, pitch가 제한된 범위 안에 머문다.
- visual AuraHull과 물리 box가 크게 어긋나 보이지 않는다.
- 좌우 동일 thrust에서 직진한다.
- 좌우 차등 thrust에서 yaw가 발생한다.
- thrust/yaw 부호가 PX4 boat control과 일치한다.

1단계에서 얻을 산출물:

- 안정적인 rectangular hull `model.sdf`
- 최소 buoyancy world 설정
- twin motor airframe mapping
- README smoke test 절차

### 4.6 2단계 목표: Trapezoid 또는 Sampled Hull 근사

2단계에서는 rectangular hull의 인위적인 안정성을 줄이고, 아래가 좁아지는 일반적인 보트 단면에 접근한다.

선택지는 두 가지다.

1. 여러 box를 조합한 trapezoid hull
2. collision은 단순하게 두고, sample point 기반 부력 plugin을 도입

우선순위는 다음과 같이 잡는다.

- 빠른 구현: 여러 box 조합 trapezoid hull
- 안정성과 튜닝성: sample point 기반 부력 plugin

### 4.7 2단계 A안: 여러 Box를 조합한 Trapezoid Hull

Gazebo SDF 기본 box collision만으로 진짜 사다리꼴 collision을 직접 만들기는 어렵다. 대신 z 높이별로 폭이 다른 box를 쌓아 근사한다.

예시:

```xml
<collision name="hull_bottom_collision">
  <pose>0 0 -0.38 0 0 0</pose>
  <geometry>
    <box>
      <size>3.5 0.30 0.20</size>
    </box>
  </geometry>
</collision>

<collision name="hull_middle_collision">
  <pose>0 0 -0.18 0 0 0</pose>
  <geometry>
    <box>
      <size>3.9 0.55 0.20</size>
    </box>
  </geometry>
</collision>

<collision name="hull_upper_collision">
  <pose>0 0 0.02 0 0 0</pose>
  <geometry>
    <box>
      <size>4.2 0.80 0.20</size>
    </box>
  </geometry>
</collision>
```

이 방식은 아래로 갈수록 폭이 좁아지는 단면을 근사한다.

장점:

- 새 C++ plugin 없이 SDF만으로 시도 가능하다.
- rectangular hull보다 roll/pitch 응답이 보트에 가까워진다.
- 단계별 box 부피를 계산해 정지 부력을 맞출 수 있다.

단점:

- box 경계에서 부력 중심이 계단식으로 변할 수 있다.
- 파도나 큰 자세 변화에서 부자연스러운 힘 변화가 생길 수 있다.
- 정확한 AuraHull 형상 재현은 아니다.

2단계 A안의 완료 기준은 1단계 완료 기준을 유지하면서 roll/pitch 응답이 지나치게 둔하지 않은지 확인하는 것이다.

### 4.8 2단계 B안: Sample Point 기반 부력

2단계 B안은 Classic 구현과 같은 철학으로 간다. collision geometry는 접촉용으로만 두고, 부력은 sample point에서 직접 계산한다.

초기 sample 설정:

| 항목 | 값 |
| --- | ---: |
| length samples | `8` 또는 `10` |
| beam samples | `2` |
| x range | `+- length * 0.45` |
| y offset | `+- beam * 0.25` 또는 `+- beam * 0.35` |
| draft | `0.45 m` |
| buoyancy scale | `1.0` |

계산 방식:

```cpp
submerged_depth = clamp(water_level - sample_world_z, 0, draft)
submerged_ratio = submerged_depth / draft
sample_force = mass * gravity * sample_weight * submerged_ratio
```

각 sample force는 world +Z 방향으로 적용하고, sample 위치와 `base_link` 중심의 lever arm으로 torque를 만든다.

장점:

- Classic boat 구현과 구조적으로 비슷하다.
- pitch/roll 복원력을 sample 위치로 직접 튜닝할 수 있다.
- collision shape와 buoyancy 계산을 분리할 수 있다.
- 추후 AuraHull 단면 table로 확장하기 쉽다.

단점:

- C++ Gazebo Harmonic plugin 구현이 필요하다.
- damping, added mass, propulsion force와의 중복을 잘 관리해야 한다.

2단계 B안은 1단계와 2단계 A안이 충분하지 않을 때 시작하거나, 초기부터 plugin 구현 시간이 허용될 때 진행한다.

### 4.9 2단계 완료 기준

2단계는 다음을 만족하면 완료로 본다.

- rectangular hull보다 실제 보트처럼 roll/pitch 응답이 나타난다.
- 무입력 상태에서 안정성은 유지된다.
- 전진 중 선수/선미가 지속적으로 가라앉지 않는다.
- 회전 중 과도한 roll이나 pitch 발산이 없다.
- tuning parameter를 SDF에서 조정할 수 있다.

### 4.10 단계별 구현 우선순위

우선 구현 순서는 다음과 같이 바꾼다.

1. rectangular hull collision 하나로 `model.sdf` 단순화
2. world buoyancy를 `basic_buoyancy`로 단순화
3. rudder/lift-drag/hydrodynamics force source 제거
4. twin motor differential thrust로 propulsion 단순화
5. `51001_gz_boat` airframe을 twin motor mapping에 맞춤
6. 무입력 부유 시험
7. 동일 thrust 직진 시험
8. 차등 thrust yaw 시험
9. 여러 box 기반 trapezoid hull 시도
10. 필요 시 sample point buoyancy plugin으로 확장

이 접근은 "정밀 모델을 먼저 만들고 안정화"가 아니라 "안정적인 baseline을 먼저 만들고 정밀도를 올리는" 방식이다.

## 5. 이외 제안 해결책

### 5.1 방향 전환

현재 방식인 `gz::sim::systems::BuoyancySystem` + box collision 기반 부력 조정은 중단한다. Aura boat에는 Classic 방식처럼 전용 USV dynamics plugin을 둔다.

새 플러그인의 책임은 다음과 같다.

- `base_link`의 world pose, body velocity, angular velocity를 읽는다.
- body frame에서 surge/sway/yaw added mass와 damping force를 계산한다.
- heave/roll/pitch 안정화를 위한 damping 또는 hydrostatic restoring 항을 계산한다.
- Aura 선체 길이 방향 sample point에서 부력을 계산한다.
- 각 sample point에 world upward force와 그에 따른 torque를 적용한다.
- collision geometry는 부력 계산용이 아니라 접촉 안정성용으로만 사용한다.

후보 이름:

- `AuraUsvDynamics`
- `GzUsvDynamics`
- `BoatDynamicsSystem`

후보 위치:

- `src/modules/simulation/gz_plugins/aura_usv_dynamics/`

### 5.2 부력 모델

초기 구현은 Classic의 sample 기반 방식을 Aura 단동선에 맞게 바꾼다.

권장 초기 모델:

- 길이 방향 sample 수: `8` 또는 `10`
- 좌우 방향 sample 수: `2`
- 좌우 sample offset: `+- beam * 0.25` 또는 `+- beam * 0.35`
- sample x 범위: `[-length * 0.45, length * 0.45]`
- 수면: `water_level = 0`
- 각 sample point에서 `submerged_depth = water_level - point_world_z`
- `submerged_depth`를 `0..draft` 범위로 clamp
- sample당 부력은 전체 중량을 기준으로 균등 분배하되, 잠긴 깊이에 따라 선형 또는 원형 단면 근사로 scale

처음부터 정확한 선형 부피를 맞추기보다, 정지 상태에서 다음 조건을 만족하는 것이 중요하다.

- 총 부력 평균이 `mass * gravity`와 거의 같다.
- 중심 부력이 중심 질량보다 약간 아래 또는 수면 방향으로 배치되어 pitch/roll 복원력이 생긴다.
- 선수/선미 sample이 모두 있어 pitch 방향 복원 모멘트가 생긴다.
- 좌/우 sample이 모두 있어 roll 방향 복원 모멘트가 생긴다.

### 5.3 유체 저항 모델

Gazebo의 generic hydrodynamics system을 계속 쓸 수도 있지만, 초기 안정화 단계에서는 전용 plugin 안에서 Classic과 같은 방식으로 force를 한 곳에서 계산하는 것이 더 낫다.

초기 반영 항목:

- surge: `x_u`, `x_uu`
- sway: `y_v`, `y_vv`
- yaw: `n_r`, `n_rr`
- heave: `z_w`
- roll: `k_p`
- pitch: `m_q`

BadaSim은 주로 3DOF maneuvering 모델이므로 heave/roll/pitch는 Gazebo에서 별도 안정화 계수로 시작한다. 이후 필요하면 BadaSim `wave_response`의 `heave_stiffness`, `heave_damping`, `roll_gain`, `pitch_gain`을 참고해 확장한다.

### 5.4 추진 구조

구현은 두 단계로 나눈다.

1차 안정화:

- Classic과 같은 twin motor differential thrust 구조를 사용한다.
- 좌우 propeller link를 `x=-3.7`, `y=+-0.9`, `z=-0.15` 근처에 둔다.
- PX4 airframe은 `CA_ROTOR_COUNT=2`로 맞춘다.
- rudder와 lift-drag plugin은 일단 제거하거나 비활성화한다.

이유:

- Classic 구현과 제어 구조가 같다.
- yaw 제어가 좌우 thrust 차이로 생기므로 rudder side force에 의한 roll/pitch 교란을 줄일 수 있다.
- 부력 안정화와 조향 모델 검증을 분리할 수 있다.

2차 확장:

- BadaSim의 최신 계약인 `steering + signed_thrust`로 되돌린다.
- single engine + rudder 구조를 별도 mode로 추가한다.
- rudder force는 generic `LiftDrag`보다 전용 plugin 안에서 수중 rudder force로 계산하는 편이 낫다.

### 5.5 구현 단위

구현은 다음 파일 단위로 나눈다.

| 파일 | 작업 |
| --- | --- |
| `src/modules/simulation/gz_plugins/aura_usv_dynamics/AuraUsvDynamics.hpp` | plugin class, parameter struct, sample point struct 정의 |
| `src/modules/simulation/gz_plugins/aura_usv_dynamics/AuraUsvDynamics.cpp` | Gazebo Harmonic `System` plugin 구현 |
| `src/modules/simulation/gz_plugins/aura_usv_dynamics/CMakeLists.txt` | plugin library target 추가 |
| `src/modules/simulation/gz_plugins/CMakeLists.txt` | `add_subdirectory(aura_usv_dynamics)` 및 aggregate target dependency 추가 |
| `Tools/simulation/gz_custom/models/boat/model.sdf` | 새 plugin SDF 설정, collision/propulsion 정리 |
| `Tools/simulation/gz_custom/worlds/boat.sdf` | generic buoyancy plugin 제거 또는 비활성화 |
| `ROMFS/px4fmu_common/init.d-posix/airframes/51001_gz_boat` | actuator mapping을 선택한 propulsion layout과 일치시킴 |
| `Tools/simulation/gz_custom/README.md` | 실행 방법, tuning parameter, 검증 절차 정리 |

초기 구현에서는 새 plugin 이름을 `AuraUsvDynamics`로 둔다. 이 이름은 Aura 전용 파라미터를 기본값으로 넣기 좋고, 이후 다른 USV로 일반화할 때 `BoatDynamicsSystem` 같은 이름으로 바꿀 수 있다.

### 5.6 Plugin SDF 인터페이스

`model.sdf`에는 다음 형태의 plugin 설정을 추가한다.

```xml
<plugin filename="libAuraUsvDynamicsPlugin.so" name="px4::gz::sim::systems::AuraUsvDynamics">
  <link_name>base_link</link_name>

  <water_level>0.0</water_level>
  <water_density>1025.0</water_density>
  <gravity>9.8</gravity>

  <mass>850.0</mass>
  <boat_length>8.688735647623002</boat_length>
  <boat_beam>2.6219928146972334</boat_beam>
  <draft>0.45</draft>
  <center_of_buoyancy>0 0 -0.15</center_of_buoyancy>

  <length_samples>10</length_samples>
  <beam_samples>2</beam_samples>
  <sample_x_fraction>0.45</sample_x_fraction>
  <sample_y_fraction>0.35</sample_y_fraction>

  <x_u_dot>-120.0</x_u_dot>
  <y_v_dot>-650.0</y_v_dot>
  <n_r_dot>-900.0</n_r_dot>
  <x_u>-80.0</x_u>
  <x_uu>-180.0</x_uu>
  <y_v>-350.0</y_v>
  <y_vv>-900.0</y_vv>
  <n_r>-450.0</n_r>
  <n_rr>-1100.0</n_rr>

  <z_w>-250.0</z_w>
  <k_p>-600.0</k_p>
  <m_q>-2200.0</m_q>

  <debug>false</debug>
</plugin>
```

초기에는 파라미터 수가 많아 보이더라도 SDF에서 조정 가능하게 두는 편이 좋다. 부력과 damping은 첫 실행에서 바로 맞기 어렵기 때문에, 재빌드 없이 값을 바꿔가며 안정화할 수 있어야 한다.

### 5.7 Plugin 내부 데이터 구조

header에는 대략 다음 구조를 둔다.

```cpp
struct AuraUsvDynamicsParams
{
	double water_level{0.0};
	double water_density{1025.0};
	double gravity{9.8};
	double mass{850.0};

	double boat_length{8.688735647623002};
	double boat_beam{2.6219928146972334};
	double draft{0.45};
	gz::math::Vector3d center_of_buoyancy{0.0, 0.0, -0.15};

	int length_samples{10};
	int beam_samples{2};
	double sample_x_fraction{0.45};
	double sample_y_fraction{0.35};

	double x_u_dot{-120.0};
	double y_v_dot{-650.0};
	double n_r_dot{-900.0};
	double x_u{-80.0};
	double x_uu{-180.0};
	double y_v{-350.0};
	double y_vv{-900.0};
	double n_r{-450.0};
	double n_rr{-1100.0};

	double z_w{-250.0};
	double k_p{-600.0};
	double m_q{-2200.0};
};

struct BuoyancySample
{
	gz::math::Vector3d body_position;
	double weight_fraction{0.0};
};
```

plugin class는 `ISystemConfigure`와 `ISystemPreUpdate`를 구현한다.

```cpp
class AuraUsvDynamics:
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{
public:
	void Configure(
		const gz::sim::Entity &_entity,
		const std::shared_ptr<const sdf::Element> &_sdf,
		gz::sim::EntityComponentManager &_ecm,
		gz::sim::EventManager &_eventMgr) override;

	void PreUpdate(
		const gz::sim::UpdateInfo &_info,
		gz::sim::EntityComponentManager &_ecm) override;
};
```

### 5.8 Force 계산 순서

`PreUpdate()`의 계산 순서는 다음과 같이 고정한다.

1. paused 상태이면 return
2. `base_link`의 world pose, world linear velocity, world angular velocity를 읽음
3. world velocity를 body frame으로 변환
4. sample point별 buoyancy force 계산
5. body frame damping force/torque 계산
6. damping을 world frame으로 변환
7. 모든 force/torque를 합산하여 `Link::AddWorldWrench()`로 적용
8. debug mode에서는 총 부력, damping force, roll/pitch/yaw 값을 주기적으로 출력

의사코드는 다음과 같다.

```cpp
const auto pose = link.WorldPose(ecm);
const auto world_linear_velocity = link.WorldLinearVelocity(ecm);
const auto world_angular_velocity = link.WorldAngularVelocity(ecm);

const auto body_linear_velocity =
	pose->Rot().Inverse().RotateVector(*world_linear_velocity);
const auto body_angular_velocity =
	pose->Rot().Inverse().RotateVector(*world_angular_velocity);

gz::math::Vector3d total_force_world{0, 0, 0};
gz::math::Vector3d total_torque_world{0, 0, 0};

ApplySampledBuoyancy(*pose, total_force_world, total_torque_world);
ApplyHydrodynamicDamping(
	*pose,
	body_linear_velocity,
	body_angular_velocity,
	total_force_world,
	total_torque_world);

link.AddWorldWrench(ecm, total_force_world, total_torque_world);
```

### 5.9 Sample 기반 부력 계산

Classic 구현의 핵심은 `water_level - sample_world_z`로 각 sample point의 잠김 정도를 계산한 뒤, 해당 지점에 upward force를 적용하는 것이다. Aura 초기 구현도 이 구조를 따른다.

```cpp
for (const auto &sample : samples) {
	const auto sample_world_position =
		pose.Pos() + pose.Rot().RotateVector(sample.body_position);

	const double submerged_depth =
		std::clamp(params.water_level - sample_world_position.Z(), 0.0, params.draft);

	const double submerged_ratio = submerged_depth / params.draft;
	const double force_n =
		params.mass * params.gravity * sample.weight_fraction * submerged_ratio;

	const gz::math::Vector3d force_world{0.0, 0.0, force_n};
	const gz::math::Vector3d r_world = sample_world_position - pose.Pos();
	const gz::math::Vector3d torque_world = r_world.Cross(force_world);

	total_force_world += force_world;
	total_torque_world += torque_world;
}
```

초기에는 `submerged_ratio`를 선형으로 둔다. 정지 부유가 안정화된 뒤에는 Classic처럼 원형 단면 면적 근사 또는 Aura 선체 단면 table로 바꿀 수 있다.

정지 상태에서 총 부력이 부족하면 보트는 바로 가라앉고, 너무 크면 튀어 오른다. 따라서 초기 안정화를 위해 `buoyancy_scale` 파라미터를 하나 더 두는 것도 좋다.

```xml
<buoyancy_scale>1.0</buoyancy_scale>
```

튜닝 순서는 다음과 같다.

1. `buoyancy_scale`로 z 방향 평형을 맞춘다.
2. `center_of_buoyancy` 또는 sample z offset으로 roll/pitch 복원력을 맞춘다.
3. `length_samples`, `sample_x_fraction`으로 선수/선미 pitch 민감도를 맞춘다.
4. `sample_y_fraction`으로 roll 안정성을 맞춘다.

### 5.10 Hydrodynamic damping 계산

초기 damping은 BadaSim 3DOF 계수를 그대로 사용한다.

```cpp
const double u = body_linear_velocity.X();
const double v = body_linear_velocity.Y();
const double w = body_linear_velocity.Z();
const double p = body_angular_velocity.X();
const double q = body_angular_velocity.Y();
const double r = body_angular_velocity.Z();

const double fx = params.x_u * u + params.x_uu * std::abs(u) * u;
const double fy = params.y_v * v + params.y_vv * std::abs(v) * v;
const double fz = params.z_w * w;

const double tx = params.k_p * p;
const double ty = params.m_q * q;
const double tz = params.n_r * r + params.n_rr * std::abs(r) * r;
```

그 다음 body frame force/torque를 world frame으로 변환한다.

```cpp
const gz::math::Vector3d damping_force_world =
	pose.Rot().RotateVector({fx, fy, fz});
const gz::math::Vector3d damping_torque_world =
	pose.Rot().RotateVector({tx, ty, tz});
```

주의할 점은 부호다. BadaSim 계수는 이미 음수 damping 계수로 저장되어 있으므로 위 식에서는 그대로 곱한다. 만약 SDF에서 양수 계수로 넣는 방식을 선택하면 코드에서 `-coefficient * velocity` 형태로 통일해야 한다. 혼동을 막기 위해 초기 구현은 BadaSim과 같은 음수 계수 convention을 따른다.

### 5.11 Added mass 처리

Classic plugin은 속도 변화량을 이용해 added mass force를 계산했다. Harmonic 초기 버전에서는 두 단계로 접근한다.

1차 구현:

- added mass 항은 SDF에 파라미터로 받되, force 계산에서는 끈다.
- 이유는 부력 안정화와 damping 안정화를 먼저 검증하기 위해서다.

2차 구현:

- 이전 step의 body velocity를 저장한다.
- `body_accel = (body_velocity - previous_body_velocity) / dt`를 계산한다.
- `x_u_dot`, `y_v_dot`, `n_r_dot` 기반 added mass force를 추가한다.

```cpp
const double fx_added = params.x_u_dot * u_dot;
const double fy_added = params.y_v_dot * v_dot;
const double tz_added = params.n_r_dot * r_dot;
```

added mass를 너무 일찍 넣으면 현재 문제의 원인인 pitch/roll 발산과 별개의 수치 진동이 섞인다. 따라서 부유 안정화가 완료된 뒤 켜는 것이 좋다.

### 5.12 SDF 정리 원칙

`boat/model.sdf`는 다음 원칙으로 정리한다.

- Aura visual mesh는 유지한다.
- visual pose는 BadaSim의 `visual_yaw_offset_deg: 180.0`을 기준으로 `base_link +X = 선수 방향`이 되도록 고정한다.
- collision은 부력용이 아니라 접촉용으로만 사용한다.
- collision은 가능한 한 낮고 단순한 box/capsule/cylinder 조합으로 둔다.
- 현재 `port_buoyancy_collision`, `starboard_buoyancy_collision`, `bow_buoyancy_collision` 이름은 제거하거나 `*_contact_collision`으로 의미를 바꾼다.
- world 파일에서 `libBuoyancySystemPlugin.so`는 제거한다.
- `gz-sim-hydrodynamics-system`은 새 plugin과 중복되므로 제거한다.
- rudder `LiftDrag`는 1차 구현에서는 제거한다.

이렇게 하면 힘을 적용하는 주체가 하나로 모인다. 부력은 buoyancy plugin, damping은 hydrodynamics plugin, rudder는 lift-drag plugin, motor는 generic motor plugin처럼 나뉘면 문제 발생 시 torque 원인을 추적하기 어렵다.

### 5.13 1차 propulsion 구현

1차 구현은 Classic airframe과 같은 좌우 propeller 2개를 둔다.

SDF 구조:

```xml
<link name="left_propeller_link">
  <pose relative_to="base_link">-3.7 0.9 -0.15 0 1.570796 0</pose>
</link>

<link name="right_propeller_link">
  <pose relative_to="base_link">-3.7 -0.9 -0.15 0 1.570796 0</pose>
</link>

<joint name="left_propeller_joint" type="revolute">
  <parent>base_link</parent>
  <child>left_propeller_link</child>
  <axis>
    <xyz>1 0 0</xyz>
  </axis>
</joint>

<joint name="right_propeller_joint" type="revolute">
  <parent>base_link</parent>
  <child>right_propeller_link</child>
  <axis>
    <xyz>1 0 0</xyz>
  </axis>
</joint>
```

motor plugin은 `actuator_number` 0/1로 나누어 설정한다. `motorConstant`는 초기에는 Classic 값보다 Aura 추력에 맞춰 낮게 시작하고, BadaSim의 `max_thrust_per_thruster_n: 250` 기준으로 full command에서 한쪽 250 N 근처가 되도록 맞춘다.

PX4 airframe은 다음과 같은 방향으로 정리한다.

```sh
param set-default CA_AIRFRAME 9
param set-default CA_ROTOR_COUNT 2
param set-default CA_ROTOR0_AX 1
param set-default CA_ROTOR0_PX -3.7
param set-default CA_ROTOR0_PY -0.9
param set-default CA_ROTOR1_AX 1
param set-default CA_ROTOR1_PX -3.7
param set-default CA_ROTOR1_PY 0.9
```

좌우 부호는 실제 yaw 방향 시험으로 최종 확정한다. 문서상 좌/우 정의와 Gazebo visual 좌표가 어긋날 수 있으므로, 이 단계에서는 부호 확인을 필수 검증 항목으로 둔다.

### 5.14 2차 propulsion 구현

1차 안정화 후에는 BadaSim과 같은 `single_engine_rudder` mode를 추가한다.

이때는 다음을 목표로 한다.

- PX4 actuator contract: `controls[0] = steering`, `controls[1] = signed_thrust`
- engine thrust: `signed_thrust * max_engine_thrust_n`
- reverse thrust clamp: `-reverse_limit * max_engine_thrust_n`
- rudder angle: `steering * max_rudder_angle`
- rudder force: water-relative velocity와 rudder angle로 직접 계산

rudder force는 다음 형태로 단순 시작할 수 있다.

```cpp
const double rudder_angle = steering_cmd * max_rudder_angle_rad;
const double speed = std::max(0.0, u);
const double side_force =
	rudder_coefficient * speed * speed * std::sin(rudder_angle);
```

force 적용 지점은 BadaSim 설정의 `stern_offset_m = -3.7`, `z = -0.15`를 사용한다. 단, 2차 구현은 1차 부력 안정성이 확보된 뒤 진행한다.

### 5.15 디버그와 계측

초기 plugin에는 debug 출력을 반드시 넣는다. 단, 매 step 출력하면 로그가 폭주하므로 1 Hz 정도로 제한한다.

출력 항목:

- world pose: z, roll, pitch, yaw
- body velocity: u, v, w, p, q, r
- total buoyancy force
- total damping force/torque
- sample별 submerged ratio의 min/max
- final applied force/torque

추가로 필요하면 Gazebo transport topic으로 debug message를 publish한다. 첫 구현에서는 `gzdbg` 또는 throttled `std::cout`만으로 충분하다.

### 5.16 구현 우선순위

구현 순서는 다음과 같이 잡는다.

1. 새 plugin skeleton 생성 및 CMake 연결
2. SDF 파라미터 parsing
3. `base_link` pose/velocity 읽기
4. damping 없이 sample buoyancy만 적용
5. 무입력 부유 안정화
6. heave/roll/pitch damping 추가
7. surge/sway/yaw damping 추가
8. twin motor propulsion 정리
9. PX4 actuator mapping 정리
10. added mass 추가
11. single engine + rudder mode 확장

이 순서를 지키는 이유는 문제를 한 번에 여러 물리 효과가 섞인 상태로 보지 않기 위해서다. 특히 현재 증상은 정지 부유 단계에서 이미 발생하므로, propulsion과 added mass는 뒤로 미룬다.

### 5.17 첫 번째 PR 또는 commit 범위

첫 구현 commit은 작게 나눈다.

1. `gazebo_fix_plan.md` 문서화
2. `AuraUsvDynamics` plugin skeleton + build 통과
3. sample buoyancy만 적용한 model/world 변경
4. damping 계수 추가
5. twin motor SDF/airframe 정렬
6. README와 smoke test 절차 추가

각 commit은 독립적으로 build 가능한 상태를 목표로 한다. 단, 물리 동작이 완전히 안정화되는 것은 3번 이후부터 기대한다.

## 6. 실행 계획

### Phase 0: 현재 실험 설정 동결 및 기준선 정리

- `Tools/simulation/gz_custom/models/boat/model.sdf`의 Aura visual mesh는 유지한다.
- 기존 `port_buoyancy_collision`, `starboard_buoyancy_collision`, `bow_buoyancy_collision` 실험값은 baseline으로 쓰지 않는다.
- 현재 single propeller + rudder 설정은 1단계 구현 중 제거하거나 비활성화한다.
- `gz-sim-hydrodynamics-system`과 `LiftDrag`는 1단계에서는 제거하여 힘의 출처를 줄인다.
- world의 `BuoyancySystem`은 사용할 수 있지만, 적용 대상과 collision 수를 단순화한다.

### Phase 1: Simple Rectangular Hull 구현

- `model.sdf`에 하나의 `rectangular_hull_collision`을 둔다.
- 초기 collision box는 `3.7 x 0.50 x 0.90 m`로 시작한다.
- collision pose는 목표 흘수 `0.45 m`를 기준으로 `z=-0.225` 근처에서 시작한다.
- mass는 `850 kg`, inertia는 BadaSim 값을 사용한다.
- 중심 질량은 `z=-0.10`에서 시작하고, 불안정하면 `z=-0.20`까지 낮춘다.
- Aura visual mesh pose는 유지하되, `base_link +X`와 선수 방향이 맞는지 확인한다.

### Phase 2: World buoyancy와 적용 대상 정리

- `worlds/boat.sdf`의 해수면을 `z=0`으로 고정한다.
- `graded_buoyancy`는 `density=1025`, air density는 `1.2`로 둔다.
- propeller link에 collision이 있으면 부력 대상이 되지 않도록 제거하거나 최소화한다.
- 가능하면 buoyancy 적용 대상을 `boat::base_link`로 제한한다.
- 물 표면 visual plane과 실제 buoyancy water level이 같은 높이인지 확인한다.

### Phase 3: Twin motor propulsion 정렬

- 좌우 propeller link를 추가한다.
- rudder link와 rudder `LiftDrag`는 1단계에서는 제거한다.
- motor plugin은 actuator 0/1로 분리한다.
- `51001_gz_boat`는 `CA_ROTOR_COUNT=2` differential thrust 기준으로 맞춘다.
- 좌우 motor 위치는 `x=-3.7`, `y=+-0.9`, `z=-0.15` 근처에서 시작한다.

### Phase 4: 1단계 안정화 시험

각 단계는 PX4 제어를 붙이기 전에 Gazebo model 단독 또는 최소 actuator command로 먼저 확인한다.

1. 무입력 60초 부유 시험
   - z 위치가 일정 범위 안에서 수렴
   - roll/pitch가 발산하지 않음
   - 선수/선미가 한쪽으로 계속 잠기지 않음

2. 대칭 thrust 시험
   - 좌우 motor 같은 명령에서 전진
   - yaw rate가 거의 0
   - pitch 변화가 제한적

3. 차등 thrust 시험
   - 좌우 motor 차이에서 yaw 발생
   - roll/pitch 발산 없음

4. PX4 manual 시험
   - arm 후 actuator output 방향 확인
   - 전진/후진/좌회전/우회전 부호 확인

5. PX4 mission 시험
   - waypoint 추종
   - loiter 반경과 속도 제어 확인

### Phase 5: Trapezoid Hull 근사

- rectangular hull이 안정화되면 여러 box를 쌓아 아래가 좁아지는 hull을 근사한다.
- bottom/middle/upper collision box의 부피 합이 `850 / 1025 = 0.829 m^3` 근처가 되도록 맞춘다.
- 각 box의 z pose를 바꿔 정지 흘수와 roll/pitch 복원성을 조정한다.
- 1단계 안정화 시험을 그대로 반복한다.

### Phase 6: 필요 시 Sample Point 부력으로 확장

- 여러 box trapezoid hull에서 force 변화가 계단식으로 나타나거나 roll/pitch 튜닝이 어렵다면 sample point 부력 plugin을 도입한다.
- 이 단계에서 `AuraUsvDynamics` 또는 `BoatDynamicsSystem` plugin 구현을 시작한다.
- 기존 5장의 전용 dynamics plugin 제안을 참고한다.
- collision은 접촉용으로만 두고, 부력/복원력은 plugin에서 계산한다.

### Phase 7: BadaSim 계약과 통합

- rectangular 또는 trapezoid baseline이 안정화된 뒤 BadaSim의 `single_engine_rudder` 계약을 다시 검토한다.
- PX4 `steering + signed_thrust` mapping을 사용할지, Classic식 twin motor mapping을 유지할지 분리한다.
- single engine + rudder로 갈 경우 rudder force는 generic `LiftDrag`보다 전용 force 계산으로 옮기는 것을 우선 검토한다.
- twin motor mode와 single engine rudder mode의 차이를 SDF, airframe, README에 명확히 남긴다.

## 7. 완료 기준

최소 완료 기준:

- `make px4_sitl gz_boat`에서 보트가 시작 직후 침몰하지 않는다.
- 무입력 상태에서 60초 이상 roll/pitch가 발산하지 않는다.
- 전진 명령에서 보트가 +X body 방향으로 움직인다.
- 좌/우 조향 명령의 yaw 방향이 PX4 boat control과 일치한다.
- Aura visual mesh의 선수 방향과 `base_link`의 +X 방향이 일치한다.

권장 완료 기준:

- 부력, damping, propulsion 파라미터가 SDF에서 조정 가능하다.
- BadaSim의 Aura 치수/질량/동역학 계수와 Gazebo SDF 값이 문서상으로 추적 가능하다.
- twin motor mode와 single engine rudder mode의 차이가 README에 명시된다.
- 최소 smoke test 절차가 `Tools/simulation/gz_custom/README.md`에 추가된다.

## 8. 결론

현재 증상은 단순한 색상, visual mesh, 또는 한두 개 pose 값의 문제가 아니라 보트 부력/복원력 모델의 구조적 문제로 보는 것이 맞다. 다만 바로 전용 dynamics plugin으로 들어가면 구현과 튜닝 변수가 많아져 원인 분리가 어렵다.

따라서 다음 구현은 먼저 `simple rectangular hull`로 안정적인 기준 모델을 만들고, 그 다음 아래가 좁아지는 `trapezoid hull` 또는 sample point 기반 부력으로 정밀도를 올리는 방향으로 진행한다. 전용 Gazebo Harmonic USV dynamics plugin은 이 단계들이 부족할 때 적용할 "이외 제안 해결책"으로 유지한다.
