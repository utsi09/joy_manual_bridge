# joy_manual_bridge

autoware_joy_controller 출력을 요즘 Autoware의 AD API 수동조작 토픽으로 바꿔주는 ROS 2 브리지 노드.

조이스틱(DS4)으로 Autoware 차량 수동조작을 해보려는데, `autoware_joy_controller`는 옛날 tier4 external API 메시지(`tier4_external_api_msgs/ControlCommandStamped`)를 뿌리고, 최신 universe의 `external_cmd_selector`는 `autoware_adapi_v1_msgs` 기반의 pedals_cmd / steering_cmd / gear_cmd를 받게 바뀌어 있어서 그대로는 안 붙는다. 그래서 중간에 끼워넣는 용도로 만든 패키지.

## 토픽 흐름

```
joy_node -> joy_controller
  -> /api/external/set/command/remote/control   (tier4 ControlCommandStamped)
  -> /api/external/set/command/remote/shift     (tier4 GearShiftStamped, 십자키 누를 때만)
  -> /api/external/set/command/remote/turn_signal (tier4 TurnSignalStamped, L1/R1/Share 누를 때만)
  -> /api/external/set/command/remote/heartbeat (tier4 Heartbeat)
  -> [이 노드]
  -> /external/remote/pedals_cmd    (adapi PedalsCommand)
     /external/remote/steering_cmd  (adapi SteeringCommand)
     /external/remote/gear_cmd      (GearCommand, 타이머로 계속 발행)
     /external/remote/turn_indicators_cmd, hazard_lights_cmd (타이머로 계속 발행)
     /external/remote/heartbeat     (adapi ManualOperatorHeartbeat, 릴레이)
  -> external_cmd_selector -> external_cmd_converter -> vehicle_cmd_gate
```

기어는 시작할 때 `gear_command` 파라미터 값(기본 DRIVE)으로 두고, 조이스틱 십자키(DS4 기준 오른쪽 DRIVE, 왼쪽 REVERSE, 위/아래 한 단씩)로 shift 메시지가 오면 그 값으로 바꿔서 10Hz로 계속 쏜다. tier4 GearShift -> autoware GearCommand 매핑은 PARKING->PARK, REVERSE->REVERSE, NEUTRAL->NEUTRAL, DRIVE->DRIVE, LOW->LOW. 필요하면 `publish_gear:=false`로 끄면 됨.

REVERSE로 놓고 R2를 밟으면 external_cmd_converter가 목표 속도를 음수로 만들고, CARLA 쪽 에이전트는 속도 부호를 보고 reverse 플래그를 켠다.

## 빌드

Autoware 워크스페이스 src 아래에 클론하고

```bash
colcon build --symlink-install --packages-select joy_manual_bridge
source install/setup.bash
```

## 실행

```bash
# joy_controller 먼저 (raw_control: true 로 해놔야 페달값이 그대로 나옴)
ros2 launch autoware_joy_controller joy_controller_param_selection.launch.xml joy_type:=ds4

# 브리지 (CARLA 등 시뮬레이터: use_sim_time:=true, 실차: 생략)
ros2 launch joy_manual_bridge joy_manual_bridge.launch.py use_sim_time:=true
```

하트비트는 joy_controller가 10Hz로 내는 `/api/external/set/command/remote/heartbeat`(tier4 Heartbeat)를 받아서 `/external/remote/heartbeat`(adapi ManualOperatorHeartbeat, ready=true)로 그대로 넘긴다. joy_controller가 죽으면 하트비트도 같이 끊긴다. joy_controller 없이 테스트할 때는 `heartbeat_always:=true`로 띄우면 타이머로 계속 쏜다.

selector는 기본이 local이라 remote로 바꿔줘야 `/external/selected/*`가 나오는데, 이 노드가 시작하면서 `/control/external_cmd_selector/select_external_command`를 REMOTE로 호출해준다 (서비스가 뜰 때까지 재시도). 끄려면 `select_remote_on_start:=false`. 손으로 하려면:

```bash
ros2 service call /control/external_cmd_selector/select_external_command \
  tier4_control_msgs/srv/ExternalCommandSelect "{mode: {data: 2}}"
```

gate mode도 이 노드가 시작하면서 `/control/current_gate_mode`가 AUTO로 보이면 EXTERNAL로 한 번 바꿔준다 (`gate_external_on_start:=false`로 끌 수 있음). 그 뒤 Options 버튼 토글은 그대로 먹는다. engage까지 되면 operation mode가 REMOTE가 되면서 페달/조향이 먹는다.

주의: gate가 AUTO인 상태에서 engage되면 operation mode가 AUTONOMOUS로 잡힌다. 그 상태에서 진단이 하나라도 빠져 있으면 mrm_handler가 MRM(비상정지)을 걸고 vehicle_cmd_gate가 `Emergency!`를 찍으면서 조이스틱을 전부 무시한다. 그러니 리더보드/에이전트를 띄우기 전에 이 노드를 먼저 띄워서 gate를 EXTERNAL로 해두고, RViz의 Autonomous 버튼은 누르지 말 것. 수동 조작 상태는 RViz 패널에서 Remote로 보이면 된다.

안 먹으면 순서대로 확인: `/external/remote/pedals_cmd` -> `/external/selected/pedals_cmd` -> `/external/selected/control_cmd` -> `/control/command/control_cmd`. 마지막 것이 acceleration -1.5/-2.4로 고정이면 gate가 정지 명령을 강제하는 것이고, `/api/fail_safe/mrm_state`의 state가 1(NORMAL)이 아니면 MRM 때문이다.

## 파라미터

- `input_topic` (기본 `/api/external/set/command/remote/control`)
- `pedals_topic`, `steering_topic`, `gear_topic`
- `shift_topic` (기본 `/api/external/set/command/remote/shift`)
- `turn_signal_topic`, `turn_indicators_topic`, `hazard_lights_topic`. L1 좌, R1 우, L1+R1 비상등, Share 끄기.
- `publish_gear` (기본 true)
- `input_heartbeat_topic` (기본 `/api/external/set/command/remote/heartbeat`)
- `heartbeat_topic` (기본 `/external/remote/heartbeat`)
- `publish_heartbeat` (기본 true)
- `heartbeat_always` (기본 false. true면 입력 하트비트 없이 타이머로 발행)
- `select_remote_on_start` (기본 true), `selector_service`
- `gate_external_on_start` (기본 true), `gate_mode_topic`, `current_gate_mode_topic`
- `gear_command` (기본 2 = DRIVE, 시작 기어. 20 = REVERSE)

- `idle_brake` (기본 0.2), `idle_pedal_threshold` (기본 0.05). 페달을 둘 다 안 밟으면 브레이크를 idle_brake로 채워서 보낸다. Autoware 기본 accel map은 스로틀 0에서도 정지 시 +0.3 m/s^2(크리프)를 내서 차가 슬금슬금 나가기 때문. 0.2면 기본 brake map 기준 정지 시 -0.38, 주행 중 -0.7~-1.0 m/s^2. 실차에서 크리프를 살리고 싶으면 0.
- `low_gear_max_speed` (기본 3.0), `drive_gear_max_speed` (기본 0 = 무제한), `reverse_gear_max_speed` (기본 3.0). 기어별 최대 속도[m/s]. external_cmd_converter는 LOW와 DRIVE를 구분하지 않으므로 이 노드가 속도를 넘으면 스로틀을 0으로 자르고 idle_brake로 감속시킨다.
- `hold_gate_external` (기본 true). 노드가 떠 있는 동안 gate가 AUTO로 돌아가면 다시 EXTERNAL로 되돌린다. gate 토글 버튼이 실수로 눌려 자율주행으로 넘어가는 것 방지. 토글을 쓰려면 false.
- `throttle_scale` (기본 1.0). Autoware 기본 accel map은 스로틀 0.5까지만 있어서 그 이상이면 converter가 `Input throttle: acc: 1 is out of range. use closest value.`를 찍으며 0.5로 자른다. 기본 맵이면 0.5로 두면 로그가 사라지고 결과는 같다.
- `gear_change_max_speed` (기본 0.5 m/s. 이 속도 넘거나 속도 정보가 없으면 D<->R 전환 거부, 0 이하면 보호 끔), `velocity_topic`

launch 인자로 `use_sim_time`(기본 false), `gear_command`, `select_remote_on_start`, `gate_external_on_start`, `gear_change_max_speed`, `heartbeat_always`를 넘길 수 있다.

## 버튼 매핑

joy_controller의 `joy_type:=ds4` 프로파일은 `/joy` 배열 인덱스로만 동작하므로 실제 패드 종류에 따라 물리 버튼이 달라진다. `ros2 topic echo /joy`로 인덱스를 확인할 것.

| 기능 | DS4 (듀얼쇼크4) | Xbox 360 모드 패드 (GameSir 등, 버튼 11개 레이아웃) |
|---|---|---|
| 가속 | R2 / ✕ / 오른쪽 스틱 위 | RT / A / 오른쪽 스틱 위 |
| 브레이크 | L2 / □ / 오른쪽 스틱 아래 | LT / Y / 오른쪽 스틱 아래 |
| 조향 | 왼쪽 스틱 좌우 | 왼쪽 스틱 좌우 |
| 기어 D / R, 한 단 위/아래 | 십자키 → / ←, ↑ / ↓ | 십자키 → / ←, ↑ / ↓ |
| 방향지시등 좌 / 우, 끄기 | L1 / R1, Share | LB / RB, Xbox(가이드) 버튼 |
| 비상등 | L1 + R1 | LB + RB |
| gate mode 토글 | Options | 왼쪽 스틱 클릭 (실수로 눌리기 쉬움, hold_gate_external 참고) |
| Autoware engage / disengage | ○ / Share + ○ | B / 가이드 + B |
| Vehicle engage / disengage | △ / Share + △ | X / 가이드 + X |
| 비상정지 / 해제 | PS / Share + PS | 오른쪽 스틱 클릭 / 가이드 + 오른쪽 스틱 클릭 |

joy_controller의 `joy_type:=xbox` 프로파일은 버튼 17개 레이아웃을 가정해서 11개짜리 Xbox 360 모드 패드에서는 인덱스 초과로 죽는다. Xbox 모드 패드도 `ds4`로 띄우고 위 표를 쓰면 된다.

## 실차에서 쓸 때

이 노드는 표준 Autoware 외부 명령 토픽(`/external/remote/*`)만 내보내고, 그 뒤는 Autoware 본체(external_cmd_selector -> external_cmd_converter -> vehicle_cmd_gate)가 처리한다. 그래서 CAN 차량 인터페이스는 자율주행 때와 똑같이 `/control/command/*`만 보면 된다. 시뮬레이터 전용 코드는 없다.

CAN 인터페이스 쪽에서 맞춰야 하는 것:

- 구독: `/control/command/control_cmd` (longitudinal.velocity/acceleration, lateral.steering_tire_angle), `/control/command/gear_cmd`, `/control/command/turn_indicators_cmd`, `/control/command/hazard_lights_cmd`, `/vehicle/engage` (조이스틱 △).
- 발행: `/vehicle/status/velocity_status` (external_cmd_converter가 이게 없으면 아무것도 안 냄, 이 노드의 기어 보호도 이걸 씀), `/vehicle/status/steering_status`, `/vehicle/status/gear_status`, `/vehicle/status/control_mode` (MANUAL/AUTONOMOUS. mrm_handler와 operation_mode_transition_manager가 본다).
- 서비스: `/control/control_mode_request` (operation mode 전환 시 Autoware가 호출. 인터페이스가 응답하지 않으면 전환이 안 될 수 있음).

실차 권장 설정:

- `use_sim_time`은 기본 false 그대로.
- `select_remote_on_start`, `gate_external_on_start`는 운전자가 Options 버튼으로 직접 넘기게 하고 싶으면 false. 기본 true는 노드가 뜨는 순간 gate를 EXTERNAL로 바꾸므로, 자율주행 중에 이 노드를 띄우면 그 즉시 조이스틱 입력(페달 0)이 차에 들어간다.
- `gear_change_max_speed`는 켜둘 것. 실제 변속기 보호는 CAN 인터페이스에서도 한 번 더 해야 한다.
- 하트비트는 joy_controller가 살아있을 때만 릴레이되므로 `heartbeat_always`는 실차에서 쓰지 말 것.
- 페달 값은 0~1 그대로 넘기고 가속도 변환은 external_cmd_converter의 accel/brake map(`autoware_launch` 설정)이 한다. 차량에 맞는 맵으로 바꿔야 한다.
