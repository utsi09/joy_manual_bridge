# joy_manual_bridge

autoware_joy_controller 출력을 요즘 Autoware의 AD API 수동조작 토픽으로 바꿔주는 ROS 2 브리지 노드.

조이스틱(DS4)으로 Autoware 차량 수동조작을 해보려는데, `autoware_joy_controller`는 옛날 tier4 external API 메시지(`tier4_external_api_msgs/ControlCommandStamped`)를 뿌리고, 최신 universe의 `external_cmd_selector`는 `autoware_adapi_v1_msgs` 기반의 pedals_cmd / steering_cmd / gear_cmd를 받게 바뀌어 있어서 그대로는 안 붙는다. 그래서 중간에 끼워넣는 용도로 만든 패키지.

## 토픽 흐름

```
joy_node -> joy_controller
  -> /api/external/set/command/remote/control   (tier4 ControlCommandStamped)
  -> [이 노드]
  -> /external/remote/pedals_cmd    (adapi PedalsCommand)
     /external/remote/steering_cmd  (adapi SteeringCommand)
     /external/remote/gear_cmd      (GearCommand, 타이머로 계속 발행)
  -> external_cmd_selector -> external_cmd_converter -> vehicle_cmd_gate
```

기어는 조이스틱 입력이랑 무관하게 파라미터로 정한 값(기본 DRIVE)을 10Hz로 계속 쏜다. 필요하면 `publish_gear:=false`로 끄면 됨.

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

# 브리지
ros2 launch joy_manual_bridge joy_manual_bridge.launch.py
```

주의: 하트비트는 이 노드가 안 보낸다. selector가 remote 명령을 받아주려면 따로 띄워야 함.

```bash
ros2 topic pub -r 10 /external/remote/heartbeat \
  autoware_adapi_v1_msgs/msg/ManualOperatorHeartbeat "{ready: true}"
```

이후 조이스틱 버튼으로 gate mode를 EXTERNAL로 바꾸고 engage 하면 페달/조향이 먹는다. 안 먹으면 `/external/selected/pedals_cmd`가 나오는지부터 확인해볼 것.

## 파라미터

- `input_topic` (기본 `/api/external/set/command/remote/control`)
- `pedals_topic`, `steering_topic`, `gear_topic`
- `publish_gear` (기본 true)
- `gear_command` (기본 2 = DRIVE)

launch 파일에 `use_sim_time: True`가 박혀 있는데 CARLA에서 돌리느라 그런 거니까 실차에서는 빼야 한다.
