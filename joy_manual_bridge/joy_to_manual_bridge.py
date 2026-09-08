#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from tier4_external_api_msgs.msg import ControlCommandStamped, GearShiftStamped
from tier4_external_api_msgs.msg import GearShift, Heartbeat
from tier4_external_api_msgs.msg import TurnSignalStamped, TurnSignal
from autoware_adapi_v1_msgs.msg import PedalsCommand, SteeringCommand
from autoware_adapi_v1_msgs.msg import ManualOperatorHeartbeat
from autoware_vehicle_msgs.msg import GearCommand, VelocityReport
from autoware_vehicle_msgs.msg import TurnIndicatorsCommand, HazardLightsCommand
from tier4_control_msgs.msg import ExternalCommandSelectorMode, GateMode
from tier4_control_msgs.srv import ExternalCommandSelect

# joy_controller가 쏘는 tier4 GearShift -> autoware GearCommand
GEAR_SHIFT_TO_GEAR_COMMAND = {
    GearShift.NONE: GearCommand.NONE,
    GearShift.PARKING: GearCommand.PARK,
    GearShift.REVERSE: GearCommand.REVERSE,
    GearShift.NEUTRAL: GearCommand.NEUTRAL,
    GearShift.DRIVE: GearCommand.DRIVE,
    GearShift.LOW: GearCommand.LOW,
}


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, float(value)))


class JoyToManualBridge(Node):
    def __init__(self):
        super().__init__("joy_to_manual_bridge")

        self.declare_parameter(
            "input_topic",
            "/api/external/set/command/remote/control",
        )
        self.declare_parameter(
            "pedals_topic",
            "/external/remote/pedals_cmd",
        )
        self.declare_parameter(
            "steering_topic",
            "/external/remote/steering_cmd",
        )
        self.declare_parameter(
            "gear_topic",
            "/external/remote/gear_cmd",
        )
        self.declare_parameter(
            "shift_topic",
            "/api/external/set/command/remote/shift",
        )
        # 방향지시등/비상등. joy_controller: L1 좌, R1 우, L1+R1 비상등, Share 끄기
        self.declare_parameter(
            "turn_signal_topic",
            "/api/external/set/command/remote/turn_signal",
        )
        self.declare_parameter(
            "turn_indicators_topic",
            "/external/remote/turn_indicators_cmd",
        )
        self.declare_parameter(
            "hazard_lights_topic",
            "/external/remote/hazard_lights_cmd",
        )
        self.declare_parameter(
            "publish_gear",
            True,
        )
        self.declare_parameter(
            "input_heartbeat_topic",
            "/api/external/set/command/remote/heartbeat",
        )
        self.declare_parameter(
            "heartbeat_topic",
            "/external/remote/heartbeat",
        )
        self.declare_parameter(
            "publish_heartbeat",
            True,
        )
        # true면 joy_controller 하트비트가 없어도 타이머로 계속 ready를 쏜다
        self.declare_parameter(
            "heartbeat_always",
            False,
        )
        # 시작할 때 external_cmd_selector를 REMOTE로 바꿔준다 (기본 selector는 local)
        self.declare_parameter(
            "select_remote_on_start",
            True,
        )
        self.declare_parameter(
            "selector_service",
            "/control/external_cmd_selector/select_external_command",
        )
        # 시작할 때 vehicle_cmd_gate를 EXTERNAL로 한 번 바꿔준다.
        # gate가 AUTO인 채로 에이전트가 engage하면 operation mode가 AUTONOMOUS로
        # 잡혀서 MRM이 걸리고 조이스틱이 무시된다. 한 번만 바꾸므로 이후
        # Options 버튼 토글은 그대로 존중된다.
        self.declare_parameter(
            "gate_external_on_start",
            True,
        )
        self.declare_parameter(
            "gate_mode_topic",
            "/control/gate_mode_cmd",
        )
        self.declare_parameter(
            "current_gate_mode_topic",
            "/control/current_gate_mode",
        )
        self.declare_parameter(
            "gear_command",
            int(GearCommand.DRIVE),
        )
        # 주행 중 기어 전환 보호. 현재 속도(절대값)가 이 값[m/s]을 넘으면
        # DRIVE<->REVERSE 같은 방향 전환을 거부한다. 속도 정보를 아직 못 받았으면
        # 역시 거부한다 (실차 안전). 0 이하로 주면 보호 끔.
        self.declare_parameter(
            "gear_change_max_speed",
            0.5,
        )
        # 페달을 안 밟았을 때(스로틀/브레이크 둘 다 idle_pedal_threshold 미만)
        # 대신 넣어줄 브레이크 값. Autoware 기본 accel map은 스로틀 0에서도
        # 정지 상태 +0.3 m/s^2(크리프)라 차가 슬금슬금 나가는데, 이 값을 주면
        # 브레이크 맵을 타서 멈춘다. 기본 맵 기준 0.2면 정지 시 -0.38 m/s^2.
        # 0이면 끔.
        self.declare_parameter(
            "idle_brake",
            0.2,
        )
        self.declare_parameter(
            "idle_pedal_threshold",
            0.05,
        )
        # 스로틀 배율. Autoware 기본 accel map은 스로틀 0.5까지만 정의돼 있어서
        # 그 이상은 converter가 "out of range" 로그를 찍으며 0.5로 자른다.
        # 기본 맵을 쓸 때는 0.5로 두면 조용해지고 결과는 같다.
        self.declare_parameter(
            "throttle_scale",
            1.0,
        )
        self.declare_parameter(
            "velocity_topic",
            "/vehicle/status/velocity_status",
        )

        input_topic = self.get_parameter("input_topic").value
        pedals_topic = self.get_parameter("pedals_topic").value
        steering_topic = self.get_parameter("steering_topic").value
        gear_topic = self.get_parameter("gear_topic").value
        shift_topic = self.get_parameter("shift_topic").value
        turn_signal_topic = self.get_parameter("turn_signal_topic").value
        turn_indicators_topic = self.get_parameter("turn_indicators_topic").value
        hazard_lights_topic = self.get_parameter("hazard_lights_topic").value

        input_heartbeat_topic = self.get_parameter("input_heartbeat_topic").value
        heartbeat_topic = self.get_parameter("heartbeat_topic").value

        self.gear_change_max_speed = float(
            self.get_parameter("gear_change_max_speed").value
        )
        self.idle_brake = clamp(self.get_parameter("idle_brake").value, 0.0, 1.0)
        self.idle_pedal_threshold = float(
            self.get_parameter("idle_pedal_threshold").value
        )
        self.throttle_scale = clamp(
            self.get_parameter("throttle_scale").value, 0.0, 1.0
        )
        velocity_topic = self.get_parameter("velocity_topic").value

        self.publish_gear = self.get_parameter("publish_gear").value
        self.publish_heartbeat = self.get_parameter("publish_heartbeat").value
        self.select_remote_on_start = self.get_parameter(
            "select_remote_on_start"
        ).value
        selector_service = self.get_parameter("selector_service").value
        self.gate_external_on_start = self.get_parameter(
            "gate_external_on_start"
        ).value
        gate_mode_topic = self.get_parameter("gate_mode_topic").value
        current_gate_mode_topic = self.get_parameter(
            "current_gate_mode_topic"
        ).value
        self.heartbeat_always = self.get_parameter("heartbeat_always").value
        self.gear_command = int(self.get_parameter("gear_command").value)

        self.pedals_pub = self.create_publisher(
            PedalsCommand,
            pedals_topic,
            10,
        )

        self.steering_pub = self.create_publisher(
            SteeringCommand,
            steering_topic,
            10,
        )

        self.gear_pub = self.create_publisher(
            GearCommand,
            gear_topic,
            10,
        )

        self.heartbeat_pub = self.create_publisher(
            ManualOperatorHeartbeat,
            heartbeat_topic,
            10,
        )

        self.control_sub = self.create_subscription(
            ControlCommandStamped,
            input_topic,
            self.on_control_cmd,
            10,
        )

        self.turn_indicators_pub = self.create_publisher(
            TurnIndicatorsCommand,
            turn_indicators_topic,
            10,
        )
        self.hazard_lights_pub = self.create_publisher(
            HazardLightsCommand,
            hazard_lights_topic,
            10,
        )
        self.turn_indicators_command = TurnIndicatorsCommand.DISABLE
        self.hazard_lights_command = HazardLightsCommand.DISABLE
        self.turn_signal_sub = self.create_subscription(
            TurnSignalStamped,
            turn_signal_topic,
            self.on_turn_signal,
            10,
        )

        self.current_speed = None
        self.velocity_sub = self.create_subscription(
            VelocityReport,
            velocity_topic,
            self.on_velocity,
            10,
        )

        # 조이스틱 십자키 기어 입력. 들어오면 그 값을 계속 발행한다.
        self.shift_sub = self.create_subscription(
            GearShiftStamped,
            shift_topic,
            self.on_shift_cmd,
            10,
        )

        # joy_controller 하트비트(tier4 Heartbeat) -> adapi ManualOperatorHeartbeat.
        # joy_controller가 죽으면 이것도 같이 끊겨서 selector/converter가 알아챈다.
        self.heartbeat_sub = self.create_subscription(
            Heartbeat,
            input_heartbeat_topic,
            self.on_heartbeat,
            10,
        )

        self.gate_mode_pub = self.create_publisher(
            GateMode,
            gate_mode_topic,
            1,
        )
        self.gate_done = not self.gate_external_on_start
        self.gate_mode_sub = self.create_subscription(
            GateMode,
            current_gate_mode_topic,
            self.on_current_gate_mode,
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self.selector_client = self.create_client(
            ExternalCommandSelect,
            selector_service,
        )
        self.selector_done = not self.select_remote_on_start
        self.selector_future = None

        self.last_gear_stamp = self.get_clock().now()
        self.gear_timer = self.create_timer(0.1, self.publish_gear_cmd)

        self.get_logger().info("joy_to_manual_bridge started")
        self.get_logger().info(f"subscribe: {input_topic}")
        self.get_logger().info(f"publish pedals: {pedals_topic}")
        self.get_logger().info(f"publish steering: {steering_topic}")
        self.get_logger().info(f"publish gear: {gear_topic}")
        self.get_logger().info(f"subscribe shift: {shift_topic}")
        self.get_logger().info(
            f"turn signal: {turn_signal_topic} -> {turn_indicators_topic},"
            f" {hazard_lights_topic}"
        )
        self.get_logger().info(f"initial gear: {self.gear_command}")
        self.get_logger().info(
            f"gate_external_on_start: {self.gate_external_on_start}"
        )
        self.get_logger().info(
            f"idle_brake: {self.idle_brake} (threshold {self.idle_pedal_threshold})"
        )
        self.get_logger().info(
            f"heartbeat: {input_heartbeat_topic} -> {heartbeat_topic}"
            f" (publish={self.publish_heartbeat}, always={self.heartbeat_always})"
        )

    def now_stamp(self):
        return self.get_clock().now().to_msg()

    def on_control_cmd(self, msg):
        control = msg.control

        stamp = msg.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.now_stamp()
        stamp = self.get_clock().now().to_msg()
        pedals_msg = PedalsCommand()
        pedals_msg.stamp = stamp
        throttle = clamp(control.throttle, 0.0, 1.0)
        brake = clamp(control.brake, 0.0, 1.0)
        if (
            self.idle_brake > 0.0
            and throttle < self.idle_pedal_threshold
            and brake < self.idle_pedal_threshold
        ):
            throttle = 0.0
            brake = self.idle_brake
        pedals_msg.throttle = throttle * self.throttle_scale
        pedals_msg.brake = brake

        steering_msg = SteeringCommand()
        steering_msg.stamp = stamp
        steering_msg.steering_tire_angle = float(control.steering_angle)
        steering_msg.steering_tire_velocity = float(
            control.steering_angle_velocity
        )

        self.pedals_pub.publish(pedals_msg)
        self.steering_pub.publish(steering_msg)

    def on_turn_signal(self, msg):
        data = msg.turn_signal.data
        if data == TurnSignal.HAZARD:
            self.turn_indicators_command = TurnIndicatorsCommand.DISABLE
            self.hazard_lights_command = HazardLightsCommand.ENABLE
        elif data == TurnSignal.LEFT:
            self.turn_indicators_command = TurnIndicatorsCommand.ENABLE_LEFT
            self.hazard_lights_command = HazardLightsCommand.DISABLE
        elif data == TurnSignal.RIGHT:
            self.turn_indicators_command = TurnIndicatorsCommand.ENABLE_RIGHT
            self.hazard_lights_command = HazardLightsCommand.DISABLE
        elif data == TurnSignal.NONE:
            self.turn_indicators_command = TurnIndicatorsCommand.DISABLE
            self.hazard_lights_command = HazardLightsCommand.DISABLE
        else:
            self.get_logger().warn(f"unknown turn signal: {data}")
            return
        self.get_logger().info(
            f"turn signal: indicators={self.turn_indicators_command}"
            f" hazard={self.hazard_lights_command}"
        )

    def publish_turn_signal_cmds(self):
        stamp = self.now_stamp()
        ti = TurnIndicatorsCommand()
        ti.stamp = stamp
        ti.command = self.turn_indicators_command
        hl = HazardLightsCommand()
        hl.stamp = stamp
        hl.command = self.hazard_lights_command
        self.turn_indicators_pub.publish(ti)
        self.hazard_lights_pub.publish(hl)

    def on_velocity(self, msg):
        self.current_speed = abs(float(msg.longitudinal_velocity))

    def is_direction_change(self, old, new):
        forward = (GearCommand.DRIVE, GearCommand.LOW)
        old_dir = 1 if old in forward else (-1 if old == GearCommand.REVERSE else 0)
        new_dir = 1 if new in forward else (-1 if new == GearCommand.REVERSE else 0)
        return old_dir != new_dir

    def on_shift_cmd(self, msg):
        shift = msg.gear_shift.data
        if shift not in GEAR_SHIFT_TO_GEAR_COMMAND:
            self.get_logger().warn(f"unknown gear shift: {shift}")
            return
        gear = GEAR_SHIFT_TO_GEAR_COMMAND[shift]
        if gear == self.gear_command:
            return

        if self.gear_change_max_speed > 0.0 and self.is_direction_change(
            self.gear_command, gear
        ):
            if self.current_speed is None:
                self.get_logger().warn(
                    "gear change rejected: no velocity report yet"
                )
                return
            if self.current_speed > self.gear_change_max_speed:
                self.get_logger().warn(
                    f"gear change rejected: speed {self.current_speed:.2f} m/s"
                    f" > {self.gear_change_max_speed:.2f} m/s"
                )
                return

        self.get_logger().info(f"gear: {self.gear_command} -> {gear}")
        self.gear_command = gear

    def on_heartbeat(self, msg):
        if self.heartbeat_always:
            return
        self.publish_heartbeat_msg()

    def publish_heartbeat_msg(self):
        if not self.publish_heartbeat:
            return
        hb = ManualOperatorHeartbeat()
        hb.stamp = self.now_stamp()
        hb.ready = True
        self.heartbeat_pub.publish(hb)

    def on_current_gate_mode(self, msg):
        # gate가 살아있고 아직 AUTO면 EXTERNAL로 한 번 바꾼다.
        if self.gate_done:
            return
        if msg.data == GateMode.EXTERNAL:
            self.get_logger().info("gate mode already EXTERNAL")
            self.gate_done = True
            return
        cmd = GateMode()
        cmd.data = GateMode.EXTERNAL
        self.gate_mode_pub.publish(cmd)
        self.get_logger().info("gate mode AUTO -> EXTERNAL requested")
        self.gate_done = True

    def select_remote(self):
        # 서비스가 뜰 때까지 타이머마다 재시도. 성공하면 끝.
        if self.selector_future is not None:
            if not self.selector_future.done():
                return
            try:
                res = self.selector_future.result()
                if res.success:
                    self.get_logger().info("external_cmd_selector -> REMOTE")
                    self.selector_done = True
                else:
                    self.get_logger().warn(
                        f"selector REMOTE failed: {res.message}, retrying"
                    )
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"selector call error: {e}, retrying")
            self.selector_future = None
            return

        if not self.selector_client.service_is_ready():
            self.get_logger().info(
                "waiting for external_cmd_selector service...",
                throttle_duration_sec=5.0,
            )
            return

        req = ExternalCommandSelect.Request()
        req.mode.data = ExternalCommandSelectorMode.REMOTE
        self.selector_future = self.selector_client.call_async(req)

    def publish_gear_cmd(self):
        if not self.selector_done:
            self.select_remote()

        if self.heartbeat_always:
            self.publish_heartbeat_msg()

        self.publish_turn_signal_cmds()

        if not self.publish_gear:
            return

        gear_msg = GearCommand()
        gear_msg.stamp = self.now_stamp()
        gear_msg.command = self.gear_command

        self.gear_pub.publish(gear_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToManualBridge()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: BLE001
        # SIGTERM 직후 rcl context가 먼저 내려가면 wait set 생성에서
        # RCLError가 튀는데, 종료 중이면 무시해도 되는 잡음이다.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()