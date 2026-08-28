#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from tier4_external_api_msgs.msg import ControlCommandStamped
from autoware_adapi_v1_msgs.msg import PedalsCommand, SteeringCommand
from autoware_vehicle_msgs.msg import GearCommand


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
            "publish_gear",
            True,
        )
        self.declare_parameter(
            "gear_command",
            int(GearCommand.DRIVE),
        )

        input_topic = self.get_parameter("input_topic").value
        pedals_topic = self.get_parameter("pedals_topic").value
        steering_topic = self.get_parameter("steering_topic").value
        gear_topic = self.get_parameter("gear_topic").value

        self.publish_gear = self.get_parameter("publish_gear").value
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

        self.control_sub = self.create_subscription(
            ControlCommandStamped,
            input_topic,
            self.on_control_cmd,
            10,
        )

        self.last_gear_stamp = self.get_clock().now()
        self.gear_timer = self.create_timer(0.1, self.publish_gear_cmd)

        self.get_logger().info("joy_to_manual_bridge started")
        self.get_logger().info(f"subscribe: {input_topic}")
        self.get_logger().info(f"publish pedals: {pedals_topic}")
        self.get_logger().info(f"publish steering: {steering_topic}")
        self.get_logger().info(f"publish gear: {gear_topic}")

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
        pedals_msg.throttle = clamp(control.throttle, 0.0, 1.0)
        pedals_msg.brake = clamp(control.brake, 0.0, 1.0)

        steering_msg = SteeringCommand()
        steering_msg.stamp = stamp
        steering_msg.steering_tire_angle = float(control.steering_angle)
        steering_msg.steering_tire_velocity = float(
            control.steering_angle_velocity
        )

        self.pedals_pub.publish(pedals_msg)
        self.steering_pub.publish(steering_msg)

    def publish_gear_cmd(self):
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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()