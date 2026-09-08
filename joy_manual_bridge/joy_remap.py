#!/usr/bin/env python3
"""/joy 버튼·축 인덱스를 바꿔서 다시 내보내는 노드.

joy_controller의 ds4 프로파일은 인덱스로만 동작하므로, 패드 물리 배치가 다르거나
(예: GameSir Xbox 모드) 어떤 버튼이 펄스만 내는 경우 여기서 자리를 바꿔준다.

기본 button_map은 Xbox 360 모드 패드용:
  출력 2 (△ 자리, vehicle engage)  <- 입력 8 (홈 버튼, 펄스라 한 번짜리 동작에 적합)
  출력 8 (Share 자리, 조합키)      <- 입력 2 (X 버튼, 누른 채 유지 가능)
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyRemap(Node):
    def __init__(self):
        super().__init__("joy_remap")
        self.declare_parameter("input_topic", "/joy")
        self.declare_parameter("output_topic", "/joy_remapped")
        # output.buttons[i] = input.buttons[button_map[i]]
        self.declare_parameter(
            "button_map", [0, 1, 8, 3, 4, 5, 6, 7, 2, 9, 10]
        )
        # output.axes[i] = input.axes[axis_map[i]] (빈 배열이면 그대로)
        self.declare_parameter("axis_map", [-1])

        self.button_map = [int(v) for v in self.get_parameter("button_map").value]
        axis_map = [int(v) for v in self.get_parameter("axis_map").value]
        self.axis_map = [] if axis_map == [-1] else axis_map

        self.pub = self.create_publisher(
            Joy, self.get_parameter("output_topic").value, 10
        )
        self.sub = self.create_subscription(
            Joy, self.get_parameter("input_topic").value, self.on_joy, 10
        )
        self.get_logger().info(
            f"button_map: {self.button_map}, axis_map: {self.axis_map or 'identity'}"
        )

    @staticmethod
    def remap(values, index_map):
        if not index_map:
            return list(values)
        out = []
        for src in index_map:
            out.append(values[src] if 0 <= src < len(values) else 0)
        return out

    def on_joy(self, msg):
        out = Joy()
        out.header = msg.header
        out.axes = [float(v) for v in self.remap(msg.axes, self.axis_map)]
        out.buttons = [int(v) for v in self.remap(msg.buttons, self.button_map)]
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JoyRemap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: BLE001
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
