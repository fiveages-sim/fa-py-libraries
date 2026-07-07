#!/usr/bin/env python3
"""虚拟 xr_target_node：仅注册节点名，供 bag 回放时让 VRInputHandler 保持启用。

不发布任何 /xr/* 话题（由 ros2 bag play 负责）。
"""

import rclpy
from rclpy.node import Node


def main() -> None:
    rclpy.init()
    node = Node("xr_target_node")
    node.get_logger().info(
        "Virtual xr_target_node stub running (bag playback helper, no VR topics published)"
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
