#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class TestROS2Subscriber(Node):
    def __init__(self):
        super().__init__('test_jointsubscriber')
        # 创建一个订阅者，订阅/joint_command话题上的JointState消息
        self.subscription = self.create_subscription(
            JointState,
            'joint_command',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用变量的警告

    def listener_callback(self, msg):
        # 打印接收到的关节名称和位置
        self.get_logger().info('Received Joint State:')
        for name, position in zip(msg.name, msg.position):
            self.get_logger().info(f'Joint: {name}, Position: {position}')

def main(args=None):
    rclpy.init(args=args)

    ros2_subscriber = TestROS2Subscriber()

    rclpy.spin(ros2_subscriber)

    # 显式销毁节点
    ros2_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
