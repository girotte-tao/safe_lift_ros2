import rclpy
from rclpy.node import Node
from livox_ros_driver2.msg import CustomMsg  # 确保这是正确的导入路径

class CustomMsgSubscriber(Node):
    def __init__(self):
        super().__init__('custom_msg_subscriber')
        self.subscription = self.create_subscription(
            CustomMsg,
            '/livox/lidar',  # 替换为实际的话题名称
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # 在这里处理接收到的CustomMsg消息
        self.get_logger().info('Received message: "%s"' % str(msg))

def main(args=None):
    rclpy.init(args=args)
    custom_msg_subscriber = CustomMsgSubscriber()
    rclpy.spin(custom_msg_subscriber)
    custom_msg_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()