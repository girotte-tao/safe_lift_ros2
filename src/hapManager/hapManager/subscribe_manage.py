import rclpy
from rclpy.node import Node
from livox_ros_driver2.msg import CustomMsg  # 确保这是正确的导入路径
import pcl  # 导入PCL库


class CustomMsgSubscriber(Node):
    def __init__(self):
        super().__init__('custom_msg_subscriber')
        self.subscription = self.create_subscription(
            CustomMsg,
            '/livox/lidar',  # 替换为实际的话题名称
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info('Received message')

        # 处理CustomMsg以获取点云数据
        cloud = self.convert_to_pcl(msg.points)

        # 保存点云数据为PCL文件
        pcl.save(cloud, 'output.pcd')

        self.get_logger().info('Point cloud saved as output.pcd')

    def convert_to_pcl(self, points):
        # 将CustomMsg中的点云数据转换为PCL格式
        # 这需要根据CustomPoint的具体结构来编写转换逻辑
        # 例如:
        pcl_cloud = pcl.PointCloud()
        for point in points:
            # 根据CustomPoint的结构添加点到pcl_cloud
            pass
        return pcl_cloud


def main(args=None):
    rclpy.init(args=args)
    custom_msg_subscriber = CustomMsgSubscriber()
    rclpy.spin(custom_msg_subscriber)
    custom_msg_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
