import rclpy
from rclpy.node import Node
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2
import pcl  # 引入PCL库
import numpy as np

class CustomMsgSubscriber(Node):
    def __init__(self):
        super().__init__('custom_msg_subscriber')
        self.subscription = self.create_subscription(
            CustomMsg,
            '/livox/lidar',
            self.listener_callback,
            10)
        self.frames_buffer = []
        self.frame_count = 0

    def listener_callback(self, msg):
        # 处理CustomMsg，转换为点云数据
        point_cloud = self.convert_to_point_cloud(msg)
        self.frames_buffer.append(point_cloud)
        self.frame_count += 1

        # 检查是否收集了足够的帧进行合成
        if self.frame_count == 10:
            combined_frame = self.combine_frames(self.frames_buffer)
            self.save_to_pcd(combined_frame)
            self.frames_buffer = []
            self.frame_count = 0

    def convert_to_point_cloud(self, msg):
        # 将CustomMsg转换为PCL点云数据
        points = []
        for point in msg.points:
            points.append([point.x, point.y, point.z])
        pcl_data = pcl.PointCloud(np.array(points, dtype=np.float32))
        return pcl_data

    def combine_frames(self, frames):
        # 合成多个帧
        combined = pcl.PointCloud()
        for frame in frames:
            combined += frame
        return combined

    def save_to_pcd(self, point_cloud):
        # 保存点云数据为PCD文件
        pcl.save(point_cloud, 'combined_frame.pcd')

def main(args=None):
    rclpy.init(args=args)
    custom_msg_subscriber = CustomMsgSubscriber()
    rclpy.spin(custom_msg_subscriber)
    custom_msg_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
