import rclpy
from rclpy.node import Node
from livox_ros_driver2.msg import CustomMsg
import numpy as np
import struct
import os

class CustomMsgSubscriber(Node):
    def __init__(self):
        super().__init__('custom_msg_subscriber')
        self.subscription = self.create_subscription(
            CustomMsg,
            '/livox/lidar',  # 替换为实际的话题名称
            self.listener_callback,
            10)
        self.frame_data = []
        self.frame_count = 0
        self.max_frames = 10  # 收集10帧数据

    def listener_callback(self, msg):
        # self.get_logger().info('Received message: "%s"' % str(msg))
        self.process_message(msg)
        if len(self.frame_data) >= self.max_frames:
            self.save_and_clear_frame_data()

    def process_message(self, msg):
        # 转换CustomMsg为点云数据
        points = []
        for point in msg.points:
            points.append([point.x, point.y, point.z, point.reflectivity])
        self.frame_data.append(points)

    def save_and_clear_frame_data(self):
        # 合并数据并保存为PCD文件
        merged_points = np.concatenate(self.frame_data, axis=0)
        pcd_filename = f'frame_{self.frame_count}.pcd'
        self.save_pcd(merged_points, pcd_filename)
        # 转换为二进制格式
        binary_filename = f'frame_{self.frame_count}.bin'
        self.save_binary(merged_points, binary_filename)
        # 清除当前帧数据并更新帧计数
        self.frame_data = []
        self.frame_count += 1

    def save_pcd(self, points, filename):
        # 简单的PCD保存实现，可能需要根据实际的PCD格式进行调整
        with open(filename, 'w') as f:
            f.write(f'HEADER .pcd\nPOINTS {len(points)}\nDATA ascii\n')
            for p in points:
                f.write(f'{p[0]} {p[1]} {p[2]} {p[3]}\n')

    def save_binary(self, points, filename):
        # 保存为二进制格式
        with open(filename, 'wb') as f:
            for p in points:
                f.write(struct.pack('fff', p[0], p[1], p[2]))  # 只保存x, y, z

def main(args=None):
    rclpy.init(args=args)
    custom_msg_subscriber = CustomMsgSubscriber()
    rclpy.spin(custom_msg_subscriber)
    custom_msg_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
