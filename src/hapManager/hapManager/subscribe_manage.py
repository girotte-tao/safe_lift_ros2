import rclpy
from rclpy.node import Node
from livox_ros_driver2.msg import CustomMsg
import numpy as np
import struct
import os
import json
import socketio
import requests

def send_get_request(url):
    try:
        response = requests.get(url)
        response.raise_for_status() # 检查请求是否成功
        return response.text # 或者 response.json() 如果响应是JSON
    except requests.RequestException as e:
        print(f"请求错误: {e}")
        return None
    
def send_post_request(url, data):
    headers = {'Content-Type': 'application/json'}
    try:
        response = requests.post(url, data=json.dumps(data), headers=headers)
        response.raise_for_status() # 检查请求是否成功
        return response.text # 或者 response.json() 如果响应是JSON
    except requests.RequestException as e:
        print(f"请求错误: {e}")
        return None


def lidar_data_generator(frame_data, batch_size):
    print(len(frame_data))
    for i in range(0, len(frame_data), batch_size):
        print(i)
        yield frame_data[i:i + batch_size]

def send_lidar_data(frame_data, batch_size):
    url_post = 'http://8.217.216.75/api/lidar_message'
    for batch in lidar_data_generator(frame_data, batch_size):
        post_response = send_post_request(url_post, {'data': batch.tolist(), 'length':batch_size})
        if post_response:
            print(f"收到post响应: {post_response}")

    
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
        self.max_frames = 1  # 收集10帧数据
        self.merged_points = []

    def listener_callback(self, msg):
        # self.get_logger().info('Received message: "%s"' % str(msg))
        self.process_message(msg)
        if len(self.frame_data) >= self.max_frames:
            self.save_and_clear_frame_data()
            send_lidar_data(self.merged_points, 10)



    def process_message(self, msg):
        # 转换CustomMsg为点云数据
        points = []
        for point in msg.points:
            points.append([point.x, point.y, point.z, point.reflectivity])
        self.frame_data.append(points)

    def save_and_clear_frame_data(self):
        # 合并数据并保存为PCD文件
        merged_points = np.concatenate(self.frame_data, axis=0)
        self.merged_points = merged_points
        pcd_filename = f'frame_{self.frame_count}.pcd'
        # self.save_pcd(merged_points, pcd_filename)
        # 转换为二进制格式
        # binary_filename = f'frame_{self.frame_count}.bin'
        # self.save_binary(merged_points, binary_filename)
        # 保存为CSV文件
        # csv_filename = f'frame_{self.frame_count}.csv'
        # self.save_csv(merged_points, csv_filename)
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
        points.tofile(filename)
        # with open(filename, 'wb') as f:
        #     for p in points:
        #         f.write(struct.pack('fff', p[0], p[1], p[2]))  # 只保存x, y, z

    def save_csv(self, points, filename):
        # 保存为CSV格式
        with open(filename, 'w') as f:
            for p in points:
                csv_line = ','.join(map(str, p))
                f.write(csv_line + '\n')

def main(args=None):
    rclpy.init(args=args)
    custom_msg_subscriber = CustomMsgSubscriber()
    rclpy.spin(custom_msg_subscriber)
    custom_msg_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
