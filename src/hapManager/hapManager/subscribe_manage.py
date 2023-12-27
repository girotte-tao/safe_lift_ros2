import rclpy
from rclpy.node import Node
from livox_ros_driver2.msg import CustomMsg
import numpy as np
import struct
import os
import json
import socketio

class wsPublisher():
    def __init__(self, url, namespace, event):
        self.url = url
        self.namespace = namespace
        self.event = event
        self.sio = socketio.Client(reconnection=True)
        self.connected = False

        @self.sio.event
        def connect():
            print("Connected to the server.")
            self.connected = True
            
        @self.sio.on('lidar_message', namespace=self.namespace)
        def on_message(data):
            print("Received response:", data)

        @self.sio.event
        def disconnect():
            print("Disconnected from the server.")
            self.connected = False

        try:
            self.sio.connect(url, namespaces=[self.namespace])
        except socketio.exceptions.ConnectionError as e:
            print(f"Connection failed: {e}")

    def publish(self, data):
        if self.connected:
            self.sio.emit(self.event, data, namespace=self.namespace)
        else:
            print("Not connected. Cannot publish data.")

    def destroy(self):
        if self.connected:
            self.sio.disconnect()

        self.connected = False

class CustomMsgSubscriber(Node):
    def __init__(self, publisher=None):
        super().__init__('custom_msg_subscriber')
        self.subscription = self.create_subscription(
            CustomMsg,
            '/livox/lidar',  # 替换为实际的话题名称
            self.listener_callback,
            10)
        self.frame_data = []
        self.frame_count = 0
        self.max_frames = 1  # 收集10帧数据
        self.publisher = publisher

    def listener_callback(self, msg):
        # self.get_logger().info('Received message: "%s"' % str(msg))
        self.process_message(msg)
        if len(self.frame_data) >= self.max_frames:
            if self.publisher:
                self.publisher.publish(json.dumps(self.frame_data))
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
    url = 'ws://8.217.216.75:80/socket-io' 
    namespace = '/ws'
    # event = 'message'
    event = 'lidar_message'

    ws_publisher = wsPublisher(url, namespace, event)

    rclpy.init(args=args)
    custom_msg_subscriber = CustomMsgSubscriber(None)
    rclpy.spin(custom_msg_subscriber)
    custom_msg_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
