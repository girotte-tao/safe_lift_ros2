import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socketio

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.sio = socketio.Client()
        self.sio.connect('http://localhost:5000')

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.sio.emit('message', msg.data)
        self.i += 1

    def destroy_node(self):
        self.sio.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
