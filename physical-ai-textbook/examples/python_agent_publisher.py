# examples/python_agent_publisher.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PythonAgentPublisher(Node):

    def __init__(self):
        super().__init__('python_agent_publisher')
        self.publisher_ = self.create_publisher(String, 'python_chatter', 10)
        timer_period = 0.75  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from Python Agent! Count: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    python_agent_publisher = PythonAgentPublisher()

    rclpy.spin(python_agent_publisher)

    python_agent_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
