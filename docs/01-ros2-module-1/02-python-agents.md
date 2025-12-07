# rclpy: Building Python Agents for ROS 2

Building upon the foundational concepts of ROS 2, this chapter delves into `rclpy`, the official Python client library for ROS 2. `rclpy` allows you to write ROS 2 nodes, publishers, subscribers, and services entirely in Python, making it accessible for rapid prototyping and complex robotic applications.

## Introduction to `rclpy`

`rclpy` provides a Pythonic interface to the core ROS 2 client library (`rcl`). It wraps the underlying C++ functionality, allowing Python developers to interact with the ROS 2 graph seamlessly. If you're familiar with Python, `rclpy` will feel natural and intuitive.

Key features of `rclpy`:
*   **Node management**: Create and manage ROS 2 nodes.
*   **Communication primitives**: Implement publishers, subscribers, service clients, and service servers.
*   **Parameter handling**: Declare, get, and set node parameters.
*   **Timers**: Execute functions periodically.
*   **Executors**: Manage the execution of callbacks within a node.

## Creating a Node with `rclpy`

Every component in a ROS 2 system is a node. In `rclpy`, you create a node by inheriting from `rclpy.node.Node`.

```python
import rclpy
from rclpy.node import Node

class MyCustomNode(Node):
    def __init__(self):
        super().__init__('my_custom_node') # Initialize the node with a unique name
        self.get_logger().info('MyCustomNode has been started!')

def main(args=None):
    rclpy.init(args=args) # Initialize rclpy
    node = MyCustomNode()
    rclpy.spin(node) # Keep the node alive, processing callbacks
    node.destroy_node() # Clean up when done
    rclpy.shutdown() # Shut down rclpy

if __name__ == '__main__':
    main()
```

In this basic example:
*   `rclpy.init(args=args)`: Initializes the ROS 2 communication layer. This must be called once at the beginning of your program.
*   `super().__init__('my_custom_node')`: Calls the constructor of the `Node` class, passing the name of your node. This name must be unique within your ROS 2 graph.
*   `self.get_logger().info(...)`: Used for logging messages from your node.
*   `rclpy.spin(node)`: Blocks the program and keeps the node alive, allowing it to process incoming messages and timer callbacks.
*   `node.destroy_node()`: Destroys the node and cleans up its resources.
*   `rclpy.shutdown()`: Shuts down the ROS 2 communication layer.

## Publishers and Subscribers in `rclpy`

As discussed in Chapter 1, topics are the primary way nodes exchange data. `rclpy` provides `create_publisher` and `create_subscription` methods for implementing this.

### Creating a Publisher

A publisher sends messages to a topic.

```python
# (Code from simple_publisher.py will go here)
```

In the `__init__` method of your node, you'd typically set up your publisher:
```python
self.publisher_ = self.create_publisher(MessageType, 'topic_name', qos_profile)
```
*   `MessageType`: The type of ROS 2 message to publish (e.g., `std_msgs.msg.String`).
*   `'topic_name'`: The name of the topic to publish to.
*   `qos_profile`: Quality of Service settings, which define how messages are handled (e.g., reliability, history, durability). A common value is `10` for a default "best effort" reliable transient local quality of service.

Then, you publish messages using `self.publisher_.publish(message_instance)`.

### Creating a Subscriber

A subscriber receives messages from a topic.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
Here, `self.create_subscription` takes the message type, topic name, a callback function that will be executed when a new message arrives, and the QoS profile.

## Service Clients and Servers with `rclpy`

For request/reply communication, `rclpy` offers service clients and servers.

### Creating a Service Server

A service server provides a service that clients can call.

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Example service type

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__('simple_service_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service server ready.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        self.get_logger().info(f'Sending back response: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SimpleServiceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
The `self.create_service` method takes the service type, service name, and a callback function to handle requests.

### Creating a Service Client

A service client calls a service server.

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys

class SimpleServiceClient(Node):
    def __init__(self):
        super().__init__('simple_service_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print('Usage: python simple_service_client.py <int> <int>')
        return

    client = SimpleServiceClient()
    response = client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    client.get_logger().info(f'Result of add_two_ints: {response.sum}')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
Here, `self.create_client` takes the service type and service name. The `send_request` method then constructs a request and calls the service asynchronously using `self.cli.call_async(self.req)`.

## Parameters in `rclpy`

`rclpy` allows nodes to declare and manage parameters. Parameters are dynamic configurations that can be changed at runtime.

```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        self.declare_parameter('my_parameter', 'hello') # Declare a parameter with a default value
        self.get_logger().info(f'Initial parameter value: {self.get_parameter("my_parameter").value}')
        
        # Set up a callback for when parameters are changed
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'my_parameter':
                self.get_logger().info(f'Parameter "my_parameter" changed to: {param.value}')
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
You can then change parameters from the command line using `ros2 param set /parameter_node my_parameter "new_value"`.

## Conclusion

`rclpy` provides a powerful and Pythonic way to build ROS 2 applications. You've learned how to create nodes, implement publishers and subscribers for asynchronous communication, and utilize service clients and servers for synchronous request/reply patterns. Understanding these core `rclpy` components will enable you to develop sophisticated robotic behaviors. In the next chapter, we'll explore practical examples using these concepts.

## Running the Example Python Agent

To run the `python_agent_publisher.py` script, follow these steps:

1.  **Source your ROS 2 environment**:
    If you haven't already, source your ROS 2 installation:
    ```bash
    source /opt/ros/humble/setup.bash
    ```
    (Replace with your actual ROS 2 path.)

2.  **Navigate to the example directory**:
    ```bash
    cd examples/
    ```

3.  **Run the Python agent publisher node**:
    ```bash
    python3 python_agent_publisher.py
    ```
    You should see output similar to the previous example, indicating that the node is publishing messages.

4.  **Verify with a ROS 2 listener (optional)**:
    Open a new terminal, source your ROS 2 environment again, and listen to the `python_chatter` topic:
    ```bash
    ros2 topic echo /python_chatter
    ```
    You should see the "Hello from Python Agent!" messages.

## Further Reading

*   [rclpy Documentation](https://docs.ros.org/en/humble/p/rclpy/index.html)
*   [ROS 2 Python Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Writing-A-Simple-Publisher-And-Subscriber-Python.html)
