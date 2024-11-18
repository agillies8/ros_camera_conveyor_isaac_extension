import rclpy
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray  # Custom YOLO detection message
from sensor_msgs.msg import JointState  # For sending joint position commands
from rclpy.time import Time

class YOLODetectionActor(Node):
    def __init__(self):
        super().__init__('yolo_detection_actor')

        # Declare and read parameters
        self.declare_parameter('target_classes', ['blue-box'])  # Classes to filter
                # Declare individual parameters for bounding box range
        self.declare_parameter('x_min', 200.0)
        self.declare_parameter('x_max', 400.0)
        self.declare_parameter('y_min', 250.0)
        self.declare_parameter('y_max', 350.0)


        self.declare_parameter('joint_command_topic', '/joint_command')  # Topic for joint commands
        self.declare_parameter('joint_name', 'kicker_joint')  # Name of the joint to command
        self.declare_parameter('no_detection_timeout', 1.0)  # Timeout in seconds for publishing zero command

        self.target_classes = self.get_parameter('target_classes').get_parameter_value().string_array_value
        # Get the bounding box range values
        self.bbox_range = {
            'x_min': self.get_parameter('x_min').get_parameter_value().double_value,
            'x_max': self.get_parameter('x_max').get_parameter_value().double_value,
            'y_min': self.get_parameter('y_min').get_parameter_value().double_value,
            'y_max': self.get_parameter('y_max').get_parameter_value().double_value,
        }
        self.joint_command_topic = self.get_parameter('joint_command_topic').get_parameter_value().string_value
        self.joint_name = self.get_parameter('joint_name').get_parameter_value().string_value
        self.no_detection_timeout = self.get_parameter('no_detection_timeout').get_parameter_value().double_value

        # Subscriber for YOLO detections
        self.subscription = self.create_subscription(
            DetectionArray,
            '/yolo/detections',  # Replace with your topic name
            self.detection_callback,
            10  # QoS
        )

        # Publisher for joint position commands
        self.joint_command_publisher = self.create_publisher(
            JointState,
            self.joint_command_topic,
            10
        )

        # Timer to check for no detections
        self.timer = self.create_timer(0.5, self.check_no_detections)

        # Last detection timestamp
        self.last_detection_time = self.get_clock().now()

        self.get_logger().info(f'YOLO Detection Actor Node started with joint command topic: {self.joint_command_topic}')

    def detection_callback(self, msg):
        """
        Callback to process YOLO detections.
        """
        
        bbox_range = self.bbox_range

        for detection in msg.detections:
            # Check if the detected class is in the target list
            if detection.class_name in self.target_classes:
                # Check if the bounding box position is within the specified range
                bbox = detection.bbox
                self.get_logger().info(f"Detected {detection.class_name} at: ")
                self.get_logger().info(f"({bbox.center.position.x}, {bbox.center.position.y})")
                self.get_logger().info(f"({bbox_range['x_min']}, {bbox_range['x_max']},{bbox_range['y_min']}, {bbox_range['y_max']})")
                if (bbox.center.position.x >= bbox_range['x_min'] and
                        bbox.center.position.x <= bbox_range['x_max'] and
                        bbox.center.position.y >= bbox_range['y_min'] and
                        bbox.center.position.y <= bbox_range['y_max']):
                    self.get_logger().info(f"Detected {detection.class_name} within range: ")
                    self.get_logger().info(f"({bbox.center.position.x}, {bbox.center.position.y})")
                    self.send_joint_position_command(1.0)
                    self.last_detection_time = self.get_clock().now()

    def send_joint_position_command(self, position):
        """
        Publish a joint position command using the JointState message.
        """
        command_msg = JointState()
        command_msg.name = [self.joint_name]  # Specify the joint name
        command_msg.position = [position]  # Set the position
        command_msg.velocity = []  # Optionally leave empty
        command_msg.effort = []  # Optionally leave empty
        self.joint_command_publisher.publish(command_msg)
        self.get_logger().info(f'Sent joint position command: {self.joint_name} -> {position}')


    def check_no_detections(self):
        """
        Timer callback to check for no detections and publish zero command if needed.
        """
        time_since_last_detection = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        if time_since_last_detection > self.no_detection_timeout:
            self.send_joint_position_command(0.0)  # Publish zero command

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectionActor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
