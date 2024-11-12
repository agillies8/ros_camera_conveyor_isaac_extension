import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')

        # Initialize CvBridge to convert ROS images to OpenCV images
        self.bridge = CvBridge()

        # Define the save path for the images
        self.save_path = os.path.expanduser("~/box_images")
        os.makedirs(self.save_path, exist_ok=True)

        # Create multiple subscriptions, all using the same callback
        self.subscription = self.create_subscription(Image, '/camera/rgb', self.image_callback, 10)
        self.subscription

    def image_callback(self, msg):
        # Identify which topic the message came from
        topic_name = msg.header.frame_id

        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

            # Define the filename with a unique name (topic name + timestamp)
            filename = f"{self.save_path}/{topic_name.replace('/', '_')}_{msg.header.stamp.sec}_{msg.header.stamp.nanosec}.jpg"

            # Save the image
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f"Saved image from {topic_name}: {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to save image from {topic_name}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
