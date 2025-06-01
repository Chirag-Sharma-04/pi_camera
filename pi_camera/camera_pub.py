import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from picamera2 import Picamera2
from libcamera import Transform
import cv2
import numpy as np

class PiCameraPublisher(Node):
    def __init__(self):
        super().__init__('picamera_publisher')

        # Create publisher
        self.publisher_ = self.create_publisher(
            CompressedImage,
            '/image/compressed',
            10
        )

        # Setup PiCamera2 with RGB format for lores
        self.picam2 = Picamera2()
        video_config = self.picam2.create_video_configuration(
            main={"size": (1280, 720)},
            lores={"size": (640, 480), "format": "BGR888"},
            transform=Transform(hflip=False, vflip=False)
        )
        self.picam2.configure(video_config)
        self.picam2.start()

        # Timer for 30 FPS publishing
        timer_period = 1.0 / 30.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Capture frame from 'lores' stream (RGB format)
        frame = self.picam2.capture_array("lores")

        # Convert RGB to BGR for OpenCV
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Flip image vertically
        frame_flipped = cv2.flip(frame_bgr, 0)

        # Encode image as JPEG
        ret, buffer = cv2.imencode('.jpg', frame_flipped)
        if not ret:
            self.get_logger().warn('Failed to compress image')
            return

        # Create and publish message
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = buffer.tobytes()
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PiCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
