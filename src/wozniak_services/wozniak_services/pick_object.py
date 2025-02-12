import re
import rclpy.time
from wozniak_interfaces.srv import PickObject
from sensor_msgs.msg import Image, CameraInfo
from openai import OpenAI
from base64 import b64encode
from geometry_msgs.msg import TransformStamped, Point
import tf_transformations as transformations
import tf2_ros
from cv_bridge import CvBridge
import cv2

import rclpy
from rclpy.node import Node


class PickObjectService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.latest_frame = None
        self.latest_depth_frame = None
        self.camera_info = None
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.srv = self.create_service(PickObject, 'pick_object', self.pick_object_callback)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.camera_info_callback, 10)

    def image_callback(self, msg):
        self.latest_frame = msg

    def depth_callback(self, msg):
        self.latest_depth_frame = self.bridge.imgmsg_to_cv2(msg, msg.encoding)

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def pick_object_callback(self, request, response):
        self.get_logger().info('Incoming request to pick object: %s' % (request.object))
        if self.latest_frame is None or self.latest_depth_frame is None or self.camera_info is None:
            self.get_logger().error('No camera data available')
            response.success = False
            response.message = 'No camera data available'
            return response

        x, y = self.image_recognition(request.object)
        # x, y = 50, 50
        self.get_logger().info('Object detected at x: %d, y: %d' % (x, y))
        position = self.get_3d_position(x, y)
        self.get_logger().info(f"Object position: {position}")
        self.publish_transform(position)
        self.get_logger().info(f"TF published")
    
        response.success = True
        response.message = 'Object picked'
        return response

    def encode_latest_frame_base64(self):
        """Converte sensor_msgs/Image em imagem OpenCV e salva em disco."""
        cv_image = self.bridge.imgmsg_to_cv2(self.latest_frame, self.latest_frame.encoding)
        # Convert OpenCV image to JPEG format in memory
        _, buffer = cv2.imencode(".jpg", cv_image)

        # Encode the image buffer in base64
        base64_str = b64encode(buffer).decode("utf-8")

        return base64_str

    def image_recognition(self, object):
        """Calls OpenAI API to recognize an object and retries up to 3 times if not found."""
        client = OpenAI(base_url="http://10.9.8.252:8000/v1")
        
        max_retries = 3
        for attempt in range(max_retries):
            self.get_logger().info(f"Attempt {attempt + 1} to recognize object: {object}")

            self.get_logger().info('Calling OpenAI API')
            response = client.chat.completions.create(
                messages=[{"role": "user", "content": [
                                {"type": "text", "text": "Point to the " + object + " in this image."},
                                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{self.encode_latest_frame_base64()}"}}
                        ]}],
                model="lcad-ica",
                max_completion_tokens=300,
            )

            result = response.choices[0].message.content
            self.get_logger().info('OpenAI response: %s' % result)

            matches = re.findall(r'<point x="([\d.]+)" y="([\d.]+)"', result)
            self.get_logger().info('Matches: %s' % matches)

            if matches:
                x = float(matches[0][0])
                y = float(matches[0][1])
                self.get_logger().info(f"x: {x}, y: {y}")

                if 0 <= x <= 100 and 0 <= y <= 100:
                    x_pixel = int(x * self.latest_frame.width / 100)
                    y_pixel = int(y * self.latest_frame.height / 100)
                    return x_pixel, y_pixel

            self.get_logger().warning(f"Object not found, retrying... ({attempt + 1}/{max_retries})")

        raise Exception("Object not found after 3 attempts")
    
    def get_3d_position(self, x, y):
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        depth = self.latest_depth_frame[y, x] * 0.001
        position_x = (x - cx) * depth / fx
        position_y = (y - cy) * depth / fy
        position_z = depth
        return (position_x, position_y, position_z)
    
    def publish_transform(self, position):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera_color_optical_frame"
        t.child_frame_id = "detected_object"

        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]

        # No rotation needed, so set quaternion to identity
        quat = transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    minimal_service = PickObjectService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()