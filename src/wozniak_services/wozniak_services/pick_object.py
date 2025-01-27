import re
import rclpy.time
from wozniak_interfaces.srv import PickObject
from sensor_msgs.msg import Image, CameraInfo
from openai import OpenAI
from base64 import b64encode
from geometry_msgs.msg import TransformStamped, Point
import tf.transformations as transformations
import tf2_ros
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node


class PickObjectService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.latest_frame = None
        self.latest_depth_frame = None
        self.camera_info = None
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.srv = self.create_service(PickObject, 'pick_object', self.pick_object_callback)
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
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
        if not self.latest_frame or not self.latest_depth_frame or not self.camera_info:
            response.success = False
            response.message = 'No camera data available'
            return response

        x, y = self.image_recognition(request.object)
        position = self.get_3d_position(x, y)
        self.publish_transform(position)

        response.success = True
        response.message = 'Object picked'
        return response

    def image_recognition(self, object):
        # Call the OpenAI API to recognize the object. Return the coordinates of the object in percent.
        client = OpenAI(base_url="http://10.9.8.252:8000/api/v1")
        response = client.chat.completions.create(
            messages=[{"role": "system", "content": "You must recognize the location of a given object."},
                      {"role": "user", "content": [
                            {"type": "text", "content": "Where is the " + object + " located?"},
                            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64encode(self.latest_frame.data)}"}}
                      ]}],
            model="lcad-ica",
            max_completion_tokens=300,
        )
        result = response.choices[0].message.content
        matches = re.findall(r'x\d+="(\d+\.\d+)" y\d+="(\d+\.\d+)"', result)
        if not matches:
            raise Exception("Object not found")
        x, y = matches[0]
        if x < 0 or x > 100 or y < 0 or y > 100:
            raise Exception("Object not found")
        x_pixel = int(float(x) * self.latest_frame.width / 100)
        y_pixel = int(float(y) * self.latest_frame.height / 100)
        return x_pixel, y_pixel
    
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