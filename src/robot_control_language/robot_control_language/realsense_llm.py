import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import tempfile
import threading
import queue
from robot_control_language.llm import OpenAIAgent
from robot_control_language.coffee.tools import hercules_functions as available_functions
from wozniak_interfaces.srv import TriggerLLM


class MultimodalLLMNode(Node):
    def __init__(self):
        super().__init__('multimodal_llm_node')

        self.declare_parameter('camera_name', 'camera')
        self.declare_parameter('camera_namespace', 'camera')
        self.declare_parameter('model', 'gpt-4o-mini')
        self.declare_parameter('settings_file', '/home/luiz/I2CA/wozniak_coffee_test/src/robot_control_language/robot_control_language/coffee/settings.yaml')

        camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        camera_namespace = self.get_parameter('camera_namespace').get_parameter_value().string_value
        model = self.get_parameter('model').get_parameter_value().string_value
        settings_file = self.get_parameter('settings_file').get_parameter_value().string_value

        # Subscription to the RealSense `image_raw` topic
        self.subscription = self.create_subscription(
            Image,
            f'/{camera_namespace}/color/image_raw',
            self.image_callback,
            10
        )

        # Initialize CvBridge to convert ROS Image messages to OpenCV format
        self.bridge = CvBridge()

        # Variable to store the latest image frame
        self.latest_frame = None

        # Thread-safe queue to handle user input
        self.input_queue = queue.Queue()

        # Initialize the OpenAIAgent
        api_key = os.getenv('OPENAI_API_KEY')
        self.agent = OpenAIAgent(model, available_functions, settings_file, api_key=api_key)

        # Initialize the service to trigger the OpenAI agent
        self.trigger_llm_srv = self.create_service(TriggerLLM, 'trigger_llm', self.trigger_llm)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def trigger_llm(self, request, response):
        if self.latest_frame is None:
            self.get_logger().warn('No image available to send.')
            response.success = False
            return response

        # Save the latest frame as a temporary image file
        with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as temp_image_file:
            image_path = temp_image_file.name
            cv2.imwrite(image_path, self.latest_frame)

        try:
            # Invoke the OpenAIAgent with the image and message
            llm_response = self.agent.invoke(request.message, image_path)
            self.get_logger().info(f"Response from OpenAI: {llm_response}")
            response.success = True
            response.message = llm_response[0].content
        except Exception as e:
            self.get_logger().error(f"Failed to send data to OpenAI: {e}")
            response.success = False
            return response
        finally:
            # Cleanup the temporary image file
            if os.path.exists(image_path):
                os.remove(image_path)
            return response

def main(args=None):
    rclpy.init(args=args)

    node = MultimodalLLMNode()

    try:
        while rclpy.ok():
            # Process ROS messages
            rclpy.spin_once(node, timeout_sec=0.1)

            # Check if "Enter" was pressed
            if not node.input_queue.empty():
                node.input_queue.get()  # Clear the queue
                node.send_to_openai()

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
