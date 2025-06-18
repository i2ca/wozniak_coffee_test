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
from wozniak_interfaces.srv import TriggerLLM, InstructionsLLM
from openai.types.chat.chat_completion_message import ChatCompletionMessage


class MultimodalLLMNode(Node):
    def __init__(self):
        super().__init__('multimodal_llm_node')

        self.declare_parameter('camera_name', 'camera')
        self.declare_parameter('camera_namespace', 'camera')
        self.declare_parameter('model', 'gpt-4o-mini')
        self.declare_parameter('settings_file', '/your/home/path/I2CA/wozniak_coffee_test/src/robot_control_language/robot_control_language/coffee/settings.yaml')

        camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        camera_namespace = self.get_parameter('camera_namespace').get_parameter_value().string_value
        model = self.get_parameter('model').get_parameter_value().string_value
        settings_file = self.get_parameter('settings_file').get_parameter_value().string_value

        # Subscription to the RealSense `image_raw` topic
        self.subscription = self.create_subscription(
            Image,
            f'/camera/camera/color/image_raw',
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
        self.agent = OpenAIAgent(model, available_functions, settings_file, self)

        # Initialize the service to trigger the OpenAI agent
        self.TriggerLLM_srv = self.create_service(TriggerLLM, 'TriggerLLM', self.TriggerLLM)
        self.InstructionsLLM_srv = self.create_service(InstructionsLLM, 'InstructionsLLM', self.InstructionsLLM)

        # Client to send instructions to Unity (or other consumer of InstructionsLLM)
        self.unity_instructions_client = self.create_client(InstructionsLLM, "InstructionsLLM")
        if not self.unity_instructions_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Serviço "InstructionsLLM" (para Unity/destino final) não disponível após 5 segundos.')
        else:
            self.get_logger().info('Cliente para "InstructionsLLM" (para Unity/destino final) conectado.')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def TriggerLLM(self, request, response):
        self.get_logger().info(f"Received request to trigger LLM with message: {request.message}")
        image_path = None

        if self.latest_frame is None:
            self.get_logger().warn('No image available to send.')
            response.success = False
            return response

        # Save the latest frame as a temporary image file
        with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as temp_image_file:
            image_path = temp_image_file.name
            rgb_frame = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2RGB)
            cv2.imwrite(image_path, rgb_frame)

        try:
            # Invoke the OpenAIAgent with the image and message
            llm_interaction_list = self.agent.invoke(request.message, image_path) 
            self.get_logger().info(f"Response from OpenAI (full interaction list): {llm_interaction_list}")
            
            response.success = True
            response.message = "" # Inicializa como string vazia

            # Extrair apenas a última mensagem de conteúdo da assistente para o usuário
            if llm_interaction_list:
                for item in reversed(llm_interaction_list): # Iterar de trás para frente
                    if isinstance(item, ChatCompletionMessage) and item.role == 'assistant' and item.content is not None:
                        response.message = item.content
                        break # Encontrou a última mensagem de conteúdo da assistente
            
            if not response.message and response.success:
                 # Se success ainda é True mas não encontramos mensagem, algo pode estar inesperado
                self.get_logger().warn("LLM processada com sucesso, mas nenhuma mensagem de conteúdo final da assistente foi extraída.")
                # Considere se isso deve ser um caso de response.success = False ou se uma mensagem vazia é aceitável.

        except Exception as e:
            self.get_logger().error(f"Failed to send data to OpenAI: {e}")
            response.success = False
        finally:
            # Cleanup the temporary image file
            if image_path and os.path.exists(image_path):
                os.remove(image_path)

        if response.success and response.message:
            if self.unity_instructions_client and self.unity_instructions_client.service_is_ready():
                self.get_logger().info(f"Enviando instrução para destino final (Unity/etc) via InstructionsLLM: '{response.message}'")
                unity_request = InstructionsLLM.Request()
                unity_request.instruction = response.message
                future = self.unity_instructions_client.call_async(unity_request)
                future.add_done_callback(self._unity_call_done_callback)
            else:
                self.get_logger().error('Não foi possível enviar instrução para destino final: cliente InstructionsLLM não está pronto ou não foi inicializado.')
        elif response.success and not response.message:
            self.get_logger().warn("LLM processada com sucesso, mas nenhuma mensagem de conteúdo final da assistente para enviar ao destino final.")
        
        self.get_logger().info(f"Final response.message (TriggerLLM): '{response.message}'")
        return response

    def InstructionsLLM(self, request, response):
        self.get_logger().info(f'Serviço InstructionsLLM chamado com instrução: "{request.instruction}"')
        response.success = True
        return response

    def _unity_call_done_callback(self, future):
        try:
            unity_service_response = future.result()
            if unity_service_response is not None:
                if unity_service_response.success:
                    self.get_logger().info('Instrução enviada para destino final (via InstructionsLLM) com sucesso (callback).')
                else:
                    self.get_logger().error('Erro ao enviar instrução para destino final (via InstructionsLLM): Falha no serviço (callback).')
            else:
                self.get_logger().error('Erro ao chamar serviço InstructionsLLM para destino final: Sem resposta (None) (callback).')
        except Exception as e:
            self.get_logger().error(f'Exceção no callback da chamada InstructionsLLM para destino final: {e}')

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
