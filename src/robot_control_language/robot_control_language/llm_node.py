#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import tempfile
from .llm import OpenAIAgent
from wozniak_interfaces.srv import PickObject


class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')
        
        # Inicializa o OpenAIAgent
        self.agent = OpenAIAgent()
        
        # Bridge para converter mensagens ROS Image para OpenCV
        self.bridge = CvBridge()
        
        # Subscriber para imagem da câmera
        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # Cliente para o serviço pick_object
        self.pick_object_client = self.create_client(PickObject, 'pick_object')
        
        # Variável para armazenar a última imagem
        self.latest_image = None
        
        self.get_logger().info('LLM Node iniciado')
        
    def image_callback(self, msg):
        """Callback para processar imagens recebidas"""
        self.latest_image = msg
        
    def pick_object(self, object_name):
        """Função que será passada para o OpenAIAgent"""
        if not self.latest_image:
            self.get_logger().error('Nenhuma imagem disponível')
            return "Erro: Nenhuma imagem disponível"
            
        # Converte a imagem ROS para OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
        
        # Salva a imagem temporariamente
        with tempfile.NamedTemporaryFile(suffix='.jpg') as temp_file:
            cv2.imwrite(temp_file.name, cv_image)
            
            # Chama o LLM com a imagem
            prompt = f"Point to the {object_name} in this image."
            responses = self.agent.invoke(prompt, temp_file.name)
            
            # O processamento da resposta e chamada do pick_object já é feito
            # dentro do método invoke do OpenAIAgent
            
            return "Processamento completo"


def main(args=None):
    rclpy.init(args=args)
    
    llm_node = LLMNode()
    
    try:
        rclpy.spin(llm_node)
    except KeyboardInterrupt:
        pass
    finally:
        llm_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 