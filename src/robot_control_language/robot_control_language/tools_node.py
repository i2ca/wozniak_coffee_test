#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from .coffee.tools import hercules_functions
from wozniak_interfaces.srv import PickObject
from .llm import OpenAIAgent
import cv2
import tempfile
import os
import threading
import sys
import time
import re


class ToolsNode(Node):
    def __init__(self):
        super().__init__('tools_node')
        
        # Configurar nível de log
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        
        # Dicionário para guardar objetos e suas coordenadas
        self.identified_objects = {}
        # Dicionário para guardar as posições dos objetos na imagem
        self.object_positions = {}
        
        # Bridge para converter mensagens ROS Image para OpenCV
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_received = False
        self.current_cv_image = None  # Para armazenar a imagem atual com as marcações
        
        # Configuração de QoS para sensores
        sensor_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # Subscriber para a câmera
        self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher para saída da LLM
        self.llm_output_pub = self.create_publisher(
            String,
            'llm_output',
            10
        )
        
        # Cliente do serviço pick_object
        self.pick_object_client = self.create_client(
            PickObject,
            "pick_object"
        )
        
        # Subscriber para entrada do usuário
        self.create_subscription(
            String,
            'llm_input',
            self.user_input_callback,
            10
        )
        
        # Inicializando o OpenAIAgent (MolmoAI)
        try:
            settings_file = "/home/luiz/I2CA/wozniak_coffee_test/src/robot_control_language/robot_control_language/coffee/settings.yaml"
            self.get_logger().info('Inicializando assistente...')
            
            # Criando o cliente do serviço pick_object
            if not self.pick_object_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('Serviço pick_object não disponível, tentando continuar...')
            
            # Registrando as funções disponíveis
            available_functions = {
                'pick_object': lambda object: self.call_pick_object(object)
            }
            
            self.agent = OpenAIAgent(
                model="gpt-4",  # GPT-4 para diálogo em texto
                available_functions=available_functions,
                settings_file_path=settings_file,
                node=self
            )
            self.get_logger().info('Assistente inicializado com sucesso')
        except Exception as e:
            self.get_logger().error(f'Erro ao inicializar assistente: {str(e)}')
            raise e
        
        # Publicar mensagem inicial
        self.publish_llm_message("Olá! Como posso ajudar você hoje?")
        
        # Iniciar thread para entrada do usuário
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()
    
    def input_loop(self):
        """Loop para receber entrada do usuário pelo terminal"""
        print("\nDigite suas mensagens (Ctrl+C para sair):")
        while True:
            try:
                user_input = input("> ")
                if user_input.strip():  # Ignora linhas vazias
                    msg = String()
                    msg.data = user_input
                    self.user_input_callback(msg)
            except (KeyboardInterrupt, EOFError):
                break
            except Exception as e:
                self.get_logger().error(f'Erro ao ler entrada do usuário: {str(e)}')
                break
    
    def image_callback(self, msg):
        """Callback para processar imagens da câmera"""
        try:
            if not self.image_received:
                self.get_logger().info(f'Primeira mensagem de imagem recebida. Encoding: {msg.encoding}')
            
            # Converte a mensagem ROS para formato OpenCV
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            if not self.image_received:
                self.image_received = True
                height, width = self.latest_image.shape[:2]
                self.get_logger().info(f'Primeira imagem convertida com sucesso. Dimensões: {width}x{height}')
        except Exception as e:
            self.get_logger().error(f'Erro ao processar imagem: {str(e)}')
            self.get_logger().error(f'Detalhes da mensagem: encoding={msg.encoding}, height={msg.height}, width={msg.width}')
    
    def user_input_callback(self, msg):
        """Callback para processar entrada do usuário"""
        self.get_logger().debug(f'Processando mensagem do usuário: {msg.data}')
        
        if self.latest_image is None:
            self.publish_llm_message("Desculpe, ainda não recebi nenhuma imagem da câmera. Verifique se a câmera está publicando no tópico /camera/color/image_raw")
            return
        
        # Salvar a imagem temporariamente
        try:
            temp_dir = tempfile.gettempdir()
            temp_image_path = os.path.join(temp_dir, 'current_scene.jpg')
            cv2.imwrite(temp_image_path, self.latest_image)
            self.get_logger().debug(f'Imagem salva temporariamente em {temp_image_path}')
            
            # Processar a mensagem com a LLM
            self.get_logger().debug('Enviando requisição para a MolmoAI...')
            response = self.agent.invoke(msg.data, temp_image_path)
            
            # Processar apenas a primeira resposta
            if response and len(response) > 0:
                item = response[0]  # Pega apenas a primeira resposta
                if isinstance(item, dict):
                    if item.get('role') == 'assistant':
                        self.get_logger().debug('Resposta da LLM recebida')
                        self.publish_llm_message(item['content'])
                    elif item.get('role') == 'tool':
                        self.get_logger().debug('Resposta da ferramenta recebida')
                        self.publish_llm_message(f"Resultado da ação: {item['content']}")
            
        except Exception as e:
            self.get_logger().error(f'Erro ao processar com LLM: {str(e)}')
            self.publish_llm_message(f"Desculpe, ocorreu um erro ao processar sua solicitação: {str(e)}")
    
    def publish_llm_message(self, message):
        """Publica uma mensagem no tópico llm_output e mostra no terminal"""
        msg = String()
        msg.data = message
        self.llm_output_pub.publish(msg)
        self.get_logger().debug('Mensagem publicada')
        # Mostra a resposta diretamente no terminal
        print(f"\nAssistente: {message}")
        print("> ", end='', flush=True)  # Restaura o prompt

    def call_pick_object(self, object):
        """Função wrapper para chamar o serviço pick_object"""
        try:
            # Verifica se o objeto já foi identificado
            if object in self.identified_objects:
                return f"O objeto '{object}' já foi identificado anteriormente"
            
            request = PickObject.Request()
            request.target_object = object
            
            self.get_logger().info(f'Chamando serviço pick_object para objeto: {object}')
            future = self.pick_object_client.call_async(request)
            
            # Aguarda a resposta do serviço
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                response = future.result()
                self.get_logger().info(f'Resposta do serviço pick_object: {response.message}')
                
                if response.success:
                    # Armazena o objeto identificado
                    self.identified_objects[object] = True
                    
                    try:
                        # Copia a imagem gerada pelo serviço pick_object
                        if os.path.exists("/tmp/current_scene.jpg"):
                            # Lê a imagem atual
                            current_scene = cv2.imread("/tmp/current_scene.jpg")
                            if current_scene is not None:
                                # Salva a imagem com um novo nome para manter o histórico
                                cv2.imwrite("/tmp/objects_detected.jpg", current_scene)
                                self.get_logger().info(f'Imagem atualizada salva em /tmp/objects_detected.jpg')
                                
                                # Atualiza a mensagem de status
                                status_msg = f"\nObjeto '{object}' identificado com sucesso!\n"
                                status_msg += f"Objetos já identificados: {', '.join(self.identified_objects.keys())}\n"
                                status_msg += f"Imagem atualizada salva em /tmp/objects_detected.jpg"
                            else:
                                status_msg = f"Objeto identificado mas não foi possível ler a imagem gerada"
                        else:
                            status_msg = f"Objeto identificado mas imagem não foi encontrada"
                            
                    except Exception as e:
                        self.get_logger().error(f'Erro ao copiar imagem: {str(e)}')
                        status_msg = f"Objeto identificado mas houve erro ao salvar imagem: {str(e)}"
                    
                    return status_msg
                else:
                    return response.message
            else:
                self.get_logger().error('Falha ao chamar serviço pick_object')
                return "Falha ao localizar o objeto"
        except Exception as e:
            self.get_logger().error(f'Erro ao chamar serviço pick_object: {str(e)}')
            return f"Erro ao tentar localizar o objeto: {str(e)}"


def main(args=None):
    rclpy.init(args=args)
    
    try:
        tools_node = ToolsNode()
        print("\nPressione Ctrl+C para encerrar o programa...")
        rclpy.spin(tools_node)
    except KeyboardInterrupt:
        print("\nEncerrando o programa por solicitação do usuário (Ctrl+C)...")
    except Exception as e:
        print(f'\nErro fatal: {str(e)}')
    finally:
        if 'tools_node' in locals():
            tools_node.destroy_node()
            print("Node destruído com sucesso.")
        rclpy.shutdown()
        print("ROS2 encerrado com sucesso.\n")


if __name__ == '__main__':
    main() 