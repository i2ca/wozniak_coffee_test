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


class ToolsNode(Node):
    def __init__(self):
        super().__init__('tools_node')
        self.get_logger().info('Tools Node iniciado')
        
        # Bridge para converter mensagens ROS Image para OpenCV
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_received = False
        
        # Subscriber para a câmera
        self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info('Aguardando imagens no tópico /camera/color/image_raw...')
        
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
            settings_file = os.path.join(os.path.dirname(__file__), 'coffee', 'settings.yaml')
            self.get_logger().info(f'Carregando configurações de: {settings_file}')
            self.agent = OpenAIAgent(
                model="lcad-ica",
                available_functions=hercules_functions,
                settings_file_path=settings_file,
                node=self
            )
            self.get_logger().info('OpenAIAgent inicializado com sucesso')
        except Exception as e:
            self.get_logger().error(f'Erro ao inicializar OpenAIAgent: {str(e)}')
            raise e
        
        # Publicar mensagem inicial
        self.publish_llm_message("Olá! Sou sua assistente de visão. Como posso ajudar você hoje? Posso identificar objetos na cena usando a câmera.")
        
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
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if not self.image_received:
                self.image_received = True
                self.get_logger().info('Primeira imagem recebida da câmera!')
            self.get_logger().debug('Nova imagem recebida da câmera')
        except Exception as e:
            self.get_logger().error(f'Erro ao processar imagem: {str(e)}')
    
    def user_input_callback(self, msg):
        """Callback para processar entrada do usuário"""
        self.get_logger().info(f'Recebida mensagem do usuário: {msg.data}')
        
        if self.latest_image is None:
            self.publish_llm_message("Desculpe, ainda não recebi nenhuma imagem da câmera. Verifique se a câmera está publicando no tópico /camera/color/image_raw")
            return
        
        # Salvar a imagem temporariamente
        try:
            temp_dir = tempfile.gettempdir()
            temp_image_path = os.path.join(temp_dir, 'current_scene.jpg')
            cv2.imwrite(temp_image_path, self.latest_image)
            self.get_logger().info(f'Imagem salva em: {temp_image_path}')
            
            # Processar a mensagem com a LLM
            self.get_logger().info('Enviando requisição para a MolmoAI...')
            response = self.agent.invoke(msg.data, temp_image_path)
            
            # Processar a resposta
            for item in response:
                if isinstance(item, dict):
                    if item.get('role') == 'assistant':
                        self.get_logger().info(f'Resposta da LLM: {item["content"]}')
                        self.publish_llm_message(item['content'])
                    elif item.get('role') == 'tool':
                        self.get_logger().info(f'Resposta da ferramenta: {item["content"]}')
                        self.publish_llm_message(f"Resultado da ação: {item['content']}")
            
        except Exception as e:
            self.get_logger().error(f'Erro ao processar com LLM: {str(e)}')
            self.publish_llm_message(f"Desculpe, ocorreu um erro ao processar sua solicitação: {str(e)}")
        finally:
            # Limpar arquivo temporário
            if os.path.exists(temp_image_path):
                os.remove(temp_image_path)
                self.get_logger().debug('Arquivo temporário removido')
    
    def publish_llm_message(self, message):
        """Publica uma mensagem no tópico llm_output e mostra no terminal"""
        msg = String()
        msg.data = message
        self.llm_output_pub.publish(msg)
        self.get_logger().info(f'Mensagem publicada: {message}')
        # Mostra a resposta diretamente no terminal
        print(f"\nAssistente: {message}")
        print("> ", end='', flush=True)  # Restaura o prompt


def main(args=None):
    rclpy.init(args=args)
    
    try:
        tools_node = ToolsNode()
        rclpy.spin(tools_node)
    except KeyboardInterrupt:
        print("\nEncerrando...")
    except Exception as e:
        print(f'\nErro fatal: {str(e)}')
    finally:
        if 'tools_node' in locals():
            tools_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 