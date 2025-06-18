import re
import rclpy.time
from wozniak_interfaces.srv import PickObject
from wozniak_interfaces.srv import Coord
from sensor_msgs.msg import Image, CameraInfo
from openai import OpenAI
from base64 import b64encode
from geometry_msgs.msg import TransformStamped, Point
import tf_transformations as transformations
import tf2_ros
from cv_bridge import CvBridge
import cv2
import traceback
import math
import numpy as np

import rclpy
from rclpy.node import Node


class PickObjectService(Node):
    def __init__(self):
        super().__init__('pick_object')
        self.srv = self.create_service(PickObject, 'pick_object', self.pick_object_callback)
        self.coord_client = self.create_client(Coord, 'Coord')
        self.latest_frame = None
        self.latest_depth_frame = None
        self.camera_info = None
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Imagem com as marcações acumuladas
        self.marked_image = None
        
        # Flags para controle de inicialização
        self.rgb_initialized = False
        self.depth_initialized = False
        self.camera_info_initialized = False
        
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.camera_info_callback, 10)
        self.get_logger().info('Serviço pick_object iniciado. Aguardando dados da câmera...')

        # Parâmetros de calibração
        self.declare_parameter('offset_pitch', 0.0)  # em graus
        self.declare_parameter('offset_roll', 0.0)   # em graus
        self.declare_parameter('offset_yaw', 0.0)    # em graus
        
        while not self.coord_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço "Coord" não disponível, aguardando...')
        
        self.get_logger().info("Serviço 'pick_object' pronto para receber requisições.")
        self.get_logger().info("Para alterar os parâmetros de offset em tempo de execução, use 'ros2 param set /pick_object <param_name> <value>'")

    def image_callback(self, msg):
        try:
            self.latest_frame = msg
            # Sempre atualiza a imagem marcada com a nova imagem
            self.marked_image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
            
            if not self.rgb_initialized:
                self.rgb_initialized = True
                self.get_logger().info('✓ Câmera RGB inicializada')
                self._check_all_initialized()
                
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')

    def depth_callback(self, msg):
        """Callback para imagem de profundidade"""
        try:
            self.latest_depth_frame = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
            if not self.depth_initialized:
                self.depth_initialized = True
                self.get_logger().info('✓ Câmera de profundidade inicializada')
                self._check_all_initialized()
        except Exception as e:
            self.get_logger().error(f'Error in depth callback: {str(e)}')

    def camera_info_callback(self, msg):
        """Callback para informações da câmera"""
        try:
            self.camera_info = msg
            if not self.camera_info_initialized:
                self.camera_info_initialized = True
                self.get_logger().info('✓ Informações da câmera recebidas')
                self._check_all_initialized()
        except Exception as e:
            self.get_logger().error(f'Error in camera info callback: {str(e)}')
            
    def _check_all_initialized(self):
        if self.rgb_initialized and self.depth_initialized and self.camera_info_initialized:
            self.get_logger().info('✓ Sistema totalmente inicializado e pronto para uso')

    def pick_object_callback(self, request, response):
        """Callback do serviço para detectar um objeto e retornar suas coordenadas."""
        target_object = request.target_object
        self.get_logger().info(f'Requisição recebida para encontrar o objeto: "{target_object}"')

        # Checagem de inicialização dos sensores
        if not all([self.rgb_initialized, self.depth_initialized, self.camera_info_initialized]):
            self.get_logger().error('Sensores não inicializados. Abortando requisição.')
            response.success = False
            return response

        self.get_logger().info(f'Executando ciclo de detecção para: "{target_object}"')
        
        try:
            pixel_coords_list = self.image_recognition(target_object)
            
            if not pixel_coords_list:
                self.get_logger().warn(f'Nenhum objeto "{target_object}" encontrado neste ciclo.')
                response.success = False
                return response

            # Processa apenas a primeira instância encontrada
            x_pixel, y_pixel = pixel_coords_list[0]
            object_identifier = f"{target_object}_1"
            self.get_logger().info(f"--- Iniciando processamento para {object_identifier} em pixels ({x_pixel}, {y_pixel}) ---")
            
            try:
                # 1. Obter posição 3D
                position_3d_camera_frame = self.get_3d_position(x_pixel, y_pixel)
                if position_3d_camera_frame is None:
                    self.get_logger().error(f"Falha ao obter posição 3D para {object_identifier}.")
                    response.success = False
                    return response

                # 2. Publicar TF e obter coordenadas finais
                final_coords = self.publish_transform(position_3d_camera_frame, object_identifier)
                
                if final_coords is None:
                    self.get_logger().error(f"Falha ao publicar TF ou calcular coordenadas finais para {object_identifier}.")
                    response.success = False
                    return response
                    
                # 3. Chamar o serviço Coord para enviar para o Unity
                if self.coord_client.service_is_ready():
                    coord_request = Coord.Request()
                    coord_request.x = final_coords[0]
                    coord_request.y = final_coords[1]
                    coord_request.z = final_coords[2]
                    
                    self.get_logger().info(f"Enviando para o serviço Coord: X={coord_request.x:.3f}, Y={coord_request.y:.3f}, Z={coord_request.z:.3f}")
                    self.coord_client.call_async(coord_request)
                    
                    # 4. Preencher a resposta do serviço pick_object
                    response.x = final_coords[0]
                    response.y = final_coords[1]
                    response.z = final_coords[2]
                    response.success = True
                    self.get_logger().info(f"Coordenadas para '{object_identifier}' enviadas para o Unity: X={response.x:.3f}, Y={response.y:.3f}, Z={response.z:.3f}")
                else:
                    self.get_logger().error("Serviço Coord não está pronto. Não foi possível enviar para o Unity.")
                    response.success = False

            except Exception as e_obj_processing:
                self.get_logger().error(f"Erro crítico durante o processamento de {object_identifier}: {str(e_obj_processing)}")
                self.get_logger().error(f'Stack trace para {object_identifier}: {traceback.format_exc()}')
                response.success = False
        
        except Exception as e_main_detection_loop:
            self.get_logger().error(f'Falha no ciclo de detecção: {str(e_main_detection_loop)}')
            if not ("MolmoAI não encontrou o objeto" in str(e_main_detection_loop) or 
                    "Nenhuma instância válida" in str(e_main_detection_loop) or 
                    "Nenhuma coordenada de pixel válida" in str(e_main_detection_loop)):
                self.get_logger().error(f'Stack trace: {traceback.format_exc()}')
            response.success = False
        
        return response

    def encode_latest_frame_base64(self):
        """Converte sensor_msgs/Image em imagem OpenCV e salva em disco."""
        cv_image = self.bridge.imgmsg_to_cv2(self.latest_frame, self.latest_frame.encoding)
        # Convert OpenCV image to JPEG format in memory
        _, buffer = cv2.imencode(".jpg", cv_image)

        # Encode the image buffer in base64
        base64_str = b64encode(buffer).decode("utf-8")

        return base64_str

    def image_recognition(self, target_object):
        client = OpenAI(
            base_url="http://10.9.8.252:8000/v1",
            api_key="not-needed"
        )
        self.get_logger().info(f'Procurando por 1 instância de: {target_object}')
        
        if self.latest_frame is None:
            self.get_logger().error('Nenhuma imagem disponível da câmera para reconhecimento.')
            raise Exception("Nenhuma imagem disponível da câmera para reconhecimento.")

        try:
            self.marked_image = self.bridge.imgmsg_to_cv2(self.latest_frame, self.latest_frame.encoding)
        except Exception as e_bridge:
            self.get_logger().error(f'Falha ao converter imagem ROS (frame atual) para OpenCV: {str(e_bridge)}')
            raise Exception(f'Falha ao converter imagem ROS (frame atual): {str(e_bridge)}')
                    
        self.get_logger().info('Convertendo imagem para base64...')
        try:
            base64_image = self.encode_latest_frame_base64()
        except Exception as e_b64:
            self.get_logger().error(f'Falha ao codificar frame para base64: {str(e_b64)}')
            raise Exception(f'Falha ao codificar frame para base64: {str(e_b64)}')
        self.get_logger().info('Imagem convertida para base64 com sucesso')
        
        # Prompt para localizar 1 objeto.
        prompt_text = (
            f"Locate one {target_object} in the image. Mark its center point using percentage coordinates. "
            f"The coordinates should be given as percentages where (0,0) is the top-left corner and (100,100) is the bottom-right corner. "
            f"Provide the coordinates in exactly this format: x=\"<percentage>\" y=\"<percentage>\". "
            f"For example: Found {target_object} at x=\"50.0\" y=\"60.0\""
        )
        self.get_logger().info(f"Prompt para MolmoAI: {prompt_text}")
        
        try:
            response = client.chat.completions.create(
                messages=[{
                    "role": "user", 
                    "content": [
                        {"type": "text", "text": prompt_text},
                        {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}}
                    ]
                }],
                model="lcad-ica",
                max_completion_tokens=500,
            )
            result_content = response.choices[0].message.content
            self.get_logger().info(f'Resposta MolmoAI: {result_content}')
        except Exception as e_openai:
            self.get_logger().error(f'Erro na chamada da API OpenAI: {str(e_openai)}')
            raise Exception(f'Erro na chamada da API OpenAI: {str(e_openai)}')

        # Tenta diferentes padrões de resposta
        detected_coords = []
        
        # Padrão 1: x="..." y="..."
        matches = re.findall(r'x="(\d+\.?\d*)" y="(\d+\.?\d*)"', result_content)
        if matches:
            detected_coords.extend([(float(x), float(y)) for x, y in matches])
            
        # Padrão 2: x1,y1,x2,y2 format (convertendo para ponto central)
        if not detected_coords:
            bbox_matches = re.findall(r'x1="(\d+\.?\d*)" y1="(\d+\.?\d*)" x2="(\d+\.?\d*)" y2="(\d+\.?\d*)"', result_content)
            if bbox_matches:
                for (x1, y1, x2, y2) in bbox_matches:
                    try:
                        x_center = (float(x1) + float(x2)) / 2.0
                        y_center = (float(y1) + float(y2)) / 2.0
                        detected_coords.append((x_center, y_center))
                        self.get_logger().info(f'Convertido bbox ({x1},{y1},{x2},{y2}) para centro ({x_center:.1f},{y_center:.1f})')
                    except ValueError as e:
                        self.get_logger().warn(f'Falha ao converter coordenadas da bbox: {str(e)}')
                        continue

        # Padrão 3: Procura por números próximos que possam ser coordenadas
        if not detected_coords:
            number_pairs = re.findall(r'(\d+\.?\d*)\D+(\d+\.?\d*)', result_content)
            for x_str, y_str in number_pairs:
                try:
                    x, y = float(x_str), float(y_str)
                    if 0 <= x <= 100 and 0 <= y <= 100:
                        detected_coords.append((x, y))
                        self.get_logger().info(f'Encontrado par de coordenadas válido: ({x:.1f}, {y:.1f})')
                except ValueError:
                    continue

        if not detected_coords:
            self.get_logger().info(f'MolmoAI não retornou coordenadas válidas. Resposta: {result_content}')
            try:
                cv2.imwrite("/tmp/current_scene_no_detection.jpg", self.marked_image)
                self.get_logger().info('Imagem (sem detecções) salva em /tmp/current_scene_no_detection.jpg')
            except Exception as e_save:
                self.get_logger().error(f'Falha ao salvar imagem sem detecções: {str(e_save)}')
            raise Exception(f'Não foi possível encontrar coordenadas válidas para "{target_object}" na imagem.')

        # Processa as coordenadas encontradas, limitado a 1.
        detected_pixels_list = []
        for i, (x_percent, y_percent) in enumerate(detected_coords[:1]): # Limita a 1 objeto
            try:
                if not (0 <= x_percent <= 100 and 0 <= y_percent <= 100):
                    self.get_logger().warn(f'Coordenadas percentuais inválidas para {target_object}_{i+1}: ({x_percent:.1f}%, {y_percent:.1f}%). Pulando.')
                    continue

                x_pixel = int(x_percent * self.latest_frame.width / 100)
                y_pixel = int(y_percent * self.latest_frame.height / 100)
                
                x_pixel = np.clip(x_pixel, 0, self.latest_frame.width - 1)
                y_pixel = np.clip(y_pixel, 0, self.latest_frame.height - 1)

                detected_pixels_list.append((x_pixel, y_pixel))

                # Desenha círculo e rótulo
                cv2.circle(self.marked_image, (x_pixel, y_pixel), 10, (0, 255, 0), 2)
                label = f"{target_object}_{i+1}"
                cv2.putText(self.marked_image, label, (x_pixel + 15, y_pixel), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                # Desenha cruz no centro para melhor visualização
                size = 5
                cv2.line(self.marked_image, 
                        (x_pixel - size, y_pixel), 
                        (x_pixel + size, y_pixel), 
                        (0, 0, 255), 2)
                cv2.line(self.marked_image, 
                        (x_pixel, y_pixel - size), 
                        (x_pixel, y_pixel + size), 
                        (0, 0, 255), 2)
                
                self.get_logger().info(f'{target_object}_{i+1} identificado em: pixel({x_pixel}, {y_pixel}) - percentual({x_percent:.1f}%, {y_percent:.1f}%)')
            except Exception as e_proc:
                self.get_logger().error(f"Erro ao processar coordenadas para {target_object}_{i+1}: {str(e_proc)}")
                continue

        if not detected_pixels_list:
            self.get_logger().error(f'Nenhuma coordenada válida foi processada com sucesso para "{target_object}"')
            try:
                cv2.imwrite("/tmp/current_scene_invalid_all.jpg", self.marked_image)
            except Exception as e_save:
                self.get_logger().error(f'Falha ao salvar imagem: {str(e_save)}')
            raise Exception(f'Falha ao processar coordenadas para "{target_object}"')

        # Salva a imagem com as marcações
        try:
            self.get_logger().info('Salvando imagem com marcações em /tmp/current_scene.jpg...')
            success = cv2.imwrite("/tmp/current_scene.jpg", self.marked_image)
            if success:
                self.get_logger().info(f'Imagem com {len(detected_pixels_list)} marcações salva com sucesso')
            else:
                self.get_logger().error('Falha ao salvar a imagem com marcações')
        except Exception as e_save:
            self.get_logger().error(f'Erro ao salvar imagem final: {str(e_save)}')

        return detected_pixels_list
    
    def get_3d_position(self, x, y):
        """
        Converte coordenadas da imagem RGB para coordenadas 3D usando a imagem de profundidade
        """
        # Ajusta as coordenadas da imagem RGB para a resolução da imagem de profundidade
        depth_height, depth_width = self.latest_depth_frame.shape
        rgb_height = self.latest_frame.height
        rgb_width = self.latest_frame.width
        
        # Converte as coordenadas proporcionalmente
        x_depth = int((x / rgb_width) * depth_width)
        y_depth = int((y / rgb_height) * depth_height)
        
        # Garante que as coordenadas estão dentro dos limites
        x_depth = min(max(0, x_depth), depth_width - 1)
        y_depth = min(max(0, y_depth), depth_height - 1)
        
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        
        depth = self.latest_depth_frame[y_depth, x_depth] * 0.001  # converte para metros
        
        # Usa as coordenadas originais da imagem RGB para o cálculo 3D
        position_x = (x - cx) * depth / fx
        position_y = (y - cy) * depth / fy
        position_z = depth
        
        self.get_logger().info(f"[DEBUG get_3d_position] x_pixel: {x}, y_pixel: {y}")
        self.get_logger().info(f"[DEBUG get_3d_position] depth_val_at_pixel: {self.latest_depth_frame[y_depth, x_depth]}, depth_meters: {depth}")
        self.get_logger().info(f"[DEBUG get_3d_position] fx: {fx}, fy: {fy}, cx: {cx}, cy: {cy}")
        self.get_logger().info(f"[DEBUG get_3d_position] Calculated ROS Coords: X={position_x}, Y={position_y}, Z={position_z}")
        
        return (position_x, position_y, position_z)
    
    def publish_transform(self, position, object_frame_id):
        # As coordenadas da câmera são usadas diretamente, sem aplicar rotação de offset.
        final_x, final_y, final_z = position

        self.get_logger().info(f"[DEBUG publish_transform] Original cam position for {object_frame_id}: {position}")
        self.get_logger().info(f"[DEBUG publish_transform] Final position (no rotation): ({final_x:.3f}, {final_y:.3f}, {final_z:.3f})")
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera_color_optical_frame" # Ou o frame base apropriado se a câmera se move com o robô
        t.child_frame_id = object_frame_id

        t.transform.translation.x = final_x
        t.transform.translation.y = final_y
        t.transform.translation.z = final_z

        # A rotação na TF permanece como identidade, significando que o frame do objeto
        # terá a mesma orientação do 'camera_color_optical_frame'.
        quat = transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"TF publicada para {object_frame_id} em ({final_x:.3f}, {final_y:.3f}, {final_z:.3f})")

        return final_x, final_y, final_z # Retorna as coordenadas finais calculadas

def main(args=None):
    rclpy.init(args=args)

    minimal_service = PickObjectService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()