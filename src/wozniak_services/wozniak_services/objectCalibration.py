# ========================================================================================
#                             CALIBRAÇÃO TWO OBJECTS
# ========================================================================================
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
        self.latest_frame = None
        self.latest_depth_frame = None
        self.camera_info = None
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.client = self.create_client(Coord, "Coord")
        
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

        # Parâmetros de controle e calibração
        self.declare_parameter('continuous_mode', True)
        self.declare_parameter('detection_frequency', 1.0) # em Hz
        self.declare_parameter('offset_pitch', 0.0)  # em graus
        self.declare_parameter('offset_roll', 0.0)   # em graus
        self.declare_parameter('offset_yaw', 0.0)    # em graus
        
        # Timer para o loop de detecção contínua
        timer_period = 1.0 / self.get_parameter('detection_frequency').get_parameter_value().double_value
        self.timer = self.create_timer(timer_period, self.continuous_detection_callback)
        self.get_logger().info(f"Modo contínuo ativado. Detectando 'xicara' a {self.get_parameter('detection_frequency').get_parameter_value().double_value} Hz.")
        self.get_logger().info("Para alterar os parâmetros em tempo de execução, use 'ros2 param set /pick_object <param_name> <value>'")

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

    def continuous_detection_callback(self):
        """Callback do timer para detecção contínua de objetos."""
        if not self.get_parameter('continuous_mode').get_parameter_value().bool_value:
            self.get_logger().info('Modo contínuo desativado. Pausando detecção.', throttle_duration_sec=10)
            return

        target_object = "xicara"

        # Checagem de inicialização dos sensores
        if not all([self.rgb_initialized, self.depth_initialized, self.camera_info_initialized]):
            self.get_logger().info('Aguardando a inicialização completa de todos os sensores...', throttle_duration_sec=10)
            return

        self.get_logger().info(f'Executando ciclo de detecção para: "{target_object}"', throttle_duration_sec=5)
        
        try:
            pixel_coords_list = self.image_recognition(target_object)
            
            if not pixel_coords_list:
                self.get_logger().warn(f'Nenhum objeto "{target_object}" encontrado neste ciclo.', throttle_duration_sec=5)
                return

            num_detected = len(pixel_coords_list)
            self.get_logger().info(f"Processando {num_detected} instância(s) de '{target_object}'.")

            for i, (x_pixel, y_pixel) in enumerate(pixel_coords_list):
                object_identifier = f"{target_object}_{i+1}"
                self.get_logger().info(f"--- Iniciando processamento para {object_identifier} em pixels ({x_pixel}, {y_pixel}) ---")
                
                try:
                    # 1. Obter posição 3D
                    position_3d_camera_frame = self.get_3d_position(x_pixel, y_pixel)
                    if position_3d_camera_frame is None:
                        self.get_logger().error(f"Falha ao obter posição 3D para {object_identifier}. Pulando este objeto.")
                        continue

                    # 2. Publicar TF
                    final_coords_for_service = self.publish_transform(position_3d_camera_frame, object_identifier)
                    
                    if final_coords_for_service is None:
                        self.get_logger().error(f"Falha ao publicar TF ou calcular coordenadas finais para {object_identifier}. Pulando este objeto.")
                        continue
                        
                    # 3. Chamar o serviço Coord
                    if self.client and self.client.service_is_ready():
                        coord_request = Coord.Request()
                        coord_request.x = final_coords_for_service[0]
                        coord_request.y = final_coords_for_service[1]
                        coord_request.z = final_coords_for_service[2]
                        
                        self.get_logger().info(f"Enviando para o serviço Coord para {object_identifier}: X={coord_request.x:.3f}, Y={coord_request.y:.3f}, Z={coord_request.z:.3f}")
                        
                        self.client.call_async(coord_request)
                    else:
                        self.get_logger().warn(f"Serviço Coord não está pronto para {object_identifier}.", throttle_duration_sec=10)

                except Exception as e_obj_processing:
                    self.get_logger().error(f"Erro crítico durante o processamento de {object_identifier}: {str(e_obj_processing)}")
                    self.get_logger().error(f'Stack trace para {object_identifier}: {traceback.format_exc()}')
        
        except Exception as e_main_detection_loop:
            self.get_logger().error(f'Falha no ciclo de detecção contínua: {str(e_main_detection_loop)}')
            if not ("MolmoAI não encontrou o objeto" in str(e_main_detection_loop) or 
                    "Nenhuma instância válida" in str(e_main_detection_loop) or 
                    "Nenhuma coordenada de pixel válida" in str(e_main_detection_loop)):
                self.get_logger().error(f'Stack trace: {traceback.format_exc()}')
        
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
        self.get_logger().info(f'Procurando por até 4 instâncias de: {target_object}')
        
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
        
        # Prompt para localizar até 4 objetos.
        prompt_text = (
            f"Locate up to four {target_object}s in the image. For each one, mark its center point using percentage coordinates. "
            f"The coordinates should be given as percentages where (0,0) is the top-left corner and (100,100) is the bottom-right corner. "
            f"Provide the coordinates in exactly this format for each {target_object}: x=\"<percentage>\" y=\"<percentage>\". "
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
            numeric_matches = re.findall(r'(\d+\.?\d*)', result_content)
            if len(numeric_matches) >= 2:
                # Assume que os números vêm em pares (x, y)
                for i in range(0, len(numeric_matches) - 1, 2):
                    try:
                        x = float(numeric_matches[i])
                        y = float(numeric_matches[i+1])
                        if 0 <= x <= 100 and 0 <= y <= 100: # Validação de range
                            detected_coords.append((x,y))
                            self.get_logger().info(f'Encontrado par numérico e adicionado como coordenadas: ({x}, {y})')
                    except (ValueError, IndexError):
                        continue # Ignora se não for um par válido

        if not detected_coords:
            self.get_logger().error('MolmoAI não encontrou o objeto ou a resposta está em um formato inesperado.')
            raise Exception(f'Não foi possível encontrar coordenadas válidas para "{target_object}" na imagem.')

        # Processa as coordenadas encontradas, limitado a 4.
        detected_pixels_list = []
        for i, (x_percent, y_percent) in enumerate(detected_coords[:4]): # Limita a 4 objetos
            try:
                if not (0 <= x_percent <= 100 and 0 <= y_percent <= 100):
                    self.get_logger().warn(f'Coordenada inválida (fora do range 0-100) recebida: ({x_percent}, {y_percent}). Pulando.')
                    continue

                img_h, img_w = self.marked_image.shape[:2]
                x_pixel = int(img_w * (x_percent / 100.0))
                y_pixel = int(img_h * (y_percent / 100.0))

                detected_pixels_list.append((x_pixel, y_pixel))
                
                # Desenha um círculo no centro do objeto detectado
                cv2.circle(self.marked_image, (x_pixel, y_pixel), radius=10, color=(0, 255, 0), thickness=-1)
                cv2.putText(self.marked_image, f"{target_object}_{i+1}", (x_pixel + 15, y_pixel + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                
            except Exception as e:
                self.get_logger().error(f"Erro ao processar e desenhar coordenada {i+1}: {str(e)}")
                continue

        # Salva a imagem com as marcações para depuração
        debug_image_path = '/tmp/current_scene.jpg'
        cv2.imwrite(debug_image_path, self.marked_image)
        self.get_logger().info(f'Imagem de depuração salva em {debug_image_path} com {len(detected_pixels_list)} objeto(s) marcado(s).')

        if not detected_pixels_list:
            raise Exception("Nenhuma coordenada de pixel válida foi processada, apesar de detecções iniciais.")
            
        return detected_pixels_list
    
    def get_3d_position(self, x, y):
        """Calcula a posição 3D a partir de coordenadas de pixel e da imagem de profundidade."""
        if self.latest_depth_frame is None or self.camera_info is None:
            self.get_logger().error('Imagem de profundidade ou camera_info não disponível.')
            return None

        try:
            depth_value = self.latest_depth_frame[y, x]
            if depth_value == 0:
                self.get_logger().warn(f'Valor de profundidade é 0 no pixel ({x}, {y}). Tentando vizinhança...')
                depth_value = self.get_depth_from_neighbors(x, y, 5)
                if depth_value == 0:
                    self.get_logger().error(f'Ainda 0 de profundidade após verificar vizinhos. Não é possível calcular a posição 3D.')
                    return None

            depth_in_meters = float(depth_value) / 1000.0  # Assumindo que a profundidade está em mm

            K = self.camera_info.k
            fx, fy, cx, cy = K[0], K[4], K[2], K[5]
            
            # Converte coordenadas de pixel para coordenadas no frame da câmera
            x_cam = (x - cx) * depth_in_meters / fx
            y_cam = (y - cy) * depth_in_meters / fy
            z_cam = depth_in_meters
            
            self.get_logger().info(f'Posição 3D calculada no frame da câmera: ({x_cam:.3f}, {y_cam:.3f}, {z_cam:.3f})')
            
            return Point(x=x_cam, y=y_cam, z=z_cam)
        
        except IndexError:
            h, w = self.latest_depth_frame.shape[:2]
            self.get_logger().error(f"Coordenadas de pixel ({x}, {y}) fora dos limites da imagem de profundidade ({w}x{h}).")
            return None
        except Exception as e:
            self.get_logger().error(f'Erro ao calcular posição 3D: {traceback.format_exc()}')
            return None

    def get_depth_from_neighbors(self, x, y, radius):
        """Pega a profundidade média de uma vizinhança ao redor de um pixel, ignorando valores 0."""
        h, w = self.latest_depth_frame.shape[:2]
        x_min, x_max = max(0, x - radius), min(w - 1, x + radius)
        y_min, y_max = max(0, y - radius), min(h - 1, y + radius)
        
        neighbors = self.latest_depth_frame[y_min:y_max+1, x_min:x_max+1]
        non_zero_neighbors = neighbors[neighbors > 0]
        
        if non_zero_neighbors.size > 0:
            return np.mean(non_zero_neighbors)
        return 0

    def publish_transform(self, position, object_frame_id):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_link'
        t.child_frame_id = object_frame_id

        # Rotação (Pitch, Roll, Yaw)
        pitch = math.radians(self.get_parameter('offset_pitch').get_parameter_value().double_value)
        roll = math.radians(self.get_parameter('offset_roll').get_parameter_value().double_value)
        yaw = math.radians(self.get_parameter('offset_yaw').get_parameter_value().double_value)
        
        # O quatérnio base da câmera é (geralmente) x=-0.5, y=0.5, z=-0.5, w=0.5
        # para alinhar com o frame do robô. Aplicamos os offsets a essa rotação.
        base_quat = transformations.quaternion_from_euler(math.pi, 0, -math.pi/2)
        offset_quat = transformations.quaternion_from_euler(roll, pitch, yaw)
        final_quat = transformations.quaternion_multiply(base_quat, offset_quat)
        
        # Define a rotação da transformada
        t.transform.rotation.x = final_quat[0]
        t.transform.rotation.y = final_quat[1]
        t.transform.rotation.z = final_quat[2]
        t.transform.rotation.w = final_quat[3]

        # Inverte os eixos para o serviço, se necessário, mas publica a TF original.
        # Rotação aplicada ao vetor de posição para alinhar com o sistema de coordenadas do robô
        pos_vec = np.array([position.x, position.y, position.z])
        rotated_pos = transformations.quaternion_matrix(final_quat)[:3, :3].dot(pos_vec)

        # Translação
        t.transform.translation.x = position.x
        t.transform.translation.y = position.y
        t.transform.translation.z = position.z
        
        # A posição para o serviço deve ser a posição transformada
        final_x_for_service = rotated_pos[0]
        final_y_for_service = rotated_pos[1]
        final_z_for_service = rotated_pos[2]
        
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"TF publicada para '{object_frame_id}' em relação a 'camera_link'")
        
        return (final_x_for_service, final_y_for_service, final_z_for_service)
        

def main(args=None):
    rclpy.init(args=args)
    pick_object_service = PickObjectService()
    rclpy.spin(pick_object_service)
    pick_object_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()