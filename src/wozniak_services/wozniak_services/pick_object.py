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
        self.client = self.create_client(Coord, "Coord")
        
        # Imagem com as marcações acumuladas
        self.marked_image = None
        
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/aligned_depth_to_color/camera_info', self.camera_info_callback, 10)
        self.get_logger().info('Pick object service initialized. Waiting for camera data...')

    def image_callback(self, msg):
        try:
            self.latest_frame = msg
            # Sempre atualiza a imagem marcada com a nova imagem
            self.marked_image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
            
            if not hasattr(self, 'first_image_received'):
                self.first_image_received = True
                self.get_logger().info(f'Primeira imagem RGB recebida. Encoding: {msg.encoding}, Dimensões: {msg.width}x{msg.height}')
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')


    def depth_callback(self, msg):
        """Callback para imagem de profundidade"""
        try:
            self.latest_depth_frame = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
            if not hasattr(self, 'first_depth_received'):
                self.first_depth_received = True
                self.get_logger().info('Primeira imagem de profundidade recebida')
        except Exception as e:
            self.get_logger().error(f'Error in depth callback: {str(e)}')

    def camera_info_callback(self, msg):
        """Callback para informações da câmera"""
        try:
            self.camera_info = msg
            if not hasattr(self, 'first_info_received'):
                self.first_info_received = True
                self.get_logger().info('Primeiras informações da câmera recebidas')
        except Exception as e:
            self.get_logger().error(f'Error in camera info callback: {str(e)}')

    def pick_object_callback(self, request, response):
        self.get_logger().info('Incoming request to pick object: %s' % (request.target_object))
        
        # Debug information
        self.get_logger().info(f'Camera data status:')
        self.get_logger().info(f'- Color image: {"Available" if self.latest_frame is not None else "Not available"}')
        self.get_logger().info(f'- Depth image: {"Available" if self.latest_depth_frame is not None else "Not available"}')
        self.get_logger().info(f'- Camera info: {"Available" if self.camera_info is not None else "Not available"}')
        
        if self.latest_frame is None or self.latest_depth_frame is None or self.camera_info is None:
            self.get_logger().error('No camera data available')
            response.success = False
            response.message = 'No camera data available'
            return response

        try:
            x, y = self.image_recognition(request.target_object)
            position = self.get_3d_position(x, y)
            self.publish_transform(position)
            response.success = True
            response.message = 'Object picked'
        except Exception as e:
            self.get_logger().warn(f'Failed to pick object: {str(e)}')
            response.success = False
            response.message = str(e)
        
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
        # Call the OpenAI API to recognize the object. Return the coordinates of the object in percent.
        client = OpenAI(
            base_url="http://10.9.8.252:8000/v1",
            api_key="not-needed"
        )
        self.get_logger().info(f'Procurando objeto: {target_object}')
        
        # Verifica se temos uma imagem válida
        if self.latest_frame is None:
            self.get_logger().error('Nenhuma imagem disponível da câmera')
            raise Exception("Nenhuma imagem disponível")
            
        self.get_logger().info('Convertendo imagem para base64...')
        base64_image = self.encode_latest_frame_base64()
        self.get_logger().info('Imagem convertida com sucesso')
        
        response = client.chat.completions.create(
            messages=[{"role": "user", "content": [
                            {"type": "text", "text": "Point to the " + target_object + " in this image."},
                            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}}
                      ]}],
            model="lcad-ica",
            max_completion_tokens=300,
        )
        result = response.choices[0].message.content
        self.get_logger().info(f'Resposta MolmoAI: {result}')
        
        matches = re.findall(r'x="(\d+\.\d+)" y="(\d+\.\d+)"', result)
        if not matches:
            self.get_logger().warn(f'Objeto {target_object} não encontrado na imagem')
            raise Exception(f"Objeto {target_object} não encontrado")
            
        x, y = matches[0]
        x = float(x)
        y = float(y)
        if x < 0 or x > 100 or y < 0 or y > 100:
            self.get_logger().warn(f'Coordenadas inválidas para {target_object}: x={x}, y={y}')
            raise Exception(f"Coordenadas inválidas para {target_object}")
            
        x_pixel = int(float(x) * self.latest_frame.width / 100)
        y_pixel = int(float(y) * self.latest_frame.height / 100)
        
        # Atualiza a imagem com as marcações
        try:
            self.get_logger().info('Convertendo imagem ROS para OpenCV...')
            
            # Se não temos uma imagem marcada ainda, cria uma nova a partir do frame atual
            if self.marked_image is None:
                self.marked_image = self.bridge.imgmsg_to_cv2(self.latest_frame, self.latest_frame.encoding)
            
            self.get_logger().info('Desenhando marcações na imagem...')
            
            # Desenha círculo verde mais visível
            cv2.circle(self.marked_image, (x_pixel, y_pixel), 10, (0, 255, 0), 2)
            # Adiciona texto com nome do objeto
            cv2.putText(self.marked_image, target_object, (x_pixel + 15, y_pixel), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            self.get_logger().info('Salvando imagem em /tmp/current_scene.jpg...')
            success = cv2.imwrite("/tmp/current_scene.jpg", self.marked_image)
            if success:
                self.get_logger().info(f'Imagem salva com sucesso em /tmp/current_scene.jpg')
            else:
                self.get_logger().error('Falha ao salvar a imagem')
                
        except Exception as e:
            self.get_logger().error(f'Erro ao processar/salvar imagem: {str(e)}')
            raise e
            
        self.get_logger().info(f'Objeto {target_object} identificado em x={x_pixel}, y={y_pixel}')
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

        if not self.client:
            self.get_logger().error("Coord client not available")
            return
        
        # Calculate the offsets
        x_offset = 0.024
        y_offset = 0.119
        z_offset = 0.045
        
        # Call the Coord service to get the coordinates
        coord_request = Coord.Request()
        coord_request.x = position[0] + x_offset
        coord_request.y = position[1] + y_offset
        coord_request.z = position[2] + z_offset
        
        #publish
        self.get_logger().info("Calling Coord service")
        future = self.client.call_async(coord_request)


def main(args=None):
    rclpy.init(args=args)

    minimal_service = PickObjectService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()