import os
import re
import base64

# ROS2
import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import tf.transformations as transformations
import tf2_ros
from geometry_msgs.msg import TransformStamped
from wozniak_interfaces.srv import PickObject

# OpenAI
from openai import OpenAI
from base64 import b64encode

# --- Dependências SAM2 (do CÓDIGO 2)
import torch
import numpy as np
import cv2
import matplotlib.pyplot as plt
from PIL import Image as PILImage

# Hydra / omegaconf
import hydra
from omegaconf import DictConfig
from hydra import initialize_config_dir, compose
from hydra.core.global_hydra import GlobalHydra

# sam2 imports
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor

def setup_sam2():
    """Configura e retorna o modelo SAM2 e seu preditor."""
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[SAM2] Usando dispositivo: {device}")

    # Ajuste estes caminhos conforme seu ambiente
    CHECKPOINT_PATH = "/home/hercules/sam2/checkpoints/sam2.1_hiera_large.pt"
    config_dir = "/home/hercules/sam2/sam2/configs"

    # Reset Hydra antes de inicializar
    if GlobalHydra.instance().is_initialized():
        GlobalHydra.instance().clear()

    initialize_config_dir(config_dir=config_dir, version_base=None)
    cfg = compose(config_name="sam2.1/sam2.1_hiera_l")

    # Verificação do arquivo de configuração
    config_file_path = os.path.join(config_dir, "sam2.1", "sam2.1_hiera_l.yaml")
    if not os.path.exists(config_file_path):
        raise FileNotFoundError(
            f"Erro: Arquivo de configuração do SAM2 não encontrado em {config_file_path}."
        )

    # Carregar o modelo SAM2
    sam2_model = build_sam2(cfg, CHECKPOINT_PATH, device=device)
    predictor = SAM2ImagePredictor(sam2_model)

    return predictor

def show_mask(mask, ax):
    """Desenha a máscara na imagem."""
    color = np.array([30/255, 144/255, 255/255, 0.6])
    h, w = mask.shape[-2:]
    mask_image = mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
    ax.imshow(mask_image)

def show_masks(image, masks, scores, output_folder="/tmp"):
    """Exibe e salva as máscaras segmentadas em arquivos .jpeg."""
    for i, (mask, score) in enumerate(zip(masks, scores)):
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.imshow(image)
        show_mask(mask, ax)
        ax.set_title(f"Segmentação - Score: {score:.3f}", fontsize=18)
        ax.axis('off')

        output_path = os.path.join(output_folder, f"segmentacao_{i+1}.jpeg")
        plt.savefig(output_path, bbox_inches='tight', pad_inches=0, dpi=300)
        print(f"[SAM2] Imagem segmentada salva em: {output_path}")

        plt.close(fig)  # Fechar o plot ao final para não bloquear

def run_sam2_segmentation(x_coord, y_coord, image_path, predictor, output_folder="/tmp"):
    """Executa a segmentação do SAM2 usando um ponto específico."""
    if not os.path.exists(image_path):
        raise FileNotFoundError(f"Erro: A imagem '{image_path}' não foi encontrada.")

    # Carrega a imagem
    image = np.array(PILImage.open(image_path))
    predictor.set_image(image)

    # Define o ponto de entrada
    input_point = np.array([[x_coord, y_coord]])
    input_label = np.array([1])

    # Executa a predição
    masks, scores, logits = predictor.predict(
        point_coords=input_point,
        point_labels=input_label,
        multimask_output=True
    )

    # Ordena pelas pontuações
    sorted_ind = np.argsort(scores)[::-1]
    masks = masks[sorted_ind]
    scores = scores[sorted_ind]

    # Exibe/Salva as máscaras
    show_masks(image, masks, scores, output_folder)

class PickObjectService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.latest_frame = None
        self.latest_depth_frame = None
        self.camera_info = None
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Inicia o servidor ROS
        self.srv = self.create_service(PickObject, 'pick_object', self.pick_object_callback)

        # Assinaturas dos tópicos
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )
        self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            10
        )

        # (Opcional) Você pode inicializar o SAM2 uma única vez aqui:
        self.sam2_predictor = setup_sam2()
        # E/ou criar uma pasta temporária para salvar imagens
        self.tmp_folder = "/tmp/ros_sam2"
        os.makedirs(self.tmp_folder, exist_ok=True)

    def image_callback(self, msg):
        self.latest_frame = msg

    def depth_callback(self, msg):
        self.latest_depth_frame = self.bridge.imgmsg_to_cv2(msg, msg.encoding)

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def pick_object_callback(self, request, response):
        self.get_logger().info(f'Incoming request to pick object: {request.object}')
        if not self.latest_frame or not self.latest_depth_frame or not self.camera_info:
            response.success = False
            response.message = 'No camera data available'
            return response

        try:
            # 1) Usa ChatGPT (OpenAI) para encontrar x, y do objeto
            x, y = self.image_recognition(request.object)

            # 2) [NOVO] Executa a segmentação SAM2 usando (x, y) na imagem
            #    a) Converter self.latest_frame em arquivo .jpeg temporário
            image_path = os.path.join(self.tmp_folder, "current_color.jpeg")
            self.save_ros_image_to_disk(self.latest_frame, image_path)

            #    b) Executar segmentação
            run_sam2_segmentation(
                x_coord=x,
                y_coord=y,
                image_path=image_path,
                predictor=self.sam2_predictor,
                output_folder=self.tmp_folder
            )

            # 3) Recuperar coordenada 3D e publicar transform
            position = self.get_3d_position(x, y)
            self.publish_transform(position)

            response.success = True
            response.message = 'Object picked (SAM2 segmentation done)'
            return response

        except Exception as e:
            self.get_logger().error(str(e))
            response.success = False
            response.message = f'Erro ao processar objeto: {str(e)}'
            return response

    def save_ros_image_to_disk(self, ros_image_msg, out_path):
        """Converte sensor_msgs/Image em imagem OpenCV e salva em disco."""
        cv_image = self.bridge.imgmsg_to_cv2(ros_image_msg, ros_image_msg.encoding)
        cv2.imwrite(out_path, cv_image)

    def image_recognition(self, object):
        # Chama a API do OpenAI para reconhecer a localização do objeto
        # Retorna as coordenadas do objeto em pixels (x, y).

        client = OpenAI(base_url="http://10.9.8.252:8000/api/v1")
        response = client.chat.completions.create(
            messages=[
                {
                    "role": "system",
                    "content": "You must recognize the location of a given object."
                },
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "content": f"Where is the {object} located?"},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{b64encode(self.latest_frame.data)}"
                            }
                        }
                    ]
                }
            ],
            model="lcad-ica",
            max_completion_tokens=300,
        )
        result = response.choices[0].message.content
        matches = re.findall(r'x\d+="(\d+\.\d+)" y\d+="(\d+\.\d+)"', result)
        if not matches:
            raise Exception("Object not found by ChatGPT")

        x_percent, y_percent = matches[0]
        x_percent, y_percent = float(x_percent), float(y_percent)

        # Validação simples
        if x_percent < 0 or x_percent > 100 or y_percent < 0 or y_percent > 100:
            raise Exception("Invalid object coordinates from ChatGPT")

        # Converte percentual da imagem para pixel
        x_pixel = int(x_percent * self.latest_frame.width / 100)
        y_pixel = int(y_percent * self.latest_frame.height / 100)
        return x_pixel, y_pixel

    def get_3d_position(self, x, y):
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        # Converte para metros (depth * 0.001)
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

        # Sem rotação, quaternion identidade
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
