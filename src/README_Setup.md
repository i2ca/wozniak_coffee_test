## Executando o Sistema Completo com MolmoAI

### 1. Preparação do Ambiente

#### Inicialização da MolmoAI
1. Siga as instruções do arquivo `MolmoAi_Setup_Remoto_Atena.md` para configurar a MolmoAI
2. Verifique se o serviço está rodando corretamente:
   ```bash
   curl http://10.9.8.252:8000/v1/health
   ```

#### Configuração da Câmera
1. Conecte a câmera RealSense ao computador
2. Verifique se a câmera está sendo reconhecida:
   ```bash
   rs-enumerate-devices
   ```

### 2. Iniciando os Serviços

Execute os seguintes comandos em terminais separados:

```bash
# Terminal 1: RealSense Camera
cd ~/I2CA/wozniak_coffee_test
source install/setup.bash
ros2 launch robot_control_language realsense.launch.py

# Terminal 2: Serviço de Detecção
source install/setup.bash
ros2 run wozniak_services pick_object

# Terminal 3: Ferramentas de Controle
source install/setup.bash
ros2 run robot_control_language tools_node
```

### 3. Fluxo de Execução

1. **Captura de Imagem**
   - A câmera RealSense captura continuamente frames RGB e depth
   - Os frames são publicados nos tópicos ROS2:
     - `/camera/camera/color/image_raw`
     - `/camera/camera/aligned_depth_to_color/image_raw`

2. **Processamento com MolmoAI**
   - O serviço `pick_object` recebe a imagem
   - Converte para base64
   - Envia para a API da MolmoAI junto com o prompt
   - Exemplo de prompt:
     ```python
     "Point to the coffee cup in this image."
     ```

3. **Interpretação e Ação**
   - A MolmoAI retorna as coordenadas do objeto
   - O sistema converte as coordenadas 2D para 3D
   - As transformações são publicadas via TF2
   - As ferramentas executam as ações necessárias

### 4. Monitoramento e Debug

1. **Visualização**
   ```bash
   # Visualizar imagem da câmera
   ros2 run rqt_image_view rqt_image_view

   # Monitorar transformações
   ros2 run tf2_ros tf2_echo camera_color_optical_frame detected_object
   ```

2. **Logs e Status**
   ```bash
   # Verificar logs do serviço
   ros2 topic echo /rosout

   # Status da MolmoAI
   curl http://10.9.8.252:8000/v1/status
   ```

### 5. Troubleshooting Comum

1. **MolmoAI não responde**
   - Verifique se o serviço está rodando
   - Confirme o endereço IP e porta
   - Verifique os logs do serviço

2. **Problemas com a Câmera**
   - Reinicie o launch file da RealSense
   - Verifique se os tópicos estão sendo publicados:
     ```bash
     ros2 topic list | grep camera
     ```

3. **Erros de Transformação**
   - Verifique se todas as transformações necessárias estão sendo publicadas
   - Confirme se os frames estão corretos
   - Valide a calibração da câmera 