# Documentação dos Scripts - Wozniak Coffee Test

## Visão Geral dos Pacotes

### 1. Wozniak Services (`wozniak_services`)

#### Serviço de Detecção de Objetos (`pick_object.py`)
Este serviço é o coração do sistema de visão computacional do robô, responsável por:

- **Processamento de Imagem**
  - Recebe imagens RGB e depth da câmera RealSense
  - Converte frames para formato base64 para processamento com IA
  - Salva imagens com marcações para debug visual

- **Integração com IA**
  - Utiliza a API OpenAI para detecção de objetos
  - Processa respostas da IA para extrair coordenadas (x,y)
  - Converte coordenadas 2D para posições 3D no espaço

- **Comunicação ROS2**
  - Publica transformações TF para o objeto detectado
  - Gerencia serviços ROS2 para coordenadas
  - Mantém sincronização entre frames RGB e depth

### 2. Controle do Robô (`robot_control_language`)

#### Gerenciador de IA (`llm.py`)
Sistema central de controle baseado em IA que:

- **Gerenciamento de Conversação**
  - Mantém histórico completo do chat
  - Processa entradas multimodais (texto + imagem)
  - Gerencia o contexto da conversa

- **Integração com OpenAI**
  - Implementa sistema de retry com backoff exponencial
  - Gerencia chamadas à API de forma robusta
  - Processa respostas e executa ações correspondentes

- **Sistema de Ferramentas**
  - Gerencia ferramentas disponíveis para a IA
  - Executa chamadas de função baseadas em respostas
  - Valida e processa resultados das ferramentas

#### Ferramentas de Café (`coffee/tools.py`)
Implementa as funcionalidades específicas para manipulação de café:

- **Funções de Manipulação**
  - `pick_object`: Serviço para pegar objetos identificados
  - Gerenciamento de timeouts e reconexões
  - Tratamento de erros e feedback

## Componentes Adicionais

### 1. Oculus-RealSense
Sistema de integração entre o Oculus Quest e a câmera RealSense que:
- Permite visualização em realidade virtual do feed da câmera
- Facilita a calibração do sistema através de interface VR
- Auxilia no desenvolvimento e teste de algoritmos de visão

### 2. Arquivos de Calibração

#### `calibration.launch.py`
Launch file responsável pela calibração do sistema:
- **Funcionalidades**
  - Inicia os nós necessários para calibração
  - Configura parâmetros da câmera RealSense
  - Sincroniza streams RGB e depth
  - Permite ajuste fino dos parâmetros de transformação

- **Uso**
  ```bash
  ros2 launch wozniak_services calibration.launch.py
  ```

- **Parâmetros Importantes**
  - Transformações entre frames da câmera
  - Configurações de resolução e FPS
  - Filtros de depth

### 3. Reconhecimento de Pontos (`red_dot_recognition.cpp`)

Módulo C++ para detecção precisa de pontos vermelhos:

- **Funcionalidades**
  - Processamento de imagem em tempo real
  - Filtragem por cor HSV
  - Detecção de centroide
  - Cálculo de coordenadas 3D

- **Algoritmo**
  - Conversão para espaço de cor HSV
  - Aplicação de thresholds para isolamento do vermelho
  - Operações morfológicas para redução de ruído
  - Detecção de contornos e cálculo de centroide

- **Integração**
  - Publica resultados em tópicos ROS2
  - Integra com sistema de transformações TF2
  - Fornece feedback visual para debugging

## Fluxo de Dados

1. A câmera RealSense captura imagens RGB e depth
2. O serviço `pick_object.py` processa estas imagens
3. A IA identifica objetos através do `llm.py`
4. As coordenadas são calculadas e transformadas
5. As ferramentas em `tools.py` executam as ações físicas

## Fluxo de Calibração

1. **Preparação**
   - Iniciar o sistema RealSense
   - Carregar `calibration.launch.py`
   - Verificar conexão com Oculus (se utilizado)

2. **Processo**
   - Posicionar marcadores de calibração
   - Executar detecção de pontos vermelhos
   - Ajustar transformações TF
   - Validar resultados com ferramentas de visualização

3. **Validação**
   - Verificar precisão da detecção
   - Testar diferentes posições
   - Confirmar alinhamento RGB-depth

## Notas de Desenvolvimento

- O sistema usa ROS2 para comunicação entre componentes
- A câmera RealSense deve estar corretamente calibrada
- As transformações TF são essenciais para manipulação precisa
- O sistema de retry protege contra falhas temporárias da API

## Dicas de Debug

1. **Imagens de Debug**
   - Verifique `/tmp/image.jpg` para visualizar detecções
   - As marcações verdes indicam pontos detectados

2. **Logs ROS**
   - Use `ros2 topic echo` para monitorar tópicos
   - Verifique transformações com `tf2_echo`

3. **Serviços**
   - Teste serviços individualmente com `ros2 service call`
   - Monitore timeouts e reconexões

4. **Calibração**
   - Use `rviz2` para visualizar transformações
   - Verifique alinhamento RGB-depth com padrão de calibração
   - Monitore latência entre streams
   - Utilize ferramentas de visualização do Oculus para validação em VR

## Usando o Sistema com LLM

### 1. Arquitetura do Sistema

O sistema utiliza dois componentes principais para comunicação com a MolmoAI:
- `llm.py`: Gerencia a comunicação com a LLM e processa as respostas
- `pick_object`: Executa as ações baseadas nas coordenadas recebidas

### 2. Iniciando os Serviços

Execute os comandos em terminais separados:

```bash
# Terminal 1: RealSense Camera
cd ~/I2CA/wozniak_coffee_test
source install/setup.bash
ros2 launch robot_control_language realsense.launch.py

# Terminal 2: Serviço LLM
source install/setup.bash
ros2 run robot_control_language llm_node

# Terminal 3: Serviço de Detecção
source install/setup.bash
ros2 run wozniak_services pick_object

# Terminal 4: Ferramentas de Controle
source install/setup.bash
ros2 run robot_control_language tools_node
```

### 3. Fluxo de Comunicação

1. **Interação com LLM**
   - O `llm.py` mantém uma conversação com a MolmoAI
   - Processa entradas de texto do usuário
   - Gerencia o contexto da conversa
   - Exemplo de uso:
     ```python
     # A LLM gera automaticamente prompts apropriados como:
     "Identify and point to the coffee mug in the scene"
     "Locate the sugar container on the table"
     ```

2. **Processamento da Resposta**
   - A LLM analisa a imagem e gera instruções
   - O sistema extrai coordenadas da resposta
   - As coordenadas são enviadas para o `pick_object`

3. **Execução de Ações**
   - O `pick_object` recebe as coordenadas
   - Converte para posição 3D
   - Executa as ações necessárias

### 4. Monitoramento do Sistema

1. **Visualização da Conversação**
   ```bash
   # Visualizar mensagens da LLM
   ros2 topic echo /llm_output

   # Monitorar status do processamento
   ros2 topic echo /llm_status
   ```

2. **Debug da Comunicação**
   ```bash
   # Verificar comunicação com MolmoAI
   ros2 topic echo /molmo_response

   # Monitorar ações do pick_object
   ros2 topic echo /pick_object_status
   ```

### 5. Troubleshooting

1. **Problemas com a LLM**
   - Verifique a conexão com a MolmoAI
   - Confirme se o contexto está sendo mantido
   - Verifique os logs do serviço LLM

2. **Erros de Comunicação**
   - Verifique se todos os nós estão ativos:
     ```bash
     ros2 node list
     ```
   - Confirme se os tópicos estão sendo publicados:
     ```bash
     ros2 topic list | grep llm
     ```

3. **Ajustes de Prompt**
   - Modifique o arquivo de configuração da LLM se necessário
   - Ajuste os parâmetros de conversação
   - Verifique o histórico de chat para debug

### 6. Configuração da LLM

O arquivo de configuração (`settings.yaml`) permite ajustar:
- Prompt base para a LLM
- Parâmetros de conversação
- Ferramentas disponíveis
- Exemplo:
  ```yaml
  system_prompt: "You are an assistant helping to identify objects..."
  tools:
    - name: "pick_object"
      description: "Identifies and picks objects in the scene"
  ``` 