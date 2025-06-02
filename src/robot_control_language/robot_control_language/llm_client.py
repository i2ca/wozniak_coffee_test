import rclpy
from rclpy.node import Node
from wozniak_interfaces.srv import TriggerLLM, InstructionsLLM


class ChatNode(Node):
    def __init__(self):
        super().__init__('chat_node')
        self.client = self.create_client(TriggerLLM, "TriggerLLM")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando o serviço TriggerLLM estar disponível...')
        
        self.instructions_llm_client = self.create_client(InstructionsLLM, "InstructionsLLM")
        while not self.instructions_llm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando o serviço InstructionsLLM estar disponível...')
            
        self.run_chat()

    def run_chat(self):
        self.get_logger().info("Chat iniciado. Digite 'sair' para encerrar.")
        while rclpy.ok():
            user_input = input("\nVocê: ")
            if user_input.strip().lower() == 'sair':
                print("Encerrando chat...")
                break

            request = TriggerLLM.Request()
            request.message = user_input

            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                response = future.result()
                if response.success:
                    print("LLM:", response.message)
                    # Enviar a mensagem da LLM para o Unity usando o serviço InstructionsLLM
                    unity_request = InstructionsLLM.Request()
                    unity_request.instruction = response.message
                    unity_future = self.instructions_llm_client.call_async(unity_request)
                    rclpy.spin_until_future_complete(self, unity_future)
                    if unity_future.result() is not None:
                        unity_response = unity_future.result()
                        if unity_response.success:
                            self.get_logger().info('Instrução enviada para o Unity (via InstructionsLLM) com sucesso.')
                        else:
                            self.get_logger().error('Erro ao enviar instrução para o Unity (via InstructionsLLM).')
                    else:
                        self.get_logger().error('Erro ao chamar o serviço InstructionsLLM para o Unity.')
                else:
                    print("Erro no processamento pelo LLM.")
            else:
                print("Erro ao chamar o serviço.")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        chat_node = ChatNode()
        print("\nPressione Ctrl+C para encerrar o programa...")
        rclpy.spin(chat_node)
    except KeyboardInterrupt:
        print("\nEncerrando o programa por solicitação do usuário (Ctrl+C)...")
    except Exception as e:
        print(f'\nErro fatal: {str(e)}')
    finally:
        if 'chat_node' in locals():
            chat_node.destroy_node()
            print("Node destruído com sucesso.")
        rclpy.shutdown()
        print("ROS2 encerrado com sucesso.\n")


if __name__ == '__main__':
    main()
