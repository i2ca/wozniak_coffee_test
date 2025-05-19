import rclpy
from rclpy.node import Node
from wozniak_interfaces.srv import TriggerLLM


class ChatNode(Node):
    def __init__(self):
        super().__init__('chat_node')
        self.client = self.create_client(TriggerLLM, "trigger_llm")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando o serviço trigger_llm estar disponível...')
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
