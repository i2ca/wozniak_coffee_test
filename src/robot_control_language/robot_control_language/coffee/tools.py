import json
import rclpy
from rclpy.node import Node
from enum import Enum
import time
from wozniak_interfaces.srv import PickObject
import traceback


# def pick_object(node: Node, object: str):
#     """Pick the object."""
#     node.get_logger().info(f"Node used: {node}, object to pick: {object}")
#     # Actual pick logic goes here

#     client = node.create_client(PickObject, "pick_object")

#     # Wait for the service to be available
#     if not client.wait_for_service(timeout_sec=5.0):
#         node.get_logger().error("Service 'pick_object' not available")
#         return "Service not available"

#     # Create a request (we're using Trigger, which doesn't take specific input)
#     request = PickObject.Request()
#     request.target_object = object

#     import pdb; pdb.set_trace()
#     # Call the service and wait for the result
#     future = client.call_async(request)
#     #rclpy.spin_until_future_complete(node, future)
#     while not future.done():
#         time.sleep(1.0)

#     if future.result() is not None:
#         # Assuming response contains a `success` boolean and `message`
#         node.get_logger().info(f"Service call succeeded: {future.result().message}")
#         return future.result().message
#     else:
#         node.get_logger().error("Service call failed")
#         return "Service call failed"

def pick_object(node: Node, object: str):
    client = node.create_client(PickObject, "pick_object")

    if not client.wait_for_service(timeout_sec=0.5):
        node.get_logger().error("Service 'pick_object' not available")
        return "Service not available"

    request = PickObject.Request()
    request.target_object = object
    future = client.call_async(request)

    def callback(fut):
        try:
            result = fut.result()
            if result:
                if result.success:
                    node.get_logger().info(f"Serviço 'pick_object' executado com sucesso. Target: '{request.target_object}', Coords: (x:{result.x:.2f}, y:{result.y:.2f}, z:{result.z:.2f})")
                else:
                    node.get_logger().error(f"Serviço 'pick_object' falhou. Target: '{request.target_object}'. (Consulte os logs do serviço PickObject para detalhes)")
            else:
                node.get_logger().error("Chamada ao serviço 'pick_object' falhou (future não retornou resultado).")
        except Exception as e:
            node.get_logger().error(f"Exceção no callback do serviço 'pick_object': {str(e)}")
            node.get_logger().error(traceback.format_exc())

    future.add_done_callback(callback)
    return "Request sent"


hercules_functions = {
    "pick_object": pick_object,
}