import json
from enum import Enum
import rclpy
from wozniak_interfaces.srv import PickObject

rclpy.init()
node = rclpy.create_node("hercules_tools")


class KitchenObjects(Enum):
    CUP = "cup"
    COFFEE_MAKER = "coffee maker"


def pick_object(object: KitchenObjects):
    """Pick the object."""
    client = node.create_client(PickObject, "pick_object")

    # Wait for the service to be available
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Service 'pick_object' not available")
        return "Service not available"

    # Create a request (we're using Trigger, which doesn't take specific input)
    request = PickObject.Request()
    request.object = object

    # Call the service and wait for the result
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        # Assuming response contains a `success` boolean and `message`
        node.get_logger().info(f"Service call succeeded: {future.result().message}")
        return future.result().message
    else:
        node.get_logger().error("Service call failed")
        return "Service call failed"
    return ""


hercules_functions = {
    "pick_object": pick_object,
}