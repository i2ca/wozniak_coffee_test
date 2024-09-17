import json
from enum import Enum


class KitchenObjects(Enum):
    CUP = "cup"
    COFFEE_MAKER = "coffee maker"



def pick_object(object: KitchenObjects):
    """Pick the object."""
    print(object)
    return ""


hercules_functions = {
    "pick_object": pick_object,
}