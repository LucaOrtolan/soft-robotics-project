import json
import os
from model import Model

this_dir = os.path.dirname(__file__)
parent_dir = os.path.abspath(os.path.join(this_dir, ".."))
jsons_dir = os.path.join(parent_dir, "jsons")

class PhysicsEngine(Model):
    def __init__(self, model_type: str, material: str):
        super().__init__(model_type, material)


