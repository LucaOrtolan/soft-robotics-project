import os
import json

this_dir = os.path.dirname(__file__)
parent_dir = os.path.abspath(os.path.join(this_dir, ".."))
jsons_dir = os.path.join(parent_dir, "jsons")

def import_materials_data():
    with open(os.path.join(jsons_dir, "materials.json"), "r") as f:
        materials = json.load(f)
    return materials

def export_robot_json(robot_data):
    robot_data.to_json(os.path.join(jsons_dir, "robot.json"))
