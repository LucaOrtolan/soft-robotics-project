import os
import json

this_dir = os.path.dirname(__file__)
parent_dir = os.path.abspath(os.path.join(this_dir, ".."))
jsons_dir = os.path.join(parent_dir, "jsons") 

def import_json_data(filename):
    with open(os.path.join(jsons_dir, filename + ".json"), "r") as f:
        data = json.load(f)
    return data

def export_json(data: dict, filename):
    with open(os.path.join(jsons_dir, filename + ".json"), "w") as f:
        json.dump(data, f)
    

