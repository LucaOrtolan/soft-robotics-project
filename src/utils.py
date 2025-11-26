import os
import json

this_dir = os.path.dirname(__file__)
parent_dir = os.path.abspath(os.path.join(this_dir, ".."))
jsons_dir = os.path.join(parent_dir, "jsons") 
images_dir = os.path.join(parent_dir, "images")

def import_json_data(filename):
    with open(os.path.join(jsons_dir, filename + ".json"), "r") as f:
        data = json.load(f)
    return data

def export_json(data: dict, filename):
    with open(os.path.join(jsons_dir, filename + ".json"), "w") as f:
        json.dump(data, f)

def save_img(fig, filename, dpi=300, bbox_inches="tight"):
    os.makedirs(os.path.join(parent_dir, ), exist_ok=True)
    path = os.path.join(images_dir, filename)
    fig.savefig(path, dpi=dpi, bbox_inches=bbox_inches)
