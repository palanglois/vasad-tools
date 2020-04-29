import cv2
import json
import numpy as np

classes = {
  "Pillar": ["Furring"],
  "Partition": ["Partition"],
  "Bearing_wall": ["Wall:Exterior", "HR"], # HR = hour-rated (fire proof)
  "Window": ["Window"],
  "Roof": ["Roof"],
  "Door": ["Door"],
  "Floor": ["Floor"],
  "Stair": ["Stair"],
  "Railing": ["Railing"],
  "Ceiling": ["Ceiling"],
  "Beam": ["Beam"]
}


classes_with_color = {}
for i, (cl, keywords) in enumerate(classes.items()):
  h = int(i*255/(len(classes) - 1))
  classes_with_color[cl] = {"keywords": keywords,
                            "color": cv2.applyColorMap(np.uint8([[h]]), cv2.COLORMAP_JET)[0, 0].tolist()}

with open("semantic_classes.json", "w") as out_file:
  json.dump(classes_with_color, out_file)

