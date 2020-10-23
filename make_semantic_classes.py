import cv2
from os.path import join
import json
import numpy as np

classes = {
  "Pillar": ["Furring"],
  "Partition": ["Partition", "M_Int"],
  "Bearing_wall": ["Wall:Exterior", "Wall:Retaining", "Wall:Foundation", "Wall:Cast", "Wall:Reinforced", "Wall:STONE", "Wall:Generic", "HR", "IfcWallStandardCase/Wall.", "M_Ext"], # HR = hour-rated (fire proof)
  "Window": ["Window", "Curtain_Wall", "System_Panel", "CurtainWall", "IfcBuildingElementProxy/Cadre", "PAN_Principal"],
  "Roof": ["Roof", "ROOF", "IfcCovering/TER"],
  "Door": ["Door", "Porte"],
  "Floor": ["Floor", "Dalle", "IfcSlab"],
  "Stair": ["Stair", "MARCHE", "IfcMember/LIM", "IfcMember/SS"],
  "Railing": ["Railing"],
  "Ceiling": ["Ceiling"],
  "Beam": ["Beam", "Hollow_Structural_Section", "Column", "IfcBuildingElementProxy/Forme"]
}


classes_with_color = {}
for i, (cl, keywords) in enumerate(classes.items()):
  h = int(i*255/(len(classes) - 1))
  classes_with_color[cl] = {"keywords": keywords,
                            "color": cv2.applyColorMap(np.uint8([[h]]), cv2.COLORMAP_JET)[0, 0].tolist()}

with open(join("data", "semantic_classes.json"), "w") as out_file:
  json.dump(classes_with_color, out_file)

