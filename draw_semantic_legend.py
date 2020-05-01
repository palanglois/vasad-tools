import argparse
import json
from os.path import join
from PIL import Image, ImageDraw, ImageFont

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("--input", default="semantic_classes.json", help="Json semantic class file")
  args = parser.parse_args()

  h_per_class = 30
  width = 300
  
  with open(args.input, "r") as in_file:
    data = json.load(in_file)
    data["Other"] = {"color": [0, 0, 0]}
    nb_classes = len(data)
    image = Image.new('RGB', (width, nb_classes*h_per_class))
    draw = ImageDraw.Draw(image)
    draw.rectangle([0, 0, width, nb_classes*h_per_class], (255, 255, 255))
    for i, (k, v) in enumerate(data.items()):
      draw.rectangle(((0, i*h_per_class), (width // 3, (i+1)*h_per_class)), fill=tuple(v["color"]))
      font = ImageFont.truetype(join('debug', 'arial.ttf'), size=30)
      draw.text((width // 3, i*h_per_class), k, font=font, fill='black')
    image.save(join("debug","sem_classes.png"))

if __name__ == "__main__":
  main()
