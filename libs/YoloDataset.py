import os.path

from ultralytics import YOLO
from libs.JsonLib import getDetecao

data = getDetecao()
modelo = None
if os.path.exists(data["datasetname"]):
    modelo = YOLO(data["datasetname"])

valores = data["ids"]



def Detetar(img):
    if modelo is None: return -4


    resultados = modelo(img)
    for r in resultados:
        for box in r.boxes:
            cls_id = int(box.cls[0])
            if cls_id not in valores:
                continue

            return 1 if data["ids"][0] == int(cls_id) else -1

    return -3
