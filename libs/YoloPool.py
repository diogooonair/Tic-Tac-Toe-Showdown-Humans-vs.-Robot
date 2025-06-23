import torch
from libs.JsonLib import getDetecao

data = getDetecao()
model = None
if len(data["repo"]) > 2:
    model = torch.hub.load(data["repo"], data["datasetname"], pretrained=True)


def Detetar(img):
    if model is None:return -4

    resultados = model(img)
    for *box, conf, cls in resultados.xyxy[0]:
        print(cls)
        if int(cls) in data["ids"]:  # 47 = maca, 49 = laranja
            return 1 if data["ids"][0] == int(cls) else -1

    return -3
