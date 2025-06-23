import json
import os


def setJson(valor):
    with open("Defaults/jogo.json", "w") as f:
        json.dump(valor, f, indent=4)


def getJson():
    with open("Defaults/jogo.json", "r") as f:
        return json.load(f)

def setCores(valor):
    with open("Defaults/detecaocor.json", "w") as f:
        json.dump(valor, f, indent=4)


def getCores():
    with open("Defaults/detecaocor.json", "r") as f:
        return json.load(f)

def getValoresPosicoes(valor):
    subpasta = os.path.dirname(__file__)

    # Go up one level to the parent directory
    pasta = os.path.dirname(subpasta) + "/Defaults/posicoes.json"

    if os.path.exists(pasta):
        with open(pasta, 'r') as f:
            posicoes = json.load(f)

    data = posicoes[valor]
    return data


def getDetecao():
    subpasta = os.path.dirname(__file__)

    # Go up one level to the parent directory
    pasta = os.path.dirname(subpasta) + "/Defaults/SetupDetecao.json"

    if os.path.exists(pasta):
        with open(pasta, 'r') as f:
            return json.load(f)

    return None


def getUsarYolo():
    subpasta = os.path.dirname(__file__)
    pasta = os.path.dirname(subpasta) + "/Defaults/SetupDetecao.json"

    if os.path.exists(pasta):
        with open(pasta, 'r') as f:
            val = json.load(f)

    return val["ativado"], val["PyPoll"]

def atualizarDetecao(valor):
    subpasta = os.path.dirname(__file__)
    pasta = os.path.dirname(subpasta) + "/Defaults/SetupDetecao.json"
    if os.path.exists(pasta):
        with open(pasta, 'w') as f:
            json.dump(valor, f, indent=4)