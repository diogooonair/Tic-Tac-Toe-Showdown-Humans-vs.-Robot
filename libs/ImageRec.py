import numpy as np
import json
import cv2
from cv2.version import contrib
from typer.main import except_hook
from libs.JsonLib import getUsarYolo

from libs.JsonLib import getCores
from vars import tabuleiro, p_aruko, localstorage
import libs.YoloPool as pool
import libs.YoloDataset as dataset


def SalvarImagem(img, path):
    """
    Saves an image to the specified path.

    This function saves the given image to the provided file path. The image is
    stored in the specified location with the name and type defined by the path.

    :param img: The image data that will be saved.
    :type img: numpy.ndarray
    :param path: The relative path where the image should be saved, including the
        file name and extension.
    :type path: str
    :return: None
    """
    cv2.imwrite(localstorage+path, img)

#detecao objeto por shape
def DetetarPorShape(img):
    """
    Analyzes the shape of detected contours in a given image and classifies them
    as squares or circles. The function additionally modifies the input image by
    drawing the detected contours onto it and saves the image.

    :param img: Input image to process
    :type img: numpy.ndarray
    :return: A value indicating the identified shape:
        - 1 if a square is detected
        - -1 if a circle is detected
    :rtype: int
    """
    #converter a imagem para cinza

    try :
        imgCinza = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    except:
        return -2
    #canny edge para os cantos
    cantos = cv2.Canny(imgCinza, 100, 200)
    contornos, _ = cv2.findContours(cantos, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print(str(len(contornos)))
    for contorno in contornos:
        approx = cv2.approxPolyDP(contorno, 0.02 * cv2.arcLength(contorno, True), True)
        area = cv2.contourArea(contorno)

        cor = (0, 0, 0)
        a = 0
        if len(approx) == 4:
            print("quadrado")
            cor = (255, 0, 0)
            a = 1
        else:
            print("circulo")
            cor = (0, 255, 0)
            a = -1

        print("here at CHECKPOINT")
        cv2.drawContours(img, [contorno], 0, cor, 5)
        cv2.imwrite("forma.png", img)
        return a
    return -1




def DetetarPorCor(img):
    """
    Detect regions in an image based on predefined HSV color thresholds.

    This function detects specific color regions in an input image using HSV
    (Hue, Saturation, Value) color space thresholds. It processes the image
    to identify regions falling within the thresholds defined for red and
    green colors. The result is processed and saved as separate images. Finally,
    it returns a specific integer value based on the presence of detected colors
    in the image.

    :param img: The input image in BGR color space to be processed.
    :type img: numpy.ndarray
    :return: An integer indicating the detected object's color:
             - -1 if green is detected
             - 1 if red is detected
             - 0 if neither are detected.
    :rtype: int
    """
    #lemos aqui o valor esta sempre atualizado, podias dar request ao gradie
    # mas e desnecessario se podemos simplesmente ler o file
    defaults = getCores()

    try:
     hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    except:
        return 0

    VerdeMin = np.array([defaults["m_G_H"], defaults["m_G_S"], defaults["m_G_V"]])
    VerdeMax = np.array([defaults["M_G_H"], defaults["M_G_S"], defaults["M_G_V"]])
    VermelhoMin1 = np.array([defaults["m_V_H"], defaults["m_V_S"], defaults["m_V_V"]])
    VermelhoMax1 = np.array([defaults["M_V_H"], defaults["M_V_S"], defaults["M_V_V"]])

    verde = cv2.inRange(hsv, VerdeMin, VerdeMax)
    vermelho = cv2.inRange(hsv, VermelhoMin1, VermelhoMax1)


    SalvarImagem(vermelho, r"vermelhodetetado.png")
    SalvarImagem(verde, r"verdedetetado.png")

    if np.any(verde):
        return -1
    elif np.any(vermelho):
        return 1

    return 0


#detecao geral
def DetetarJogada(img):
    """
    Detects a player's move based on specific attributes in the provided image.
    The detection process involves evaluating both the shape and color attributes
    of the player repeatedly until a match is found.

    :param img: The input image used for detecting the player's move.
                 It should contain identifiable shapes and colors for
                 accurate processing.
    :type img: Any

    :return: The player's color detected, represented as an integer.
    :rtype: int
    """
    jogadorshape = -1
    jogadorcor = 1
    i = 0
    try:
     cv2.imwrite("as.png", img)
    except:
        print("here")
        return -3
    while(jogadorshape != jogadorcor):
        jogadorshape = DetetarPorShape(img)
        jogadorcor = DetetarPorCor(img)
        i += 1

        if(i == 5 and jogadorshape != jogadorcor):
            jogadorcor = -3
            break
        print("loop ", jogadorcor, " ", jogadorshape)

    print(jogadorcor)
    return jogadorcor

#Detetar Arukos Proxy
def DetetarArukosProxy(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()

    # Criar o detetor
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    # Detetar marcadores
    corners, ids, rejected = detector.detectMarkers(gray)

    return ids

#Detetar arukos
def detetarAruko(imagem):
    """
    Determiens and processes ArUco markers in an image. This function detects ArUco markers
    within the given image and performs specific operations based on the number of markers
    detected. If exactly nine markers are detected, it initializes a game board by saving
    each marker individually from the image and updating relevant positional coordinates.
    If fewer than nine markers are detected, it computes the next move in a game logic
    by modifying the board array and uses helper functions for relevant processing.

    :param imagem: ndarray
        The input image (color or grayscale) in which ArUco markers are to be detected
        and analyzed.

    :return: None
    """
    gray = cv2.cvtColor(imagem, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()

    # Criar o detetor
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    # Detetar marcadores
    corners, ids, rejected = detector.detectMarkers(gray)

    cv2.imwrite("arukos.png", cv2.aruco.drawDetectedMarkers(imagem, corners, ids))
    #se forem 9 marcadores detetados = tabuleiro inicial

    try:
        if len(ids) == 9:
            print("inic")
            for n,idmarcador in enumerate(ids):
                idmarcador = idmarcador[0]
                x_min, y_min = np.min(corners[n][0], axis=0).astype(int)
                x_max, y_max = np.max(corners[n][0], axis=0).astype(int)
                SalvarImagem(gray[y_min:y_max, x_min:x_max],r"aruko" + str(idmarcador) + ".png")
                p_aruko[idmarcador] = [x_min, y_min, x_max, y_max]

            return -3
        else:


            jogadaList = [i for i, v in enumerate(tabuleiro) if v == 0 and i not in ids]
            print("len jogadas =  "+str(len(jogadaList)))
            if(len(jogadaList) != 1):
                return -3

            jogada = jogadaList[0]
            print("jogado no aruko " + str(jogada))
            print(p_aruko[jogada])

            img = imagem[p_aruko[jogada][1]-40:p_aruko[jogada][3]+40, p_aruko[jogada][0]-40:p_aruko[jogada][2]+40]


            cv2.imwrite("as2.png", img)
            usaryolo, usarpool = getUsarYolo()
            if usaryolo == False:
                jogador = DetetarJogada(img)
                if(jogador == -3): return -3
                tabuleiro[jogada] = jogador
            else:
                if usarpool == True:
                    jogador = pool.Detetar(img)
                else:
                    jogador = dataset.Detetar(img)

                if jogador == -3: return -3
                tabuleiro[jogada] = jogador

            return jogador
    except:
        jogada = [k for k,i in enumerate(tabuleiro) if i == 0][0]
        tabuleiro[jogada] = -1
        return -1

