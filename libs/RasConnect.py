import numpy as np
import requests
import cv2

#Dar request ao ras da camera e ir buscar a ft
def getPhoto():
    img = requests.get("http://192.168.98.14:5000/getPhoto").content
    img_array = np.frombuffer(img, dtype=np.uint8)
    img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
    return img
