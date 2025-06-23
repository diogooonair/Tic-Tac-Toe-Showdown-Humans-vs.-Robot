import time

from Externo.move import main
import Externo.robotMove
from libs.JsonLib import getValoresPosicoes


"""
Ficheiro temporario usado num evento até reparo do pi0
"""
while True:
        data1 = getValoresPosicoes("cubo_1")
        print("cubo_1")
        data2 = getValoresPosicoes("aruko_2")
        data3 = getValoresPosicoes("approach_point")
        data4 = getValoresPosicoes("ponto_escondido")
        data5 = getValoresPosicoes("ponto_arrumar")

        from Externo.robotMove import ismoving

        time.sleep(1.0)
        main(data1)
        while not ismoving(data1, True):
            main(data1)
        time.sleep(3.0)
        main([], False)
        time.sleep(1.0)
        main(data5)
        while not ismoving(data5, True):
            main(data5)

        time.sleep(1.0)
        main(data3)
        while not ismoving(data3, True):
            main(data3)

        time.sleep(1.0)
        main(data2)
        while not ismoving(data2, True):
            main(data2)
        time.sleep(1.0)
        main([], True)



        time.sleep(1.0)
        main(data5)
        while not ismoving(data5, True):
            main(data5)

        time.sleep(1.0)
        main(data3)
        while not ismoving(data3, True):
                main(data3)

        time.sleep(1.0)
        main(data2)
        while not ismoving(data2, True):
                main(data2)

        time.sleep(1.0)
        main([], False)

        time.sleep(1.0)
        main(data3)
        while not ismoving(data3, True):
                main(data3)

        time.sleep(1.0)
        main(data5)
        while not ismoving(data5, True):
                main(data5)

        time.sleep(1.0)
        main(data1)
        while not ismoving(data1, True):
                main(data1)

        time.sleep(1.0)
        main([], True)
        main(data5)
        while not ismoving(data5, True):
                main(data5)

        time.sleep(5.0)



