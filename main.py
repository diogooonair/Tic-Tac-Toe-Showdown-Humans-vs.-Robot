from time import sleep

#Local import
from libs import JsonLib, RasConnect, NegaMax, ImageRec
from libs.ImageRec import DetetarArukosProxy
from libs.JsonLib import getJson, setJson, getValoresPosicoes
from vars import tabuleiro, vitorias


#Libs
import time
import random


#Atualizar json do jogo para comeco, dar reset
def comecar_jogo(turno):
    """
    Starts a new game by initializing the game board and setting the starting player.

    This function initializes the game state, including the game board as an empty
    state and the starting turn. It then stores this data in a JSON format for
    further use in the game's logic.

    :param turno: Indicates the starting player's turn, where 1 or 2 represents a
                  respective player.
    :type turno: int
    :return: None
    """
    tabuleiro = [0,0,0,
             0,0,0,
             0,0,0]
    json_val = {
        "turno": turno,
        "tabuleiro": tabuleiro,
        "vencedor": None
    }
    JsonLib.setJson(json_val)


#funcao temp mostrar tabuleiro
def mostrartabuleiro(tabuleiro):
    """
    Displays a formatted version of a tic-tac-toe board. This function takes a
    list representing the game board and prints it row by row, showing either
    the values of the cells or blank spaces for empty cells. Rows are separated
    by horizontal lines for better visual representation.

    :param list tabuleiro: A list of integers representing the tic-tac-toe
        board. The board should have exactly nine elements, with 0 representing
        an empty cell and other integers representing occupied cells.
    :return: None
    """
    for i in range(0, 9, 3):
        linha = tabuleiro[i:i+3]
        linha_formatada = [str(celula) if celula != 0 else " " for celula in linha]
        print(" | ".join(linha_formatada))
        if i < 6:
            print("-" * 5)


#retorna o vencedor, 0 em caso de empate
def getVencedor(tabuleiro):
    """
    Determines the winner of a given tic-tac-toe board if one exists.

    This function checks predefined winning combinations on the board to determine
    if any player has achieved a winning state. If no winning combination is found,
    it returns 0 indicating no winner yet.

    :param tabuleiro: A list of integers representing the current state of the
                      tic-tac-toe board, where each integer indicates the state
                      of a cell.
                      - 0 represents an empty cell
                      - 1 represents a cell occupied by player 1
                      - 2 represents a cell occupied by player 2
    :return: An integer indicating the winner of the game or no winner.
             - Returns 0 if there is no winner yet
             - Returns 1 if player 1 is the winner
             - Returns 2 if player 2 is the winner
    """
    for (a, b, c) in vitorias:
        if tabuleiro[a] == tabuleiro[b] == tabuleiro[c] != 0:
            return tabuleiro[a]
    return 0

#mover robo










#Escolhe dificuldade
def Dificuldade():
    """
    Gets the difficulty level from the user input.

    This function prompts the user to input a difficulty level. The levels
    are represented as integers and are mapped as follows:
    - 0 for "Facil" (easy),
    - 1 for "Medio" (medium),
    - 2 for "Dificil" (hard).

    A value is then returned that represents the user's chosen difficulty.

    :return: Difficulty level selected by the user
    :rtype: int
    """
    valor = int(input("0 - Facil,\n 1 - Medio ,\n 2 - Dificil \n Insira o valor do dificuldade: \n"))
    return valor

#Cara ou coroa retorna true quando o player ganha
def CaraOuCoroa(valor):
    """
    Determine the result of a coin flip game based on the user's choice.

    This function simulates a coin flip by generating a random number and checks
    if the result corresponds to the user's choice of either heads (0) or tails (1).
    If the user's choice matches the outcome of the random number's parity
    (odd/even), the function returns True; otherwise, it returns False.

    :param valor: The user's choice for the coin flip:
                  - 0 for heads (even result)
                  - 1 for tails (odd result)
                  Must be an integer value.
    :return:
        - True if the result of the coin flip matches the user's choice.
        - False otherwise.
    """
    resultado = random.randint(1, 1000)
    match int(valor):
        case 0:
            if resultado % 2 == 0:
                return True
        case 1:
            if resultado % 2 != 0:
                return True
    return False

#pede a dificuldade
depth = Dificuldade() + 1
#caso o jogador comece 2 caso o AI comece 1
i = 1 if CaraOuCoroa(0) else 2
comecar_jogo(i)
#inicializar os aruko markers
ImageRec.detetarAruko(RasConnect.getPhoto())


#Game loop
while not NegaMax.estadoterminal(tabuleiro):
    if i % 2 == 0:
        mostrartabuleiro(tabuleiro)
        print("JOGADOR")
        data = -3
        while data == -3:
            print("LOOP")
            try:
                data = ImageRec.detetarAruko(RasConnect.getPhoto())
            except:
                while True:
                    time.sleep(5.0)
                    data = ImageRec.detetarAruko(RasConnect.getPhoto())

            print(data)
            time.sleep(1.0)
        turnos = JsonLib.getJson()
        turnos["tabuleiro"] = tabuleiro
        turnos["turno"] += 1
        i+=1
        JsonLib.setJson(turnos)
        mostrartabuleiro(tabuleiro)

    else:
        jogada, _ = NegaMax.negamax(tabuleiro, 1, depth)
        print("ROBO JOGA " + str(jogada) + "    " + str(i))
        NegaMax.jogarf(tabuleiro, jogada, 1)
        #time.sleep(30.0)
        turnos = JsonLib.getJson()
        turnos["tabuleiro"] = tabuleiro
        turnos["turno"] += 1
        i += 1
        JsonLib.setJson(turnos)

        mostrartabuleiro(tabuleiro)

    #esperar passagem de turno (Nao necessario)
    """while True:
        turnos = JsonLib.getJson()
        if i != turnos["turno"]:
            if(i % 2 == 0):
                #verificar onde foi jogado
                ImageRec.detetarAruko(RasConnect.getPhoto())
            #atualizar tab no json
            turnos["tabuleiro"] = tabuleiro
            JsonLib.setJson(turnos)
            i+=1
            break
        time.sleep(0.5)"""

    print('\n')

turnos = getJson()
turnos["vencedor"] = getVencedor(tabuleiro)
setJson(turnos)

