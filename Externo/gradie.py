"""
gradio

"""
import json
import subprocess
import time

import gradio as gr
import numpy as np
import os

from aiofiles import stdin

from libs.ImageRec import detetarAruko, DetetarPorCor
from libs.JsonLib import getCores,getDetecao, getJson, setCores, setJson, getValoresPosicoes, atualizarDetecao
from libs.RasConnect import getPhoto
from vars import tabuleiro, localstorage
from PIL import Image, ImageDraw
#import pyRobotiqGripper


def getDificuldade(c):
    """
    Determines the difficulty level based on the given category.

    This function takes a string input representing a difficulty category and
    matches it to a corresponding numeric difficulty level. If the category
    is "Facil", the difficulty level is set to 0. If the category is "Medio",
    the difficulty level is set to 1. For any other input, the difficulty is
    set to 2.

    :param c: The category of difficulty. Accepted values are "Facil",
              "Medio", or any other string.
    :type c: str
    :return: The numeric difficulty level. Returns 0 for "Facil", 1
             for "Medio", and 2 for any other input.
    :rtype: int
    """
    global dificuldade
    match c:
        case "Facil":
            dificuldade = 0
        case "Medio":
            dificuldade = 1
        case _:
            dificuldade = 2




defaults = getCores()
def carregar_config():
    detecao = getDetecao()
    return (
        detecao["ativado"],
        detecao["PyPoll"],
        detecao["datasetname"],
        detecao["repo"],
        ",".join(str(i) for i in detecao["ids"]),
    )

def Imagem():
    """
    Detects ArUco markers in an image and returns the processed image.

    This function retrieves a photo using the `getPhoto` function, detects
    ArUco markers present in the given image through the `detetarAruko`
    function, and then returns the processed image with the detected ArUco
    markers included.

    :return: The processed image with detected ArUco markers
    :rtype: Any
    """
    img = getPhoto()
    detetarAruko(img)
    return img


##############################################
##CRIAR TABULEIRO TICTACTOE para representacao
def lerJogo():
    """
    Reads the game board configuration from a JSON source, processes it, and
    returns it as a reshaped numpy array.

    This function extracts the board data stored under the key "tabuleiro" in
    the JSON object returned by `getJson()`. It reshapes the extracted data
    into a 3x3 numpy array to represent the game board.

    :returns: A 3x3 numpy array representing the game board configuration.
    :rtype: numpy.ndarray
    """
    data = getJson()
    return np.array(data["tabuleiro"]).reshape(3, 3)

def criarTabuleiro():
    """
    Generates a graphical representation of a Tic-Tac-Toe game board.

    This function reads a previously saved game state and creates a 300x300 graphical
    image of a Tic-Tac-Toe board, divided into three equal sections, representing rows
    and columns. The function draws grid lines to form the board and visualizes the
    game state with specific shapes and colors: red rectangles for one player and
    green circles for the other. The resulting image can be used to visually represent
    the current state of the game.

    :rtype: Image
    :return: A PIL Image object representing the Tic-Tac-Toe game board with the
             current game state drawn onto it.
    """
    tab = lerJogo()
    quadrado = 300 // 3
    img = Image.new("RGB", (300, 300), "white")
    draw = ImageDraw.Draw(img)

    # fazer as linhas
    for i in range(1, 3):
        draw.line([(0, i * quadrado), (300, i * quadrado)], fill="black", width=3)
        draw.line([(i * quadrado, 0), (i * quadrado, 300)], fill="black", width=3)

    # desenhos bonitinhos
    for r in range(3):
        for c in range(3):
            x, y = c * quadrado, r * quadrado
            if tab[r, c] == 1:
                draw.rectangle([x + 20, y + 20, x + quadrado - 20, y + quadrado - 20], fill="red")
            elif tab[r, c] == -1:
                draw.ellipse([x + 20, y + 20, x + quadrado - 20, y + quadrado - 20], fill="green")

    return img

def atualizar():
    return gr.update(value=lerTurno()), gr.update(value=criarTabuleiro())
##########################3
##########################


def atualizarHSV(m_G_H, m_G_S, m_G_V, M_G_H, M_G_S, M_G_V, m_V_H, m_V_S, m_V_V, M_V_H, M_V_S, M_V_V):
    """
    Updates HSV color thresholds and detects colors in a given photo.

    This function takes in the minimum and maximum HSV values for two categories
    of colors (e.g., green and red), updates these thresholds in a configuration
    dictionary, applies these thresholds to a given photo using a color detection
    algorithm, and returns the updated HSV dictionary along with the paths of
    the images where detected colors are saved.

    :param m_G_H: Minimum Hue value for the green color range.
    :type m_G_H: int or float
    :param m_G_S: Minimum Saturation value for the green color range.
    :type m_G_S: int or float
    :param m_G_V: Minimum Value (brightness) for the green color range.
    :type m_G_V: int or float
    :param M_G_H: Maximum Hue value for the green color range.
    :type M_G_H: int or float
    :param M_G_S: Maximum Saturation value for the green color range.
    :type M_G_S: int or float
    :param M_G_V: Maximum Value (brightness) for the green color range.
    :type M_G_V: int or float
    :param m_V_H: Minimum Hue value for the red color range.
    :type m_V_H: int or float
    :param m_V_S: Minimum Saturation value for the red color range.
    :type m_V_S: int or float
    :param m_V_V: Minimum Value (brightness) for the red color range.
    :type m_V_V: int or float
    :param M_V_H: Maximum Hue value for the red color range.
    :type M_V_H: int or float
    :param M_V_S: Maximum Saturation value for the red color range.
    :type M_V_S: int or float
    :param M_V_V: Maximum Value (brightness) for the red color range.
    :type M_V_V: int or float
    :return: A tuple containing the updated HSV thresholds dictionary and file
             paths for the images with detected colors.
    :rtype: tuple(dict, str, str)
    """
    defaults = {
        "m_G_H": m_G_H,
        "m_G_S": m_G_S,
        "m_G_V": m_G_V,
        "M_G_H": M_G_H,
        "M_G_S": M_G_S,
        "M_G_V": M_G_V,
        "m_V_H": m_V_H,
        "m_V_S": m_V_S,
        "m_V_V": m_V_V,
        "M_V_H": M_V_H,
        "M_V_S": M_V_S,
        "M_V_V": M_V_V,
    }
    setCores(defaults)

    DetetarPorCor(getPhoto())
    return defaults,"vermelhodetetado.png","verdedetetado.png"

#da start ao main.py que e o jogo
def iniciarjogo():
    """
    Starts the game by navigating to the parent directory of the current script location
    and executing the "main.py" script from there.

    :return: A tuple with three elements:
        - The first element is a Gradio update object for visibility set to False.
        - The second element is a Gradio update object for visibility set to True.
        - A string message indicating the game has started and the turn number.
    :rtype: tuple
    """
    # sudo chmod a+rw /dev/ttyACM0
    GRIPPER_PORT = '/dev/ttyACM0'
    # Initialize the Robotiq Gripper
    #gripper = pyRobotiqGripper.RobotiqGripper(portname=GRIPPER_PORT)
    #gripper.resetActivate()
    
    subpasta = os.path.dirname(__file__)

    global process
    try:
        if process.poll() is not None:
            process.kill()
    except:
        print("skip")
    # Go up one level to the parent directory
    pasta = os.path.dirname(subpasta)
    process = subprocess.Popen(
        ["python", "main.py"],
        cwd=pasta,
        stdin=subprocess.PIPE,
        text=True
    )
    global dificuldade
    print(f"ISTO E HOMEM {dificuldade}")
    process.stdin.write(f"{dificuldade}\n")
    process.stdin.flush()
    #process.communicate(input=f"{dificuldade}")

    return gr.update(visible=True),gr.update(visible=True), f"Jogo iniciado! Turno: 1"

#Incrementa 1 no json para passar ao proximo turno
def terminar_turno():
    """
    Increments the turn counter stored in a JSON object and updates the game
    state by creating a new board configuration.

    The function retrieves the current state of the game from a JSON file or
    data source, updates the `turno` key by incrementing its value, and saves
    the updated state. It then pauses execution for a short time to simulate
    a delay before returning the updated game board and a new game board
    configuration.

    :raises KeyError: If the `turno` key does not exist in the JSON object.
    :raises AnyOtherExceptions: Any exceptions raised by `getJson`, `setJson`,
                                or `criarTabuleiro`.

    :return: A tuple containing the updated game board and a newly created
             game board configuration.
    :rtype: Tuple
    """
    turnos = getJson()
    #turnos["turno"] += 1
    #setJson(turnos)
    time.sleep(2.0)
    return tabuleiro, criarTabuleiro()


def moverrobo(data, b = False):
    from Externo.move import main
    from Externo.robotMove import ismoving
    main(data)
    while not ismoving(data, b):
        main(data)

#limpar tabuleiro
def arrumar():
    tabuleiro = getJson()["tabuleiro"]
    listapecas = [i for i,valor in enumerate(tabuleiro) if valor == 1]
    npecas = len(listapecas)
    data1 = getValoresPosicoes("approach_point")
    data2 = getValoresPosicoes("ponto_arrumar")
    from Externo.move import main
    moverrobo(data1)
    for i in listapecas:
        b = True
        if i < 2: b = False
        data3 = getValoresPosicoes("aruko_"+str(i))
        data4 = getValoresPosicoes("cubo_"+str(npecas))
        npecas -=1
        time.sleep(1.0)
        moverrobo(data3, True)
        time.sleep(2.0)
        main([], False)
        time.sleep(1.0)
        moverrobo(data1, True)
        time.sleep(2.0)
        moverrobo(data2,True)
        time.sleep(2.0)
        moverrobo(data4, False)
        time.sleep(2.0)
        main([], True)
        time.sleep(2.0)
        moverrobo(data1, True)
        time.sleep(2.0)

    moverrobo(data2)
    return gr.update(visible=True), gr.update(visible=True)


def lerTurno():
    turnos = getJson()

    if turnos["vencedor"] != 1 and turnos["vencedor"] != 0:
        name = "Humano" if turnos["turno"] % 2 == 0 else "Robo"
        return "É a vez do "+name
    else:
        name = "Humano" if turnos["vencedor"] == -1 else "Robo"
        return "O vencedor foi o " + name



def atualizar_detecao(usar_yolo_val, pypoll_val, datasetname_val, repo_val, ids_str, detecao):
    try:
        ids_list = [int(i.strip()) for i in ids_str.split(",") if i.strip().isdigit()]
    except ValueError:
        return "Erro: IDs devem ser números separados por vírgula."

    # Atualiza o JSON
    detecao["ativado"] = usar_yolo_val
    detecao["PyPoll"] = pypoll_val
    detecao["datasetname"] = datasetname_val
    detecao["repo"] = repo_val
    detecao["ids"] = ids_list

    atualizarDetecao(detecao)

    return "Detecção atualizada com sucesso!", detecao


################################
#ros conf
##########
posicoes_ros = ["aruko_0", "aruko_1", "aruko_2", "aruko_3", "aruko_4", "aruko_5", "aruko_6", "aruko_7", "aruko_8", "cil_1", "cil_2", "cil_3", "cil_4", "cil_5","cubo_1", "cubo_2", "cubo_3", "cubo_4", "cubo_5", "approach_point", "ponto_escondido", "ponto_arrumar"]
def comecar_setup():
    global rosprocess
    global setup_index
    setup_index = len(posicoes_ros) - 1
    subpasta = os.path.dirname(__file__)
    rosprocess = subprocess.Popen(
        ["python", "getPosicaoRos.py"],
        cwd=subpasta,
        stdin=subprocess.PIPE,
        text=True
    )

    return f"Mova para a posição: {posicoes_ros[setup_index]}"

def salvar_posicao():

    global rosprocess
    global setup_index
    if rosprocess == -1: return "Erro process not started"


    if setup_index >= len(posicoes_ros):
        rosprocess.kill()
        return "Setup finalizado."

    try:
        rosprocess.stdin.write(f"salvarposicao {posicoes_ros[setup_index]}\n")
        rosprocess.stdin.flush()
        setup_index += 1
        if setup_index < len(posicoes_ros):
            return f"Mova para a posição: {posicoes_ros[setup_index]}"
        else:
            return "Setup finalizado."
    except Exception as e:
        return f"Falha ao comunicar com o nó ROS: {e}"


def moverpara(valor):
    '''
    subpasta = os.path.dirname(__file__)
    processo = subprocess.Popen(
        ["python", "movePosicaoRos.py"],
        cwd=subpasta,
        stdin=subprocess.PIPE,
        text=True
    )
    processo.stdin.write(f"{valor}\n")
    processo.stdin.flush()
    '''
    from Externo.move import main

    data = getValoresPosicoes(valor)
    main(data)

def moverpara_gradio(selected_pos):
    moverpara(selected_pos)
    return f"Movendo para {selected_pos}"

def abrirgripper():
    from Externo.move import main
    main([], True)

def fechargripper():
    from Externo.move import main
    main([], False)

def main():
    """
    Creates a Gradio-based user interface with multiple tabs to handle different configurations
    and functionalities, including HSV settings, processing, game management, and calculations.

    Summary
    -------
    This function initializes and launches a Gradio Blocks-based interface with the following
    tabs:
    - "HSV": Provides a set of sliders to configure HSV (Hue, Saturation, and Value) settings for
      image detection and outputs visual feedback in the form of images and JSON data.
    - "Processamento": Displays the processed images from local storage representing various
      detection results.
    - "Jogo": Manages game functionality, allowing the user to initiate and terminate turns,
      select difficulty levels, and manipulate the game board.
    - "Cal": Offers a series of buttons to handle various calculation-related actions and displays
      an image related to the operation.

    Attributes
    ----------
    demo : gradio.Blocks
        Instance of the Gradio Blocks used to organize the interface structure.

    Tab Sections
    ------------
    1. HSV
        - Includes sliders for adjusting minimum and maximum values of HSV for two groups
          of detection inputs (`m_G_H`, `m_G_S`, etc. for the first group, and `m_V_H`, `m_V_S`, etc.
          for the second group).
        - Outputs include JSON data and images with detected colors visualized.

    2. Processamento
        - Displays images stored locally representing results from image processing.

    3. Jogo
        - Provides game-related features with a text output, game board, selectable difficulty,
          and interactive controls for initiating or concluding game turns.

    4. Cal
        - Contains multiple buttons for specialized operations and updates an image output
          upon interaction.

    Errors Raised
    -------------
    No errors are raised explicitly by the function.

    Parameters
    ----------
    No parameters are taken by the function.

    Returns
    -------
    None
        Function has no return value, as it primarily configures and launches the Gradio interface.
    """
    with gr.Blocks() as demo:
        gr.Markdown("### Settings")
        gr.HTML("<script>setTimeout(() => location.reload(), 500);</script>")

        with gr.Tabs():
            with gr.Tab("Jogo"):
                with gr.Row():
                    output = gr.Textbox(value=lerTurno())

                    tab = gr.Image(label="Tabuleiro", value=criarTabuleiro())
                    dif = gr.Dropdown(
                        ["Facil", "Medio", "Dificil"], label="Dificuldade",allow_custom_value=True, multiselect=False
                    )
                    dif.select(getDificuldade, inputs=dif)
                    global dificuldade
                    dificuldade = 0
                    btn_iniciar = gr.Button("Iniciar Jogo", visible=True)
                    btn_arrumar = gr.Button("Arrumar Tabuleiro", visible=True)
                    btn_atualizar = gr.Button("Atualizar", visible=True)
                    btn_iniciar.click(iniciarjogo, inputs=[], outputs=[btn_iniciar,
                                                                       btn_arrumar,
                                                                        output])

                    btn_arrumar.click(arrumar, inputs=[], outputs=[btn_iniciar, btn_arrumar])
                    btn_atualizar.click(atualizar, inputs=[], outputs=[output, tab])
            with gr.Tab("HSV"):
                with gr.Row():
                    m_G_H = gr.Slider(0, 255, value=defaults["m_G_H"], label="m_G_H")
                    m_G_S = gr.Slider(0, 255, value=defaults["m_G_S"], label="m_G_S")
                    m_G_V = gr.Slider(0, 255, value=defaults["m_G_V"], label="m_G_V")

                with gr.Row():
                    M_G_H = gr.Slider(0, 255, value=defaults["M_G_H"], label="M_G_H")
                    M_G_S = gr.Slider(0, 255, value=defaults["M_G_S"], label="M_G_S")
                    M_G_V = gr.Slider(0, 255, value=defaults["M_G_V"], label="M_G_V")

                with gr.Row():
                    m_V_H = gr.Slider(0, 255, value=defaults["m_V_H"], label="m_V_H")
                    m_V_S = gr.Slider(0, 255, value=defaults["m_V_S"], label="m_V_S")
                    m_V_V = gr.Slider(0, 255, value=defaults["m_V_V"], label="m_V_V")

                with gr.Row():
                    M_V_H = gr.Slider(0, 255, value=defaults["M_V_H"], label="M_V_H")
                    M_V_S = gr.Slider(0, 255, value=defaults["M_V_S"], label="M_V_S")
                    M_V_V = gr.Slider(0, 255, value=defaults["M_V_V"], label="M_V_V")

                output = gr.JSON()
                image_output1 = gr.Image(value="vermelhodetetado.png", label="Vermelho")
                image_output2 = gr.Image(value="verdedetetado.png", label="Verde")

                btn = gr.Button("Atualizar")
                btn.click(atualizarHSV,
                          inputs=[m_G_H, m_G_S, m_G_V, M_G_H, M_G_S, M_G_V, m_V_H, m_V_S, m_V_V, M_V_H, M_V_S, M_V_V],
                          outputs=[output, image_output1, image_output2])

            with gr.Tab("Processamento"):
                with gr.Row():
                    image_output1 = gr.Image(value=localstorage+"vermelhodetetado.png", label="Vermelho")
                    image_output2 = gr.Image(value=localstorage+"verdedetetado.png", label="Verde")
                    image_output3 = gr.Image(value=localstorage+"forma.png", label="Objetos")
                    image_output4 = gr.Image(value=localstorage+"arukos.png", label="Arukos")






            with gr.Tab("Cal"):
                with gr.Row():
                    btn13 = gr.Button("Inicio_Humano")
                    btn14 = gr.Button("Fim_Humano")
                    btn15 = gr.Button("Inicio_Robo")
                    btn16 = gr.Button("Fim_Robo")
                    btn17 = gr.Button("Ver Camera")
                    img = gr.Image()
                    btn17.click(Imagem,
                              inputs=[],
                              outputs=img)
            with gr.Tab("Setup"):
                with gr.Row():
                    comecar_btn = gr.Button("Começar Setup")
                    salvar_btn = gr.Button("Salvar Posição")
                    dropdown = gr.Dropdown(choices=posicoes_ros, label="Posições ROS")
                    btn = gr.Button("Mover para posição")
                    btng = gr.Button("Abrir Gripper")
                    btng2 = gr.Button("Fechar Gripper")

                status_display = gr.Textbox(label="Status", interactive=False)
                output_text = gr.Textbox(label="Status", interactive=False)

                btng.click(abrirgripper)
                btng2.click(fechargripper)
                comecar_btn.click(comecar_setup, outputs=status_display)
                salvar_btn.click(salvar_posicao, outputs=status_display)
                btn.click(
                    moverpara_gradio,
                    inputs=dropdown,
                    outputs=output_text
                )

                detecao_state = gr.State(getDetecao())
                with gr.Row():
                    usar_yolo = gr.Checkbox(label="Usar Yolo")
                    pypoll = gr.Checkbox(label="PyPoll")

                with gr.Row():
                    datasetname = gr.Textbox(label="Dataset Name")
                    repo = gr.Textbox(label="Repo")

                with gr.Row():
                    ids = gr.Textbox(label="IDs (separados por vírgula, ex: 1,2,3)", placeholder="1,2,3")

                with gr.Row():
                    atualizar_btn = gr.Button("Atualizar Detecção")

                demo.load(
                    fn=carregar_config,
                    inputs=[],
                    outputs=[usar_yolo, pypoll, datasetname, repo, ids]
                )

                atualizar_btn.click(
                    fn=atualizar_detecao,
                    inputs=[usar_yolo, pypoll, datasetname, repo, ids, detecao_state],
                    outputs=[status_display, detecao_state]
                )


        demo.launch()

if __name__ == "__main__":
    main()
