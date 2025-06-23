import time
from time import sleep

from vars import vitorias, Heuristica

#verifica se o jogo terminou
def estadoterminal(tabuleiro):
    for (a, b, c) in vitorias:
        if tabuleiro[a] == tabuleiro[b] == tabuleiro[c] != 0:
            return True

    #EMPATE
    if all(celula != 0 for celula in tabuleiro):
        return True


    return False



#Funcao de avaliacao com elementos estaticos, adicionando utilidade com base na posicao
def avaliacao(tabuleiro, jogador):
    utilidade = 0
    for (a,b,c) in vitorias:
        pecasjogador = sum([tabuleiro[pos] == jogador for pos in (a, b, c)])
        pecasoponente = sum([tabuleiro[pos] == jogador * -1 for pos in (a, b, c)])
        utilidade += Heuristica[pecasjogador][pecasoponente]

    return utilidade


#Retorna todas as jogadas possiveis num tabuleiro
def jogadasdisponiveis(tabuleiro):
    t = []
    for i,j in enumerate(tabuleiro):
        if(j == 0):
            t.append(i)
    return t

#Executa uma jogada, da replace nao verifica se a jogada e possivel de fazer, pois num tabuleiro real nao acontece
def jogar(tabuleiro, jogada, jogador):
    tabuleiro[jogada] = jogador


    return tabuleiro


def jogarf(tabuleiro, jogada, jogador):
    tabuleiro[jogada] = jogador
    if jogador == 1:
        from Externo.move import main
        from libs.JsonLib import getValoresPosicoes
        num = len([k for k in tabuleiro if k == 1])
        data1 = getValoresPosicoes("cubo_"+str(num))
        print("cubo_"+str(num))
        data2 = getValoresPosicoes("aruko_" + str(jogada))
        data3 = getValoresPosicoes("approach_point")
        data4 = getValoresPosicoes("ponto_escondido")
        data5 = getValoresPosicoes("ponto_arrumar")

        from Externo.robotMove import ismoving

        import operator
        b = False if operator.lt(num, 3) else True

        time.sleep(1.0)
        main(data1)
        while not ismoving(data1, b):
            main(data1)
        time.sleep(3.0)
        main([], False)
        time.sleep(1.0)
        main(data5)
        while not ismoving(data5, True):
            main(data5)

        #Devido ao braco robotico nem sempre responder aos comandos atravez do controlador colocou se uma verificacao de movimento, para reenviar o comando
        time.sleep(2.0)
        main(data3)
        while not ismoving(data3, True):
            main(data3)

        time.sleep(2.0)
        main(data2)
        while not ismoving(data2, True):
            main(data2)

        time.sleep(2.0)
        main([], True)

        time.sleep(1.0)
        main(data3)
        while not ismoving(data3, True):
            main(data3)

        time.sleep(3.0)
        main(data5)
        while not ismoving(data5, True):
            main(data5)

        time.sleep(2.0)

    return tabuleiro

#cancela uma jogada pois o negamax altera o tabuleiro
def cancelarjogada(tabuleiro, jogada):
    tabuleiro[jogada] = 0
    return tabuleiro

#Definicao negamax
def negamax(tabuleiro, jogador, depth):
    if estadoterminal(tabuleiro) or depth == 0:
        return -1,avaliacao(tabuleiro, jogador)

    melhorutilidade = float('-inf')
    melhorjogada = -1
    jogadas = jogadasdisponiveis(tabuleiro)
    for i in jogadas:
        jogar(tabuleiro, i, jogador)
        _, pontuacao = negamax(tabuleiro, jogador * -1, depth-1)
        cancelarjogada(tabuleiro, i)

        #aplicar negamax utilidade de um jogador e autilidade contratia para outro
        #min(a, b) = -max(-a, -b)
        pontuacao *= -1
        if pontuacao > melhorutilidade:
            melhorjogada = i
            melhorutilidade = pontuacao

    return melhorjogada, melhorutilidade
