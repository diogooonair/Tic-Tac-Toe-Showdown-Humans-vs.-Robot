#defenicao do tabuleiro de jogo
tabuleiro = [0,0,0,
             0,0,0,
             0,0,0]


#Define todas as possiveis combinacoes no tabuleiro
vitorias = [
  { 0, 1, 2 },
  { 3, 4, 5 },
  { 6, 7, 8 },
  { 0, 3, 6 },
  { 1, 4, 7 },
  { 2, 5, 8 },
  { 0, 4, 8 },
  { 2, 4, 6 }
]

#utilidade de estados
# call [pecas tuas, pecas do adversario]
#Para cada peca tua num dos padroes vencedores teras utilidade se tiveres vantagem
#caso o padrao seja impossivel pecas dos 2 utilidade 0, ou ninguem tenha jogado nesse padrao
Heuristica = [
  [0, -10, -100, -1000],
  [10, 0, 0, 0],
  [100, 0, 0, 0],
  [1000, 0, 0, 0]
]


p_aruko = [[0,0,0,0], [0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]

localstorage = "TestStore/"