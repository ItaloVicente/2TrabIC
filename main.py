from Grafo import Grafo

grafo = Grafo("grafo.gr")
#Busca em Largura
print(f"Quantidade de arestas para chegar em t a partir de s: {str(grafo.bfs("s")[0]["t"])}")
#Busca custo uniforme
print(f"Custo para chegar em t a partir de s: {str(grafo.djikstra("s")[0]["t"])}")
#Busca em profundidade
print(f"Tempo para chegar em t a partir de s: {grafo.inicia_dfs()[1]['t']}")