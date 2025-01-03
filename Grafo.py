import heapq #uso para o metodo dijkstra
import math #uso para numeros infinitos
tempo = 0 #usei o tempo de forma global para tentar diminuir o espaco de recursividade usado pelo computador em dfs

class Grafo: #classe criada para grafos nao horientados
    def __init__(self, caminho): #dou o caminho de onde estara o grafo
        self.tipo = "grafo"
        self.listaAdj = ListaAdj(self.tipo)
        self.qnt_arestas = 0
        with open(caminho, 'r') as arquivo: #adicionando os vertices e arestas a lista, e contabiliza a qnt de arestas
            for linha in arquivo:
                splitted_linha = linha.split(" ")
                if splitted_linha[0] == "a": #as arestas começam quando se encontra o caractere "a"
                    self.qnt_arestas += 1
                    if splitted_linha[1] not in self.listaAdj.lista:
                        self.listaAdj.addVertice(splitted_linha[1])
                    if splitted_linha[2] not in self.listaAdj.lista:
                        self.listaAdj.addVertice(splitted_linha[2])
                    self.listaAdj.addAresta(splitted_linha[1], splitted_linha[2], splitted_linha[3])

    def bfs(self, vertice):  # deixarei o usuario escolher qual vertice ele quer iniciar
        vertices = {}  # aqui inicializo um dicionario em que o vertice seria a chave, o valor seria uma lista com cor, distancia e predecessor, respectivamente
        d = {}  # aqui inicializo um dicionario em que a key sera o vertice em que o laco passou e o valor sera a distancia em relacao ao vertice passado como parametro
        pi = {}  # aqui inicializo um dicionario em que a key sera o vertice em que o laco passou e o valor sera o seu predecessor
        Q = []  # lista com os vertices a serem visitados
        temp = {}
        explorados=[]
        for key in self.listaAdj.lista:
            vertices[key] = ["branco", math.inf, None]
        Q.append(vertice)
        vertices[vertice] = ["cinza", 0,
                             None]  # inicializa o vertice de origem com cor cinza, distancia 0 e sem predecessor
        while len(Q) > 0:
            print("\n--- Iteração da BFS ---")
            print(f"Borda atual: {Q}")
            print(f"Explorados: {explorados}")
            print(f"Árvore de busca (predecessores): {temp}")
            Q.remove(vertice)
            explorados.append(vertice)
            for i in self.listaAdj.lista[vertice]:  # verifica se seus vizinhos sao brancos (ainda n visitados)
                if vertices[i[0]][0] == "branco":
                    vertices[i[0]] = ["cinza", vertices[vertice][1] + 1, vertice]  # altera sua cor, distancia e predecessor
                    temp[i[0]] = [vertices[vertice][1] + 1, vertice]
                    Q.append(i[0])
            vertices[vertice][0] = "preto"
            if len(Q) > 0:
                vertice = Q[0]
        for i in vertices:  # por fim, retorna dois dicionários com as distancias e predecessores dos vertices
            d[i] = vertices[i][1]
            pi[i] = vertices[i][2]
        print("\n=== Fim da Busca em Largura (BFS) ===")
        return d, pi

    def inicia_dfs(self):  # Inicializa a DFS
        global tempo
        vertices = {}  # Inicializa os dicionários de cor, tempos e predecessores
        pi = {}
        temp_ini = {}
        temp_final = {}
        dfs_tree = {}  # Estrutura para armazenar a árvore do DFS
        tempo = 0

        print("\n=== Iniciando DFS ===")

        for key in self.listaAdj.lista:
            vertices[key] = ["branco", [], None]  # Branco: ainda não visitado
            dfs_tree[key] = []  # Cada vértice tem uma lista vazia de filhos

        for v in vertices:
            if vertices[v][0] == "branco":
                print(f"\n>>> Iniciando busca a partir do vértice: {v}")
                self.busca_dfs(v, vertices, dfs_tree)

        for v in vertices:  # Preenche os resultados
            pi[v] = vertices[v][2]
            temp_ini[v] = vertices[v][1][0]
            temp_final[v] = vertices[v][1][1]

        print("\n=== DFS concluída ===")
        print("Predecessores:", pi)
        print("Tempo de início:", temp_ini)
        print("Tempo de finalização:", temp_final)
        print("Árvore do DFS:", dfs_tree)

        return pi, temp_ini, temp_final, dfs_tree

    def busca_dfs(self, v, vertices, dfs_tree):
        global tempo
        tempo += 1
        vertices[v][1].append(tempo)  # Tempo inicial
        vertices[v][0] = "cinza"  # Em processamento
        print(f"\nVisitando vértice {v}, tempo inicial: {tempo}")
        temp = {}
        for i in vertices:
            if len(vertices[i][1]) != 0:
                temp[i] = vertices[i][1:]
        print(f"Estado atual dos vértices: {temp}")

        for vizinho in self.listaAdj.lista[v]:  # Explorar vizinhos
            if vertices[vizinho[0]][0] == "branco":
                print(f"Vértice {vizinho[0]} descoberto a partir de {v}.")
                vertices[vizinho[0]][2] = v
                dfs_tree[v].append(vizinho[0])  # Adiciona o vizinho como filho do vértice atual na árvore
                self.busca_dfs(vizinho[0], vertices, dfs_tree)

        tempo += 1
        vertices[v][1].append(tempo)  # Tempo final
        vertices[v][0] = "preto"  # Finalizado
        print(f"Finalizando vértice {v}, tempo final: {tempo}")
        print(f"Estado dos vértices após finalizar {v}: {vertices}")

    def cria_heap(self,v): #função criada para organizar os vértices numa fila de prioridade com base em suas distancias
        Q = []
        Q.append((0, v))
        for i in self.listaAdj.lista.keys():
            if i == v:
                continue
            else:
                Q.append((math.inf, i))
        return Q
    def mantem_heap(self, Q, tupla): #funcao chamada caso a distancia do vertice tenha sido atualizada
        Q.append(tupla)
        heapq.heapify(Q)
        return Q
    def djikstra(self, v): #funcao que representa o algoritmo de Djikstra
        S=set()
        vertices = self.inicializa(v)
        Q = self.cria_heap(v) #chamada da funcao para criar uma fila de proridade
        d = {}
        pi = {}
        temp = {}
        explorados = []
        while len(Q)>0: #enquanto tiver elementos na fila
            print("\n--- Iteração do Djikstra ---")
            print(f"Borda atual: {Q}")
            print(f"Explorados: {explorados}")
            print(f"Árvore de busca (predecessores): {temp}")
            if v[0] not in explorados:
                explorados.append(v[0])
            u = Q.pop(0)
            if u[1] in S:
                continue
            S.add(u[1])
            for v in self.listaAdj.lista[u[1]]: #novamente a  funcao relaxa é chamada duas vezes para o par de vertices no caso do grafo nao orientado
                vertices = self.relaxa(u[1],v[0],vertices)
                vertices = self.relaxa(v[0], u[1], vertices)
                tupla = (vertices[v[0]][0],v[0]) #atualiza a informcao do vertice
                Q = self.mantem_heap(Q, tupla) #atualiza a lista de proridade Q
            temp[u[1]] = list(tupla)
        for i in vertices:#funcao retorna dois dicionarios d e pi contendo as distancias e predecessores dos vertices, respectivamente
            d[i] = vertices[i][0]
            pi[i] = vertices[i][1]
        print("\n=== Fim da Busca de Custo Uniforme (Djikstra) ===")
        return d, pi

    def relaxa(self, origem, destino,vertices):  # funcao relaxa usada para atualizar a distancia entre vertices através dos pesos
        for valor in self.listaAdj.lista[origem]:
            if valor[0] == destino:
                peso = int(valor[1])
        if float(vertices[destino][0]) > float(vertices[origem][0]) + int(peso):  # verifica se a distancia do vertice de destino é maior que a do vertice de origem mais seu peso
            vertices[destino][0] = int(vertices[origem][0]) + peso  # se sim, atualiza sua distancia e seu predecssor
            vertices[destino][1] = origem
        return vertices

    def inicializa(self,v):  # funcao usada para inicializar os vertices (exceto a origem) com distancia infinita e sem predecessor
        vertices = {}
        vertices[v] = [0, None]  # o vertice origem tem disatancia 0 e sem predecessor
        for key in self.listaAdj.lista:
            if key != v:
                vertices[key] = [math.inf, None]  # pos 0 = distancia, pos 1 = pai
        return vertices
class ListaAdj: #resolvemos criar uma classe de lista adj para evitar repeticao de codigo, tambem escolhemos a lista por menor complexidade
    def __init__(self, tipo):
        self.lista = {} #dicionario, a chave seria o vertice, o valor seria uma lista [[vertice de chegada, peso da aresta], ...]
        self.tipo = tipo #diz o tipo do grafo, digito "grafo" para grafo e "digrafo" para digrafo.

    def addVertice(self, vertice): #adicionando um vertice a lista adjacente.
        self.lista[vertice] = []

    def addAresta(self, origem, destino, peso): #adicionando uma aresta
        aresta = [destino, peso]
        arestaVolta=[origem, peso]
        self.lista[origem].append(aresta)
        if self.tipo == "grafo": #se nao for digrafo, tambem deve-se adicionar o vizinho no vertice de destino
            self.lista[destino].append(arestaVolta)
