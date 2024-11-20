import sys
import numpy as np
import os
import heapq
import networkx as nx

matrix = None
algo = 0

# in : matrice d'adjacence d'un graphe
# out : matrice d'adjacence contenant les distances des plus court chemin entre un noeud et les autres noeuds du graphe
def Djisktra(arr):
    nb_nodes = len(arr)
    result_matrix = np.zeros((len(arr),len(arr)), dtype=object) # Matrice des distances des plus-court chemin
    for i in range(nb_nodes):
        shortest_dist = sten(i, arr) # Pour chaque noeud, cette fonction va nous donner un vecteur contenant les distances avec chaque noeud
        print(shortest_dist)
        print(i)
        result_matrix[i] = np.matrix(shortest_dist)
    return result_matrix

def sten(source, adjacence_matrix):
    stnd = np.zeros(len(adjacence_matrix), dtype="int") # Vecteur contenant les distances avec les autres noeuds (STND: source to node distance)
    stnd.fill(10**12) # Les valeurs des distances sont initialisées à la valur symbolique "Infinity" (concept clef: aucune valeur n'est supérieure à celle-ci) mais à 10**12 dans notre algorithme (directive des consignes)
    visited_nodes = np.zeros(len(adjacence_matrix), dtype="int") # Vecteur contenant les noeuds qui ont été visités (0 : non-visité, 1 : visité)
    stnd[source] = 0 # Distance entre la source et la source = 0
    iter = 0
    while iter < len(adjacence_matrix): # On itère n fois, n = nombre de noeuds
        next_node = 0
        # Calcul du noeud le plus proche du noeud courant (-> prochain noeud que l'algorithme visitera)
        heap = [] # Utilisation d'un priority queue (inspiration provenant du livre "Algorithms, fourth edition") qui contiendra la liste des distances des noeuds qui sont accessible par le noeud source ET qui n'ont pas encore été visités
        for i in range(len(adjacence_matrix)):
            if visited_nodes[i] == 0: # Si le noeud n'a pas encore été visité...
                heapq.heappush(heap,(stnd[i], i)) # ... il est rajouté dans la heap
        next_node = heapq.heappop(heap)[1] # Nous prenons l'index du noeud correspondant à celui le plus proche de la source
        visited_nodes[next_node] = 1 # Ce noeud sera "visité" donc la valeur associée au noeud de la liste visited_nodes est actualisée à 1
        # Calcul du vecteur des distances
        for i in range(len(adjacence_matrix)):
            if not visited_nodes[i] and stnd[i] != 0 and stnd[i] > stnd[next_node] + adjacence_matrix[next_node][i]: # Ici nous avons pour objectif d'actualiser le vecteur des distances, cela se fait sous 3 conditions
                # ```not visited_nodes[i]``` : Le noeud en question ne doit pas être visité car cela voudrait dire qu'on rajouterai du chemin à un noeud qui  à déjà été visité (la valeur des distances ne serait plus la valeur des distances les plus courte)
                # ```stnd[i] != 0            : Lorsque stnd[i] = 0, cela correspond à la valeur de distance entre le noeud source et le noeud source, on ne calcule pas de valeur du plus court chemin dans ce cas-ci
                # ```stnd[i] > stnd[next_node] + adjacence_matrix[next_node][i]```: Si jamais le noeud est à visiter nous venons rajouter le coût du parcour de ce noeud avec le coût du chemin pour arriver au noeud précédent
                stnd[i] = stnd[next_node] + adjacence_matrix[next_node][i]
        iter+=1
    return stnd

def Bellman_Ford(arr):
    # TODO
    result_matrix = np.zeros((len(arr), len(arr)), dtype=object)  # Matrice des distances des plus-court chemin
    return result_matrix

def Floyd_Warshall(arr):
    # TODO
    result_matrix = np.zeros((len(arr), len(arr)), dtype=object)  # Matrice des distances des plus-court chemin
    return result_matrix

if __name__ == '__main__':
    arr = np.loadtxt('data.csv', delimiter=",")
    arr2 = arr.copy()
    arr[arr == 0] = 10 ** 12
    np.fill_diagonal(arr, 0)
    matrix_g = np.asmatrix(arr) #convertir np.array en np.matrix pour respecter la signature
    print(matrix_g)
    print("By default, the shortest path of every pair will be calculated using Djisktra, Bellman-Ford and "
          "Floyd_Warshall")

    print("Djiskstra algorithm array :")
    a = Djisktra(arr)
    print(a)

    G = nx.from_numpy_matrix(np.array(arr), create_using=nx.DiGraph)
    shortest_path_matrix = dict(nx.all_pairs_dijkstra_path_length(G))

    # Conversion des résultats en matrice pour comparer
    validated_matrix = np.zeros((len(arr), len(arr)), dtype=int)
    for i in range(len(arr)):
        for j in range(len(arr)):
            validated_matrix[i, j] = shortest_path_matrix[i][j]
            
    #print("Bellman_Ford algorithm array :")
    b = Bellman_Ford(arr)
    #print(Bellman_Ford(arr))
    #print("Floyd_Warshall algorithm array :")
    c = Floyd_Warshall(matrix_g)
    #print(Floyd_Warshall(matrix_g))

    #if np.array_equal(a, b) and np.array_equal(a, c):
    #    print("All algorithms calculated the same shortest paths")