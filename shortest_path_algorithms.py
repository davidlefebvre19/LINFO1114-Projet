import numpy as np
import heapq


# in : matrice d'adjacence d'un graphe
# out : matrice d'adjacence contenant les distances des plus court chemin entre un noeud et les autres noeuds du graphe
def Djisktra(matrix):
    nb_nodes = len(matrix)
    result_matrix = np.zeros(
        (len(matrix), len(matrix)), dtype=int
    )  # Matrice des distances des plus-court chemin
    for i in range(nb_nodes):
        shortest_dist = sten(
            i, matrix
        )  # Pour chaque noeud, cette fonction va nous donner un vecteur contenant les distances avec chaque noeud
        # print(shortest_dist)
        # print(i)
        result_matrix[i] = np.matrix(shortest_dist)
    return result_matrix


def sten(source, adjacence_matrix):
    stnd = np.zeros(len(adjacence_matrix), dtype="int")  # Vecteur contenant les distances avec les autres noeuds (STND: source to node distance)
    stnd.fill(10**12) # Les valeurs des distances sont initialisées à la valur symbolique "Infinity" (concept clef: aucune valeur n'est supérieure à celle-ci) mais à 10**12 dans notre algorithme (directive des consignes)
    visited_nodes = np.zeros(len(adjacence_matrix), dtype="int")  # Vecteur contenant les noeuds qui ont été visités (0 : non-visité, 1 : visité)
    stnd[source] = 0  # Distance entre la source et la source = 0
    iter = 0
    while iter < len(adjacence_matrix):  # On itère n fois, n = nombre de noeuds
        next_node = 0
        # Calcul du noeud le plus proche du noeud courant (-> prochain noeud que l'algorithme visitera)
        heap = []  # Utilisation d'un priority queue (inspiration provenant du livre "Algorithms, fourth edition") qui contiendra la liste des distances des noeuds qui sont accessible par le noeud source ET qui n'ont pas encore été visités
        for i in range(len(adjacence_matrix)):
            if visited_nodes[i] == 0:  # Si le noeud n'a pas encore été visité...
                heapq.heappush(heap, (stnd[i], i))  # ... il est rajouté dans la heap
        next_node = heapq.heappop(heap)[1]  # Nous prenons l'index du noeud correspondant à celui le plus proche de la source
        visited_nodes[next_node] = (1)# Ce noeud sera "visité" donc la valeur associée au noeud de la liste visited_nodes est actualisée à 1
        # Calcul du vecteur des distances
        for i in range(len(adjacence_matrix)):
            if (
                not visited_nodes[i]
                and stnd[i] != 0
                and stnd[i] > stnd[next_node] + adjacence_matrix[next_node][i]
            ):  # Ici nous avons pour objectif d'actualiser le vecteur des distances, cela se fait sous 3 conditions
                # ```not visited_nodes[i]``` : Le noeud en question ne doit pas être visité car cela voudrait dire qu'on rajouterai du chemin à un noeud qui  à déjà été visité (la valeur des distances ne serait plus la valeur des distances les plus courte)
                # ```stnd[i] != 0            : Lorsque stnd[i] = 0, cela correspond à la valeur de distance entre le noeud source et le noeud source, on ne calcule pas de valeur du plus court chemin dans ce cas-ci
                # ```stnd[i] > stnd[next_node] + adjacence_matrix[next_node][i]```: Si jamais le noeud est à visiter nous venons rajouter le coût du parcour de ce noeud avec le coût du chemin pour arriver au noeud précédent
                stnd[i] = stnd[next_node] + adjacence_matrix[next_node][i]
        iter += 1
    return stnd


def Bellman_Ford(matrix):
    """
    L'algorithme fonctionne en itérant sur les arêtes du graphe et en relaxant
    (mettant à jour) les distances à chaque noeud. L'algorithme
    effectue au maximum |V| - 1 itérations, où |V| est le nombre de noeuds.
    Après ces itérations, les distances les plus courtes entre le noeud source
    et tous les autres noeuds sont calculées.
    """

    num_nodes = len(matrix)

    result_matrix = np.zeros((num_nodes, num_nodes), dtype=int)

    for source in range(num_nodes):
        distances = [10**12] * num_nodes  # set all distances to infinity
        distances[source] = 0  # distance to self is 0

        # Relax edges |V| - 1 times
        for _ in range(num_nodes - 1):
            for u in range(num_nodes):  # starting node
                for v in range(num_nodes):  # end node
                    weigth = matrix[u, v]
                    if weigth < 10**12 and distances[u] + weigth < distances[v]:
                        distances[v] = distances[u] + weigth

        result_matrix[source] = distances

    return result_matrix


def Floyd_Warshall(matrix):
    """
    L'algorithme fonctionne en itérant sur tous les noeuds intermédiaires possibles
    (k) et en vérifiant, pour chaque paire de noeuds (i, j), si un chemin passant
    par l'intermédiaire k offre un chemin plus court que celui actuellement connu.
    Si oui, on met à jour la distance entre i et j.
    """

    num_nodes = len(matrix)
    distances = matrix.copy()

    for k in range(num_nodes):  # intermediate nodes
        for i in range(num_nodes):  # sources nodes
            for j in range(num_nodes):  # destination nodes
                # If the path through k is shorter, update the distance
                if distances[i, k] < 10**12 and distances[k, j] < 10**12:
                    distances[i, j] = min(
                        distances[i, j], distances[i, k] + distances[k, j]
                    )

    return distances


def compare_algorithms(bellman: np.ndarray, djisktra: np.ndarray, floyd: np.ndarray):
    bellman_djisktra = np.array_equal(bellman, djisktra)
    djisktra_floyd = np.array_equal(djisktra, floyd)

    are_equals = bellman_djisktra and djisktra_floyd

    if are_equals:
        return True, None
    else:
        if bellman_djisktra:
            return False, "djisktra - floyd"
        else:
            return (
                False,
                "bellman - djisktra",
            )


def print_matrix(matrix):
    for row in matrix:
        row_print = []
        for x in row:
            if x == 10**12:
                row_print.append("∞")
            else:
                row_print.append(x)

        print(", ".join(map(str, row_print)))
