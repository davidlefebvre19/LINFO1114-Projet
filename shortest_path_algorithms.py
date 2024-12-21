import numpy as np
import heapq


# in : matrice d'adjacence d'un graphe
# out : matrice d'adjacence contenant les distances des plus court chemin entre un noeud et les autres noeuds du graphe
def djikstra_one_node(adj, src):
    # Initialisation
    n = len(adj)
    S = []  # Ensemble des sommets traités
    L = [10 ** 12] * n  # Labels de distance initialisés à un grand nombre
    L[src] = 0  # La distance au sommet source est 0

    while len(S) != n:
        # Trouver u : sommet non traité avec le coût minimal
        u = None
        u_cost = 10 ** 12
        for idx, curr in enumerate(L):
            if idx not in S and curr < u_cost:
                u = idx
                u_cost = curr

        # Ajouter u à S
        if u is None:
            break  # Tous les sommets accessibles ont été traités
        S.append(u)

        # Mettre à jour les labels des voisins de u
        for idx in range(n):
            if idx not in S and adj[u][idx] != 10 ** 12:
                # Vérifier si un chemin plus court est trouvé
                if L[idx] > L[u] + adj[u][idx]:
                    L[idx] = L[u] + adj[u][idx]
                    
    return L


def Djikstra(adj):
    # Construire la matrice des plus courts chemins pour toutes les paires de sommets
    n = len(adj)
    D = np.zeros_like(adj)
    for src in range(n):
        # Calculer les distances à partir du sommet src
        L = djikstra_one_node(adj, src)
        for dest in range(n):
            D[src][dest] = L[dest]
    return D


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
