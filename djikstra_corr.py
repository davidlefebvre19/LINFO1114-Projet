import numpy as np


def dijkstra(adj, src):
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
        L = dijkstra(adj, src)
        for dest in range(n):
            D[src][dest] = L[dest]
    return D


if __name__ == "__main__":
    # Charger la matrice depuis le fichier
    arr = np.loadtxt("data.csv", delimiter=",", dtype=int)
    if arr.shape[0] != arr.shape[1]:
        raise ValueError("La matrice doit être carrée.")

    # Remplacer les 0 par une grande valeur pour indiquer l'absence de connexion (sauf la diagonale)
    arr[arr == 0] = 10 ** 12
    np.fill_diagonal(arr, 0)  # La diagonale représente les distances nulles

    # Calculer la matrice des plus courts chemins
    shortest_paths = Djikstra(arr)
    print("Matrice des plus courts chemins :")
    print(shortest_paths)
