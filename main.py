from shortest_path_algorithms import (
    Djisktra,
    Floyd_Warshall,
    Bellman_Ford,
    print_matrix,
    compare_algorithms,
)
import numpy as np


def main():
    # Load CSV
    matrix = np.loadtxt("data.csv", delimiter=",", dtype=int)

    # Creating the costs matrix
    matrix[matrix == 0] = 10**12
    np.fill_diagonal(matrix, 0)

    print("Matrice de coûts :")
    print_matrix(matrix)
    print()

    print("Matrice de distances via Djisktra :")
    a = Djisktra(matrix.copy())
    print_matrix(a)
    print()

    print("Matrice de distances via Bellman_Ford :")
    b = Bellman_Ford(matrix.copy())
    print_matrix(b)
    print()

    print("Matrice de distances via Floyd_Warshall :")
    c = Floyd_Warshall(matrix)
    print_matrix(c)
    print()

    are_equals, failed = compare_algorithms(b, a, c)
    if are_equals:
        print("Les 3 implémentations renvoient le même résultat")
    elif failed == "djisktra - floyd":
        print("Les algorithmes djisktra and floyd ne renvoient pas le même résultat")
        print("Djisktra: \n", a)
        print("Floyd: \n", c)
    else:
        print("Les algorithmes djisktra and bellman ne renvoient pas le même résultat")
        print("Bellman: \n", b)
        print("Djisktra: \n", a)


if __name__ == "__main__":
    main()
