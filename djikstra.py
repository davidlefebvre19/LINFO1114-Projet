import numpy as np

def djikstra(adj):
    # init S and L
    S = []
    L = []
    for i in range(len(adj[0])):
        L.append(10**12)
    L[0] = 0

    while (len(S) != len(adj)):
        # find u : u not in S and with minimal cost
        u = 10**12
        u_idx = 0
        for idx, curr in enumerate(L):
            if idx not in S and curr < u:
                u = curr
                u_idx = idx
        S.append(u_idx)
        # update labels not in S and connected to u
        for idx, v in enumerate(L):
            if idx not in S:
                # check if neighboor
                if adj[u_idx][idx] != 10**12:
                    # check if shortest path has been found between u and v than previously
                    if v > u + adj[u_idx][idx]:
                        L[idx] = u + adj[u_idx][idx]
    D = adj.copy()
    for i in range(len(adj)):
        for j in range(len(adj)):
            sp = L[j] - L[i]
            D[i][j] = abs(sp)
    return D

if __name__ == "__main__":
    arr = np.loadtxt("data.csv", delimiter=",", dtype=int)
    print(arr)
    arr[arr == 0] = 10**12
    np.fill_diagonal(arr, 0)
    print(djikstra(arr))
