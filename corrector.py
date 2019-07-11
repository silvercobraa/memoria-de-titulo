import numpy as np
import numpy.linalg as la

edges = np.array([
    [0, 1], [0, 2], [0, 4], [0, 5],
    [1, 0], [2, 0], [4, 0], [5, 0],
    [3, 1], [3, 2], [3, 4], [3, 5],
    [1, 3], [2, 3], [4, 3], [5, 3],
    [1, 2], [1, 5], [2, 4], [4, 5],
    [2, 1], [5, 1], [4, 2], [5, 4],
])

corners = np.array([
    [0, 1, 2], [1, 2, 0], [2, 0, 1],
    [0, 2, 4], [2, 4, 0], [4, 0, 2],
    [0, 4, 5], [4, 5, 0], [5, 0, 4],
    [0, 5, 1], [5, 1, 0], [1, 0, 5],
    [3, 2, 1], [2, 1, 3], [1, 3, 2],
    [3, 1, 5], [1, 5, 3], [5, 3, 1],
    [3, 4, 2], [4, 2, 3], [2, 3, 4],
    [3, 5, 4], [5, 4, 3], [4, 3, 5],
])

def best_edge(cent, edge):
    best = float('inf')
    ret = None
    for e in edges:
        # print(e, cent[e[0]], cent[e[1]], edge)
        diff = la.norm(edge - cent[e[0]]) + la.norm(edge - cent[e[1]])
        if diff < best:
            # print(diff)
            best = diff
            ret = e

    return ret


def best_corner(cent, corner):
    best = float('inf')
    ret = None
    for c in corners:
        # print(e, cent[e[0]], cent[e[1]], corner)
        diff = la.norm(corner - cent[c[0]]) + la.norm(corner - cent[c[1]]) + la.norm(corner - cent[c[2]])
        if diff < best:
            # print(diff)
            best = diff
            ret = c

    return ret


def get_edges(x, centroids):
    edge_order = [
        [ 5, 10], [ 7, 19], [ 3, 37], [ 1, 46],
        [28, 25], [30, 43], [34, 52], [32, 16],
        [12, 23], [21, 41], [39, 50], [48, 14],
    ]
    es = np.array([
        [x[e[0]], x[e[1]]] for e in edge_order
    ])
    # print('cs', cs)
    for xx in es:
        print(best_edge(centroids, xx))


def get_corners(x, centroids):
    corner_order = [
        [ 8,  9, 20], [ 6, 18, 38], [ 0, 36, 47], [ 2, 45, 11],
        [15, 29, 26], [24, 27, 44], [42, 33, 53], [51, 35, 17],
    ]
    cs = np.array([
        [x[c[0]], x[c[1]], x[c[2]]] for c in corner_order
    ])
    for xx in cs:
        print(best_corner(centroids, xx))


# print(edges. reshape(-1))
x = np.load('pca.npy')
centroids = np.array([x[i] for i in range(4, 54, 9)])
# print(x)
# get_edges(x, centroids)
get_corners(x, centroids)
