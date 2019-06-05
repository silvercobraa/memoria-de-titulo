import os
import numpy as np
import pylab as pl
from mpl_toolkits.mplot3d import Axes3D
#from sklearn.discriminant_analysis import QuadraticDiscriminantAnalysis
from sklearn.cluster import KMeans, MeanShift
from sklearn.mixture import GaussianMixture

DATASET_DIR = 'dataset_tarde'
# TEST_FILE = 'test_set.npy'
# TEST_FILE = 'capturas_sin_luz/representatives.npy'
TEST_FILE = 'capturas_temp/representatives.npy'

figure = pl.figure()
axis = figure.add_subplot(111, projection='3d')

train_points = np.empty((0, 3))
train_labels = np.empty((0, 1), dtype=int)
cluster_colors = []

def labels2perm(labels):
    # mapeo del orden en el que son capturados los facelets al orden requierido por 2 phase solver
    facelet_order = [
        # orden de los facelets segun la camara de la cabeza del baxter
        # 42, 39, 36,   43, 40, 37,   44, 41, 38, # U
        # 9, 10, 11,   12, 13, 14,   15, 16, 17, # R
        # 0, 1, 2,   3, 4, 5,   6, 7, 8, # F
        # 47, 50, 53,   46, 49, 52,   45, 48, 51, # D
        # 27, 28, 29,   30, 31, 32,   33, 34, 35, # L
        # 18, 19, 20,   21, 22, 23,   24, 25, 26, # B

        # orden de los facelets webcam (F, R, B, L, U, D)
        42, 39, 36,   43, 40, 37,   44, 41, 38, # U
        9, 10, 11,   12, 13, 14,   15, 16, 17, # R
        0, 1, 2,   3, 4, 5,   6, 7, 8, # F
        47, 50, 53,   46, 49, 52,   45, 48, 51, # D
        27, 28, 29,   30, 31, 32,   33, 34, 35, # L
        18, 19, 20,   21, 22, 23,   24, 25, 26, # B

    	# 41, 1,   37, 10,   39, 19,   43, 28, # UF, UR, UB, UL
    	# 50, 7,   52, 16,   48, 25,   46, 34, # DF, DR, DB, DL
    	# 5, 12,   3, 32,   21, 14,   23, 30, # FR, FL, BR, BL
    	# 38, 2, 9,   36, 11, 18,   42, 20, 27,   44, 29, 0, # UFR, URB, UBL, ULF
    	# 53, 15, 8,   47, 6, 35,   45, 33, 26,   51, 24, 17,
    ]
    # mapeo de las etiquetas asignadas por GMM a etuiquetas en orden FRDLUB
    center_labels = [0 for i in range(6)]
    centers = labels[4::9]
    for i, label in enumerate(centers):
        center_labels[label] = i

    # mapeo de etiquetas de color (numeros) a caras
    faces = 'FRBLUD'
    # faces = 'URFDLB'
    print(facelet_order)
    print(center_labels)
    permutation = [faces[center_labels[labels[facelet]]] for facelet in facelet_order]
    print(permutation)
    permutation = ''.join(permutation)
    print(permutation)
    for f in range(6):
        for i in range(3):
            print(permutation[9*f + 3*i : 9*f + 3*i + 3])
        print('')
    return permutation


for root, dirs, files in os.walk(DATASET_DIR):
    for file in files:
        fullpath = os.path.join(DATASET_DIR, file)
        cluster = np.load(fullpath)
        cluster_labels = np.full((len(cluster), 1), len(cluster_colors), dtype=int)
        mean_color = cluster.mean(axis=0)
        cluster_colors.append(mean_color)
        # axis.scatter(cluster[:, 0], cluster[:, 1], cluster[:, 2], color=mean_color/255)

        train_points = np.append(train_points, cluster, axis=0)
        train_labels = np.append(train_labels, cluster_labels)

# pl.show()

grid_step = 32
grid = np.asarray([
    (i, j, k)
    for i in range(0, 256, grid_step)
    for j in range(0, 256, grid_step)
    for k in range(0, 256, grid_step)
])
test = np.load(TEST_FILE)
# test = grid

centroids = np.array([test[i] for i in range(4, 54, 9)])
print('centroides:', centroids)

# model = QuadraticDiscriminantAnalysis()
# model = KMeans(n_clusters=6)
model = GaussianMixture(n_components=6)

# model.fit(train_points, train_labels)
model.fit(test)

predictions = model.predict(test)
print('predicciones:\n', predictions.reshape(-1, 3, 3))

cluster_colors = np.asarray([
    [233, 232, 233], # blanco
    [154, 201, 147], # verde
    [220, 68, 93], # rojo
    [236, 217, 139], # amarillo
    [138, 112, 96], # cafe
    [89, 63, 213], # azul
])

for label, color in enumerate(cluster_colors):
    # if label == 0:
    if True:
        indexes = np.array([i for i, val in enumerate(predictions) if label == val])
        region = test[indexes.astype(int)]
        axis.scatter(region[:, 0], region[:, 1], region[:, 2], color=color/255)
        # axis.scatter(color[0], color[1], color[2], color=color/255, s=100)

cluster_colors = np.asarray(cluster_colors)

pl.show()

labels2perm(predictions)
