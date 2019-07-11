import numpy as np
from sklearn.cluster import KMeans
from sklearn.mixture import GaussianMixture
from sklearn.decomposition import PCA
from mpl_toolkits.mplot3d import Axes3D
import pylab as pl

class Classifier(object):
    """docstring for Classifier."""

    FACELET_ORDER = [
        42, 39, 36,   43, 40, 37,   44, 41, 38, # U
        9, 10, 11,   12, 13, 14,   15, 16, 17, # R
        8,  7,  6,    5,  4,  3,    2,  1,  0, # F
        45, 46, 47,   48, 49, 50,   51, 52, 53, # D
        27, 28, 29,   30, 31, 32,   33, 34, 35, # L
        26, 25, 24,   23, 22, 21,   20, 19, 18, # B
    ]

    def __init__(self, face_order='URFDLB'):
        self._fig = pl.figure()
        self._axis = self._fig.add_subplot(111, projection='3d')
        self._face_order = face_order


    def fit(self, raw_colors):
        self._X = [raw_colors[i] for i in Classifier.FACELET_ORDER]

        np.save('X', self._X)
        for label, color in enumerate(self._X):
            color[0], color[2] = color[2], color[0]
            self._axis.scatter(*color, color=color/384)
        pl.show()

        self._X_pca = PCA(n_components=2).fit_transform(self._X)
        print(self._X_pca)
        np.save('pca', self._X_pca)
        for i, point in enumerate(self._X_pca):
            pl.scatter(*point, color=self._X[i]/384)
        pl.show()

        centroids = np.array([self._X_pca[i] for i in range(4, 54, 9)])
        for i, point in enumerate(centroids):
            pl.scatter(*point, color=self._X[9*i+4]/384)
        pl.show()

        print(centroids)
        self._model = KMeans(n_clusters=6, init=centroids)
        self._model.fit(self._X_pca)


    def get_state(self):
        if self._X is None:
            raise Error('Se debe hacer fit primero.')
        pred2 = self._model.predict(self._X_pca)
        state = ''.join([self._face_order[i] for i in pred2])
        print(state)
        pred2 = pred2.reshape(-1, 3, 3)
        print(pred2)
        return state



def main():
    reps = np.load('capturas/reps.npy')
    cls = Classifier()
    cls.fit(reps)
    ans = cls.get_state()
    print(ans)


if __name__ == '__main__':
    main()
