import numpy as np
from sklearn.cluster import KMeans
from sklearn.decomposition import PCA
from mpl_toolkits.mplot3d import Axes3D
import pylab as pl
import matplotlib as mpl
import cv2
from GMM import GMM

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
        # self._model = GaussianMixture(n_components=6, means_init=centroids)
        self._model.fit(self._X_pca)

    def fit2(self, raw_colors):
        from sklearn.mixture import GaussianMixture
        self._X = [raw_colors[i] for i in Classifier.FACELET_ORDER]
        for label, color in enumerate(self._X):
            color[0], color[2] = color[2], color[0]
            self._axis.scatter(*color, color=color/384)
        pl.show()

        centroids = np.array([self._X[i] for i in range(4, 54, 9)])

        print(centroids)
        self._model = GaussianMixture(n_components=6, means_init=centroids)
        self._model.fit(self._X)
        self._X_pca = self._X


    def fit3(self, raw_colors):
        self._X = [raw_colors[i] for i in Classifier.FACELET_ORDER]
        np.save('X', self._X)
        X = np.load('X.npy').astype('uint8')
        for i in range(len(X)):
            X[i][0], X[i][2] = X[i][2], X[i][0]

        X_hsv = X[np.newaxis, ...]
        hsv = cv2.cvtColor(X_hsv, cv2.COLOR_RGB2HSV)[0]
        centroid_id = list(range(4, 54, 9))
        feats = hsv[:,::2]
        alphas = np.empty((6, 1))
        means = np.empty((6, feats.shape[1]))
        covs = np.empty((6, feats.shape[1], feats.shape[1]))
        for i in range(6):
            mean = feats[9*i + 4]
            cov = np.array([[0.5*10, 0.0], [0.0, 2.5*10]])
            alphas[i] = 1 / 6.0
            means[i] = mean
            covs[i] = cov
            print('alpha:', alphas[i])
            print('mean:', mean)
            print('cov:', cov)
        self._gmm = GMM(6, feats, mu=means, sigma=covs, alpha=alphas)
        self._gmm.execute()
        print(self._gmm.alpha)
        print(self._gmm.mu)
        print(self._gmm.sigma)

        self._ans = []
        for xi in feats:
            probas = np.array([self._gmm.Normal(xi, self._gmm.mu[k], self._gmm.sigma[k], len(xi)) for k in range(6)])
            self._ans.append(probas.argmax())
        print(np.array(self._ans).reshape(6, 3, 3))

        def eigsorted(cov):
            vals, vecs = np.linalg.eigh(cov)
            order = vals.argsort()[::-1]
            return vals[order], vecs[:,order]

        nstd = 2
        ax = pl.subplot(111)
        for i, c in enumerate(self._ans):
            if i % 9 == 4:
                pl.scatter(*feats[i], color=X[centroid_id[c]]/255.0, marker='x')
            else:
                pl.scatter(*feats[i], color=X[centroid_id[c]]/255.0, marker='o')

        for k in range(6):
            cov = self._gmm.sigma[k]
            vals, vecs = eigsorted(cov)
            theta = np.degrees(np.arctan2(*vecs[:,0][::-1]))
            w, h = 2 * nstd * np.sqrt(vals)
            ell = mpl.patches.Ellipse(xy=self._gmm.mu[k],
                          width=w, height=h,
                          angle=theta, color=X[centroid_id[k]]/255.0, alpha=0.5)
            ell.set_facecolor(X[centroid_id[k]]/255.0)
            ax.add_artist(ell)
        for k in range(6):
            cov = covs[k]
            vals, vecs = eigsorted(cov)
            theta = np.degrees(np.arctan2(*vecs[:,0][::-1]))
            w, h = 2 * nstd * np.sqrt(vals)
            ell = mpl.patches.Ellipse(xy=self._gmm.mu[k],
                          width=w, height=h,
                          angle=theta, color=X[centroid_id[k]]/255.0, alpha=0.5)
            # ell.set_facecolor(X[centroid_id[k]]/255.0)
            ax.add_artist(ell)
        pl.show()



    def get_state(self):
        # if self._X is None:
        #     raise Error('Se debe hacer fit primero.')
        # pred2 = self._model.predict(self._X_pca)
        # state = ''.join([self._face_order[i] for i in pred2])
        # print(state)
        # pred2 = pred2.reshape(-1, 3, 3)
        # print(pred2)
        # return state
        FACES = 'URFDLB'
        state = ''.join([FACES[i] for i in np.array(self._ans).reshape(-1)])
        print(state)
        return state



def main():
    reps = np.load('capturas/reps.npy')
    cls = Classifier()
    # cls.fit2(reps)
    cls.fit3(reps)
    state = cls.get_state()
    print(state)


if __name__ == '__main__':
    main()
