import numpy as np
import pylab as pl
import cv2
import sys
from mpl_toolkits.mplot3d import Axes3D
from sklearn.cluster import KMeans, MeanShift
# from sklearn.mixture import GaussianMixture
from sklearn.decomposition import PCA
import matplotlib as mpl
from GMM import GMM
import solver.solver as Solver

X = np.load('X.npy').astype('uint8')
print(X)
for i in range(len(X)):
    X[i][0], X[i][2] = X[i][2], X[i][0]

figure = pl.figure()
axis = figure.add_subplot(111, projection='3d')

for color in X:
    print(color)
    axis.scatter(*color, color=color/255.0)
    pass
pl.show()

X_hsv = X[np.newaxis, ...]
hsv = cv2.cvtColor(X_hsv, cv2.COLOR_RGB2HSV)[0]

for i, color in enumerate(hsv):
    # axis.scatter(*color, color=X[i]/255.0)
    if (i + 5) % 9 == 0:
        pl.scatter(color[0], color[2]/255.0, color=X[i]/255.0, marker='x')
        # pl.scatter(color[0], 0, color=X[i]/255.0, marker='x')
    else:
        pl.scatter(color[0], color[2]/255.0, color=X[i]/255.0, marker='o')
        # pl.scatter(color[0], 0, color=X[i]/255.0, marker='.')
# pl.show()
print(hsv)

for color in hsv:
    axis.scatter(color[0], color[1], 0, color=color/255.0)
    pass
pl.show()

centroid = list(range(4, 54, 9))
feats = hsv[:,::2]
# feats = hsv
# feats = PCA(n_components=2).fit_transform(hsv)
# feats = PCA(n_components=2).fit_transform(X)
# feats = hsv[:,:2]
# feats[:, 0] *= 10
print('Feats:', feats)
model = KMeans(n_clusters=6, init=feats[4::9,:])
# model = MeanShift(n_clusters=6, init=feats[4::9,:])
# model = GaussianMixture(n_components=6, means_init=feats[4::9,:])
model.fit(feats)
Y = model.predict(feats)
for i, y in enumerate(Y):
    pl.scatter(*feats[i], color=X[centroid[y]]/255.0)
pl.show()
print(Y.reshape(-1, 3, 3))

alphas = np.empty((6, 1)) # priors?
means = np.empty((6, feats.shape[1]))
covs = np.empty((6, feats.shape[1], feats.shape[1]))
for i in range(6):
    f = feats[Y == i]
    print(len(f))
    # mean = f.mean(axis=0)
    mean = feats[9*i + 4]
    # cov = np.cov(f, rowvar=False)
    cov = np.array([[0.3*50, 0.0], [0.0, 5.0*50]])
    # alphas[i] = float(len(f)) / len(feats)
    alphas[i] = 1 / 6.0
    means[i] = mean
    covs[i] = cov
    print('alpha:', alphas[i])
    print('mean:', mean)
    print('cov:', cov)

print('MEANS:', means)
print('COVS:', covs)

gmm = GMM(6, feats, mu=means, sigma=covs, alpha=alphas)
gmm.execute()
gmm_likelihood = gmm.likelihood
gmm_alpha = gmm.alpha
gmm_mu = gmm.mu
gmm_sigma = gmm.sigma
print("The likelihood is:", gmm.likelihood)
print("The amplitudes are:", gmm.alpha)
print("The means are:", gmm.mu)
print("The covariances are:", gmm.sigma)
# sys.exit()

ans = []
for xi in feats:
    probas = np.array([gmm.Normal(xi, gmm_mu[k], gmm_sigma[k], len(xi)) for k in range(6)])
    ans.append(probas.argmax())
    # print(probas.argmax())

print(np.array(ans).reshape(-1, 3, 3))

for i, c in enumerate(ans):
    pl.scatter(*feats[i], color=X[centroid[c]]/255.0, marker='o')
# pl.show()




def eigsorted(cov):
    vals, vecs = np.linalg.eigh(cov)
    order = vals.argsort()[::-1]
    return vals[order], vecs[:,order]

nstd = 2
ax = pl.subplot(111)

for k in range(6):
    cov = gmm_sigma[k]
    # cov = np.array([[0.3*50, 0.0], [0.0, 5.0*50]])
    vals, vecs = eigsorted(cov)
    theta = np.degrees(np.arctan2(*vecs[:,0][::-1]))
    w, h = 2 * nstd * np.sqrt(vals)
    ell = mpl.patches.Ellipse(xy=gmm_mu[k],
                  width=w, height=h,
                  angle=theta, color=X[centroid[k]]/255.0, alpha=0.5)
    ell.set_facecolor(X[centroid[k]]/255.0)
    ax.add_artist(ell)
pl.show()

FACES = 'URFDLB'
state = ''.join([FACES[i] for i in np.array(ans).reshape(-1)])
print(state)
moves = Solver.solve(state, 20, 2)
print(moves)
