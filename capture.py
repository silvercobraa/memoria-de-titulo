import math
import sys
import numpy as np
import cv2
import pylab as pl
from sklearn.cluster import KMeans
from sklearn.decomposition import PCA


cap = cv2.VideoCapture(0)

ret, bgr_frame = cap.read()
width = bgr_frame.shape[1]
height = bgr_frame.shape[0]
square_size = height // 8
gap_size = square_size
border_color = (255, 0, 255)
thickness = 4

square_gap = square_size + gap_size
# top left
base_square = (
    width // 2 - square_size - gap_size // 2 - square_size // 2,
    height // 2 - square_size - gap_size // 2 - square_size // 2,
)

squares = [
    (base_square[0], base_square[1], square_size, square_size), # top left
    (base_square[0] + square_gap, base_square[1], square_size, square_size), # top center
    (base_square[0] + 2*square_gap, base_square[1], square_size, square_size), # top right
    (base_square[0], base_square[1] + square_gap, square_size, square_size), # middle left
    (base_square[0] + square_gap, base_square[1] + square_gap, square_size, square_size), # middle center
    (base_square[0] + 2*square_gap, base_square[1] + square_gap, square_size, square_size), # middle right
    (base_square[0], base_square[1] + 2*square_gap, square_size, square_size), # bottom left
    (base_square[0] + square_gap, base_square[1] + 2*square_gap, square_size, square_size), # bottom center
    (base_square[0] + 2*square_gap, base_square[1] + 2*square_gap, square_size, square_size), # bottom right
]

captured_colors = []

while True:
    ret, bgr_frame = cap.read()
    rgb_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
    rgb_frame = cv2.GaussianBlur(rgb_frame, (9,9), cv2.BORDER_DEFAULT)
    # rgb_frame = bgr_frame

    for square in squares:
        # segundo argumento: rec = tupla de 4 elementos = (offset_ancho, offset_alto, ancho, alto)
        cv2.rectangle(bgr_frame, square, border_color, thickness)

    cv2.imshow('bgr_frame', bgr_frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord(' '):
        for square in squares:
            quadrant = rgb_frame[square[1] : square[1]+square[3], square[0] : square[0]+square[2]]
            mean_color = quadrant.mean(axis=0).mean(axis=0)
            print('#%02x%02x%02x' % (int(mean_color[2]), int(mean_color[1]), int(mean_color[0])))

            captured_colors.append(mean_color)

print(captured_colors)

captured_colors = np.asarray(captured_colors)
np.save('test_set', captured_colors)
normalized_colors = captured_colors / 255
sys.exit()

for color in normalized_colors:
    pl.plot(color[0], color[1], 'o', color=color)
pl.show()
for color in normalized_colors:
    pl.plot(color[0], color[2], 'o', color=color)
pl.show()
for color in normalized_colors:
    pl.plot(color[1], color[2], 'o', color=color)
pl.show()

from mpl_toolkits.mplot3d import Axes3D
fig = pl.figure()
ax = fig.add_subplot(111, projection='3d')

for color in normalized_colors:
    ax.scatter(color[0], color[1], color[2], color=color)

ax.set_xlabel('Eje rojo')
ax.set_ylabel('Eje verde')
ax.set_zlabel('Eje azul')

pl.show()

cap.release()
cv2.destroyAllWindows()

pca = PCA(n_components=2)
reduced = pca.fit_transform(captured_colors)

for i, color in enumerate(reduced):
    pl.plot(color[0], color[1], 'o', color=normalized_colors[i])
pl.show()

centers = np.array([captured_colors[i] for i in range(4, 54, 9)])
print('centros:', centers)
km = KMeans(n_clusters=6, init=centers)
km.fit(captured_colors)
colors = km.predict(captured_colors)
print(colors)

normal = np.array([1.0, 1.0, 1.0])
normal /= np.linalg.norm(normal)

print(captured_colors @ normal)
dist = [color * normal for color in captured_colors @ normal]
print(dist)
proyected_colors = captured_colors - np.array(dist)
fig = pl.figure()
ax = fig.add_subplot(111, projection='3d')

for i, color in enumerate(proyected_colors):
    ax.scatter(color[2], color[1], color[0], color=normalized_colors[i])
pl.show()

for i, color in enumerate(proyected_colors):
    pl.plot(color[0], color[1], 'o', color=normalized_colors[i])
pl.show()

print(captured_colors.mean(axis=0))
print(captured_colors.std(axis=0))
