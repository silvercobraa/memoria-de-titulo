#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import pylab as pl
from mpl_toolkits.mplot3d import Axes3D


# FACES = 'FBDRLU'
FACES = 'FRBLUD'
FACE = 'F'


COLOR = (255, 0, 255)
THICKNESS = 2

representatives = []

def representative(frame, circle):
    """Retorna un color que represente la región interior del círculo en el frame. """
    x, y, r = circle
    region = frame[y, x-r:x+r]
    figure = pl.figure()
    axis = figure.add_subplot(111, projection='3d')
    for color in region:
        rgb = np.array([color[2], color[1], color[0]])
        axis.scatter(*rgb, color=rgb.astype(float)/255)
    pl.show()
    median = np.median(region, axis=0)
    return median

def gridsort(circles):
    circles = np.asarray(sorted(circles, key=lambda x: x[1]))
    print('FIRST SORT:', circles)
    circs = np.zeros(circles.shape)
    for i in range(3):
        circs[3*i : 3*i + 3,:] = np.asarray(sorted(circles[3*i : 3*i + 3,:], key=lambda x: x[0]))
        print(circs)
    return circs

for face in FACES:
    frame = np.load(face + '.npy')
    circles = np.load(face + '_circles.npy')

    print 'UNSORTED:\n', circles
    # circs = np.asarray(sorted(circles[0,:], key=lambda x: (x[0], x[1])))
    circs = gridsort(circles[0,:])
    # print 'SORTED:\n', circs
    # for circle in circles[0,:]:
    for circle in circs:
        x, y, r = circle.astype(int)
        median = representative(frame, circle)
        representatives.append(median)

        cv2.circle(frame, (x, y), r, COLOR, THICKNESS)
        cv2.circle(frame, (x, y), r, COLOR, THICKNESS)
        # # draw the center of the circle
        cv2.circle(frame, (x, y), 2, COLOR, THICKNESS)

    # cv2.imshow('test', frame)
    # cv2.waitKey()

print(representatives)
figure = pl.figure()
axis = figure.add_subplot(111, projection='3d')
for color in representatives:
    col = np.array([color[2], color[1], color[0]])
    axis.scatter(col[0], col[1], col[2], color=col.astype(float)/255)
pl.show()
# pl.show()
np.save('representatives', representatives)
