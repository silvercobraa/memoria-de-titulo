#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import pylab as pl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


class ColorExtractor():

    def __init__(self, dir, faces):
        self._dir = dir
        self._faces = faces
        self._circle_color = (0, 255, 0) # verde
        self._circle_thickness = 2


    def _representative(self, frame, circle):
        """Retorna un color que represente la región interior del círculo en el frame. """
        x, y, r = circle
        region = frame[y-r/2:y+r/2, x-r/2:x+r/2].reshape(-1, frame.shape[2])
        print('REGION:', region)

        # Descomentar para ver los colores de cada circulo
        # figure = pl.figure()
        # axis = figure.add_subplot(111, projection='3d')
        # for color in region:
        #     rgb = np.array([color[2], color[1], color[0]])
        #     axis.scatter(*rgb, color=rgb.astype(float)/255)
        # pl.show()

        median = np.median(region, axis=0)
        # median = np.mean(region, axis=0)
        print(median)
        return median


    def _gridsort(self, circles):
        circles = np.asarray(sorted(circles, key=lambda x: x[1]))
        print('FIRST SORT:', circles)
        circs = np.zeros(circles.shape)
        for i in range(3):
            circs[3*i : 3*i + 3,:] = np.asarray(sorted(circles[3*i : 3*i + 3,:], key=lambda x: x[0]))
            print(circs)
        return circs


    def get_representative_colors(self):
        representatives = []
        for face in self._faces:
            frame = np.load(self._dir + face + '.npy')
            circles = np.load(self._dir + face + '_circles.npy')
            circs = self._gridsort(circles[0,:])

            for circle in circs:
                x, y, r = circle.astype(int)
                median = self._representative(frame, circle)
                representatives.append(median)

                cv2.circle(frame, (x, y), r, self._circle_color, self._circle_thickness)
                # # draw the center of the circle
                cv2.circle(frame, (x, y), 2, self._circle_color, self._circle_thickness)

            # Descomentar para ver los círculos sobre cada cara
            cv2.imshow('test', frame)
            cv2.waitKey()
        self._reps = representatives

        return representatives


    def show(self):
        print(self._reps)
        figure = pl.figure()
        axis = figure.add_subplot(111, projection='3d')
        for color in self._reps:
            col = np.array([color[2], color[1], color[0]])
            axis.scatter(col[0], col[1], col[2], color=col.astype(float)/255)
        pl.show()

    def save(self, file):
        np.save(self._dir + file, self._reps)



def main():
    ext = ColorExtractor('capturas/', 'FRBLUD')
    colors = ext.get_representative_colors()
    ext.show()
    ext.save('reps')


if __name__ == '__main__':
    main()
