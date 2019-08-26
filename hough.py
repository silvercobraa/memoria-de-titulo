import numpy as np
import numpy.linalg as la
import cv2
import time
import cv


def hough(img):
    # img = cv2.imread('pepe-arch.png',0)
    # img = cv2.GaussianBlur(img, (5, 5), 5)
    img = cv2.medianBlur(img, 5)
    # print(img)
    canny = cv2.Canny(img,100,200)
    gray_img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)


    # radios para los brazos
    # circles = cv2.HoughCircles(gray_img, method=cv.CV_HOUGH_GRADIENT, dp=1, minDist=30, param1=80, param2=30, minRadius=10, maxRadius=30)
    circles = cv2.HoughCircles(gray_img, method=cv.CV_HOUGH_GRADIENT, dp=1, minDist=30,
        param1=80, param2=30, minRadius=10, maxRadius=35)
        # param1=80, param2=30, minRadius=5, maxRadius=40)
    # circles = cv2.HoughCircles(gray_img, method=cv.CV_HOUGH_GRADIENT, dp=1, minDist=20,
    #     param1=80, param2=30, minRadius=10, maxRadius=40)
    # radios para la cabeza
    # circles = cv2.HoughCircles(img,cv.CV_HOUGH_GRADIENT,1,30,
    #                             param1=50,param2=30,minRadius=3,maxRadius=10)

    if circles is None:
        cv2.imshow('detected circles', img)
        if cv2.waitKey(100) >= 0:
            pass
        return
    # print('circles:', circles[0])
    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
        # draw the outer circle
        cv2.circle(img, (i[0],i[1]), i[2], (0,255,0), 2)
        # draw the center of the circle
        cv2.circle(img, (i[0],i[1]), 2, (0,0,255), 3)

    cv2.imshow('detected circles', img)
    cv2.imwrite('many_circles.png', img)
    if cv2.waitKey(100) >= 0:
        pass
    return circles


def rectangles(img):
    img = cv2.medianBlur(img,5)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(gray,127,255,1)
    contours,h = cv2.findContours(thresh,1,2)
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        print(len(approx))
        if len(approx)==4:
            print("square", approx)
            norma = la.norm(approx[0] - approx[3])
            if 1 < norma < 500:
                cv2.drawContours(img,[cnt],0,(0,0,255),-1)
        # elif len(approx)==5:
        #     print("pentagon")
        #     cv2.drawContours(img,[cnt],0,255,-1)
        # elif len(approx)==3:
        #     print("triangle")
        #     cv2.drawContours(img,[cnt],0,(0,255,0),-1)
        # elif len(approx) == 9:
        #     print("half-circle")
        #     cv2.drawContours(img,[cnt],0,(255,255,0),-1)
        elif len(approx) > 15:
            print("circle")
            cv2.drawContours(img,[cnt],0,(0,255,255),-1)

    # cv2.imshow('img', img)


def main():
    img = np.load('capturas/' + 'F.npy')
    # cv2.imwrite('camara_externa.png', img)
    gray_frame = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imshow('asdasd', gray_frame)
    circles = hough(img)


if __name__ == '__main__':
    main()
