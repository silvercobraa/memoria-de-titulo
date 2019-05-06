import numpy as np
import numpy.linalg as la
import cv2
import time
import cv


def hough(img):
    # img = cv2.imread('pepe-arch.png',0)
    img = cv2.medianBlur(img,5)
    # print(img)
    canny = cv2.Canny(img,100,200)
    cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)


    circles = cv2.HoughCircles(img,cv.CV_HOUGH_GRADIENT,1,30,
                                param1=50,param2=30,minRadius=3,maxRadius=30)

    if circles is None: return
    # print('circles:', circles[0])
    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
        # draw the outer circle
        cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center of the circle
        cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

    cv2.imshow('detected circles',cimg)
    # cv2.imshow('canny', canny)
    return circles
    # time.sleep(1)

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

    cv2.imshow('img', img)

def main():
    cap = cv2.VideoCapture(0)
    ret, bgr_frame = cap.read()
    while True:
        ret, bgr_frame = cap.read()
        rgb_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('bgr_frame', bgr_frame)
        hough(rgb_frame)
        # rectangles(bgr_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

if __name__ == '__main__':
    main()