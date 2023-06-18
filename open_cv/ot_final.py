import numpy as np
import cv2 as cv
cap=cv.VideoCapture(0)
while True:
    ret,frame = cap.read()
    lower =np.array([20,70,80])
    upper=np.array([38,255,255])
    lower1 =np.array([50,0,146])
    upper1=np.array([255,134,191])
    gauss = cv.GaussianBlur(frame, (9,9), 10)
    median = cv.medianBlur(gauss, 5, 0)
    framehsv = cv.cvtColor(median, cv.COLOR_BGR2HSV)
    framelab = cv.cvtColor(median, cv.COLOR_BGR2LAB)
    maskinglab = cv.inRange(framelab, lower1, upper1)
    maskinghsv = cv.inRange(framehsv, lower, upper)
    mask = cv.bitwise_or(maskinglab, maskinglab, mask=maskinghsv)
    cv.imshow('mask',mask)
    mask_erode = cv.erode(mask, None, iterations=1)
    mask_dilate = cv.dilate(mask_erode, None, iterations=1)
    gauss2 = cv.GaussianBlur(mask_dilate, (9,9), 20)
    circles = cv.HoughCircles(gauss2, cv.HOUGH_GRADIENT, 1.06, 60,param1=65,param2=32, minRadius=15,maxRadius=100)    
    
    if circles is not None:
     circles = np.uint16(np.around(circles))
     for circle in circles[0, :]:
            x, y, r = circle[0], circle[1], circle[2]
            cv.circle(frame, (int(x), int(y)), int(r), (255, 25, 255), 3)
            cv.circle(frame, (int(x), int(y)), 3, (0, 0, 255), -1)
    cv.imshow("Frame", frame)
    #cv.imshow('mask',res)
    if cv.waitKey(1) == ord('q'):
        break
cap.release()
cv.destroyAllWindows() 
