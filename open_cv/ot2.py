import numpy as np
import cv2 as cv
cap=cv.VideoCapture(0)
while True:
    ret,frame = cap.read()
    gauss=cv.GaussianBlur(frame, (7,7),0)
    hsv = cv.cvtColor(gauss, cv.COLOR_BGR2HSV)
    lower =np.array([25,50,70])
    upper=np.array([35,255,255])
    mask=cv.inRange(hsv,lower,upper)
    mask = cv.erode(mask, None, iterations=10)
    mask = cv.dilate(mask, None, iterations=10)
    median = cv.medianBlur(mask,9,10)
    gauss1 = cv.GaussianBlur(median, (9, 9), 0)
    res = cv.bitwise_and(frame, frame, mask=mask)
    cnts, _ =cv.findContours(mask.copy(),cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
    circles = cv.HoughCircles(gauss1,cv.HOUGH_GRADIENT,1,10,param1=50,param2=30,minRadius=10,maxRadius=100)
    if circles is not None:
     circles = np.uint16(np.around(circles))
     for circle in circles[0, :]:
            x, y, r = circle[0], circle[1], circle[2]
            cv.circle(frame, (int(x), int(y)), int(r), (0, 0, 255), 2)
            cv.circle(frame, (int(x), int(y)), 2, (255, 0, 0), -1)
    cv.imshow("Frame", frame)
    cv.imshow('mask',res)
    if cv.waitKey(1)== ord('q'):
        break
cap.release()
cv.destroyAllWindows() 
