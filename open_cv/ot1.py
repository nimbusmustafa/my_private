import numpy as np
import cv2 as cv
import imutils
cap=cv.VideoCapture(0)
while True:
    ret,frame = cap.read()
    gauss=cv.GaussianBlur(frame, (11,11),0)
    hsv = cv.cvtColor(gauss, cv.COLOR_BGR2HSV)
    lower =np.array([25,50,70])
    upper=np.array([35,255,255])
    mask=cv.inRange(hsv,lower,upper)
    mask = cv.erode(mask, None, iterations=3)
    mask = cv.dilate(mask, None, iterations=3)
    res = cv.bitwise_and(frame, frame, mask=mask)
    cnts=cv.findContours(mask.copy(),cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
    cnts=imutils.grab_contours(cnts)
    center=None
    if len(cnts)>0:
        c=max(cnts,key=cv.contourArea)
        ((x,y),radius)=cv.minEnclosingCircle(c)
        M=cv.moments(c)
        print(x,y)
        center=(int(M["m10"] / M["m00"]) , int(M["m01"] / M["m00"]))
        if radius >1:
            cv.circle(frame,(int(x),int(y)),int(radius),(255,25,255),2)
            cv.circle(frame, center, 5, (0, 0, 255), -1)
    cv.imshow("Frame", frame)
    #cv.imshow("Result view", res)
    if cv.waitKey(1)== ord('q'):
        break
cap.release()
cv.destroyAllWindows() 