"""

The following code detects a particular colored object and
gives the location of that object.

Also returns the difference between that object and the centre of the camera frame

This particular code is inspired from pyimagesearch.com

"""

import cv2
import numpy as np

cap = cv2.VideoCapture(1)

#change the range of the desired color (given green)
# greenLower = (85-10, 86, 6) #Hue(0-180) Saturation(0-255) Value(0-255)
# greenUpper = (85+10, 255, 255)
# yeLower = (20, 100, 100)
# yeUpper = (30, 255, 255)
blueUpper = (113+10, 255, 255)
blueLower = (113-10, 100, 100)

while True:

    _ , frame = cap.read()

    frame = cv2.resize(frame, (640,480))

    #crop
    frame = frame[18:355, 1:620]

    #flip 180 degrees
    frame = cv2.flip(frame, flipCode=-1)

    blur = cv2.GaussianBlur(frame,(9,9),0)

    hsv = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)

    ma = cv2.inRange(hsv,blueLower,blueUpper)

    #Opening
    mas = cv2.erode(ma, None, iterations=2)
    mask = cv2.dilate(mas, None, iterations=2)

    # ing,
    cnts, hie = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    center = []

    #center of the camera frame
    x1 = int(np.size(frame,0)/2) #300
    y1 = int(np.size(frame,1)/2)
    print(len(cnts))
    if len(cnts) > 0:

        for i in range(len(cnts)):
            c = cnts[i]
            M = cv2.moments(c)
            cv2.drawContours(frame,c,-1,(0,255,255),2)
            center.append((int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))) # center of the object
            cv2.circle(frame, center[i], 5, (0, 0, 0), -1)

            cv2.putText(frame,'Center:'+str(center[i]),(20,50+(i*30)),cv2.FONT_HERSHEY_SIMPLEX,0.5,
                    (255, 255, 255), lineType=cv2.LINE_AA)

    cv2.imshow('cam',frame)
    # fps = cap.get(cv2.CAP_PROP_FPS)
    # print(fps)
    if cv2.waitKey(1) & 0xFF == ord('z'): #ord(char) returns ASCII
        break

cap.release()
cv2.destroyAllWindows()
