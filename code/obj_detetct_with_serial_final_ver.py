

"""

The following code detects a particular colored object and
sends the (x,y) coordinate of that object to arduio via pyserial.

This particular code is inspired from pyimagesearch.com

"""

import cv2
import numpy as np
import serial, time, struct
import timeit

# global messages
cap = cv2.VideoCapture(0)
color = 113
# color = int(color/2)

ser = serial.Serial('COM5', 9600)
time.sleep(1)
state = True

#change the range of the desired color (given green)
greenLower = (color-10, 100, 100) #Hue(0-180) Saturation(0-255) Value(0-255)
greenUpper = (color+10, 255, 255)

def pixToxy(x,y):
    """
    :param x: x axis pixel value
    :param y: y axis pixel value
    :return: exact centimeter value from pixel value
    """
    #------------------------------
    # 310 --> the x axis mid value of the camera frame
    # 444 --> the y axis length + the dist betw end of workspace to arm center
    # 1 px = 0.0635 cm
    #---------------------------------
    a,b = x-310, 444-y
    a,b = a*0.0635, (b*0.0635)
    return a,b

while True:
    _ , frame = cap.read()

    frame = cv2.resize(frame, (640,480))

    frame = frame[18:355, 1:620]

    frame = cv2.flip(frame, flipCode=-1)
    frame = frame + 30
    frame_cent = (int((620-1)/2), int((355-18)/2))

    blur = cv2.GaussianBlur(frame,(9,9),0)

    hsv = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)

    ma = cv2.inRange(hsv,greenLower,greenUpper)

    #Opening
    mas = cv2.erode(ma, None, iterations=2)
    mask = cv2.dilate(mas, None, iterations=2)

    cnts, hie = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    center = []
    real_center = []

    #center of the camera frame
    x1 = int(np.size(frame,0)/2) #300
    y1 = int(np.size(frame,1)/2)

    if len(cnts) > 0:
        for i in range(len(cnts)):

            c = cnts[i]

            M = cv2.moments(c)

            # cv2.drawContours(frame,c,-1,(0,255,255),2)


            cenx, ceny = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])) #pixel x,y
            cv2.rectangle(frame, ((cenx-30),(ceny-30)), ((cenx+30),(ceny+30)), (0,223,200), 2)
            corx, cory = pixToxy(cenx,ceny) #centimeter x,y
            corx, cory = int(corx), int(cory)
            #---------------------------------------------
            #sending message to arduino
            messages = "<"+"Data"+","+str(corx)+","+str(cory)+">"

            if state:
                # time.sleep(1)
                ser.write(messages.encode('utf-8'))
                # time.sleep()
                state = False
                ser.close()




            #--------------------------------------------
            center.append((cenx,ceny)) # center of the object

            real_center.append((int(corx),int(cory)))

            cv2.circle(frame, center[i], 5, (0, 0, 0), -1)

            cv2.putText(frame,'Real_Center:' + str(real_center[i]), (20, 50 + (i*30) ), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), lineType = cv2.LINE_AA)
            #---------------------------------------------------------------------------

            #-------------------------------------------------------------
    cv2.imshow('cam',frame)

    if cv2.waitKey(1) & 0xFF == ord('a'):
                ser.write('a')
                ser.close()

    if cv2.waitKey(1) & 0xFF == ord('z'): #ord(char) returns ASCII
        break

cap.release()
cv2.destroyAllWindows()
