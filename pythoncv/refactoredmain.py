import csv
import cv2
import cv2.aruco as aruco
# import numpy as np
# import math
import time
import randompath
import robotClassSerial
import calc

#constants
DICT_USED = aruco.DICT_6X6_50
WINDOW_WIDTH = 700 #pixelsπ
WINDOW_HEIGHT = 550
X_DIST = 40
Y_DIST = 50
FILENAME = "./test2.csv"

dictionary = aruco.getPredefinedDictionary(DICT_USED)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WINDOW_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, WINDOW_HEIGHT)
start_time = time.time()
setRefAngle = False
refAngle = 0

bot = robotClassSerial.INRbot("a6:5d:28:70:b8:e2", ["b440","ba89","c81a","0ef0"])
pather = randompath.botPath(bot)

csvfile = open(FILENAME, "a")
write = csv.writer(csvfile)

count = 5
num = 0

foundCorners = False

while True:

    success, frame = cap.read()

    if success:
       markerCorners, markerIDs, rejectedCandidates = detector.detectMarkers(frame)
    else: continue


    #============= SLOWDOWNS ===========
    # these values should never be used (overridden)
    mX = mY = mx1 = my1 = centerX = centerY = float(-1.0)
    xFactor = yFactor = 1
  
    if markerIDs is not None:
        index0 = index3 = -1

        #get reference markers
        for i in range(len(markerIDs)):
            if (index0 != -1 and index3 != -1): break
            if (markerIDs[i] == 0):
                index0 = i
                continue
            elif (markerIDs[i] == 3):
                index3 = i
                continue
  
        if foundCorners != True: #only execute once
            m0x = markerCorners[index0][0][0][0]
            m0y = markerCorners[index0][0][0][1]
            m3x = markerCorners[index3][0][0][0]
            m3y = markerCorners[index3][0][0][1]
  
            if ((m3x - m0x) != 0 and (m3y - m0y) != 0):
                xFactor = X_DIST/(m3x - m0x)
                yFactor = Y_DIST/(m3y - m0y)
            
            foundCorners = True
        
        for i in range(len(markerIDs)):
            if setRefAngle==False and markerIDs[i] == 3:
                #    refAngle = calc.getAngle(float(x1), float(y1), float(x2), float(y2), float(x3), float(y3), float(x4), float(y4))
                refAngle = calc.getAngle(markerCorners[i])
                setRefAngle = True

            if (markerIDs[i] != 0) and (markerIDs[i] != 3): # exclude reference markers
                mc = markerCorners[i]

                # Calculate x,y coordinate and angle from Camera Data
                xCoor = (m3x - mc[0][0][0]) * xFactor
                yCoor = (m3y - mc[0][0][1])* yFactor

                x1 = mc[0][0][0]
                y1 = mc[0][0][1]
                x2 = mc[0][1][0]
                y2 = mc[0][1][1]
                x3 = mc[0][2][0]
                y3 = mc[0][2][1]
                x4 = mc[0][3][0]
                y4 = mc[0][3][1]

                angle = calc.getAngle(markerCorners[i]) - refAngle

                # Poll for IMU data and write to CSV if received. 1/30 -> max 30 Hz data
                # due to bluepy slowing, actual data rate is ~2 Hz :(
                success, data = bot.getData(1/30)
                if success:
                    print(data)
                    # ax, ay, az, gx, gy, trash, gz, yaw, omegaL, omegaR = data
                    # write.writerow([float(time.time() - start_time), int(markerIDs[i]),
                    #                 ax, ay, az, gx, gy, gz,
                    #                 yaw, omegaL[0], omegaR[0],
                    #                 xCoor, yCoor, angle])

    QueryImg = aruco.drawDetectedMarkers(frame, markerCorners, markerIDs)

    cv2.imshow('cam', QueryImg)

    if cv2.waitKey(1) & 0xFF == ord('q'): #press 'q' to quit
       break

    num += 1
    if num == count:
       pather.takePath()
       num = 0
    else:
       time.sleep(0.1)

cv2.destroyAllWindows()
cap.release()
