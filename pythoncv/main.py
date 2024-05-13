import csv
import cv2
import cv2.aruco as aruco
import numpy as np
import math
import time
import serial
import re
import serial.tools.list_ports

#returns unit vector
def unit_vector(vector):
    return vector / np.linalg.norm(vector)

#input as floats, angle as radians
def getAngle(x1, y1, x2, y2, x3, y3, x4, y4): 
    mx1 = (x1 + x2)/ 2.0
    my1 = (y1 + y2) / 2.0
    mx2 = (x3 + x4) / 2.0
    my2 = (y3 + y4) / 2.0
    mX = (x3 + x4) / 2.0
    mY = (y3 + y4) / 2.0
    centerX = (mx1 + mx2)/ 2.0
    centerY = (my1 + my2) / 2.0
    
    v1 = unit_vector([centerX-mX, centerY-mY])
    v2 = unit_vector([-1, 0])
    angle = np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))
    
    if mY - centerY > 0: 
        return 2 * math.pi - angle

    return angle
    
#constants
DICT_USED = aruco.DICT_6X6_50
WINDOW_WIDTH = 700 #pixelsπ
WINDOW_HEIGHT = 550
X_DIST = 40
Y_DIST = 50
FILENAME = "./test1.csv"

# print(list(serial.tools.list_ports.comports()))

dictionary = aruco.getPredefinedDictionary(DICT_USED)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WINDOW_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, WINDOW_HEIGHT)
start_time = time.time()
setRefAngle = False
refAngle = 0

with open(FILENAME, "a") as csvfile:
    write = csv.writer(csvfile)

    while True:
        success, frame = cap.read()
        ser = serial.Serial(port='/dev/ttyACM0', baudrate=9600)

        # ser.write(bytes("circle 2", 'utf-8'))


        if success:
            markerCorners, markerIDs, rejectedCandidates = detector.detectMarkers(frame)

        #these values should never be used (overridden)
        mX = mY = mx1 = my1 = centerX = centerY = float(-1.0)
        xFactor = yFactor = 1

        if markerIDs is not None:
            index0 = index3 = -1

            #get reference markers
            for i in range(len(markerIDs)):
                if (markerIDs[i] == 0):
                    index0 = i
                    continue
                if (markerIDs[i] == 3):
                    index3 = i
                    continue

            m0x = markerCorners[index0][0][0][0]
            m0y = markerCorners[index0][0][0][1]
            m3x = markerCorners[index3][0][0][0]
            m3y = markerCorners[index3][0][0][1]

            if ((m3x - m0x) != 0 and (m3y - m0y) != 0):
                xFactor = X_DIST/(m3x - m0x)
                yFactor = Y_DIST/(m3y - m0y)

            for i in range(len(markerIDs)):
                mc = markerCorners[i]

                # joystick = "doesn't work"
                line = ((ser.readline()).decode('utf-8'))
                line = re.sub(r"\s+", '', line)
                list = line.split(',', 12)
                if (len(list) == 13):
                    ax, ay, az, gx, gy, gz, qw, qx, qy, qz, y, p, r = line.split(',', 12)

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

                if (setRefAngle == False and markerIDs[i] == 3):
                    refAngle = getAngle(float(x1), float(y1), float(x2), float(y2), float(x3), float(y3), float(x4), float(y4))
                    setRefAngle = True

                angle = getAngle(float(x1), float(y1), float(x2), float(y2), float(x3), float(y3), float(x4), float(y4)) - refAngle

                if ((markerIDs[i] != 0) and (markerIDs[i] != 3)):
                    write.writerow([float(time.time()-start_time), int(markerIDs[i]), ax, ay, az, gx, gy, gz, qw, qx, qy, qz, y, p, r, xCoor, yCoor, angle])

        QueryImg = aruco.drawDetectedMarkers(frame, markerCorners, markerIDs)

        cv2.imshow('cam', QueryImg)

        if cv2.waitKey(1) & 0xFF == ord('q'): #press 'q' to quit
            break

        ser.close()

cap.release()
cv2.destroyAllWindows()

# rvecs = []
# tvecs = []

# QueryImg = cv2.line(QueryImg, (int(mX), int(mY)), (int(centerX), int(centerY)), (255,0,0),5)

    # markerPoints = np.array([[-MARKER_SIZE / 2, MARKER_SIZE / 2, 0],
    #                           [MARKER_SIZE / 2, MARKER_SIZE / 2, 0],
    #                           [MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
    #                           [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0]], dtype=np.float32)

    # for c in markerCorners:
    #     trash, R, t = cv2.solvePnP(markerPoints, c, mtx, distortion) #todo: camera distortion
    #     rvecs.append(R)
    #     tvecs.append(t)
