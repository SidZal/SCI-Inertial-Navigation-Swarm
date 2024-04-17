import csv
import cv2
import cv2.aruco as aruco
import numpy as np
import math
import time
import serial

#returns unit vector
def unit_vector(vector):
    return vector / np.linalg.norm(vector)

#input as floats, angle as radians
def getAngle(x1, y1, x2, y2, x3, y3, x4, y4): 
    mx1 = (x1 + x2)/ 2.0
    my1 = (y1 + y2) / 2.0
    mx2 = (x3 + x4) / 2.0
    my2 = (y3 + y4) / 2.0
    mX = (x2 + x3) / 2.0
    mY = (y2 + y3) / 2.0
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
x_dist = 40
y_dist = 50
filename = "/Users/lindsayqin/Desktop/scidata/test.csv"

dictionary = aruco.getPredefinedDictionary(DICT_USED)
parameters =  aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WINDOW_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, WINDOW_HEIGHT)
start_time = time.time()

ser = serial.Serial(port='/dev/tty.usbmodem11201', baudrate=9600)
# ser = arduinoserial.SerialPort('/dev/tty.usbmodem11301', 9600)

with open(filename, "a") as csvfile:
    write = csv.writer(csvfile)

    while True:
        success, frame = cap.read()
    
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
                xFactor = x_dist/(m3x - m0x)
                yFactor = y_dist/(m3y - m0y)

            for i in range(len(markerIDs)):
                mc = markerCorners[i]
            
                # joystick = "doesn't work"
                joystick = ((ser.readline()).decode('utf-8')).replace("\n", "")
                # joystick = (ser.read_until('\n')).decode('utf-8').replace("\n", "")
            
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
            
                angle = getAngle(float(x1), float(y1), float(x2), float(y2), float(x3), float(y3), float(x4), float(y4))
                
                write.writerow([str(time.time()-start_time), i, joystick, xCoor, yCoor, angle])
    
        QueryImg = aruco.drawDetectedMarkers(frame, markerCorners, markerIDs)
        
        cv2.imshow('cam', QueryImg)

        if cv2.waitKey(1) & 0xFF == ord('q'): #press 'q' to quit
            break

cap.release()
cv2.destroyAllWindows()

#todo: camera calibration step
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