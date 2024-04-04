import cv2
import cv2.aruco as aruco
import numpy as np
import math
import serial
import time

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
WINDOW_WIDTH = 640 #pixels
WINDOW_HEIGHT = 480
REAL_WIDTH = 640 #measure distance of frame in real life
REAL_HEIGHT = 480

# MARKER_SIZE = 2 #change if needed

dictionary = aruco.getPredefinedDictionary(DICT_USED)
parameters =  aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WINDOW_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, WINDOW_HEIGHT)

ser = serial.Serial(port='/dev/tty.usbmodem11301', baudrate=9600)
file = open("/Users/lindsayqin/Downloads/test.txt", "a")

while True:
    success, frame = cap.read()
    
    if success:
        markerCorners, markerIDs, rejectedCandidates = detector.detectMarkers(frame)
    
    mX = mY = mx1 = my1 = centerX = centerY = float(-1.0)
    
    if markerIDs is not None:
        for i in range(len(markerIDs)):
            mc = markerCorners[i]
            
            coor = ser.readline()
            file.write(coor.decode('utf-8') + "\n")
            
            coorString = "ID " + str(markerIDs[i]) + ": " + str(mc[0][0]) + '\n'
            file.write(coorString)
            # ser.write(bytes(coorString, 'utf-8'))
             
            x1 = mc[0][0][0]
            y1 = mc[0][0][1]
            x2 = mc[0][1][0]
            y2 = mc[0][1][1]
            x3 = mc[0][2][0]
            y3 = mc[0][2][1]
            x4 = mc[0][3][0]
            y4 = mc[0][3][1]
            
            a = getAngle(float(x1), float(y1), float(x2), float(y2), float(x3), float(y3), float(x4), float(y4))
            angleString = "ID " + str(markerIDs[i]) + ": " + str(a) + "\n"
            file.write(angleString)
            # ser.write(bytes(angleString, 'utf-8'))
    
    QueryImg = aruco.drawDetectedMarkers(frame, markerCorners, markerIDs)
        
    cv2.imshow('cam', QueryImg)

    if cv2.waitKey(1) & 0xFF == ord('q'): #press 'q' to quit
        break
file.close()
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