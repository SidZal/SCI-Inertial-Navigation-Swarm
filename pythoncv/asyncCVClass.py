import cv2
import cv2.aruco as aruco
import asyncio

class asyncCV:

    def __init__(self, cap, dict_used):
        self.cap = cap

        dictionary = aruco.getPredefinedDictionary(dict_used)
        parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(dictionary, parameters)

    def detect(self):
        success, frame = self.cap.read()

        if success:
            markerCorners, markerIDs, _ = self.detector.detectMarkers(frame)
        else:
            return #not sure if this is the right thing