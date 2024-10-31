import cv2
import cv2.aruco as aruco
import calc
import numpy as np
import asyncio

class asyncCV:

    def __init__(self, cap, dict_used, origin_id, ref_id, x_dist, y_dist):
        self.cap = cap

        dictionary = aruco.getPredefinedDictionary(dict_used)
        parameters = aruco.DetectorParameters()

        self.detector = aruco.ArucoDetector(dictionary, parameters)

        self.ORIGIN_ID = origin_id
        self.REFERENCE_ID = ref_id
        self.X_DIST = x_dist
        self.Y_DIST = y_dist

        self.foundReferences = False
        self.originX = -1
        self.originY = -1
        self.m3x = -1
        self.m3y = -1
        self.refAngle = -1
        self.xFactor = -1
        self.yFactor = -1

        self.origin_index = -1
        self.reference_index = -1

    #only called if origin index or ref index are -1. finds indices of origin and reference markers
    def findIndices(self, markerIDs):
        self.origin_index = self.reference_index = -1

        # Check for reference markers
        for i in range(len(markerIDs)):
            if markerIDs[i] == self.ORIGIN_ID:
                self.origin_index = i
            elif markerIDs[i] == self.REFERENCE_ID:
                self.reference_index = i

        # Only return true if both references are found and indexed
        return self.origin_index != -1 and self.reference_index != -1

    #finds origin coordinaates, scaling factors, reference angle. maybe split up?
    def findReferences(self, marker_corners):
        if self.origin_index < len(marker_corners):
            self.originX = marker_corners[self.origin_index][0][0][0]
            self.originY = marker_corners[self.origin_index][0][0][1]
        if self.reference_index < len(marker_corners):
            self.m3x = marker_corners[self.reference_index][0][0][0]
            self.m3y = marker_corners[self.reference_index][0][0][1]

        dist_x = self.m3x - self.originX
        dist_y = self.m3y - self.originY
        print(f"{dist_x} {dist_y}")
        if dist_x != 0 and dist_y != 0:
            self.xFactor = self.X_DIST / dist_x
            self.yFactor = self.Y_DIST / dist_y
            print(f"Scaling Factor X: {self.xFactor} cm/pixel, Y: {self.yFactor} cm/pixel")

            self.refAngle = calc.get_angle(marker_corners[self.origin_index])
            print(f"Reference Angle: {np.degrees(self.refAngle)}°")

            self.foundReferences = True

    def detect(self):
        success, frame = self.cap.read()
        marked_img = xCoor = yCoor = angle = None

        if success:
            marker_corners, marker_ids, _ = self.detector.detectMarkers(frame)

            if marker_ids is not None:
                if self.foundReferences:
                    for i in range(len(marker_ids)):
                        if marker_ids[i] not in [self.ORIGIN_ID, self.REFERENCE_ID]:
                            mc = marker_corners[i]

                            # Get the center of the third marker (agent)
                            center_x = np.mean(mc[0][:, 0])
                            center_y = np.mean(mc[0][:, 1])

                            # Find pixel difference between origin and bot, scale to cm
                            xCoor = (center_x - self.originX) * self.xFactor
                            yCoor = (center_y - self.originY) * self.yFactor
                            angle = calc.get_angle(marker_corners[i]) + self.refAngle

                            # print(f"X: {xCoor:.2f} cm, Y: {yCoor:.2f} cm, Orientation: {np.degrees(angle):.2f}°")
                elif self.findIndices(marker_ids):
                    self.findReferences(marker_corners)
                else:
                    # This will be run before reference is defined and the CV is searching for the reference markers
                    pass

            marked_img = aruco.drawDetectedMarkers(frame, marker_corners, marker_ids)
        return marked_img, xCoor, yCoor, angle

