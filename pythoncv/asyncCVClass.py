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

        self.originX = self.originY = self.m3x = self.m3y = None
        self.xFactor = self.yFactor = self.refAngle = None

    def findRefs(self, markerIDs, marker_corners):
        if self.originX is None or self.m3x is None: # Check for reference markers
            for i in range(len(markerIDs)):
                if markerIDs[i] == self.ORIGIN_ID:
                    self.originX = marker_corners[i][0][0][0]
                    self.originY = marker_corners[i][0][0][1]
                    self.refAngle = calc.get_angle(marker_corners[i])
                    print(f"Reference Angle: {np.degrees(self.refAngle)}°")
                elif markerIDs[i] == self.REFERENCE_ID:
                    self.m3x = marker_corners[i][0][0][0]
                    self.m3y = marker_corners[i][0][0][1]

            if self.originX is not None and self.m3x is not None:
                dist_x = self.m3x - self.originX
                dist_y = self.m3y - self.originY

                self.xFactor = self.X_DIST / dist_x
                self.yFactor = self.Y_DIST / dist_y
                print(f"Scaling Factor X: {self.xFactor} cm/pixel, Y: {self.yFactor} cm/pixel")
        else:
            return True

        return False

    def detect(self):
        success, frame = self.cap.read()
        marked_img = xCoor = yCoor = angle = None

        if success:
            marker_corners, marker_ids, _ = self.detector.detectMarkers(frame)

            if marker_ids is not None:
                if self.findRefs(marker_ids, marker_corners):
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

            marked_img = aruco.drawDetectedMarkers(frame, marker_corners, marker_ids)
        return marked_img, xCoor, yCoor, angle

