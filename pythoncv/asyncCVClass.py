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

        self.foundIndices = False
        self.origin_index = -1
        self.reference_index = -1

    #only called if origin index or ref index are -1. finds indices of origin and reference markers
    def findIndices(self, markerIDs):
        # Check for reference markers
        for i in range(len(markerIDs)):
            if self.origin_index == -1 and markerIDs[i] == self.ORIGIN_ID:
                self.origin_index = i
            elif self.reference_index == -1 and markerIDs[i] == self.REFERENCE_ID:
                self.reference_index = i

        if self.origin_index != -1 and self.reference_index != -1:
            self.foundIndices = True

    #finds origin coordinaates, scaling factors, reference angle. maybe split up?
    def findReferences(self, markerCorners):

        if (self.origin_index < len(markerCorners)):
            self.originX = markerCorners[self.origin_index][0][0][0]
            self.originY = markerCorners[self.origin_index][0][0][1]
        if (self.reference_index < len(markerCorners)):
            self.m3x = markerCorners[self.reference_index][0][0][0]
            self.m3y = markerCorners[self.reference_index][0][0][1]

        dist_x = self.m3x - self.originX
        dist_y = self.m3y - self.originY

        if dist_x != 0 and dist_y != 0:
            self.xFactor = self.X_DIST / dist_x
            self.yFactor = self.Y_DIST / dist_y
            print(f"Scaling Factor X: {self.xFactor} cm/pixel, Y: {self.yFactor} cm/pixel")

            if (self.origin_index < len(markerCorners)):
                self.refAngle = calc.get_angle(markerCorners[self.origin_index]) - np.radians(90)
                print(f"Reference Angle: {np.degrees(self.refAngle)}°")

            if self.refAngle != -1: #finished final step in finding references
                self.foundReferences = True

    def detect(self):
        success, frame = self.cap.read()

        if success:
            markerCorners, markerIDs, _ = self.detector.detectMarkers(frame)
        else:
            return None #not sure if this is the right thing

        if markerIDs is not None and not self.foundIndices:
            self.findIndices(markerIDs)

        # Record corner references if they have not been recorded
        if not self.foundReferences and self.foundIndices:
            self.findReferences(markerCorners)

        for i in range(len(markerIDs)):
            xCoor = yCoor = angle = -1

            if self.foundReferences and markerIDs[i] not in [self.ORIGIN_ID, self.REFERENCE_ID]:
                mc = markerCorners[i]

                # Get the center of the third marker (agent)
                centerX = np.mean(mc[0][:, 0])
                centerY = np.mean(mc[0][:, 1])

                # Find pixel difference between origin and bot, scale to cm
                xCoor = (centerX - self.originX) * self.xFactor
                yCoor = (centerY - self.originY) * self.yFactor

                angle = calc.get_angle(markerCorners[i]) - self.refAngle

                print(f"X: {xCoor:.2f} cm, Y: {yCoor:.2f} cm, Orientation: {np.degrees(angle):.2f}°")

                # Poll for IMU data and write to CSV if received. 1/X -> max X Hz data
                # success, data = bot.receiveNotification(1 / 60)
                # if success:
                #     # print(data)
                #     ax, ay, az, gx, gy, gz, yaw, omegaL, omegaR = data
                #
                #     timeSinceStart = time.time() - start_time
                #     print(f"Time: {timeSinceStart:.3f}")
                #     write.writerow([timeSinceStart, int(markerIDs[i]),
                #                             ax, ay, az, gx, gy, gz,
                #                             yaw, omegaL, omegaR,
                #                             xCoor, yCoor, angle])

        marked_img = aruco.drawDetectedMarkers(frame, markerCorners, markerIDs)
        return marked_img, xCoor, yCoor, angle

