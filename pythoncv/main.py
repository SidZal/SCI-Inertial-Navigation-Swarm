import csv
import cv2
import cv2.aruco as aruco
import time
import calc
import asyncio
import numpy as np
import robotClassBluePy


# Width/Height dictates window size and resolution OpenCV will analyze
WINDOW_WIDTH = 700
WINDOW_HEIGHT = 550

# ArUco Reference Markers
ORIGIN_ID = 0
X_DIST = 40
Y_DIST = 50
REFERENCE_ID = 3

# Open file for data logging
timestr = time.strftime("%Y%m%d-%H%M%S")
FILENAME = f"../.generateddata/{timestr}.csv"

# OpenCV / ArUco Parameters
DICT_USED = aruco.DICT_6X6_50
dictionary = aruco.getPredefinedDictionary(DICT_USED)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)

# Load camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WINDOW_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, WINDOW_HEIGHT)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")) # changes codec to allow real-time video capture
print(cv2.cuda.getCudaEnabledDeviceCount()) # 0 -> integrated graphics, >0 -> GPU-enabled

# ec:62:60:8e:60:16
# a6:5d:28:70:b8:e2
# bot = robotClassBluePy.INRbot("ec:62:60:8e:60:16", ["bac3","78d3","2bef"])
bot = robotClassBluePy.INRbot("a6:5d:28:70:b8:e2", ["bac3","78d3","2bef"])

setRefAngle = False
refAngle = 0
start_time = time.time()

csvfile = open(FILENAME, "a")
write = csv.writer(csvfile)


def wheel_ref_to_bytes(omega_left, omega_right):
    return omega_left.to_bytes(4, 'little', signed=True) + omega_right.to_bytes(4, 'little', signed=True)


async def main():
    foundCorners = False
    setRefAngle = False

    while True:
        success, frame = cap.read()

        if success:
           markerCorners, markerIDs, _ = detector.detectMarkers(frame)
        else:
            continue

        if markerIDs is not None:
            origin_index = reference_index = -1

            # Check for reference markers
            for i in range(len(markerIDs)):
                if origin_index != -1 and reference_index != -1: break
                if markerIDs[i] == ORIGIN_ID:
                    origin_index = i
                    continue
                elif markerIDs[i] == REFERENCE_ID:
                    reference_index = i
                    continue

            # Record corner references if they have not been recorded
            if not foundCorners and origin_index != -1 and reference_index != -1:
                m0x = markerCorners[origin_index][0][0][0]
                m0y = markerCorners[origin_index][0][0][1]
                m3x = markerCorners[reference_index][0][0][0]
                m3y = markerCorners[reference_index][0][0][1]

                dist_x = m3x - m0x
                dist_y = m3y - m0y

                if dist_x != 0 and dist_y != 0:
                    xFactor = X_DIST / dist_x
                    yFactor = Y_DIST / dist_y
                    print(f"Scaling Factor X: {xFactor} cm/pixel, Y: {yFactor} cm/pixel")

                    refAngle = calc.get_angle(markerCorners[origin_index]) - np.radians(90)
                    print(f"Reference Angle: {np.degrees(refAngle)}°")

                    foundCorners = True

            for i in range(len(markerIDs)):
                if foundCorners and markerIDs[i] not in [ORIGIN_ID, REFERENCE_ID]:
                    mc = markerCorners[i]

                    # Get the center of the third marker (agent)
                    centerX = np.mean(mc[0][:, 0])
                    centerY = np.mean(mc[0][:, 1])

                    # Find pixel difference between origin and bot, scale to cm
                    xCoor = (centerX - m0x) * xFactor
                    yCoor = (centerY - m0y) * yFactor

                    angle = calc.get_angle(markerCorners[i]) - refAngle

                    # print(f"X: {xCoor:.2f} cm, Y: {yCoor:.2f} cm, Orientation: {np.degrees(angle):.2f}°")

                    # Poll for IMU data and write to CSV if received. 1/X -> max X Hz data
                    success, data = bot.receiveNotification(1/60)
                    if success:
                        #print(data)
                        ax, ay, az, gx, gy, gz, yaw, omegaL, omegaR = data

                        timeSinceStart = time.time() - start_time
                        print(f"Time: {timeSinceStart:.3f}")
                        write.writerow([timeSinceStart, int(markerIDs[i]),
                                        ax, ay, az, gx, gy, gz,
                                        yaw, omegaL, omegaR,
                                        xCoor, yCoor, angle])

        marked_img = aruco.drawDetectedMarkers(frame, markerCorners, markerIDs)
        cv2.imshow('cam', marked_img)

        # 'Q' to end program
        if cv2.pollKey() & 0xFF == ord('q'):
           break

    cv2.destroyAllWindows()
    cap.release()

asyncio.run(main())