import csv
import cv2
import cv2.aruco as aruco
import time
import calc
import asyncio
import numpy as np
from robotClassBleak import Cart
import robotClassBluePy
from bleak import BleakClient
import struct


#constants
DICT_USED = aruco.DICT_6X6_50
WINDOW_WIDTH = 700#pixelsπ
WINDOW_HEIGHT = 550
X_DIST = 40
Y_DIST = 50

ORIGIN_ID = 0
REFERENCE_ID = 3

timestr = time.strftime("%Y%m%d-%H%M%S")
FILENAME = f"../.generateddata/{timestr}.csv"

dictionary = aruco.getPredefinedDictionary(DICT_USED)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WINDOW_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, WINDOW_HEIGHT)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")) # add this line
start_time = time.time()
setRefAngle = False
refAngle = 0
print(cv2.cuda.getCudaEnabledDeviceCount())

bot = robotClassBluePy.INRbot("a6:5d:28:70:b8:e2", ["bac3","78d3","2bef"])
#pather = randompath.botPath(bot)

# # 2D array for each cart: address, service UUID, wheel Reference UUID, sensor UUID
# cartIDs = [["a6:5d:28:70:b8:e2", "bac3", "2bef", "78d3"]]
# #a6:5d:28:70:b8:e2 arduino address on cart
# #f7:ae:59:0b:bf:08 arduino address on breadboard
# print(cartIDs[0])
# # TODO: dynamically create array of cart objects given length of cartIDS
# cart1 = Cart(cartIDs[0])

def wheel_ref_to_bytes(omega_left, omega_right):
    return omega_left.to_bytes(4, 'little', signed=True) + omega_right.to_bytes(4, 'little', signed=True)


# def sensor_notification(cartUUID, data):
#     #print("Notified")
#     if newData:
#         global start_time
#         print(time.time()-start_time)
#         print(list(struct.iter_unpack('f', data)))
#         newData = False



csvfile = open(FILENAME, "a")
write = csv.writer(csvfile)
newData = False
xCoor = 0.
yCoor = 0.
angle = 0.
count = 5
num = 0


async def main():
    # client = BleakClient(cart1.address)
    # await client.connect()
    foundCorners = False
    setRefAngle = False

    #await client.start_notify(cart1.sensor, sensor_notification)
    while True:
        success, frame = cap.read()

        if success:
           markerCorners, markerIDs, rejectedCandidates = detector.detectMarkers(frame)
        else:
            continue

        if markerIDs is not None:
            origin_index = reference_index = -1

            # get reference markers
            for i in range(len(markerIDs)):
                if (origin_index != -1 and reference_index != -1): break
                if (markerIDs[i] == ORIGIN_ID):
                    origin_index = i
                    continue
                elif (markerIDs[i] == REFERENCE_ID):
                    reference_index = i
                    continue

            if origin_index != -1 and reference_index != -1 and not foundCorners:
                global m0x
                m0x = markerCorners[origin_index][0][0][0]
                global m0y
                m0y = markerCorners[origin_index][0][0][1]
                m3x = markerCorners[reference_index][0][0][0]
                m3y = markerCorners[reference_index][0][0][1]

                dist_x = m3x - m0x
                dist_y = m3y - m0y

                if dist_x != 0 and dist_y != 0:
                    print("Factors Calculated")
                    xFactor = X_DIST / dist_x
                    yFactor = Y_DIST / dist_y
                    foundCorners = True
                    print(f"Scaling Factors - X: {xFactor} cm/pixel, Y: {yFactor} cm/pixel")

            for i in range(len(markerIDs)):
                if setRefAngle == False and markerIDs[i] == ORIGIN_ID:
                    print("Ref Angle Calculated")
                    refAngle = calc.getAngle(markerCorners[i]) - np.radians(90)
                    setRefAngle = True

                if (markerIDs[i] != ORIGIN_ID) and (markerIDs[i] != REFERENCE_ID) and setRefAngle and foundCorners:
                    mc = markerCorners[i]

                    # Get the center of the third marker (agent)
                    centerX = np.mean(mc[0][:, 0])
                    centerY = np.mean(mc[0][:, 1])

                    xCoor = (centerX - m0x) * xFactor  # Corrected scaling
                    yCoor = (centerY - m0y) * yFactor  # Corrected scaling

                    angle = calc.getAngle(markerCorners[i]) - refAngle

                    #print(f"Real-World Position - X: {xCoor:.2f} cm, Y: {yCoor:.2f} cm")
                    #print(f"Orientation Angle: {np.degrees(angle):.2f}°")

                    # Poll for IMU data and write to CSV if received. 1/60 -> max 60 Hz data
                    success, data = bot.receiveNotification(1/60)
                    if success:
                        #print(data)
                        ax, ay, az, gx, gy, gz, yaw, omegaL, omegaR = data

                        timeSinceStart = time.time() - start_time
                        print(f"Logging Data. Time: {timeSinceStart}")
                        write.writerow([timeSinceStart, int(markerIDs[i]),
                                        ax, ay, az, gx, gy, gz,
                                        yaw, omegaL, omegaR,
                                        xCoor, yCoor, angle])

        QueryImg = aruco.drawDetectedMarkers(frame, markerCorners, markerIDs)
        cv2.imshow('cam', QueryImg)

        if cv2.waitKey(1) & 0xFF == ord('q'): #press 'q' to quit
           break

    cv2.destroyAllWindows()
    cap.release()

asyncio.run(main())