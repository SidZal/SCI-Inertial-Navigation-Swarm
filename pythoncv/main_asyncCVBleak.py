import csv
import cv2
import cv2.aruco as aruco
import time
import asyncio
import numpy as np
import asyncCVClass
from robotClassBleak import Cart
from bleak import BleakClient
import struct

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

# Load camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WINDOW_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, WINDOW_HEIGHT)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))  # changes codec to allow real-time video capture
print(cv2.cuda.getCudaEnabledDeviceCount())  # 0 -> integrated graphics, >0 -> GPU-enabled
DICT_USED = aruco.DICT_6X6_50

# ec:62:60:8e:60:16 RP2040 on turqoise cart
# a6:5d:28:70:b8:e2 33 BLE on white cart
# bot = robotClassBluePy.INRbot("ec:62:60:8e:60:16", ["bac3","78d3","2bef"])
#bot = robotClassBluePy.INRbot("a6:5d:28:70:b8:e2", ["bac3","78d3","2bef"])

setRefAngle = False
refAngle = 0
start_time = time.time()

csvfile = open(FILENAME, "a")
write = csv.writer(csvfile)

cartIDs = [["a6:5d:28:70:b8:e2", "bac3", "2bef", "78d3"]]
cart1 = Cart(cartIDs[0])

aCV = asyncCVClass.asyncCV(cap, DICT_USED, ORIGIN_ID, REFERENCE_ID, X_DIST, Y_DIST)


def wheel_ref_to_bytes(omega_left, omega_right):
    return omega_left.to_bytes(4, 'little', signed=True) + omega_right.to_bytes(4, 'little', signed=True)


async def sensor_notification(cartUUID, data):
    data = np.array(list(struct.iter_unpack('f', data)), dtype=float).flatten()
    ax, ay, az, gx, gy, gz, yaw, omegaL, omegaR = data
    print(yaw)


async def bluetooth_loop():
    print("Attempting Connection")
    async with BleakClient(cart1.address) as client:
        print("Connected")
        await client.start_notify(cart1.sensor, sensor_notification)
        await camera_loop()
        # client = BleakClient(cart1.address)
        # await client.connect()
        # await client.start_notify(cart1.sensor, sensor_notification)
        # await asyncio.sleep(300)
        # await client.stop_notify(cart1.sensor)
        # await client.disconnect()


async def camera_loop():
    while True:
        marked_img, xCoor, yCoor, angle = aCV.detect()

        if marked_img is not None:
            cv2.imshow('cam', marked_img)

        await asyncio.sleep(1/60)

        if cv2.pollKey() & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
    cap.release()

asyncio.run(bluetooth_loop())