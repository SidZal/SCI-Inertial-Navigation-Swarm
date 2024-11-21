import csv

import bleak
import cv2
import cv2.aruco as aruco
import time
import asyncio
import numpy as np
import asyncCVClass
from robotClassBleak import Cart
from bleak import BleakClient
import struct
from pyquaternion import Quaternion
import os
import json
import randompath

# Width/Height dictates window size and resolution OpenCV will analyze
WINDOW_WIDTH = 700
WINDOW_HEIGHT = 550

# ArUco Reference Markers
ORIGIN_ID = 3
X_DIST = 40
Y_DIST = 50
REFERENCE_ID = 0

# Open file (s) for data logging
timestr = time.strftime("%Y%m%d%H%M%S")
DIR = f"../.tliodata/{timestr}"
os.makedirs(DIR)
TRAIN_LIST = open(f"../.tliodata/train_list.txt", 'a')

IMU_FILE_PATH = f"../.tliodata/{timestr}/imu_samples_0.csv"
CV_FILE_PATH = f"../.tliodata/{timestr}/imu0_resampled.npy"

IMU_FILE = csv.writer(open(IMU_FILE_PATH, 'w'))
# IMU_FILE.writerow("#timestamp [ns], temperature [degC], w_RS_S_x [rad s^-1], w_RS_S_y [rad s^-1], w_RS_S_z [rad s^-1], a_RS_S_x [m s^-2], a_RS_S_y [m s^-2], a_RS_S_z [m s^-2]")
CV_FILE = open(CV_FILE_PATH, 'ab')

# Load camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WINDOW_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, WINDOW_HEIGHT)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))  # changes codec to allow real-time video capture
print(cv2.cuda.getCudaEnabledDeviceCount())  # 0 -> integrated graphics, >0 -> GPU-enabled
cap.set(cv2.CAP_PROP_FPS, 30)
print(cap.get(cv2.CAP_PROP_FPS))
DICT_USED = aruco.DICT_6X6_50

startTime = 0
cv_data = []

# ec:62:60:8e:60:16 RP2040 on black cart
# a6:5d:28:70:b8:e2 33 BLE on white cart
cartIDs = [["a6:5d:28:70:b8:e2", "bac3", "2bef", "78d3"]]
cart1 = Cart(cartIDs[0])
bot = randompath.botPath(cart1)

aCV = asyncCVClass.asyncCV(cap, DICT_USED, ORIGIN_ID, REFERENCE_ID, X_DIST, Y_DIST)


class sharedCoords:
    def __init__(self):
        self.xCoor = 0
        self.yCoor = 0

angle = 0
coords = sharedCoords()

async def sensor_notification(cartUUID, data):
    clock = time.perf_counter_ns() - startTime
    data = np.array(list(struct.iter_unpack('f', data)), dtype=float).flatten()
    ax, ay, az, gx, gy, gz, yaw, omegaL, omegaR = data
    IMU_FILE.writerow([clock, 0, gx, gz, gy, ax, az, ay])  # negating y and z axis due to IMU orientation
    # print(f"IMU: {clock/1e9}")



async def bluetooth_loop():
    print("Attempting Connection")
    async with BleakClient(cart1.address) as client:
        print("Connected")
        await client.start_notify(cart1.sensor, sensor_notification)

        global startTime
        startTime = time.perf_counter_ns()
        tasks = [bot.takePath(client, coords), camera_loop()]

        task_objects = [asyncio.create_task(t) for t in tasks]

        # Wait for the first task to complete
        done, pending = await asyncio.wait(task_objects, return_when=asyncio.FIRST_COMPLETED)

        # Handle the result of the completed task

        # Cancel remaining tasks
        for pending_task in pending:
            pending_task.cancel()
            try:
                await pending_task
            except asyncio.CancelledError:
                print("A pending task was cancelled.")

        await client.stop_notify(cart1.sensor)


async def camera_loop():
    cam_clock = time.perf_counter_ns()

    while True:
        # global xCoor
        # global yCoor

        old_x = coords.xCoor
        old_y = coords.yCoor
        marked_img, xCoor, yCoor, angle = aCV.detect()
        print(f"x {xCoor} y {yCoor}")

        if marked_img is not None:
            cv2.imshow('cam', marked_img)
            if xCoor is not None:
                coords.xCoor = xCoor
                coords.yCoor = yCoor
                old_clock = cam_clock
                cam_clock = time.perf_counter_ns() - startTime

                dt = (old_clock - cam_clock) / 1e9
                cv_quat = Quaternion(axis=[0, 0, 1], angle=angle)

                if old_x is None:
                    xVel = yVel = 0
                else:
                    xVel = (coords.xCoor - old_x) / dt
                    yVel = (coords.yCoor - old_y) / dt

                cv_data.append([cam_clock / 1e3, 0,0,0, 0,0,0,
                                  cv_quat[1], cv_quat[2], cv_quat[3], cv_quat[0],
                                  coords.xCoor/100, coords.yCoor/100, 0,
                                  xVel, yVel, 0])
                print(f"Cam: {cam_clock/1e9}")

        await asyncio.sleep(1/60)

        if cv2.pollKey() & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    cap.release()

asyncio.run(bluetooth_loop())

# Save data for TLIO
np.save(CV_FILE, cv_data)
TRAIN_LIST.write(timestr + '\n')

if len(cv_data) > 0:
    camJSON = {
        "columns_name(width)": [
            "ts_us(1)",
            "gyr_compensated_rotated_in_World(3)",
            "acc_compensated_rotated_in_World(3)",
            "qxyzw_World_Device(4)",
            "pos_World_Device(3)",
            "vel_World(3)"
        ],
        "num_rows": len(cv_data),
        "approximate_frequency_hz": 30.0,
        "t_start_us": cv_data[0][0],
        "t_end_us": cv_data[-1][0]
    }
    camJSONFile = open(f"../.tliodata/{timestr}/imu0_resampled_description.json", 'w')
    json.dump(camJSON, camJSONFile, indent=4)

    camJSONFile.close()
CV_FILE.close()
TRAIN_LIST.close()
