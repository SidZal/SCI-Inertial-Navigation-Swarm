import asyncio
from robotClassBleak import Cart
from bleak import BleakClient
import struct
import numpy as np
import random
import time

# 2D array for each cart: address, service UUID, wheel Reference UUID, sensor UUID
cartIDs = [["a6:5d:28:70:b8:e2", "bac3", "2bef", "78d3"]]
#a6:5d:28:70:b8:e2 arduino address on cart
#f7:ae:59:0b:bf:08 arduino address on breadboard
print(cartIDs[0])

cart1 = Cart(cartIDs[0])
def wheel_ref_to_bytes(omega_left, omega_right):
    return omega_left.to_bytes(4, 'little', signed=True) + omega_right.to_bytes(4, 'little', signed=True)

startTime = time.perf_counter()
async def main():
    client = BleakClient(cart1.address)
    await client.connect()
    await client.start_notify(cart1.sensor, sensor_notification)
    await asyncio.sleep(60)
    await client.stop_notify(cart1.sensor)
    await client.disconnect()
    # async with BleakClient(cart1.address) as client:
    #     await client.start_notify(cart1.sensor, sensor_notification)
    #     # while True:
    #     #     print("Writing...")
    #     #     the_data = wheel_ref_to_bytes(int(10*random.random()), int(10*random.random()))
    #     #     await client.write_gatt_char(cart1.wheel_reference, the_data, response=True)
    #     #     await asyncio.sleep(.1)
    #     await asyncio.sleep(60)
    #
    #     await client.stop_notify(cart1.sensor)

    # success, data = cart1.read_sensor()
    # if success:
    #     print(data)
    # else:
    #     pass#print("oh naur")
    # print(await cart1.manual_read())

def sensor_notification(cartUUID, data):
    print(time.perf_counter()-startTime)
    data = np.array(list(struct.iter_unpack('f', data)), dtype=float).flatten()
    ax, ay, az, gx, gy, gz, yaw, omegaL, omegaR = data
    #print(f"Ax: {ax:.3f}")

asyncio.run(main())