from bluepy import btle
import numpy as np
import struct
import signal

class INRbot:
    # Pass in device address + UUIDlist (service, omega, omegaData, dmpData, rawData)
    def __init__(self, address, UUIDlist):
        # Connect to device

        print("Attempting BLE Connection to: " + address)
        self.dev = btle.Peripheral(address)
        self.dev.setMTU(60)
        print("Setting up:")
        print(self.dev)

        # Find service and verify characteristics
        self.service_UUID = btle.UUID(UUIDlist[0])
        self.service = self.dev.getServiceByUUID(self.service_UUID)
        print("\nService Detected: " + str(self.service))

        self.characteristics = self.dev.getCharacteristics()
        # For each characteristic, assign to class characteristic
        for char in self.characteristics:
            if char.uuid == UUIDlist[1]:
                self.onBoardData = char
                self.dev.writeCharacteristic(self.onBoardData.valHandle+1, b"\x01\x00")
            elif char.uuid == UUIDlist[2]:
                self.charBackup = UUIDlist[2]
                self.omegaChar = char

        self.dev.setDelegate(self)
        self.sensorData = None
        print("INR Setup Complete\n")

    # simple destructor to disconnect from BLE
    def __del__(self):
        self.dev.disconnect()

    # def tunePID(self, kp, ki, kd):
    #     gain_byte_array = struct.pack('f', kp) + struct.pack('f', ki) + struct.pack('f', kd)
    #     self.PID.write(gain_byte_array, True)

    def setWheelSpeed(self, wL, wR):
        omega_byte_array = wL.to_bytes(4, 'little', signed=True) + wR.to_bytes(4, 'little', signed=True)
        self.omegaChar.write(omega_byte_array, False)

        # signal.signal(signal.SIGALRM, self.dataTimeoutHandler)
        # signal.setitimer(signal.ITIMER_REAL, timeout)
        #
        # try:
        #     self.omegaChar.write(omega_byte_array, True)
        #     signal.setitimer(signal.ITIMER_REAL, 0)
        # except Exception as exc:
        #     signal.setitimer(signal.ITIMER_REAL, 0)


    def manualReadData(self, timeout):
        signal.signal(signal.SIGALRM, self.dataTimeoutHandler)
        signal.setitimer(signal.ITIMER_REAL, timeout)

        try:
            data = list(struct.iter_unpack('f', self.onBoardData.read()))
            signal.setitimer(signal.ITIMER_REAL, 0)
            return True, data
        except Exception as exc:
            signal.setitimer(signal.ITIMER_REAL, 0)
            return False, 0

    def dataTimeoutHandler(signum, frame, gunk):
        raise Exception("Signal Timed Out")

    def receiveNotification(self, timeout):
        if self.dev.waitForNotifications(timeout):
            return True, self.sensorData
        else:
            return False, None

    def handleNotification(self, cHandle, data):
        if cHandle == self.onBoardData.getHandle():
            self.sensorData = np.array(list(struct.iter_unpack('f', data)), dtype=float).flatten()
        else:
            print("ERROR: Unexpected Notification Received")
            print(cHandle)
            while True:
                pass
