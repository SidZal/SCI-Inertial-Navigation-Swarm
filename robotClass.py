from bluepy import btle
import struct
import signal

class INRbot:
    # Pass in device address + UUIDlist (service, omega, omegaData, dmpData, rawData)
    def __init__(self, address, UUIDlist):
        # Connect to device
        print("Attempting BLE Connection to: " + address)
        self.dev = btle.Peripheral(address)
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
                self.omegaChar = char
            elif char.uuid == UUIDlist[2]:
                self.onBoardData = char
            elif char.uuid== UUIDlist[3]:
                self.PID = char

        print("INR Setup Complete\n")

    # simple destructor to disconnect from BLE
    def __del__(self):
        self.dev.disconnect()

    def tunePID(self, kp, ki, kd):
        gain_byte_array = struct.pack('f', kp) + struct.pack('f', ki) + struct.pack('f', kd)
        self.PID.write(gain_byte_array, True)

    def setWheelSpeed(self, wL, wR):
        omega_byte_array = wL.to_bytes(4, 'little', signed=True) + wR.to_bytes(4, 'little', signed=True)
        self.omegaChar.write(omega_byte_array, True)

    def getData(self,waitForData):
        signal.signal(signal.SIGALRM, self.dataTimeoutHandler)
        signal.setitimer(signal.ITIMER_REAL, waitForData)

        try:
            data = list(struct.iter_unpack('f', self.onBoardData.read()))
            signal.setitimer(signal.ITIMER_REAL, 0)
            return True, data
        except Exception as exc:
            signal.setitimer(signal.ITIMER_REAL, 0)
            return False, 0

    def dataTimeoutHandler(signum, frame, gunk):
        raise Exception("Signal Timed Out")

    # Bluepy notifications not working
    # def handleNotification(self, cHandle, data):
    #     print("Notification")
    #     if cHandle == self.rawDataChar.getHandle():
    #         self.motion6Raw = list(struct.iter_unpack('i', data))
    #     elif cHandle == self.floatDataChar.getHandle():
    #         self.floatData = list(struct.iter_unpack('f', data))
    #     else:
    #         print("ERROR: Unexpected Notification Received")
    #         print(cHandle)
    #         while True:
    #             pass
