import serial
import struct
import signal

class INRbot:
    # Pass in device address + UUIDlist (service, omega, omegaData, dmpData, rawData)
    def __init__(self):
        self.ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200)

    # simple destructor to disconnect from BLE
    def __del__(self):
        self.ser.close()

    def tunePID(self, kp, ki, kd):
        self.ser.write(bytes('P' + str(kp) + ',' + str(ki) + ',' + str(kd) + '\n', 'utf-8'))

    def setWheelSpeed(self, wL, wR):
        self.ser.write(bytes(str(wL) + ',' + str(wR),'utf-8'))

    def getData(self,waitForData):
        signal.signal(signal.SIGALRM, self.dataTimeoutHandler)
        signal.setitimer(signal.ITIMER_REAL, waitForData)

        try:
            data = (self.ser.readline()).decode('utf-8').split(',', 8)
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
