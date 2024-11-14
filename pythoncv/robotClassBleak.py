from bleak import BleakClient
from bleak import BleakGATTCharacteristic
import struct

# Basic class to hold cart identifiers, no functions
class Cart:
    __slots__ = ("address", "service", "sensor", "wheel_reference")

    def __init__(self, IDs):
        self.address = IDs[0]
        self.service = IDs[1]
        self.wheel_reference = IDs[2]
        self.sensor = IDs[3]

    def wheel_ref_to_bytes(self, omega_left, omega_right):
        return omega_left.to_bytes(4, 'little', signed=True) + omega_right.to_bytes(4, 'little', signed=True)

    async def set_wheel_speed(self, omega_left, omega_right, client):
        omega_byte_array = self.wheel_ref_to_bytes(omega_left, omega_right)
        print("here")
        await client.write_gatt_char(self.wheel_reference, omega_byte_array)

# class below DOES NOT work
class INRBot:
    def __init__(self, address, uuids):
        # Initialize device at address and connect
        self.dev = BleakClient(address)

        # BLE Characteristics identified by UUID
        # Bleak also has a GATTCharacteristic class, not sure if applicable
        self.wheel_reference_uuid = uuids[1]
        self.sensor_data_uuid = uuids[2]
        self.sensor_data = None

    async def setup(self):
        await self.dev.connect()
        # Verify connection and prepare characteristic properties
        if self.dev.is_connected:
            print("Connected to " + self.dev.address)

            # Subscribe to sensor Notifications
            print(self.sensor_data_uuid)
            await self.dev.start_notify(self.sensor_data_uuid, self.notification_handler)
        else:
            print("Failed to connect to " + self.dev.address)

    def __del__(self):
        print("Disconnecting from " + self.dev.address)
        self.dev.disconnect()

    # Converts wheel speeds to bytes and writes to characteristic
    async def set_wheel_speed(self, omega_left, omega_right):
        omega_byte_array = omega_left.to_bytes(4, 'little', signed=True) + omega_right.to_bytes(4, 'little', signed=True)
        await self.dev.write_gatt_char(self.wheel_reference_uuid, omega_byte_array)

    async def manual_read(self):
        data = list(struct.iter_unpack('f', await self.dev.read_gatt_char(self.sensor_data_uuid)))
        return data

    # Accesses and acknowledges notification data on class side
    def read_sensor(self):
        if self.sensor_data:
            data = list(struct.iter_unpack('f', self.sensor_data))
            self.sensor_data = None
            return True, data
        else:
            return False, None

    # Saves notification data for future use
    def notification_handler(self, sender: BleakGATTCharacteristic, data: bytearray):
        print("Notified!!")
        self.sensor_data = data
