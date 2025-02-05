"""
Not really used, kind of overkill
Could be used later for tests and error handling.
BMI160_Handler is not dependant on it
"""
import uasyncio as asyncio
from machine import I2C, Pin

class I2CHandler:
    def __init__(self, scl_pin=Pin(5), sda_pin=Pin(4), freq=400_000):
        """
        Initialize I2C communication.
        :param scl_pin: SCL Pin object
        :param sda_pin: SDA Pin object
        :param freq: I2C frequency (default 400kHz)
        """
        self.i2c = I2C(0, scl=scl_pin, sda=sda_pin, freq=freq)

    async def read_register(self, address, reg, length):
        """
        Read data from a specific register on an I2C device.
        :param address: I2C device address
        :param reg: Register address
        :param length: Number of bytes to read
        :return: Bytes read from the register or None on error
        """
        try:
            return self.i2c.readfrom_mem(address, reg, length)
        except OSError as e:
            print(f"Error reading register {reg} from device {hex(address)}: {e}")
            return None

    async def write_register(self, address, reg, value):
        """
        Write a value to a specific register on an I2C device.
        :param address: I2C device address
        :param reg: Register address
        :param value: Value to write
        """
        try:
            self.i2c.writeto_mem(address, reg, bytes([value]))
        except OSError as e:
            print(f"Error writing to register {reg} on device {hex(address)}: {e}")
