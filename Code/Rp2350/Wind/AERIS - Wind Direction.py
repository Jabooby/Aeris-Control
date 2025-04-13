from machine import I2C, Pin
from AS5600_Handler import AS5600
import uasyncio as asyncio

class WindVane:
    def __init__(self):
        i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=100000)
        self.sensor = AS5600(i2c)
        self.zero_angle = 0  # default value, will be set in `initialize()`
        
    async def initialize(self):
        self.zero_angle = await self.sensor.get_angle()
        
    async def get_wind_direction(self):
        angle = await self.sensor.get_angle()
        return (angle - self.zero_angle) % 360

    async def set_0(self):
        self.zero_angle = await self.sensor.get_angle()


# Example usage:
if __name__ == "__main__":     
    async def main():
        windvane = WindVane()
        await windvane.initialize()

        while True:
            direction = await windvane.get_wind_direction()
            print(f"Wind direction: {direction:.2f}°")
            await asyncio.sleep(0.1)

    asyncio.run(main())

        
        
        
        
    