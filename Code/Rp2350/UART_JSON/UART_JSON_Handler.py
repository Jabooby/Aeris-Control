import ujson
from machine import UART, Pin
import uasyncio as asyncio

class UARTJsonHandler:
    def __init__(self, uart_id=0, baudrate=115600, tx_pin=Pin(0), rx_pin=Pin(1)):
        """
        Initialize UART communication for JSON handling.
        :param uart_id: UART bus ID (0 or 1 for RP2040)
        :param baudrate: Communication speed (default 9600)
        :param tx_pin: TX Pin object
        :param rx_pin: RX Pin object
        """
        self.uart = UART(uart_id, baudrate=baudrate, tx=tx_pin, rx=rx_pin)

    async def send_json(self, data):
        """
        Serialize a dictionary to JSON and send over UART.
        :param data: Dictionary containing data to send
        """
        try:
            json_data = ujson.dumps(data) + "\n"  # Newline separates JSON packets
            self.uart.write(json_data)
            print(f"Sent: {json_data}")
        except (ValueError, TypeError) as e:
            print(f"Error serializing JSON: {e}")

    async def read_json(self):
        """
        Read incoming UART data, parse it as JSON, and return the parsed dictionary.
        :return: Parsed dictionary or None if no valid JSON received
        """
        while True:
            if self.uart.any():
                try:
                    line = self.uart.readline().decode().strip()  # Read and decode JSON string
                    if line:
                        parsed_data = ujson.loads(line)
                        print(f"Received JSON: {parsed_data}")
                        return parsed_data
                except (ValueError, UnicodeError) as e:
                    print(f"Error decoding JSON: {e}")
            await asyncio.sleep(0.1)  # Yield control to other tasks

# Example usage:
if __name__ == "__main__":
    async def async_send_data_periodically(uart_handler):
        """
        Periodically send JSON data over UART (simulating sensor data).
        """
        while True:
            simulated_data = {
                "sensor_1": 23.5,
                "sensor_2": 45.0,
                "status": "normal"
            }
            await uart_handler.send_json(simulated_data)
            await asyncio.sleep(2)  # Send data every 2 seconds

    async def main():
        # Initialize UART communication for JSON handling
        uart_handler = UARTJsonHandler()

        # Task to handle periodic UART data transmission
        asyncio.create_task(async_send_data_periodically(uart_handler))

        # Continuously read and print incoming JSON data from UART
        await uart_handler.read_json()

    # Run the main coroutine
    asyncio.run(main())
