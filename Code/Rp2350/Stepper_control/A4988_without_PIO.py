"""
A4988 Stepper Motor Driver Control (Non-PIO Solution)

This script provides a class-based solution for controlling a stepper motor 
using the A4988 driver without relying on the PIO (Programmable I/O) 
functionality of the Raspberry Pi Pico. Instead, it uses GPIO pins 
for step and direction signals and synchronous blocking code to manage motor movement.

Key Features:
-------------
1. **A4988 Class:** Manages the interaction with the A4988 stepper motor driver.
2. **Synchronous Step Control:** The `move_sync()` function allows precise control of
   stepper motor steps with customizable speed.
3. **Direction and Step Pins:** Two GPIO pins are used:
    - **DIR Pin:** Sets the motor direction (forward or backward).
    - **STEP Pin:** Emits the pulse to step the motor.
4. **Blocking Execution:** The solution uses time delays (`time.sleep`) for motor control.
   This method is simple but non-optimal for multitasking environments.

Hardware Requirements:
----------------------
- Raspberry Pi Pico 3.3V
- A4988 Stepper Motor Driver 3.3V 
- Stepper motor
- Wiring connections:
  - **DIR (GPIO 15)** connected to A4988 DIR input
  - **STEP (GPIO 14)** connected to A4988 STEP input
  - Proper power supply for the motor 12V

Usage Instructions:
-------------------
- Instantiate the `A4988` class to control the motor.
- Use the `move_sync(steps, speed)` method to move the motor forward or backward
  at a specified speed (steps per second).
- The speed should ideally be between 300 and 900 steps per second for reliable operation.

Example Usage:
--------------
stepper = A4988()
stepper.move_sync(800, 500)  # Move 800 steps forward at 500 steps/sec
stepper.move_sync(-800, 500) # Move 800 steps backward at 500 steps/sec

Code Details:
-------------
1. **Class Methods:**
   - `step(forward=True)`: Emits a single step pulse.
   - `move_sync(steps, speed)`: Moves the motor by the specified steps at the given speed.
   - `deinit()`: Cleans up resources associated with the motor pins.
   - Context manager support (`__enter__` and `__exit__`) for automatic cleanup.

2. **Main Execution:** 
   - The main loop demonstrates forward and backward motor control with gradually increasing speed.
   - If speed exceeds 2000 steps per second, it resets to 200 to maintain motor stability.

Performance Considerations:
----------------------------
- **Speed Limitation:** Speeds above 1000 steps/sec may be unreliable due to blocking code delays.
- **PIO Recommendation:** For high-frequency and asynchronous control, consider a PIO-based solution.

Note:
-----
This is a simple and hardware-efficient solution but lacks the precision and multitasking capabilities that PIO can provide.

"""
#Thanks CHAT-GPT for that gorgeous header

import time

import machine


#---------------------------------INIT CLASS A4988-----------------------------------------------
class A4988:
    def __init__(self, DIR=machine.Pin(15, machine.Pin.OUT), STEP=machine.Pin(14, machine.Pin.OUT)):
        """This class represents an A4988 stepper motor driver.  It uses two output pins
        for direction and step control signals."""

        self._dir  = DIR
        self._step = STEP

        self._dir.value(0)
        self._step.value(0)


    def move_sync(self, steps, speed=1000.0):
        """Move the stepper motor the signed number of steps forward or backward at the
        speed specified in steps per second.  N.B. this function will not return
        until the move is done, so it is not compatible with asynchronous event
        loops.
        """

        self._dir.value(steps >= 0)
        time_per_step = 1.0 / speed
        for count in range(abs(steps)):
            self._step.value(1)
            time.sleep(time_per_step)
            self._step.value(0)
            time.sleep(time_per_step)

    def deinit(self):
        """Manage resource release as part of object lifecycle."""
        self._dir  = None
        self._step = None

    def __enter__(self):
        return self

    def __exit__(self):
        # Automatically deinitializes the hardware when exiting a context.
        self.deinit()

#--------------------------------- MAIN CODE-----------------------------------------------
stepper = A4988()
print("Starting stepper motor test.")

speed = 200
#Speed of 300 to 900 are good. Above 1000 is shit
while True:
    print(f"Speed: {speed} steps/sec.")
    stepper.move_sync(800, speed)
    time.sleep(1.0)

    stepper.move_sync(-800, speed)
    time.sleep(1.0)

    speed *= 1.1
    if speed > 1000:
        speed = 200