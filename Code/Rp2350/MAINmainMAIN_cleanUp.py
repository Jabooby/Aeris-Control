import time
import math
import uasyncio as asyncio
from ulab import numpy as np

# Import custom modules
import A4988_with_PIO_with_class as A4988
import UART_JSON_Handler as UART_JSON
import BMI160_Handler as BMI160
import AS5600_Handler as AS5600
from BMI_and_timeManager import *

# ========================
# Constants and Parameters
# ========================
STEP_REV = 900  # Steps per revolution
PULLEY_DIA = 36  # mm (Pulley Diameter)
CIRCUMFERENCE = math.pi * PULLEY_DIA  # Pulley circumference
DIST_STEP = CIRCUMFERENCE / STEP_REV  # Distance per step
L_MAX = 100  # mm (Maximum displacement of the wire)
DEFAULT_SPEED = 900

# Angle and Trigonometric Values
ANGLE_M = math.radians(60)  # Mast rotation angle in degrees
COS_M = math.cos(ANGLE_M / 2)
SIN_M = math.sin(ANGLE_M / 2)
THETA_MAX = math.radians(45)  # Max angle between mast and vertical
COS_THETA_MAX = math.cos(THETA_MAX)

# Structural Measurement
H_CRWNST = 0.133775  # Height of crow's nest from u-joint (meters)

# =========================
# JSON Data for Communication
# =========================
JSONData = {}  # Received JSON from Raspberry Pi 4 (controller data)
DataSending = {}  # JSON to send to Raspberry Pi 4 (sensor data)

# ==================
# UART Communication
# ==================
uart = None

# ===================
# Stepper Motor Setup
# ===================
motorZ = None
motorR = None
motorL = None

# ==================
# IMU Sensor Setup
# ==================
imu_0 = None

# ==========================
# Time Management and Tracking
# ==========================
time_manager = None
sum_time_delta = 0.0  # Initialize time tracking variable

def init():
    global motorZ, motorR, motorL, imu_0, uart, time_manager
    
    # Initialize UART communication
    uart = UART_JSON.UARTJsonHandler()
    
    # Initialize stepper motors
    motorZ = A4988.StepperControl(sm_id=0, stpPin=12, dirPin=13)
    motorR = A4988.StepperControl(sm_id=1, stpPin=16, dirPin=17)
    motorL = A4988.StepperControl(sm_id=4, stpPin=14, dirPin=15)

    # Initialize IMU Sensor and calibrate
    imu_0 = IMUSensor(i2c_0)
    imu_0.auto_calibrate()
    time.sleep(0.5)

    # Initialize time manager
    time_manager = TimeManager()
    

def currentPosition(roll, pitch, yaw):
    # Yaw (about Z-axis (up))
    c_R_b = np.array([
        [np.cos(yaw), np.sin(yaw), 0],
        [-np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ], dtype=np.float)
    # Pitch (about X-axis (east))
    b_R_a = np.array([
        [1, 0, 0],
        [0, np.cos(pitch), -np.sin(pitch)],
        [0, np.sin(pitch), np.cos(pitch)]
    ], dtype=np.float)

    # Roll (about Y-axis (north))
    a_R_n = np.array([
        [np.cos(roll), 0, np.sin(roll)],
        [0, 1, 0],
        [-np.sin(roll), 0, np.cos(roll)]
    ], dtype=np.float)
    # ENU rotation matrix
    c_R_n = np.dot(c_R_b, np.dot(b_R_a, a_R_n))
    a = np.array([[0],[0],[H_CRWNST]])
    currentPosition = np.dot(c_R_n, a)
    return currentPosition

async def calculateNextPosition(JSONData):
    imu_0.update(time_manager.get_time_delta())
    accx, accy, accz = imu_0.accx, imu_0.accy, imu_0.accz
    roll, pitch, yaw = imu_0.roll, imu_0.pitch, imu_0.yaw
    
    # Gestion XY
    JoystickX = JSONData.get("axeX", 0)
    JoystickY = -(JSONData.get("axeY", 0))
    print("JoystickX value: ", JoystickX)
    print("JoystickY value: ", JoystickY)
    
    newX = JoystickX * H_CRWNST * COS_THETA_MAX
    newY = JoystickY * H_CRWNST * COS_THETA_MAX
    currentXY = currentPosition(roll, pitch, yaw)
    currentX = currentXY[0]  # Accelerometer
    currentY = currentXY[1]  # Accelerometer
    deltaX = newX - currentX
    deltaY = newY - currentY

    # Define new position as a linear combination of vectors p and q
    left_side = np.array([[H_CRWNST * COS_M, H_CRWNST * COS_M], 
                          [H_CRWNST * SIN_M, -H_CRWNST * SIN_M]])
    right_side = np.array([deltaX, deltaY])

    # Solve for m and n
    MN = np.linalg.inv(left_side)
    MN = np.dot(MN, right_side)

    # Ensure MN is a 1D array
    MN = MN.flatten()  # Converts any shape into a 1D array

    # Extract scalars correctly
    M = float(MN[0])  # Now guaranteed to be a scalar
    N = float(MN[1])  # Now guaranteed to be a scalar

    # Compute norm safely
    normMN = math.sqrt(M**2 + N**2)

    motR = M / normMN
    motL = N / normMN

    DataSending = {
        "Yaw": yaw,
        "Pitch": pitch,
        "Roll": roll,
        "accx": accx,
        "accy": accy,
        "accz": accz,
        "motR": motR,
        "motL": motL
    }

    await uart.send_json(DataSending)  # Awaiting the async call

    positionL = round((((JoystickX / COS_M) + (JoystickY / SIN_M)) * L_MAX) / DIST_STEP)
    positionR = round(((-(JoystickX / COS_M) + (JoystickY / SIN_M)) * L_MAX) / DIST_STEP)

    return positionL, positionR


async def read_UART_JSON():
    global JSONData  # Ensure we modify the shared JSONData
    while True:
        JSONData = await uart.read_json()  # Update global variable
        await asyncio.sleep(0.0001)

async def motorMOVE():
    global JSONData  # Ensure we read from the shared JSONData
    stepSizeZ = 50
    
    positionL = 0
    positionR = 0
    
    stepZ = 0
    speedZ = 0

    motorL.set_speed(DEFAULT_SPEED)
    motorR.set_speed(DEFAULT_SPEED)
    
    while True:
        time_manager.update()
        
        #Gestion du moteur en Z
        if (JSONData.get("ZRight") and not JSONData.get("ZLeft")):
            stepZ = stepSizeZ
            speedZ = DEFAULT_SPEED
        elif (JSONData.get("ZLeft") and not (JSONData.get("ZRight"))):
            stepZ = -stepSizeZ
            speedZ = DEFAULT_SPEED
        elif (JSONData.get("ZRight") and JSONData.get("ZLeft")):
            stepZ = 0
        else:
            stepZ = 0
            speedZ = 1
        #Gestion des moteurs XY (gauche et droite)    
        positionL, positionR = await calculateNextPosition(JSONData)
            
        print("Motor LEFT wanted position:", positionL)
        print("Motor RIGHT wanted position:", positionR)
        print("Motor Z wanted position:", stepZ)
        
        
        motorZ.set_step_and_speed(stepZ, speedZ)
        motorL.go_to_position(positionL)
        print(motorL.state_as_json())
        motorR.go_to_position(positionR)
        print(motorR.state_as_json())
        
        while motorZ.inMovement:
            await asyncio.sleep(0.0001)  # Use asyncio sleep to avoid blocking

        #print("Second step sequence complete!")
        #await asyncio.sleep(0.01)
# Example usage:
if __name__ == "__main__":
    async def main():
        init()
        # Run both tasks simultaneously
        await asyncio.gather(read_UART_JSON(), motorMOVE())

    # Run the main coroutine
    asyncio.run(main())

    # Reset the motorZ
    motorZ.reset_motorZ()

