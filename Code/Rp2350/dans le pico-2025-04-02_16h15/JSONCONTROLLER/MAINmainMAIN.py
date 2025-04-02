import A4988_with_PIO_with_class as A4988
import UART_JSON_Handler as UART_JSON
import BMI160_Handler as BMI160
import AS5600_Handler as AS5600
import time
import math
import uasyncio as asyncio
from BMI_and_timeManager import *
from ulab import numpy as np
#import mip
#import array
#import math

STEP_REV = 900
PULLEY_DIA = 36 #mm
CIRCUMFERENCE = math.pi * PULLEY_DIA
DIST_STEP = CIRCUMFERENCE / STEP_REV
L_MAX = 100 #mm Déplacement maximum du fils
ANGLE_M = math.radians(60) #degrees
COS_M = math.cos(ANGLE_M/2)
SIN_M = math.sin(ANGLE_M/2)
THETA_MAX = math.radians(45) #degrees maximum angle between mast and vertical
COS_THETA_MAX = math.cos(THETA_MAX)
H_CRWNST = 0.133775 #height of crows nest from ujoint 

# Shared JSON data. This is the JSON we get from the pi4 for the controller data
JSONData = {}

#this is the JSON we will send to the PI4 that contains the sensor data
DataSending = {}
uart = UART_JSON.UARTJsonHandler()
motorZ = A4988.StepperControl(sm_id=0, stpPin=12, dirPin=13)
motorR = A4988.StepperControl(sm_id=1, stpPin=16, dirPin=17)
motorL = A4988.StepperControl(sm_id=4, stpPin=14, dirPin=15)

# Create IMU Sensor object + Calibration
imu_0 = IMUSensor(i2c_0)

imu_0.auto_calibrate()

time_manager = TimeManager()

# Initialize time tracking variable
sum_time_delta = 0.0

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

def calculateNextPosition(newX, newY, currentX, currentY):
    return None

async def test_read():
    global JSONData  # Ensure we modify the shared JSONData
    while True:
        JSONData = await uart.read_json()  # Update global variable
        await asyncio.sleep(0.0001)

async def motorZMOVE():
    global JSONData  # Ensure we read from the shared JSONData
    stepSizeZ = 50
    
    positionL = 0
    positionR = 0
    
    stepZ = 0
    speedZ = 0
    speedDefaultVal = 900
    motorL.set_speed(speedDefaultVal)
    motorR.set_speed(speedDefaultVal)
    while True:
        time_manager.update()
        imu_0.update(time_manager.get_time_delta())
        accx, accy, accz = imu_0.accx, imu_0.accy, imu_0.accz
        roll, pitch, yaw = imu_0.roll, imu_0.pitch, imu_0.yaw
        
        
        print("data sent")
        print(f"roll: {roll:.3f}, yaw: {yaw:.3f}, pitch: {pitch:.3f}")
        update_json_file(roll, pitch, yaw)
        #time.sleep(0.5)
        #print(JSONData)
        #JSON arrive avec des valeurs variant entre -1 à 1 pour le Joystick
        
        #print(JSONData["ZRight"])
        #if JSONData.get("ZRight"):  # Use .get() to avoid KeyError
        #    motorZ.set_step_and_speedZ(100*int((JSONData["axeX"]*10)), 450)  
        #else:
        #    motorZ.set_step_and_speedZ(0, 450)
        #Gestion du moteur en Z
        if (JSONData.get("ZRight") and not JSONData.get("ZLeft")):
            stepZ = stepSizeZ
            speedZ = speedDefaultVal
        elif (JSONData.get("ZLeft") and not (JSONData.get("ZRight"))):
            stepZ = -stepSizeZ
            speedZ = speedDefaultVal
        elif (JSONData.get("ZRight") and JSONData.get("ZLeft")):
            stepZ = 0
        else:
            stepZ = 0
            speedZ = 1
        
        #Gestion XY
        JoystickX = JSONData.get("axeX",0)
        JoystickY = -(JSONData.get("axeY",0))
        print("JoystickX value: ", JoystickX)
        print("JoystickY value: ", JoystickY)
        newX = JoystickX*H_CRWNST*COS_THETA_MAX
        newY = JoystickY*H_CRWNST*COS_THETA_MAX
        currentXY= currentPosition(roll, pitch, yaw)
        currentX = currentXY[0] #accelerometer
        currentY = currentXY[1] #accelerometer
        deltaX = newX-currentX
        deltaY = (newY-currentY)

                #define newposition as a linear combination of vectors p and q that represent right and left motors effect
        #define left-hand side of equation
        left_side = np.array([[H_CRWNST*COS_M, H_CRWNST*COS_M], [H_CRWNST*SIN_M, -H_CRWNST*SIN_M]])

        #define right-hand side of equation
        right_side = np.array([deltaX, deltaY])

        #solve for m and n
        MN = np.linalg.inv(left_side)
        MN = np.dot(MN,right_side)
        
        #lin. comb. ratio for the right motor
        M = MN[0][0]
        #lin. comb. ratio for the left motor
        N = MN[1][0]
        #norm of MN
        normMN = math.sqrt(pow(M, 2) + pow(N, 2))
        #motor ratio to get the proper velocity
        motR = M/normMN
        motL = N/normMN
        
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
        await uart.send_json(DataSending)
        #moteur r va a current m/ new m 
        #moteur l va a current n/ new n


        positionL = round((((JoystickX/COS_M)+(JoystickY/SIN_M))*L_MAX)/DIST_STEP)
        positionR = round(((-(JoystickX/COS_M)+(JoystickY/SIN_M))*L_MAX)/DIST_STEP)
        
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
        # Run both tasks simultaneously
        await asyncio.gather(test_read(), motorZMOVE())

    # Run the main coroutine
    asyncio.run(main())

    # Reset the motorZ
    motorZ.reset_motorZ()
