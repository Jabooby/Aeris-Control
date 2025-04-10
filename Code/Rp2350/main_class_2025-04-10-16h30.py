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
DEFAULT_STEP = 50

# Angle and Trigonometric Values
ANGLE_M = math.radians(60)  # Mast rotation angle in degrees
COS_M = math.cos(ANGLE_M / 2)
SIN_M = math.sin(ANGLE_M / 2)
THETA_MAX = math.radians(45)  # Max angle between mast and vertical
COS_THETA_MAX = math.cos(THETA_MAX)

# Structural Measurement
H_CRWNST = 0.133775  # Height of crow's nest from u-joint (meters)


class MotorController:
    def __init__(self):
        """ Initialize motors, sensors, and communication. """
        self.JSONData = {}
        self.DataSending = {}

        # UART Communication
        self.uart = UART_JSON.UARTJsonHandler()

        # Initialize Stepper Motors
        self.motorZ = A4988.StepperControl(sm_id=0, stpPin=12, dirPin=13)
        self.motorR = A4988.StepperControl(sm_id=1, stpPin=16, dirPin=17)
        self.motorL = A4988.StepperControl(sm_id=4, stpPin=14, dirPin=15)

        # IMU Sensor
        self.imu_0 = IMUSensor(i2c_0)
        self.imu_0.auto_calibrate()
        
        # Array creation optimization
        self.left_side_inv  = np.linalg.inv(np.array([[H_CRWNST * COS_M, H_CRWNST * COS_M], 
                              [H_CRWNST * SIN_M, -H_CRWNST * SIN_M]]))

        # Time Management
        self.time_manager = TimeManager()
        time.sleep(0.5)  # Allow time for calibration
        

    async def read_UART_JSON(self):
        while True:
            try:
                #print("Waiting on data...")
                self.JSONData = await self.uart.read_json()
                print("✔️ JSON Received and stored:", self.JSONData)
            except Exception as e:
                print(f"❌ UART read error: {e}")
            await asyncio.sleep(0.1)



    def currentPosition(self, roll, pitch, yaw):
        """ Compute the current mast position based on IMU data. """
        c_R_b = np.array([
            [np.cos(yaw), np.sin(yaw), 0],
            [-np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ], dtype=np.float)

        b_R_a = np.array([
            [1, 0, 0],
            [0, np.cos(pitch), -np.sin(pitch)],
            [0, np.sin(pitch), np.cos(pitch)]
        ], dtype=np.float)

        a_R_n = np.array([
            [np.cos(roll), 0, np.sin(roll)],
            [0, 1, 0],
            [-np.sin(roll), 0, np.cos(roll)]
        ], dtype=np.float)

        c_R_n = np.dot(c_R_b, np.dot(b_R_a, a_R_n))
        return np.dot(c_R_n, np.array([[0], [0], [H_CRWNST]]))

    async def calculateNextPosition(self):
        """ Compute the next position for the motors based on joystick input. """
        self.imu_0.update(self.time_manager.get_time_delta())
        roll, pitch, yaw = self.imu_0.roll, self.imu_0.pitch, self.imu_0.yaw

        JoystickX = self.JSONData.get("axeX", 0)
        JoystickY = -self.JSONData.get("axeY", 0)

        newX = JoystickX * H_CRWNST * COS_THETA_MAX
        newY = JoystickY * H_CRWNST * COS_THETA_MAX

        currentXY = self.currentPosition(roll, pitch, yaw)
        currentX = currentXY[0]  # Accelerometer
        currentY = currentXY[1]  # Accelerometer
        deltaX = newX - currentX
        deltaY = newY - currentY

        MN = np.dot(self.left_side_inv, np.array([deltaX, deltaY]))

        # Ensure MN is a 1D array
        MN = MN.flatten()  # Converts any shape into a 1D array

        # Extract scalars correctly
        M = float(MN[0])  # Now guaranteed to be a scalar
        N = float(MN[1])  # Now guaranteed to be a scalar
        normMN = math.sqrt(M**2 + N**2)
        motR, motL = M / normMN, N / normMN

        self.DataSending = {
            "Yaw": yaw, "Pitch": pitch, "Roll": roll,
            "motR": motR, "motL": motL,
            "accx": self.imu_0.accx, "accy": self.imu_0.accy, "accz": self.imu_0.accz
        }
        await self.uart.send_json(self.DataSending)

        positionL = round((((JoystickX / COS_M) + (JoystickY / SIN_M)) * L_MAX) / DIST_STEP)
        positionR = round(((-(JoystickX / COS_M) + (JoystickY / SIN_M)) * L_MAX) / DIST_STEP)

        return positionL, positionR
    
    async def motorZManual(self):
        try:
            speedZ = self.JSONData.get("speedZ", 0)
            stepZ = self.JSONData.get("axeZ", 0)
        except:
            print("Variable speedZ was not received...")
            speedZ = DEFAULT_SPEED
            stepZ = 0

        stepZ = stepZ*DEFAULT_STEP
        #print("Z axis values. step, speed", stepZ, speedZ)
        return stepZ, speedZ
        
    def resetAll(self):
        # Reset Stepper Motors
        self.motorZ.reset_motor()
        self.motorR.reset_motor()
        self.motorL.reset_motor()
        #Reset/Calibrate IMU
        self.imu_0.auto_calibrate()
            #imu_1
        #Reset/Calibrate encoder

    async def motorMOVE(self):
        """ Controls motors based on joystick input. """
        positionL, positionR = 0, 0
        stepZ, speedZ = 0, 0

        self.motorL.set_speed(DEFAULT_SPEED)
        self.motorR.set_speed(DEFAULT_SPEED)

        while True:
            #Time
            self.time_manager.update()

            #Reset control
            try:
                if self.JSONData.get("Calib"):
                    self.resetAll()
                    print("Reset has been done")
            except:
                print("No reset has been received... Keeping the configuration as is...")
            
            #Moteur control
            try:
                #Manual
                if self.JSONData.get("Automatic") == False:
                    stepZ, speedZ = await self.motorZManual()
                    positionL, positionR = await self.calculateNextPosition()
                    speedL, speedR = self.JSONData.get("speedLR"), self.JSONData.get("speedLR")
                #Automatic
                else:
                    print("Mode automatique")
            except:
                print("No automatic or manual JSON receivded... Keeping the configuration as is...")
                stepZ, speedZ = (0,0)
                positionL, positionR = (0,0)
                
            self.motorZ.set_step_and_speed(stepZ, speedZ)
            self.motorL.go_to_position(positionL)
            self.motorR.go_to_position(positionR)
            
            #Print system
            #print("Motor LEFT wanted position:", positionL)
            #print("Motor RIGHT wanted position:", positionR)
            #print("Motor Z wanted position:", stepZ)

            await asyncio.sleep(0)
            #Delay for other task to take the thread
            while self.motorZ.inMovement:
                await asyncio.sleep(0.1)

    async def main(self):
        """ Run both tasks simultaneously. """
        await asyncio.gather(self.read_UART_JSON(), self.motorMOVE())

    def run(self):
        """ Start the controller. """
        asyncio.run(self.main())
        
    
        
        
        

# ========== MAIN EXECUTION ==========
if __name__ == "__main__":
    controller = MotorController()
    controller.run()
    

