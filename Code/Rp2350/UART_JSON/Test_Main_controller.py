import A4988_with_PIO_with_class as A4988
import UART_JSON_Handler as UART_JSON
import BMI160_Handler as BMI160
import AS5600_Handler as AS5600
import time
import math
import uasyncio as asyncio

STEP_REV = 200
PULLEY_DIA = 36 #mm
CIRCUMFERENCE = math.pi * PULLEY_DIA
DIST_STEP = CIRCUMFERENCE / STEP_REV
L_MAX = 100 #mm Déplacement maximum du fils
ANGLE_M = math.radians(60) #degrees
COS_M = math.cos(ANGLE_M/2)
SIN_M = math.sin(ANGLE_M/2)

# Shared JSON data
JSONData = {}

uart = UART_JSON.UARTJsonHandler()
motorZ = A4988.StepperControl(sm_id=0, stpPin=17, dirPin=16)
motorR = A4988.StepperControl(sm_id=1, stpPin=14, dirPin=15)
motorL = A4988.StepperControl(sm_id=4, stpPin=12, dirPin=13)

async def test_read():
    global JSONData  # Ensure we modify the shared JSONData
    while True:
        JSONData = await uart.read_json()  # Update global variable
        await asyncio.sleep(0.0001)

async def motorZMOVE():
    global JSONData  # Ensure we read from the shared JSONData
    stepSize = 50
    
    positionL = 0
    posMaxL = 1000
    positionR = 0
    posMaxR = 1000
    
    stepZ = 0
    speedZ = 0
    speedDefaultVal = 450
    motorL.set_speed(speedDefaultVal)
    motorR.set_speed(speedDefaultVal)
    while True:
        print(JSONData)
        #JSON arrive avec des valeurs variant entre -1 à 1 pour le Joystick
        
        #print(JSONData["ZRight"])
        #if JSONData.get("ZRight"):  # Use .get() to avoid KeyError
        #    motorZ.set_step_and_speedZ(100*int((JSONData["axeX"]*10)), 450)  
        #else:
        #    motorZ.set_step_and_speedZ(0, 450)
        
        #Gestion du moteur en Z
        if (JSONData.get("ZRight") and not JSONData.get("ZLeft")):
            stepZ = stepSize
            speedZ = speedDefaultVal
        elif (JSONData.get("ZLeft") and not (JSONData.get("ZRight"))):
            stepZ = -stepSize
            speedZ = speedDefaultVal
        elif (JSONData.get("ZRight") and JSONData.get("ZLeft")):
            stepZ = 0
        else:
            stepZ = 0
            speedZ = 1
        
        #Gestion XY
        JoystickX = JSONData.get("axeX")
        JoystickY = JSONData.get("axeY")
        
        positionL = round((((JoystickY/SIN_M)+(JoystickX/COS_M))*L_MAX)/DIST_STEP)
        positionR = round((((JoystickY/SIN_M)-(JoystickX/COS_M))*L_MAX)/DIST_STEP)
        
        
        motorZ.set_step_and_speedZ(stepZ, speedZ)
        motorL.go_to_position(positionL)
        motorR.go_to_position(positionLR)
        
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
