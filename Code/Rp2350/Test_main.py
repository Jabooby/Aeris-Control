import A4988_with_PIO_with_class as A4988
import UART_JSON_Handler as UART_JSON
import BMI160_Handler as BMI160
import AS5600_Handler as AS5600
import time

uart = UART_JSON.UARTJsonHandler()
motor = A4988.StepperControl(sm_id=0, stpPin=14, dirPin=15)

motor.set_step_and_speed(300,300)  # Set initial frequency to 300 Hz
#motor.wait_for_completion()
while motor.inMovement:
    print(motor.state_as_json())
    time.sleep(1)
#print(motor.state_as_json())
motor.set_speed(800)
motor.go_to_position(-400)
#motor.wait_for_completion()
while motor.inMovement:
    print(motor.state_as_json())
    time.sleep(1)        
print("First step sequence complete!")
print(motor.state_as_json())


motor.set_step_and_speed(900,900)  # Set frequency to 900 Hz
#motor.wait_for_completion()
while motor.inMovement:
    time.sleep(1)
print("Second step sequence complete!")

# Reset the motor
motor.reset_motor()
