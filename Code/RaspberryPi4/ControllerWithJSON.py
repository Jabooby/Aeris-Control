import pygame
import time
import json
import serial

#me when serial
#ser = serial.Serial('/dev/ttyACM0',115200,timeout = 2)
#ser.flushInput()
# Initialize pygame
pygame.init()

# Initialize the joystick module
pygame.joystick.init()

# Check if there are any joysticks connected
if pygame.joystick.get_count() == 0:
    print("No joystick/ controller detected.")
    pygame.quit()
    exit()

# Get the first joystick (PS3 controller is usually the first one)
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Print the name of the controller
print("Controller found: ", joystick.get_name())

# Function to display the state of buttons and axes
def test_controller():
    try:
        while True:
            # Process events
            pygame.event.pump()
            
            # Display the state of the buttons
            JoystickButtons = {
                "Manual": joystick.get_button(3),
                "ZLeft": joystick.get_button(6),
                "ZRight": joystick.get_button(7)
                }
            JoystickButtonJson = json.dumps(JoystickButtons)
            print(JoystickButtonJson)
            # Display the state of the axes (joystick movements)
            JoystickAxe = {
                "axeX": round(joystick.get_axis(0),2),
                "axeY": round(-joystick.get_axis(1),2),
                }
            JoystickAxeJson = json.dumps(JoystickAxe)
            print(JoystickAxeJson)
            #time.sleep(1)
            #ser.write(JoystickButtonJson)
            #time.sleep(1)
            #ser.write(JoystickAxeJson)
    except KeyboardInterrupt:
        print("\nExiting the controller test.")
        pygame.quit()

# Run the test
test_controller()

