import rp2
import time
from machine import Pin
import ujson
from _thread import allocate_lock

irq_lock = allocate_lock()


FORWARD = 1
BACKWARD = 0

class StepperControl:
    
    def wait_for_completion(self, sm_irq_state):
        """
        Poll the IRQ flag to ensure the state machine has completed its current operation.
        """
        with irq_lock: #prevents another irq to be activated 
            x = self.sm.irq().flags()
            print("IRQ flag status:", x)
            self.inMovement = False
            self.currentStepValue = self.desiredStepValue
        """
        Things I learned: irq is cleared automatically when leaving the handler.
        The print won't work outside the handler and will always show 0
        More info: https://github.com/micropython/micropython/wiki/Hardware-API#irqs 
        """
     

    @rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
    def stepper_control_step_freq_sm0():
        pull(block)           # Load step count from TX FIFO
        mov(x, osr)           # x contains the number of steps
        pull(block)           # Load delay value from TX FIFO
        
        label("step_loop")
        #y contains the 1/2 of the wanted period
        mov(y, osr)           # Load delay value from OSR into y
        set(pins, 1)          # Set GPIO high
        label("high_delay")
        nop() [31]
        jmp(y_dec, "high_delay")

        mov(y, osr)           # Load delay value from OSR into y
        set(pins, 0)          # Set GPIO low
        label("low_delay")
        nop() [31]
        jmp(y_dec, "low_delay")
        
        jmp(x_dec, "step_loop")  # Continue if x != 0

        irq(0)                   # Raise IRQ to notify CPU when x == 0
        #irq(n) n needs to be equal to the 2^sm_id 
        
    @rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
    def stepper_control_step_freq_sm1():
        pull(block)           # Load step count from TX FIFO
        mov(x, osr)           # x contains the number of steps
        pull(block)           # Load delay value from TX FIFO
        
        label("step_loop")
        #y contains the 1/2 of the wanted period
        mov(y, osr)           # Load delay value from OSR into y
        set(pins, 1)          # Set GPIO high
        label("high_delay")
        nop() [31]
        jmp(y_dec, "high_delay")

        mov(y, osr)           # Load delay value from OSR into y
        set(pins, 0)          # Set GPIO low
        label("low_delay")
        nop() [31]
        jmp(y_dec, "low_delay")
        
        jmp(x_dec, "step_loop")  # Continue if x != 0

        irq(1)                   # Raise IRQ to notify CPU when x == 0
        #irq(n) n needs to be equal to the 2^sm_id 
        
    @rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
    def stepper_control_step_freq_sm4():
        pull(block)           # Load step count from TX FIFO
        mov(x, osr)           # x contains the number of steps
        pull(block)           # Load delay value from TX FIFO
        
        label("step_loop")
        #y contains the 1/2 of the wanted period
        mov(y, osr)           # Load delay value from OSR into y
        set(pins, 1)          # Set GPIO high
        label("high_delay")
        nop() [31]
        jmp(y_dec, "high_delay")

        mov(y, osr)           # Load delay value from OSR into y
        set(pins, 0)          # Set GPIO low
        label("low_delay")
        nop() [31]
        jmp(y_dec, "low_delay")
        
        jmp(x_dec, "step_loop")  # Continue if x != 0

        irq(16)                   # Raise IRQ to notify CPU when x == 0
        #irq(n) n needs to be equal to the 2^sm_id 

    def set_step_and_speed(self, step, hz):
        """
        Set the period dynamically based on the desired frequency.
        And sends the amount of steps wanted.
        """
        self.set_speed(hz)
        self.set_step(step)
        
    def set_speed(self, hz):
        """
        Set the period dynamically based on the desired frequency.
        """
        self.speed = hz
        
    def set_direction(self, dir):
        """
        Set the direction pin according to wanted direction
        """
        self.dirPin.value(dir)
        
    def set_step(self, step):
        """
        Set the direction according to the value step received.
        Sends the number of steps wanted to the SM with the required speed.
        Stores all necessary data for logging later.
        """
        if(step < 0):
            self.set_direction(BACKWARD)#negative value goes backward
        else:
            self.set_direction(FORWARD) #positive value goas forward
        self.sm.put(abs(step))          #Send the number of steps to the SM
        self.desiredStepValue += step   #Store the desired position 
        #Better way to do this is to do the math whenever the speed is changed and store it
        half_period_cycles = int(self.clock_freq / (50 * self.speed))  # Calculate half-period in clock cycles
        self.sm.put(half_period_cycles) #Send the delay value to the PIO state machine
        self.inMovement = True          #SM is occupied and should not be communicated with till it is done. Not yet implemented...
    
    def reset_motor(self):
        """
        Reset the motor by reactivating the state machine and clearing any previous settings.
        """
        self.stop_motor()
        time.sleep(0.1)  # Short delay to ensure state machine stops
        self.sm.active(1)  # Reactivate the state machine
        self.desiredStepValue = 0
        self.currentStepValue = 0
        print("Motor reset.")

    def stop_motor(self):
        """
        Stop the motor. More specifically, stops the state machine.
        """
        self.sm.active(0)  # Stop the state machine
        
    def start_motor(self):
        """
        Start the motor. More specifically, starts the state machine.
        """
        self.sm.active(1)  # Stop the state machine
        
    def change_pin(self, stpPin, dirPin):
        """
        Change the GPIO pin used by the state machine.
        """
        self.stpPin = Pin(stpPin, Pin.OUT)
        self.dirPin = Pin(dirPin, Pin.OUT)
        
        self.stop_motor()
        self.sm = rp2.StateMachine(sm_id, self.stepper_control_step_freq, freq=self.clock_freq, set_base=self.stpPin)
        self.start_motor()
        
        print(f"GPIO pin changed to {self.pin}.")
        
    def state_as_json(self):
        """
        Return the state of the instance variables in JSON format.
        """
        state = {
            "stpPin": self.stpPin,
            "dirPin": self.dirPin,
            "clock_freq": self.clock_freq,
            "inMovement": self.inMovement,
            "desiredStepValue": self.desiredStepValue,
            "currentStepValue": self.currentStepValue,
            "speed": self.speed,
            "state_machine_id": self.sm_id,
            "irq_status": self.sm.irq().flags() if self.sm else None
        }
        
        return state
        
    def __init__(self, sm_id, stpPin, dirPin, clock_freq=150_000_000):
        self.stpPin = Pin(stpPin, Pin.OUT)
        self.dirPin = Pin(dirPin, Pin.OUT)
        self.clock_freq = clock_freq
        
        self.sm_id = sm_id
        if sm_id == 0:
            self.sm = rp2.StateMachine(
            sm_id,
            self.stepper_control_step_freq_sm0,
            freq=self.clock_freq,
            set_base=self.stpPin,
            )
        elif sm_id == 1:
            self.sm = rp2.StateMachine(
                sm_id,
                self.stepper_control_step_freq_sm1,
                freq=self.clock_freq,
                set_base=self.stpPin,
                )
        else:
             self.sm = rp2.StateMachine(
                sm_id,
                self.stepper_control_step_freq_sm4,
                freq=self.clock_freq,
                set_base=self.stpPin,
                )
        
        self.start_motor()
        self.sm.irq(handler = self.wait_for_completion)
        
        self.dir = FORWARD
        self.set_direction(dir)
        
        self.inMovement = False
        self.speed = 0
        self.desiredStepValue = 0
        self.currentStepValue =  0
        
# Example usage:
if __name__ == "__main__":
    motor = StepperControl(sm_id=4, stpPin=14, dirPin=15)

    motor.set_step_and_speed(300,300)  # Set initial frequency to 300 Hz
    #motor.wait_for_completion()
    while motor.inMovement:
        print(motor.state_as_json())
        time.sleep(1)
    #print(motor.state_as_json())
    motor.set_speed(400)
    motor.set_step(-400)
    #motor.wait_for_completion()
    while motor.inMovement:
        print(motor.state_as_json())
        time.sleep(1)        
    print("First step sequence complete!")
    
    
    motor.set_step_and_speed(900,900)  # Set frequency to 900 Hz
    #motor.wait_for_completion()
    while motor.inMovement:
        time.sleep(1)
    print("Second step sequence complete!")

    # Reset the motor
    motor.reset_motor()

