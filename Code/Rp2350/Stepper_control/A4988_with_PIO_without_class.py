import rp2
import time
from machine import Pin

@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def stepper_control_step_freq():
    pull(block)           # Load step count from TX FIFO
    mov(x, osr)           
    pull(block)           # Load delay value from TX FIFO
    label("step_loop")
    mov(y, osr)           # Load delay value from OSR into y
    set(pins, 1)          # Set GPIO high
    label("high_delay")
    nop() [31]
    jmp(y_dec, "high_delay")

    mov(y, osr)
    set(pins, 0)          # Set GPIO low
    label("low_delay")
    nop() [31]
    jmp(y_dec, "low_delay")
    
    jmp(x_dec, "step_loop")  # Continue if x != 0

    irq(0)                   # Raise IRQ to notify CPU when x == 0



def set_step_speed(step, hz):
    """
    Set the period dynamically based on the desired frequency.
    """
    sm.put(step)
    half_period_cycles = int(FREQ / (50 * hz))  # Calculate half-period in clock cycles
    sm.put(half_period_cycles)  # Send the delay value to the PIO state machine

# GPIO pin configuration
PIN_NUM = 14  # GPIO pin to toggle
FREQ = 150_000_000  # RP2350 default clock frequency

# Initialize the state machine
sm = rp2.StateMachine(0, stepper_control_step_freq, freq=FREQ, set_base=Pin(PIN_NUM))

# Start the state machine
sm.active(1)

# Example usage
while(1):
    set_step_speed(300, 300)  # Set initial frequency to 300 Hz
    time.sleep(2)
    set_step_speed(900, 900)
    time.sleep(2)