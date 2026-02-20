//Code used for the project

# File: FinalProjectECE1000
#  Description:
#    Robot claw arm controlled by a joystick
#    - X-axis of the joystick moves the base servo moving horizontally
#    - Y-axis of the joystick moves the arm joint vertically
#    - Joystick button opens/closes the claw
# Authors: Alek Presley, Alex Eisenbeck, Erick Ortiz, Jesse Mofield
# Due Date: 12/9/2025

import machine  
import time    

# ===========================================================================
# Joystick axes
# ===========================================================================

x_Joystick = machine.ADC(machine.Pin(26))   # horizontal
y_Joystick = machine.ADC(machine.Pin(27))   # vertical

# Joystick button
#   Not pressed = reads 1
#   Pressed     = reads 0

JSW = machine.Pin(16, machine.Pin.IN, machine.Pin.PULL_UP)

# Servos:
#   1) Base servo  = rotates arm left/right
#   2) Y1  =  arm joints that move up/down
#   3) Claw servo  = open/close robot claw

servoBase = machine.PWM(machine.Pin(15))   # base rotation
servoY1   = machine.PWM(machine.Pin(14))   # arm joint 1
servoClaw = machine.PWM(machine.Pin(11))   # claw

servos = [servoBase, servoY1, servoClaw]
for s in servos:
    s.freq(50)


MIN_DUTY = 1638    # 0°
MAX_DUTY = 8191    # 180°

# def angle_to_duty(angle_degrees) converts a servo angle in degrees to a 16-bit
# duty value. The function assums a linear relationshit between angle and duty
# cycles over the range 0°-180°.

# angle_degrees (float or int):
#   Desired servo angle in degrees
# returns:
#  int: corresponding 16-bit duty value for duty_u16

def angle_to_duty(angle_degrees):
   
    # Clamp angle to protect the servo
    if angle_degrees < 0:
        angle_degrees = 0
    elif angle_degrees > 180:
        angle_degrees = 180

    duty_span = MAX_DUTY - MIN_DUTY

    # Linear map:
    #   0°   = MIN_DUTY
    #   180° = MAX_DUTY
    duty_value = MIN_DUTY + duty_span * angle_degrees / 180.0

    return int(duty_value)


# Claw positions definitions:
CLAW_OPEN_ANGLE  = 360   # angle where claw is open
CLAW_CLOSE_ANGLE = 0     # angle where claw is closed

# Initial bas and arm joint angles (in degrees)
base_angle = 90
y_angle    = 120

# Move servos to initial positions
servoBase.duty_u16(angle_to_duty(base_angle))
servoY1.duty_u16(angle_to_duty(y_angle))
servoClaw.duty_u16(angle_to_duty(CLAW_OPEN_ANGLE))


time.sleep(0.5)  # allow servos to reach initial position

# ===========================================================================
# Main Control Loop
# ===========================================================================
while True:

    # Read Joystick Values
    x_value = x_Joystick.read_u16()
    y_value = y_Joystick.read_u16()

    # Map joystick readings to angles using a linear calibration model.
    # The constants were obtained by hand
    x_angle_raw = 0.00276 * x_value - 0.75
    y_angle_raw = 0.00276 * y_value - 0.75

    # Convert to integers
    x_angle = int(x_angle_raw)
    y_angle = int(y_angle_raw)

    # Clamp to [0, 180] range
    if x_angle < 0:
        x_angle = 0
    elif x_angle > 180:
        x_angle = 180

    if y_angle < 0:
        y_angle = 0
    elif y_angle > 180:
        y_angle = 180
       
# ===========================================================================      
# Update Servo Positions
# ===========================================================================

    # Base rotation controlled by joystick X-axis.        
    servoBase.duty_u16(angle_to_duty(x_angle))

    # Base rotation controlled by joystick Y-axis.
    servoY1.duty_u16(angle_to_duty(y_angle))

# ===========================================================================
# Claw control via joystick button
# ===========================================================================
    # JSW is active low:
    #   JSW.value() == 0  => button pressed  = close claw
    #   JSW.value() == 1  => button released = open claw
    if JSW.value() == 0:
        # Button pressed = CLOSE claw
        servoClaw.duty_u16(angle_to_duty(CLAW_CLOSE_ANGLE))
    else:
        # Button released = OPEN claw
        servoClaw.duty_u16(angle_to_duty(CLAW_OPEN_ANGLE))
       
# ===========================================================================
# Debug output
# ===========================================================================
   
    print("X_angle:", x_angle,
          " Y_angle:", y_angle,
          " Button:", JSW.value())

    time.sleep(0.1)
