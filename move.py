#!/usr/bin/env python3
# File name   : move.py
# Description : Refactored motor control library with numeric API
# Author      : William → (2025 refactor)

import time
import RPi.GPIO as GPIO

# -----------------------------------------------------------------------------
# GPIO pin assignments
Motor_A_EN    = 4     # PWM enable for right motor (A)
Motor_B_EN    = 17    # PWM enable for left  motor (B)
Motor_A_Pin1  = 14
Motor_A_Pin2  = 15
Motor_B_Pin1  = 27
Motor_B_Pin2  = 18

# direction constants for motor_left / motor_right
left_forward   = 0
left_backward  = 1
right_forward  = 0
right_backward = 1

# -----------------------------------------------------------------------------
def setup():
    """Initialize GPIO pins & PWM objects."""
    global pwm_A, pwm_B
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    # configure all pins as outputs
    for pin in (Motor_A_EN, Motor_B_EN,
                Motor_A_Pin1, Motor_A_Pin2,
                Motor_B_Pin1, Motor_B_Pin2):
        GPIO.setup(pin, GPIO.OUT)
    motorStop()
    # create PWM channels at 1 kHz
    pwm_A = GPIO.PWM(Motor_A_EN, 1000)
    pwm_B = GPIO.PWM(Motor_B_EN, 1000)

def motorStop():
    """Immediately stop both motors (kill PWM & set all outputs LOW)."""
    for pin in (Motor_A_Pin1, Motor_A_Pin2,
                Motor_B_Pin1, Motor_B_Pin2,
                Motor_A_EN,    Motor_B_EN):
        GPIO.output(pin, GPIO.LOW)

def motor_left(status, direction, speed):
    """
    Low-level left motor driver.
      status    0→stop, 1→run
      direction left_forward or left_backward
      speed     0–100 (duty cycle %)
    """
    if status == 0:
        # off
        GPIO.output(Motor_B_Pin1, GPIO.LOW)
        GPIO.output(Motor_B_Pin2, GPIO.LOW)
        GPIO.output(Motor_B_EN,   GPIO.LOW)
    else:
        # set direction pins
        if direction == left_backward:
            GPIO.output(Motor_B_Pin1, GPIO.HIGH)
            GPIO.output(Motor_B_Pin2, GPIO.LOW)
        else:  # forward
            GPIO.output(Motor_B_Pin1, GPIO.LOW)
            GPIO.output(Motor_B_Pin2, GPIO.HIGH)
        # start PWM at given duty cycle
        pwm_B.start(speed)
        pwm_B.ChangeDutyCycle(speed)

def motor_right(status, direction, speed):
    """
    Low-level right motor driver.
      status    0→stop, 1→run
      direction right_forward or right_backward
      speed     0–100 (duty cycle %)
    """
    if status == 0:
        GPIO.output(Motor_A_Pin1, GPIO.LOW)
        GPIO.output(Motor_A_Pin2, GPIO.LOW)
        GPIO.output(Motor_A_EN,   GPIO.LOW)
    else:
        if direction == right_forward:
            GPIO.output(Motor_A_Pin1, GPIO.HIGH)
            GPIO.output(Motor_A_Pin2, GPIO.LOW)
        else:  # backward
            GPIO.output(Motor_A_Pin1, GPIO.LOW)
            GPIO.output(Motor_A_Pin2, GPIO.HIGH)
        pwm_A.start(speed)
        pwm_A.ChangeDutyCycle(speed)

# -----------------------------------------------------------------------------
def drive(left_speed, right_speed):
    """
    Drive each wheel independently.
      left_speed, right_speed ∈ [–100 … +100]
        >0 → forward at that duty cycle
        <0 → backward at abs(duty cycle)
         0 → stop
    """
    # LEFT MOTOR
    if left_speed == 0:
        motor_left(0, left_forward, 0)
    else:
        dir_flag = left_forward if left_speed > 0 else left_backward
        motor_left(1, dir_flag, abs(left_speed))

    # RIGHT MOTOR
    if right_speed == 0:
        motor_right(0, right_forward, 0)
    else:
        dir_flag = right_forward if right_speed > 0 else right_backward
        motor_right(1, dir_flag, abs(right_speed))

def spin_left(speed):
    """
    Spin in place to the left:
      left wheel  forward @ speed,
      right wheel backward @ speed
    """
    drive(speed, -speed)

def spin_right(speed):
    """
    Spin in place to the right:
      left wheel  backward @ speed,
      right wheel forward @ speed
    """
    drive(-speed, speed)

# -----------------------------------------------------------------------------
def destroy():
    """Cleanup on exit (stop motors & reset GPIO)."""
    motorStop()
    GPIO.cleanup()
