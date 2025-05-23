#!/usr/bin/env python3
# File name   : move.py
# Description : Motor control library
# Author      : William (with refactor)
# Date        : 2019/07/24 → 2025/05/23

import time
import RPi.GPIO as GPIO

# GPIO pins
Motor_A_EN    = 4
Motor_B_EN    = 17
Motor_A_Pin1  = 14
Motor_A_Pin2  = 15
Motor_B_Pin1  = 27
Motor_B_Pin2  = 18

# directions
Dir_forward   = 0
Dir_backward  = 1
left_forward  = 0
left_backward = 1
right_forward = 0
right_backward= 1

# ----------------------------------------------------------------------------
def setup():
    """Initialize GPIO & PWM objects."""
    global pwm_A, pwm_B
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    for pin in (Motor_A_EN, Motor_B_EN,
                Motor_A_Pin1, Motor_A_Pin2,
                Motor_B_Pin1, Motor_B_Pin2):
        GPIO.setup(pin, GPIO.OUT)
    motorStop()
    pwm_A = GPIO.PWM(Motor_A_EN, 1000)
    pwm_B = GPIO.PWM(Motor_B_EN, 1000)


def motorStop():
    """Stop both motors immediately."""
    for pin in (Motor_A_Pin1, Motor_A_Pin2, Motor_B_Pin1, Motor_B_Pin2,
                Motor_A_EN, Motor_B_EN):
        GPIO.output(pin, GPIO.LOW)


def motor_left(status, direction, speed):
    """Control left motor (B)."""
    if status == 0:
        GPIO.output(Motor_B_Pin1, GPIO.LOW)
        GPIO.output(Motor_B_Pin2, GPIO.LOW)
        GPIO.output(Motor_B_EN,   GPIO.LOW)
    else:
        if direction == Dir_backward:
            GPIO.output(Motor_B_Pin1, GPIO.HIGH)
            GPIO.output(Motor_B_Pin2, GPIO.LOW)
        else:  # forward
            GPIO.output(Motor_B_Pin1, GPIO.LOW)
            GPIO.output(Motor_B_Pin2, GPIO.HIGH)
        pwm_B.start(speed)
        pwm_B.ChangeDutyCycle(speed)


def motor_right(status, direction, speed):
    """Control right motor (A)."""
    if status == 0:
        GPIO.output(Motor_A_Pin1, GPIO.LOW)
        GPIO.output(Motor_A_Pin2, GPIO.LOW)
        GPIO.output(Motor_A_EN,   GPIO.LOW)
    else:
        if direction == Dir_forward:
            GPIO.output(Motor_A_Pin1, GPIO.HIGH)
            GPIO.output(Motor_A_Pin2, GPIO.LOW)
        else:  # backward
            GPIO.output(Motor_A_Pin1, GPIO.LOW)
            GPIO.output(Motor_A_Pin2, GPIO.HIGH)
        pwm_A.start(speed)
        pwm_A.ChangeDutyCycle(speed)


def move(speed, direction, turn, radius=0.6):
    """
    Generic move:
      - direction ∈ {'forward','backward','no'}
      - turn      ∈ {'left','right','no'}
      - radius    ∈ (0,1]
    """
    if direction == 'forward':
        if turn == 'right':
            motor_left (0, left_backward, int(speed*radius))
            motor_right(1, right_forward, speed)
        elif turn == 'left':
            motor_left (1, left_forward, speed)
            motor_right(0, right_backward, int(speed*radius))
        else:
            motor_left (1, left_forward, speed)
            motor_right(1, right_forward, speed)

    elif direction == 'backward':
        if turn == 'right':
            motor_left (0, left_forward, int(speed*radius))
            motor_right(1, right_backward, speed)
        elif turn == 'left':
            motor_left (1, left_backward, speed)
            motor_right(0, right_forward, int(speed*radius))
        else:
            motor_left (1, left_backward, speed)
            motor_right(1, right_backward, speed)

    elif direction == 'no':
        if turn == 'right':
            motor_left (1, left_backward, speed)
            motor_right(1, right_forward, speed)
        elif turn == 'left':
            motor_left (1, left_forward, speed)
            motor_right(1, right_backward, speed)
        else:
            motorStop()


def destroy():
    """Cleanup GPIO on exit."""
    motorStop()
    GPIO.cleanup()
