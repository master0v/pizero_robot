#!/usr/bin/python3
# File name   : Ultrasonic.py
# Description : Detection distance and tracking with ultrasonic
# Website     : www.gewbot.com
# Author      : William
# Date        : 2019/02/23
import RPi.GPIO as GPIO
import time

Tr = 11 # transmitter attached to pin 11?
Ec = 8  # receiver attache to pin 8


def initUltrasonicSensor():
  #GPIO.setwarnings(False)
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(Tr, GPIO.OUT,initial=GPIO.LOW)
  GPIO.setup(Ec, GPIO.IN)


#Read the distance using ultrasonic time of flight
def getDistance():
    
  initUltrasonicSensor()
  
  # send a pulse
  GPIO.output(Tr, GPIO.HIGH)
  time.sleep(0.000015)
  GPIO.output(Tr, GPIO.LOW)
  
  # measure the time between the high and the low
  while not GPIO.input(Ec):
      pass
  t1 = time.time()
  while GPIO.input(Ec):
      pass
  t2 = time.time()
  
  # return the distance derived from flight time
  return round((t2-t1)*340/2,2)


if __name__ == '__main__':
    while 1:
        print(getDistance())
        time.sleep(1)
