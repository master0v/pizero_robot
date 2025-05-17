import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Initialize I2C and PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # Standard for servo control

# Set up servo on channel 11
my_servo = servo.Servo(pca.channels[11])

# Move between 0 and 90 degrees repeatedly
while True:
    my_servo.angle = 0
    time.sleep(1)
    my_servo.angle = 90
    time.sleep(1)
