import time
import ultra

try:
    while True:
        distance = ultra.getDistance()
        print(f"Distance: {distance:.2f} meters")
        time.sleep(0.2)

except KeyboardInterrupt:
    print("Stopped by user")
