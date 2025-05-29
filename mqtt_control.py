#!/home/aim/myenv/bin/python

import sys
import time
import json
import base64
import logging
import threading

import cv2
import paho.mqtt.client as mqtt
from picamera2 import Picamera2
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo as ServoModule

from ledControl import ledControl
import move
import ultra

class Config:
    BROKER        = "192.168.0.31"
    PORT          = 1883

    RES           = (640, 480)
    JPEG_QUAL     = 60

    TOPIC_IMG_REQ = "robot/image/request"
    TOPIC_IMG     = "robot/image"
    TOPIC_LED     = "robot/led"
    TOPIC_MOVE    = "robot/move"
    TOPIC_DIST_REQ= "robot/distance/request"
    TOPIC_DIST    = "robot/distance"
    TOPIC_SERVO   = "robot/servo"
    TOPIC_STATUS  = "robot/status"

    SERVO_CHANNEL = 11
    SERVO_MIN     = 30
    SERVO_MAX     = 90

# ─── Logging Setup ─────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.DEBUG,
    format='[%(asctime)s] [%(name)s] %(levelname)s: %(message)s',
    datefmt='%H:%M:%S'
)
log = logging.getLogger("robot")


def safe_json(payload: bytes) -> dict:
    try:
        return json.loads(payload.decode('utf-8'))
    except Exception as e:
        log.error("Invalid JSON payload: %s", e)
        return {}


class RobotController:
    def __init__(self):
        # Camera init & lock exposure
        self.picam2 = Picamera2()
        conf = self.picam2.create_still_configuration(main={"size": Config.RES, "format": "RGB888"})
        self.picam2.configure(conf)
        self.picam2.start(); time.sleep(2.0)
        req = self.picam2.capture_request(); md = req.get_metadata()
        exp, gain = int(md.get("ExposureTime", 0)), md.get("AnalogueGain", 1.0)
        req.release(); self.picam2.stop()
        self.picam2.set_controls({
            "AwbEnable": False,
            "ExposureTime": exp,
            "AnalogueGain": gain
        })
        log.info("Camera locked EX=%dµs, AG=%.2f", exp, gain)

        # Hardware init
        self.lc = ledControl()
        move.setup()
        ultra.init_sensor()

        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50
        self.servo = ServoModule.Servo(
            self.pca.channels[Config.SERVO_CHANNEL],
            min_pulse=500, max_pulse=2500
        )
        log.info("Servo on channel %d (angles %d–%d°)",
                 Config.SERVO_CHANNEL, Config.SERVO_MIN, Config.SERVO_MAX)

        # MQTT setup
        self.client = mqtt.Client()
        self.client.will_set(
            Config.TOPIC_STATUS,
            json.dumps({"status":"offline"}),
            qos=1, retain=True
        )
        self.client.on_connect    = self.on_connect
        self.client.on_message    = self.on_message
        self.client.on_disconnect = self.on_disconnect

    def on_connect(self, client, userdata, flags, rc):
        log.info("Connected to MQTT broker (rc=%s)", rc)
        for topic in (
            Config.TOPIC_IMG_REQ,
            Config.TOPIC_LED,
            Config.TOPIC_MOVE,
            Config.TOPIC_DIST_REQ,
            Config.TOPIC_SERVO
        ):
            client.subscribe(topic)
            log.debug("Subscribed to '%s'", topic)

        # announce online
        client.publish(
            Config.TOPIC_STATUS,
            json.dumps({"status":"online"}),
            qos=1, retain=True
        )

    def on_disconnect(self, client, userdata, rc):
        log.warning("MQTT disconnected (rc=%s)", rc)

    def on_message(self, client, userdata, msg):
        log.debug("on_message: topic=%s payload=%s", msg.topic, msg.payload)
        if msg.topic == Config.TOPIC_IMG_REQ:
            self.handle_image_request()
        elif msg.topic == Config.TOPIC_LED:
            self.handle_led(msg)
        elif msg.topic == Config.TOPIC_MOVE:
            self.handle_move(msg)
        elif msg.topic == Config.TOPIC_DIST_REQ:
            self.handle_distance_request()
        elif msg.topic == Config.TOPIC_SERVO:
            self.handle_servo(msg)
        else:
            log.warning("Unhandled topic '%s'", msg.topic)

    def handle_image_request(self):
        log.debug("handle_image_request()")
        try:
            self.picam2.start()
            time.sleep(0.05)
            frame = self.picam2.capture_array("main")
            self.picam2.stop()

            ret, buf = cv2.imencode(
                '.jpg', frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), Config.JPEG_QUAL]
            )
            if not ret:
                log.error("JPEG encoding failed")
                return

            jpg = buf.tobytes()
            payload = json.dumps({
                "timestamp": time.time(),
                "image_b64": base64.b64encode(jpg).decode('ascii')
            })
            self.client.publish(Config.TOPIC_IMG, payload)
            log.info("Published image (%d bytes)", len(jpg))

        except Exception as e:
            log.error("Error in handle_image_request: %s", e, exc_info=True)

    def handle_led(self, msg):
        log.debug("handle_led payload=%s", msg.payload)
        p = safe_json(msg.payload)
        cmd = p.get("cmd", "").lower()
        wait = int(p.get("wait_ms", 50))
        try:
            if   cmd == "red":     self.lc.redColorWipe(wait)
            elif cmd == "green":   self.lc.greenColorWipe(wait)
            elif cmd == "blue":    self.lc.blueColorWipe(wait)
            elif cmd == "wipeclean":   self.lc.wipeClean()
            elif cmd == "rainbow":     self.lc.rainbow(self.lc.strip, wait, iterations=1)
            elif cmd == "rainbowcycle":self.lc.rainbowCycle(self.lc.strip, wait, iterations=1)
            elif cmd == "theaterchase":   self.lc.theaterChase(self.lc.strip, wait_ms=wait, iterations=5)
            elif cmd == "theaterchaserainbow": self.lc.theaterChaseRainbow(self.lc.strip, wait_ms=wait)
            else:
                log.warning("Unknown LED cmd '%s'", cmd)
                return
            log.info("Executed LED cmd '%s'", cmd)
        except Exception as e:
            log.error("LED handler error: %s", e, exc_info=True)

    def handle_move(self, msg):
        """
        Drive motors with exactly the server’s values.
        JSON: { "left":<int>, "right":<int>, [ "duration":<secs> ] }.
        """
        log.debug("handle_move payload=%s", msg.payload)
        p = safe_json(msg.payload)
        left = int(p.get("left", 0))
        right = int(p.get("right", 0))
        dur = p.get("duration", None)
        log.debug("Parsed move → left=%d right=%d duration=%s", left, right, dur)

        try:
            move.drive(left, right)
            log.info("Driving L=%d%% R=%d%%%s", left, right,
                     f" for {dur:.2f}s" if dur else "")
            if dur is not None and dur > 0:
                threading.Timer(dur,
                                lambda: (move.drive(0, 0),
                                         log.info("Auto-stopped motors"))).start()
        except Exception as e:
            log.error("Move handler error: %s", e, exc_info=True)

    def handle_distance_request(self):
        log.debug("handle_distance_request()")
        try:
            dist = ultra.get_filtered_distance()
            payload = json.dumps({
                "timestamp": time.time(),
                "distance_m": dist
            })
            self.client.publish(Config.TOPIC_DIST, payload, qos=1)
            log.info("Published distance: %s", dist)
        except Exception as e:
            log.error("Distance handler error: %s", e, exc_info=True)

    def handle_servo(self, msg):
        log.debug("handle_servo payload=%s", msg.payload)
        p = safe_json(msg.payload)
        angle = float(p.get("angle", Config.SERVO_MIN))
        angle = max(Config.SERVO_MIN, min(Config.SERVO_MAX, angle))
        try:
            self.servo.angle = angle
            log.info("Servo angle set to %.1f°", angle)
        except Exception as e:
            log.error("Servo handler error: %s", e, exc_info=True)

    def shutdown(self):
        log.info("Shutting down…")
        try:
            self.client.loop_stop()
            self.client.disconnect()
        except Exception as e:
            log.error("MQTT shutdown error: %s", e, exc_info=True)

        try: move.destroy()
        except: pass
        try: self.pca.deinit()
        except: pass
        try: ultra.GPIO.cleanup()
        except: pass

        log.info("Clean exit")


def main():
    controller = RobotController()
    controller.client.connect(Config.BROKER, Config.PORT, keepalive=60)
    controller.client.loop_start()
    log.info("MQTT loop started, awaiting commands…")

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        controller.shutdown()
        sys.exit(0)


if __name__ == "__main__":
    main()
