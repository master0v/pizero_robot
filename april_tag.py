#!/usr/bin/env python3
"""
april_tag_detect.py

Live PiCamera2 → OpenCV preview with real-time AprilTag detection
(using pupil_apriltags) and full console output of each tag’s data.
"""

import os
# suppress libcamera INFO/WARN
os.environ["LIBCAMERA_LOG_LEVELS"]  = "ERROR"
os.environ["LIBCAMERA_LOG_NO_COLOR"] = "1"

import signal
import time
import numpy as np
import cv2
from picamera2 import Picamera2
from pupil_apriltags import Detector

def main():
    # ——— 1) Initialize PiCamera2 ————————————————
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={
        "format": "RGB888",    # 3‐byte RGB
        "size":   (640, 480)
    })
    picam2.configure(config)
    picam2.start()

    # ——— 2) Initialize AprilTag detector —————————
    detector = Detector(
        families="tag36h11",
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0
    )

    def shutdown(signum, frame):
        picam2.stop()
        picam2.close()
        cv2.destroyAllWindows()
        exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print("Starting AprilTag detection. Press Ctrl-C or 'q' in the window to quit.")

    while True:
        # — Capture frame as RGB array ——————————
        frame = picam2.capture_array("main")  # shape=(480,640,3), channels=R,G,B

        # — Convert to grayscale for detection —————
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # — Detect AprilTags ————————————————
        detections = detector.detect(
            gray,
            estimate_tag_pose=False,
            camera_params=None,
            tag_size=None
        )
        # (returns list of Detection objects with attributes: 
        #  tag_family, tag_id, hamming, decision_margin, center, corners, homography, pose_R, pose_t, pose_err) 

        # — Draw overlays & print data ——————————
        disp = frame.copy()
        for det in detections:
            # 1) Draw polygon around tag
            corners = det.corners.astype(int)
            for i in range(4):
                pt1 = tuple(corners[i])
                pt2 = tuple(corners[(i+1)%4])
                # disp is RGB; OpenCV wants BGR, but green (0,255,0) works both ways
                cv2.line(disp, pt1, pt2, (0,255,0), 2)

            # 2) Draw tag ID text
            cx, cy = int(det.center[0]), int(det.center[1])
            cv2.putText(disp, str(det.tag_id), (cx+5, cy-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

            # 3) Print every relevant field to the console
            print("— AprilTag Detected —")
            print(f"tag_id:         {det.tag_id}")
            print(f"tag_family:     {det.tag_family}")
            print(f"hamming:        {det.hamming}")
            print(f"decision_margin:{det.decision_margin:.3f}")
            print(f"center:         ({det.center[0]:.1f}, {det.center[1]:.1f})")
            print("corners:")
            for i, (x, y) in enumerate(det.corners):
                print(f"  corner {i}:   ({x:.1f}, {y:.1f})")
            print("homography:")
            print(det.homography)
            # optional pose fields (only if estimate_tag_pose=True)
            if hasattr(det, "pose_R") and det.pose_R is not None:
                print("pose_R:")
                print(det.pose_R)
            if hasattr(det, "pose_t") and det.pose_t is not None:
                print("pose_t:")
                print(det.pose_t)
            if hasattr(det, "pose_err") and det.pose_err is not None:
                print(f"pose_err:       {det.pose_err:.3f}")
            print()

        # — Show live window ———————————————
        # convert RGB→BGR for display
        disp_bgr = cv2.cvtColor(disp, cv2.COLOR_RGB2BGR)
        cv2.imshow("AprilTag Detection", disp_bgr)

        # exit on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            shutdown(None, None)

        # throttle loop a bit
        time.sleep(3)

if __name__ == "__main__":
    main()
