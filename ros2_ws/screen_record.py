"""Screen-record a region of DISPLAY :1 (the Gazebo GUI) to MP4 via mss+OpenCV.

Usage: python3 screen_record.py LEFT TOP WIDTH HEIGHT [DURATION_S] [FPS] [OUT]
Duration-bounded loop finalises the file in normal flow (no signal dependency).
"""
import sys
import time

import cv2
import mss
import numpy as np

L, T, W, H = (int(x) for x in sys.argv[1:5])
DUR = float(sys.argv[5]) if len(sys.argv) > 5 else 100.0
FPS = float(sys.argv[6]) if len(sys.argv) > 6 else 12.0
OUT = sys.argv[7] if len(sys.argv) > 7 else \
    '/workspace/ros2_ws/rl_eval_results/drone_flight_3rdperson.mp4'

mon = {'left': L, 'top': T, 'width': W, 'height': H}
vw = cv2.VideoWriter(OUT, cv2.VideoWriter_fourcc(*'mp4v'), FPS, (W, H))
interval = 1.0 / FPS
end = time.time() + DUR
n = 0
with mss.mss() as sct:
    while time.time() < end:
        t0 = time.time()
        frame = np.array(sct.grab(mon))[:, :, :3]
        vw.write(frame)
        n += 1
        slack = interval - (time.time() - t0)
        if slack > 0:
            time.sleep(slack)
vw.release()
print(f'REC DONE frames={n} -> {OUT}', flush=True)
