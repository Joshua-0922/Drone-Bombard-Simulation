"""Single-motor step test: PX4 actuator test + laptop-mic recording (bench, props on ONE motor).

Runs on the Windows host (the Pixhawk and the mic are Windows devices, not visible in WSL):
    python motor_step.py --motor 2 --out data/run3_motor2

Sequence: 5 s countdown, spin-up to LO (discarded), then HI/LO steps of HOLD s, then off.
Writes <out>_audio.wav (16-bit mono) and <out>_meta.json (command/ACK times, SERVO_OUTPUT_RAW,
audio block callback times), all on the host perf_counter clock. Analyse with analyze_motor_step.py.

Safety: aborts if armed, if battery voltage is unreadable or < 13 V (unless --no-batt-check),
or if the first command is not ACCEPTED. Always sends 'off' (NaN) on exit.
"""
import argparse, json, math, threading, time, wave

import numpy as np
import sounddevice as sd
from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument('--motor', type=int, default=2, help='motor index (PX4 Motor n, MAVLink ACTUATOR_OUTPUT_FUNCTION_MOTORn)')
ap.add_argument('--out', required=True, help='output prefix')
ap.add_argument('--port', default='COM7', help='Pixhawk USB serial port')
ap.add_argument('--mic', type=int, default=9, help='sounddevice index of the WASAPI mic')
ap.add_argument('--lo', type=float, default=0.30)
ap.add_argument('--hi', type=float, default=0.50)
ap.add_argument('--hold', type=float, default=1.5)
ap.add_argument('--reps', type=int, default=8)
ap.add_argument('--no-batt-check', action='store_true')
args = ap.parse_args()

FS, BLOCK = 48000, 480
SEQ = [args.lo] + [args.hi, args.lo] * args.reps   # values 0..1 map to PWM_MAIN_MINn..MAXn

m = mavutil.mavlink_connection(args.port, baud=115200, source_system=255, source_component=190)
hb = m.wait_heartbeat(timeout=10)
if hb.base_mode & 128:
    raise SystemExit("ABORT: vehicle is armed")

volt = None
t = time.time()
while time.time() - t < 4:
    s = m.recv_match(type='SYS_STATUS', blocking=True, timeout=1)
    if s:
        volt = s.voltage_battery
        break
if not args.no_batt_check and (volt is None or volt == 65535 or volt < 13000):
    raise SystemExit(f"ABORT: battery voltage unreadable or low ({volt} mV)")
print("battery mV:", volt)

# SERVO_OUTPUT_RAW (id 36) at 200 Hz on this link
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 36, 5000, 0, 0, 0, 0, 0)

events, pwm = [], []
stop = threading.Event()

def reader():
    while not stop.is_set():
        r = m.recv_match(type=['SERVO_OUTPUT_RAW', 'COMMAND_ACK'], blocking=True, timeout=0.2)
        if r is None:
            continue
        if r.get_type() == 'SERVO_OUTPUT_RAW':
            pwm.append((time.perf_counter(), r.time_usec, getattr(r, f"servo{args.motor}_raw")))
        elif r.command == mavutil.mavlink.MAV_CMD_ACTUATOR_TEST:
            events.append(('ack', time.perf_counter(), r.result))

def send(value):
    # PX4 Commander maps param5 1.. to Motor1.. (NOT the 101.. output-function ids)
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_ACTUATOR_TEST, 0,
                            value, 2.5, 0, 0, args.motor, 0, 0)
    events.append(('cmd', time.perf_counter(), None if math.isnan(value) else value))

blocks = []
def cb(indata, frames, tinfo, status):
    blocks.append((time.perf_counter(), indata[:, 0].copy()))

threading.Thread(target=reader, daemon=True).start()
# WASAPI exclusive bypasses the Windows mic-array noise suppression, which erases a steady motor tone
stream = sd.InputStream(device=args.mic, samplerate=FS, channels=1, dtype='float32', blocksize=BLOCK,
                        callback=cb, extra_settings=sd.WasapiSettings(exclusive=True))
stream.start()
try:
    print("countdown 5 s"); time.sleep(5.0)
    for i, v in enumerate(SEQ):
        send(v)
        if i == 0:
            time.sleep(0.5)
            acks = [e for e in events if e[0] == 'ack']
            if not acks or acks[0][2] != 0:
                raise RuntimeError(f"first command not accepted: {acks}")
        time.sleep(args.hold)
finally:
    for _ in range(3):
        send(float('nan')); time.sleep(0.1)
    time.sleep(1.0)
    lat = stream.latency
    stop.set(); stream.stop(); stream.close()

audio = np.concatenate([b[1] for b in blocks])
with wave.open(args.out + '_audio.wav', 'wb') as w:
    w.setnchannels(1); w.setsampwidth(2); w.setframerate(FS)
    w.writeframes((np.clip(audio, -1, 1) * 32767).astype('<i2').tobytes())
json.dump({'fs': FS, 'block': BLOCK, 'block_cb_time': [b[0] for b in blocks],
           'stream_latency': lat, 'events': events, 'pwm': pwm, 'seq': SEQ, 'hold': args.hold,
           'battery_mV': volt, 'motor': args.motor},
          open(args.out + '_meta.json', 'w'))
print("acks:", [e for e in events if e[0] == 'ack'][:3])
print("pwm samples:", len(pwm), "audio s:", len(audio) / FS)
