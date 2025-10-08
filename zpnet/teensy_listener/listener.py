import serial, json, time, logging
from zpnet.shared.events import create_event

SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 115200
HEARTBEAT_TIMEOUT = 120

def listen():
    logging.info("🔌 ZPNet Teensy Listener started")
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    last_heartbeat = time.time()

    while True:
        line = ser.readline().decode("utf-8").strip()
        if not line:
            if time.time() - last_heartbeat > HEARTBEAT_TIMEOUT:
                create_event("SYSTEM_ERROR", {
                    "component": "teensy",
                    "message": "Heartbeat timeout"
                })
                last_heartbeat = time.time()
            continue

        try:
            evt = json.loads(line)
        except json.JSONDecodeError:
            logging.warning(f"Bad JSON from Teensy: {line}")
            continue

        event_type = evt.pop("event_type", "UNKNOWN")
        create_event(event_type, evt)

        if event_type == "HEARTBEAT":
            last_heartbeat = time.time()
