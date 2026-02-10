import serial
from datetime import datetime

port = "COM5"   # or "COM3" on Windows
baud = 57600

ser = serial.Serial(port, baud, timeout=1)
import os

# ensure tm directory exists
os.makedirs("tm", exist_ok=True)

# create filename with current datetime
timestamp_file = datetime.now().strftime("%Y%m%d_%H%M%S")
filename = f"tm/data/telemetry_log_{timestamp_file}.txt"

# Open output file
with open(filename, "a") as f:
    while True:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if not line:
            continue

        # prepend timestamp
        timestamp = datetime.now().isoformat()
        out = f"{timestamp}, {line}"

        print(out)      # show on screen
        f.write(out + "\n")
        f.flush()       # force write to disk