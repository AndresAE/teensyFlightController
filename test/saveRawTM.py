import serial
import struct
from datetime import datetime

# --- packet format ---
HEADER = b'\xAB\xCD'          # new header
STRUCT_FORMAT = "<d bb 17d bb 11d H"
STRUCT_SIZE = struct.calcsize(STRUCT_FORMAT)

ser = serial.Serial("COM5", 57600, timeout=5)


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def read_packet():
    # sync to header
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b == HEADER[1:2]:
            b2 = ser.read(1)
            if b2 == HEADER[0:1]:
                break

    # read struct payload
    payload = ser.read(STRUCT_SIZE)

    data_without_crc = payload[:-2]
    received_crc = int.from_bytes(payload[-2:], "little")

    # compute CRC
    computed_crc = crc16_ccitt(data_without_crc)

    if (len(payload) != STRUCT_SIZE) or (received_crc != computed_crc):
        return None

    return struct.unpack(STRUCT_FORMAT, payload)


while True:
    timestamp_file = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"tm/data/telemetry_log_raw_{timestamp_file}.txt"
    log = open(filename, "a")

    while True:
        pkt = read_packet()
        if pkt is None:
            continue

        log.write(str(pkt) + "\n")
        log.flush()
