"""
logger.py  —  Serial data logger for prosthetic hand data collection
=====================================================================
Captures CSV rows streamed from the OpenRB-150 over USB Serial.
Lines beginning with '#' (Arduino status messages) are printed to
the console but NOT written to the CSV file.

Usage:
    pip install pyserial
    python logger.py                        # auto-detects port
    python logger.py --port COM3            # Windows
    python logger.py --port /dev/ttyACM0   # Linux / macOS
"""

import serial
import serial.tools.list_ports
import argparse
import sys
import os
from datetime import datetime

BAUD_RATE  = 115200
OUTPUT_DIR = "grasp_data"


def find_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("ERROR: No serial ports found.")
        sys.exit(1)
    for p in ports:
        desc = (p.description or "").lower()
        if any(k in desc for k in ["openrb", "samd21", "usbmodem", "arduino"]):
            print(f"Auto-detected: {p.device}  ({p.description})")
            return p.device
    print("Available ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} — {p.description}")
    return ports[int(input("Enter port number: "))].device


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=BAUD_RATE)
    args = ap.parse_args()

    port = args.port or find_port()
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    ts_str   = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = os.path.join(OUTPUT_DIR, f"grasps_{ts_str}.csv")

    print(f"\nConnecting to {port} at {args.baud} baud ...")
    print(f"Saving to:     {filename}")
    print("Ctrl+C to stop.\n")

    header_written = False
    row_count      = 0
    grasp_count    = 0

    try:
        with serial.Serial(port, args.baud, timeout=1) as ser, \
             open(filename, "w", newline="", encoding="utf-8") as f:

            while True:
                raw = ser.readline()
                if not raw:
                    continue
                try:
                    line = raw.decode("utf-8").rstrip("\r\n")
                except UnicodeDecodeError:
                    continue

                # Status lines from Arduino
                if line.startswith("#"):
                    print(f"  {line}")
                    if "Grasp ID:" in line:
                        grasp_count += 1
                    continue

                # CSV header (only write once)
                if line.startswith("grasp_id,"):
                    if not header_written:
                        f.write(line + "\n")
                        f.flush()
                        header_written = True
                        print(f"  [Header written]")
                    continue

                # Data rows
                if header_written and "," in line:
                    f.write(line + "\n")
                    row_count += 1
                    if row_count % 25 == 0:
                        cols = line.split(",")
                        print(f"  grasp={cols[0]}  label={cols[1]}  "
                              f"t={cols[2]}ms  "
                              f"t1={cols[-4]}  t2={cols[-3]}  "
                              f"[{row_count} rows total]")
                        f.flush()

    except KeyboardInterrupt:
        print(f"\n{'─'*50}")
        print(f"Stopped. Grasps: {grasp_count}  Rows: {row_count}")
        print(f"File: {filename}")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
