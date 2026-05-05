"""
logger.py - Interactive serial data logger for prosthetic hand grasps
========================================================================

This script fixes the earlier data-collection workflow issue:
- You no longer need Arduino Serial Monitor for labelled data collection.
- This script sends validated labels to the OpenRB-150 and logs the CSV
  rows returned from data_collection.ino over the same serial connection.

Usage:
    pip install pyserial
    python logger.py
    python logger.py --port COM3
    python logger.py --port /dev/ttyACM0

During collection, type one of:
    cylinder_stiff, cylinder_soft, cube_stiff, cube_soft
or use shortcuts:
    1, 2, 3, 4

Commands:
    q    quit
    r    recalibrate baseline on the OpenRB-150
    h    ask firmware to reprint the CSV header
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from datetime import datetime
from typing import Optional

import serial
import serial.tools.list_ports

BAUD_RATE = 115200
OUTPUT_DIR = "grasp_data"

VALID_LABELS = [
    "cylinder_stiff",
    "cylinder_soft",
    "cube_stiff",
    "cube_soft",
]

LABEL_SHORTCUTS = {
    "1": "cylinder_stiff",
    "2": "cylinder_soft",
    "3": "cube_stiff",
    "4": "cube_soft",
}

CSV_HEADER = (
    "grasp_id,label,timestamp_ms,enc_deg,"
    "s0_dx,s0_dy,s0_dz,"
    "s1_dx,s1_dy,s1_dz,"
    "s2_dx,s2_dy,s2_dz,"
    "s3_dx,s3_dy,s3_dz,"
    "s4_dx,s4_dy,s4_dz,"
    "s5_dx,s5_dy,s5_dz,"
    "t1_flag,t2_flag,t1_ms,t2_ms"
)


def find_port() -> str:
    """Auto-detect an Arduino/OpenRB-like serial port, otherwise ask user."""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("ERROR: No serial ports found.")
        sys.exit(1)

    for port in ports:
        desc = (port.description or "").lower()
        if any(key in desc for key in ["openrb", "samd21", "usbmodem", "arduino", "usb serial"]):
            print(f"Auto-detected: {port.device} ({port.description})")
            return port.device

    print("Available serial ports:")
    for i, port in enumerate(ports):
        print(f"  [{i}] {port.device} - {port.description}")

    while True:
        choice = input("Enter port number: ").strip()
        if choice.isdigit() and 0 <= int(choice) < len(ports):
            return ports[int(choice)].device
        print("Invalid selection.")


class SerialLogger:
    def __init__(self, ser: serial.Serial, output_file: str):
        self.ser = ser
        self.output_file = output_file
        self.header_written = False
        self.row_count = 0
        self.grasp_count = 0
        self.current_label: Optional[str] = None
        self.last_status = ""
        self._file = open(output_file, "w", newline="", encoding="utf-8")

    def close(self) -> None:
        self._file.flush()
        self._file.close()

    def ensure_header(self) -> None:
        if not self.header_written:
            self._file.write(CSV_HEADER + "\n")
            self._file.flush()
            self.header_written = True
            print("  [CSV header written]")

    def process_line(self, line: str) -> None:
        """Print status lines and save CSV rows."""
        if not line:
            return

        if line.startswith("#"):
            self.last_status = line
            print(f"  {line}")
            if "Grasp ID:" in line:
                self.grasp_count += 1
            return

        if line.startswith("grasp_id,"):
            self.ensure_header()
            return

        if "," not in line:
            print(f"  [ignored] {line}")
            return

        # If a firmware restart caused the header to be missed, still keep CSV valid.
        self.ensure_header()
        self._file.write(line + "\n")
        self.row_count += 1

        if self.row_count % 25 == 0:
            cols = line.split(",")
            if len(cols) >= 26:
                print(
                    f"  row={self.row_count:5d} grasp={cols[0]} label={cols[1]} "
                    f"t={cols[2]}ms t1={cols[-4]} t2={cols[-3]}"
                )
            self._file.flush()

    def read_available(self, duration_s: float = 0.0) -> None:
        """Read any currently available lines for up to duration_s seconds."""
        end_time = time.time() + max(duration_s, 0.0)
        while time.time() < end_time or self.ser.in_waiting:
            raw = self.ser.readline()
            if not raw:
                continue
            try:
                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            except UnicodeDecodeError:
                continue
            self.process_line(line)

    def send_line(self, text: str) -> None:
        self.ser.write((text.strip() + "\n").encode("utf-8"))
        self.ser.flush()

    def wait_for_grasp_done(self, timeout_s: float) -> None:
        """Read and log lines until firmware reports READY or timeout occurs."""
        end_time = time.time() + timeout_s
        while time.time() < end_time:
            raw = self.ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            self.process_line(line)

            if line.startswith("# READY") or "Invalid label rejected" in line:
                self._file.flush()
                return

        print("  # WARNING: logger timed out waiting for firmware READY message.")
        self._file.flush()


def normalize_user_input(text: str) -> Optional[str]:
    value = text.strip()
    if not value:
        return None
    if value in LABEL_SHORTCUTS:
        return LABEL_SHORTCUTS[value]
    return value


def print_menu() -> None:
    print("\nLabels:")
    for i, label in enumerate(VALID_LABELS, start=1):
        print(f"  {i}. {label}")
    print("Commands: q=quit, r=recalibrate baseline, h=print CSV header")


def main() -> None:
    parser = argparse.ArgumentParser(description="Interactive logger for prosthetic hand data collection")
    parser.add_argument("--port", default=None, help="Serial port, e.g. COM3 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=BAUD_RATE, help="Serial baud rate")
    parser.add_argument("--outdir", default=OUTPUT_DIR, help="Output directory for CSV files")
    parser.add_argument("--startup-wait", type=float, default=4.0, help="Seconds to read startup messages")
    parser.add_argument("--grasp-timeout", type=float, default=15.0, help="Seconds to wait for one grasp to finish")
    args = parser.parse_args()

    port = args.port or find_port()
    os.makedirs(args.outdir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    output_file = os.path.join(args.outdir, f"grasps_{timestamp}.csv")

    print(f"\nConnecting to {port} at {args.baud} baud...")
    print(f"Saving CSV to: {output_file}")
    print("Do not open Arduino Serial Monitor while this script is connected.\n")

    logger = None

    try:
        with serial.Serial(port, args.baud, timeout=1) as ser:
            # Many Arduino boards reset when the serial port opens.
            # Give the firmware time to boot and print its header.
            time.sleep(1.5)
            logger = SerialLogger(ser, output_file)
            try:
                print("Reading startup messages...")
                logger.read_available(args.startup_wait)
                print_menu()

                while True:
                    user_text = input("\nEnter label/command: ").strip()
                    if not user_text:
                        continue

                    command = user_text.lower()
                    if command in {"q", "quit", "exit"}:
                        break

                    if command in {"r", "h"}:
                        logger.send_line(command)
                        logger.wait_for_grasp_done(timeout_s=8.0)
                        continue

                    label = normalize_user_input(user_text)
                    if label not in VALID_LABELS:
                        print("Invalid label. Use one of the labels below.")
                        print_menu()
                        continue

                    print(f"Sending label: {label}")
                    logger.current_label = label
                    logger.send_line(label)
                    logger.wait_for_grasp_done(timeout_s=args.grasp_timeout)

            finally:
                logger.close()

    except KeyboardInterrupt:
        print("\nStopped by user.")
    except serial.SerialException as exc:
        print(f"Serial error: {exc}")
        sys.exit(1)

    if logger is not None:
        print("\n" + "-" * 56)
        print(f"Done. Grasps reported: {logger.grasp_count}")
        print(f"Rows saved:       {logger.row_count}")
        print(f"CSV file:         {output_file}")
        print("-" * 56)


if __name__ == "__main__":
    main()
