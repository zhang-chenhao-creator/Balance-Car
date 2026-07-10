import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path


CSV_HEADER = [
    "record", "ms", "state", "mode", "ax_raw", "ay_raw", "az_raw",
    "gx_raw", "gy_raw", "gz_raw", "ax_g", "ay_g", "az_g",
    "gx_dps", "gy_dps", "gz_dps", "pitch_acc", "pitch_dmp",
    "pitch_comp", "pitch_kalman", "dmp_hz", "dt_ms", "status",
]


def import_serial():
    try:
        import serial
        from serial.tools import list_ports
        return serial, list_ports
    except ModuleNotFoundError:
        print("pyserial is required: py -m pip install pyserial", file=sys.stderr)
        raise


def choose_port(list_ports, requested):
    if requested:
        return requested
    ports = list(list_ports.comports())
    for port in ports:
        text = f"{port.device} {port.description}".upper()
        if "CH9102" in text or "USB-SERIAL" in text:
            return port.device
    if ports:
        return ports[0].device
    raise RuntimeError("no serial port found")


def main():
    parser = argparse.ArgumentParser(description="Capture B585 MPU6050 CSV output.")
    parser.add_argument("--port", default=None, help="serial port, for example COM7")
    parser.add_argument("--baud", default=115200, type=int)
    parser.add_argument("--seconds", default=60, type=int)
    parser.add_argument("--state", default="STATIC", choices=["STATIC", "TILT", "SHAKE"])
    parser.add_argument("--mode", default="ALL", choices=["RAW", "DMP", "COMP", "KALMAN", "ALL"])
    parser.add_argument("--rate", default=50, type=int, choices=[20, 50, 100])
    args = parser.parse_args()

    serial, list_ports = import_serial()
    port = choose_port(list_ports, args.port)
    out_dir = Path(__file__).resolve().parent / "实验数据"
    out_dir.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = out_dir / f"MPU6050_{args.state}_{stamp}.csv"

    with serial.Serial(port, args.baud, timeout=0.2) as ser, csv_path.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)
        # CH9102 one-key-download circuit: RTS low selects Flash boot, DTR pulse resets MCU.
        ser.rts = False
        ser.dtr = True
        time.sleep(0.2)
        ser.dtr = False
        time.sleep(0.18)
        ser.dtr = True
        time.sleep(6.0)
        ser.reset_input_buffer()
        for command in [f"STATE {args.state}", f"MODE {args.mode}", f"RATE {args.rate}", "LOG 1", "STATUS"]:
            ser.write((command + "\r\n").encode("ascii"))
            ser.flush()
            time.sleep(0.1)

        print(f"connected {port} @ {args.baud}")
        print(f"writing {csv_path}")
        end_at = time.time() + args.seconds
        count = 0
        while time.time() < end_at:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            try:
                print(line)
            except UnicodeEncodeError:
                enc = sys.stdout.encoding or "utf-8"
                print(line.encode(enc, errors="replace").decode(enc, errors="replace"))
            if line.startswith("A,"):
                fields = line.split(",")
                if len(fields) == len(CSV_HEADER):
                    writer.writerow(fields)
                    count += 1
        ser.write(b"LOG 0\r\n")
        ser.flush()
        print(f"captured {count} samples")


if __name__ == "__main__":
    main()
