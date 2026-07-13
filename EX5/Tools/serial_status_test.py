import argparse
import time

import serial


def send_command(ser, command, wait=0.5):
    ser.write((command + "\r\n").encode("ascii"))
    time.sleep(wait)
    data = ser.read(4096)
    print(f">>> {command}")
    text = data.decode("utf-8", errors="replace").strip()
    print(text)
    return text


def validate_status(text):
    required = (
        "DIST=",
        "UOK=",
        "OBS=",
        "UGUARD=",
        "USTATE=",
        "MISS=",
        "USTOP=",
        "USLOW=",
        "VREQ=",
        "VSAFE=",
    )
    missing = [field for field in required if field not in text]
    if missing:
        raise RuntimeError("STATUS missing fields: " + ", ".join(missing))


def main():
    parser = argparse.ArgumentParser(description="EX5 serial status helper")
    parser.add_argument("--port", default="COM7", help="Serial port, default COM7")
    parser.add_argument("--baud", default=115200, type=int, help="Baudrate, default 115200")
    parser.add_argument(
        "--auto",
        action="store_true",
        help="Send UGUARD ON after STATUS. Optional because the guard is enabled at power-on.",
    )
    args = parser.parse_args()

    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        time.sleep(0.4)
        ser.reset_input_buffer()
        validate_status(send_command(ser, "STATUS"))
        if args.auto:
            send_command(ser, "UGUARD ON")
            validate_status(send_command(ser, "STATUS"))


if __name__ == "__main__":
    main()
