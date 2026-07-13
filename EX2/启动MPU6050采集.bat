@echo off
cd /d "%~dp0"
py -3 mpu6050_serial_capture.py --seconds 60 --state STATIC --mode ALL --rate 50
