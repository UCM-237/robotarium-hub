import serial;
import sys
port = '/dev/bus/usb/001/021'
baudrate = 9600
ser = serial.Serial(port,baudrate,timeout=0.001)
while True:
    data = ser.read(1)
    data+= ser.read(ser.inWaiting())
    sys.stdout.write(data)
    sys.stdout.flush()

    