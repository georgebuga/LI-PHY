import serial
import time
PORT = '/dev/serial0'
ser = serial.Serial(port = PORT, baudrate = 10000)

while True:
    ser.write(bytearray('c'))
    time.sleep(0.01)
