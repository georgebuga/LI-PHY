import serial

ser = serial.Serial(port='/dev/serial0',
                    timeout=0.1,
                    baudrate=10000)

while True:    
    data = ser.read(1)
    if data:
        print(data)
    else:
        print(' ')