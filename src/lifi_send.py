#
# Copyright (c) 2019, George Buga & Ciprian Mindru
# All rights reserved.
#
import serial
import numpy as np
import time
import os
import struct

# Serial connection parameters
LIFI_SERIAL_PORT = '/dev/serial0'
LIFI_SERIAL_BAUD = 80000

# Image path
IMAGE_PATH = './images'

# Sync byte sequence
LIFI_SYNC_DATA = bytearray([40, 248, 245, 124, 204, 36, 107, 234, 202, 245])

# Create serial connection
ser = serial.Serial(port=LIFI_SERIAL_PORT,
                    baudrate=LIFI_SERIAL_BAUD,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=None,
                    xonxoff=False,
                    rtscts=False,
                    write_timeout=None,
                    dsrdtr=False,
                    inter_byte_timeout=None)

# ------------------------------------------------ Data transmission ---------------------------------------------------
while True:
    # Iterate through all the images
    for file in os.listdir('./images/'):    
        
        # Current file path
        file_path = os.path.join(IMAGE_PATH, file)
        
        # Read file data
        fh = open(file_path,'rb')
        data_bytes = fh.read()
        fh.close()

        # Scramble data
        data_bytes = bytearray(data_bytes)
        for idx in range(len(data_bytes)):
            data_bytes[idx] =  data_bytes[idx] ^ 0xAA

        # Print file stats
        print('File path = %s' % file_path)
        print('File size = %d' % len(data_bytes))
        
        # Send sync
        if ser.write(LIFI_SYNC_DATA) != len(LIFI_SYNC_DATA):
            print('TX SYNC FAILED ...')
            exit(1)

        # Build and send preamble
        data_size = np.uint32(len(data_bytes))
        preamble_bytes = struct.pack('<L', data_size)
        if ser.write(preamble_bytes) != len(preamble_bytes):
            print('TX PREAMBLE FAILED ...')
            exit(1)
            
        # Send data
        if ser.write(data_bytes) != len(data_bytes):
            print('TX DATA FAILED ...')
            exit(1)
        
        # Wait some time before next transmissions
        time.sleep(1)
