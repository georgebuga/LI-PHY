#
# Copyright (c) 2019, George Buga & Ciprian Mindru
# All rights reserved.
#
import serial
import numpy as np
import imageio
import matplotlib.pyplot as plt
import struct
import time
import os
from Pillow import Image

# Serial connection parameters
LIFI_SERIAL_PORT = '/dev/serial0'
LIFI_SERIAL_BAUD = 80000

# Sync byte sequence
LIFI_SYNC_DATA = bytearray([40, 248, 245, 124, 204, 36, 107, 234, 202, 245])

# Serial read timeout (seconds)
SERIAL_READ_TIMEOUT = 0.01

# Temporary file path
TEMP_FILE_PATH = 'file.data'

# Denial of service file path
DOS_FILE_PATH = 'dos.jpeg'
DOS_FILE_DATA = imageio.imread(DOS_FILE_PATH).astype(np.uint8)

# Descramble byte sequence
SCRAMBLE_SEQ = 0xAA

# Preamble parameters
PREAMBLE_LEN = 4

# Create serial connection
ser = serial.Serial(port=LIFI_SERIAL_PORT,
                    baudrate=LIFI_SERIAL_BAUD,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=SERIAL_READ_TIMEOUT,
                    xonxoff=False,
                    rtscts=False,
                    write_timeout=None,
                    dsrdtr=False,
                    inter_byte_timeout=None)

# -------------------------------------------------- Data reception ----------------------------------------------------
# Force figure to be fullscreen
mng = plt.get_current_fig_manager()
#mng.full_screen_toggle()

# Remove plot toolbar
import matplotlib as mpl
mpl.rcParams['toolbar'] = 'None'

# Remove axis and set interactive
plt.axis('off')
plt.ion()

# Initialize RX buffer
RX_DATA_BUFF_SIZE = 100000
rx_data_buff = np.zeros(RX_DATA_BUFF_SIZE)
rx_data_buff = rx_data_buff.astype(np.uint8)

# Start reception
while True:

    # Flush input data buffer for fresh reception
    ser.reset_input_buffer()

    # RX synchronization
    print('----------------')
    print('RX SYNC wait ...')
    rx_sync_idx = 0
    while True:
        byte = bytearray(ser.read(1))
        if not byte:
            continue
        if byte[0] == LIFI_SYNC_DATA[rx_sync_idx]:
            if rx_sync_idx == (len(LIFI_SYNC_DATA) - 1):
                break
            else:
                rx_sync_idx += 1
        else:
            rx_sync_idx = 0
    print('RX SYNC OK ...')

    # Preamble reception
    rx_preamble_buff = ser.read(PREAMBLE_LEN)
    if len(rx_preamble_buff) != PREAMBLE_LEN:
        print('RX PREAMBLE TIMEOUT  ...')
        plt.imshow(DOS_FILE_DATA)
        plt.show()
        plt.pause(0.01)
        continue
    rx_data_size = struct.unpack('<L', rx_preamble_buff) 
    rx_data_size = rx_data_size[0]
    print('RX PREAMBLE OK ...')
    print('-> Data size = %d' % rx_data_size)

    # Data reception & write to file
    rx_data_num = 0
    rx_timeout = False
    time_start = time.time()
    while rx_data_num < rx_data_size:
        byte = bytearray(ser.read(1))
        if byte:
            rx_data_buff[rx_data_num] = byte[0]
            rx_data_num += 1
        else:
            rx_timeout = True
            break
    if rx_timeout:
        print('RX DATA TIMEOUT  ...')
        plt.imshow(DOS_FILE_DATA)
        plt.show()
        plt.pause(0.01)
        continue
    time_elapsed = (time.time() - time_start)
    speed = rx_data_size/time_elapsed/1024
    print('Transfer rate = %f kB/s' %speed)
    print('RX DATA OK ...')

    time_start = time.time()
    # Data descramblng
    for idx in range(rx_data_size):
        rx_data_buff[idx] = rx_data_buff[idx] ^ SCRAMBLE_SEQ

    # Write data to file
    with open(TEMP_FILE_PATH, 'wb') as temp_file:
        temp_file.write(rx_data_buff[0:rx_data_size])

    # Format and vizualize data
    try:
        rx_data_img = imageio.imread(TEMP_FILE_PATH).astype(np.uint8)
        
        if rx_data_img.ndim == 2:
            plt.imshow(rx_data_img, cmap='gray')
        else:
            plt.imshow(rx_data_img)
        plt.show()
        plt.pause(0.001)
        
    except:
        print('RX DATA CORRUPT ...')
        
    image = Image.open(TEMP_FILE_PATH)
    image.show
    #os.system('gpicview -single-window %s &' % TEMP_FILE_PATH)
        
    time_elapsed = (time.time() - time_start)
    print('time for image plot %f '%time_elapsed)
        