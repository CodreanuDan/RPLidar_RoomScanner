import serial as pyserial_lib 
import time
import os

FILE_NAME = "sample_data.txt"
DEBUG = True

with open(FILE_NAME, 'a') as f:
    
    print(f"Record LIDAR data in file: '{os.path.abspath(f.name)}'.")
    print("-" * 40)

    try:
        serial_com = pyserial_lib.Serial(port="COM5", baudrate=115200, timeout=1) 
        print(f"PORT: {serial_com.port} is open, baudrate = {serial_com.baudrate} bps.")
        time.sleep(2) 
        
        while True:

            if serial_com.in_waiting > 0:
                data_raw = serial_com.read()
                
                if data_raw:
                    data_hex_line = ' '.join(f'{b:#04x}' for b in data_raw) + '\n'
                    
                    if not DEBUG:
                        f.write(data_hex_line)
                    
                    print(data_hex_line.strip())  
            else:
                time.sleep(0.01) 

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt")

    finally:
        if 'serial_com' in locals() and serial_com.is_open:
            serial_com.close()
            print(f"{serial_com.port} closed !")