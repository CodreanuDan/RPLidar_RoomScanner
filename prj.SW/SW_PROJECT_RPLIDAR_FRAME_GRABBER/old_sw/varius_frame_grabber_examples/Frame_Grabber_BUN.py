import serial as pyserial_lib 
import time
import os

FILE_NAME = "sample_data.txt"

# If DEBUG is set to true the data is not wrriten in the sample file 
DEBUG = False
# DEBUG = True

class CommandFlags:
    STOP   = 0x00
    START  = 0x01
    RESUME = 0x02 

class SerialReader():
    
    def __init__(self):
        self.flags = CommandFlags()
        
    def get_cmd_name(self, flag):
        """Helper function for printing command names."""
        if flag == self.flags.START: return "START"
        if flag == self.flags.RESUME: return "RESUME"
        if flag == self.flags.STOP: return "STOP"
        return "UNKNOWN"
        
    def send_command(self, serial_com: pyserial_lib.Serial, command_flag: int):
        """ Send 1 byte command to MSP430 """
        
        cmd_bytes = pyserial_lib.to_bytes([command_flag])
        print(f"[CMD] Sending command 0x{command_flag:02x} ('{self.get_cmd_name(command_flag)}')...")
        serial_com.write(cmd_bytes)
        # Wait for echo
        echo_response = serial_com.read(1)
        if echo_response == cmd_bytes:
            print(f"[ACK] Command 0x{command_flag:02x} confirmed.")
        else:
            print(f"[ERR] Command echo mismatch: Expected {cmd_bytes}, received {echo_response}.")
        
    def run_scan_cycle(self):
        """ Manage communication with MS430 """
        
        with open(FILE_NAME, 'a') as f:
            
            print(f"Record LIDAR data in file: '{os.path.abspath(f.name)}'.")
            print("-" * 40)

            try:
                serial_com = pyserial_lib.Serial(port="COM5", baudrate=115200, timeout=1) 
                print(f"PORT: {serial_com.port} is open, baudrate = {serial_com.baudrate} bps.")
                time.sleep(2) 
                
                print(f"PORT: {serial_com.port}, send start command {self.flags.START}")
                self.send_command(serial_com, self.flags.START)
                start_time = time.time()
                
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
                    
                    
if __name__ == "__main__":
    
    serialReader = SerialReader();
    serialReader.run_scan_cycle()