import serial as pyserial_lib
import time
import os

FILE_NAME = "sample_data.txt"
OUTPUT_DIR = "frames"
BAUDRATE = 115200
PORT = "COM5"

DEBUG = False

END_MARKER = bytes([0xFF, 0xFF, 0xFF, 0xFF])  # STOP frame marker


class CommandFlags:
    STOP   = 0x00
    START  = 0x01
    RESUME = 0x02


class SerialReader:
    def __init__(self):
        self.flags = CommandFlags()
        self.frame_no = 0
        os.makedirs(OUTPUT_DIR, exist_ok=True)

    def get_cmd_name(self, flag: int) -> str:
        if flag == self.flags.START: return "START"
        if flag == self.flags.RESUME: return "RESUME"
        if flag == self.flags.STOP: return "STOP"
        return "UNKNOWN"

    def send_command(self, serial_com: pyserial_lib.Serial, command_flag: int):
        """Send 1-byte command to MSP430."""
        cmd_bytes = pyserial_lib.to_bytes([command_flag])
        print(f"[CMD] Sending 0x{command_flag:02X} ({self.get_cmd_name(command_flag)})")
        serial_com.write(cmd_bytes)
        echo_response = serial_com.read(1)
        if echo_response == cmd_bytes:
            print(f"[ACK] Command 0x{command_flag:02X} confirmed.")
        else:
            print(f"[WARN] Echo mismatch. Got {echo_response}")

    def save_bin_from_sample(self):
        """Convert sample_data.txt to binary file."""
        with open(FILE_NAME, "r") as f:
            text = f.read()

        # Extract only valid hex bytes (ignores # and newlines)
        hex_bytes = [b for b in text.split() if b.startswith("0x")]
        if not hex_bytes:
            print("[WARN] sample_data.txt is empty or invalid.")
            return None

        data = bytes(int(b, 16) for b in hex_bytes)
        fname = f"frame_{self.frame_no:04d}.bin"
        bin_path = os.path.join(OUTPUT_DIR, fname)

        with open(bin_path, "wb") as fbin:
            fbin.write(data)

        print(f"[SAVE] Frame {self.frame_no} saved -> {bin_path} ({len(data)} bytes)")
        return bin_path

    def clear_sample_file(self):
        """Erase sample_data.txt content."""
        open(FILE_NAME, "w").close()
        print("[INFO] Cleared sample_data.txt")

    def run_test_cycle(self):
        """Main interactive test loop."""
        with open(FILE_NAME, "a") as f:
            print(f"[INFO] Recording LIDAR data to '{os.path.abspath(f.name)}'")
            print("-" * 50)

            try:
                serial_com = pyserial_lib.Serial(port=PORT, baudrate=BAUDRATE, timeout=1)
                print(f"[OPEN] {serial_com.port} @ {BAUDRATE}")
                time.sleep(2)
                self.send_command(serial_com, self.flags.START)
                print("[INFO] Started measurement cycle.")

                buffer = bytearray()

                while True:
                    if serial_com.in_waiting > 0:
                        chunk = serial_com.read(serial_com.in_waiting or 1)
                        if chunk:
                            buffer.extend(chunk)
                            hex_line = ' '.join(f'0x{b:02X}' for b in chunk) + '\n'
                            if not DEBUG:
                                f.write(hex_line)
                            print(hex_line.strip())

                            # detect end marker (A5 25)
                            if END_MARKER in buffer:
                                self.frame_no += 1
                                print(f"\n[INFO] End marker detected! Frame {self.frame_no} ready.")
                                
                                # interactive control
                                user_input = input("Scriem în bin? (ok / skip): ").strip().lower()
                                if user_input == "ok":
                                    self.save_bin_from_sample()
                                    self.clear_sample_file()

                                user_input = input("Trimitem resume? (ok / skip): ").strip().lower()
                                if user_input == "ok":
                                    self.send_command(serial_com, self.flags.RESUME)
                                    print("[INFO] Resume command sent.")
                                else:
                                    print("[PAUSE] Waiting... Press Ctrl+C to exit or continue manually.")

                                buffer.clear()
                                print("-" * 40)
                    else:
                        time.sleep(0.01)

            except KeyboardInterrupt:
                print("\n[INTERRUPT] Stopping by user.")

            finally:
                if 'serial_com' in locals() and serial_com.is_open:
                    serial_com.close()
                    print(f"[CLOSE] {serial_com.port} closed.")


if __name__ == "__main__":
    SerialReader().run_test_cycle()
