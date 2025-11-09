import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog, messagebox
import threading
import serial as pyserial_lib
import time
import os
from datetime import datetime
import pandas as pd
import math
import matplotlib.pyplot as plt
import plotly.graph_objects as go
from typing import List, Dict
import webbrowser

# Configuration
BAUDRATE = 115200
DEFAULT_PORT = "COM5"
MEASUREMENTS_BASE = "measurements"

# Command flags
class CommandFlags:
    STOP = 0x00
    START = 0x01
    RESUME = 0x02

# End markers
END_MARKER = bytes([0xFF, 0xFF, 0xFF, 0xFF])
END_CYCLE_MARKER = bytes([0xFF, 0xFF, 0xFA, 0xFA])

# Processing parameters
FILTER_STD_FACTOR = 2.5


class LIDARScannerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("LIDAR 3D Scanner Control")
        self.root.geometry("900x700")
        
        # Variables
        self.is_scanning = False
        self.serial_port = None
        self.current_measurement_folder = None
        self.frame_count = 0
        self.target_angle = 180
        self.com_port = tk.StringVar(value=DEFAULT_PORT)
        self.angle_step = tk.IntVar(value=1)
        self.show_settings = tk.BooleanVar(value=False)
        self.expected_frames = 180
        
        self.setup_gui()
        
    def setup_gui(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(4, weight=1)
        
        # Connection Frame
        conn_frame = ttk.LabelFrame(main_frame, text="Connection", padding="5")
        conn_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Label(conn_frame, text="COM Port:").grid(row=0, column=0, padx=5)
        ttk.Entry(conn_frame, textvariable=self.com_port, width=10).grid(row=0, column=1, padx=5)
        
        # Measurement Frame
        meas_frame = ttk.LabelFrame(main_frame, text="Measurement Control", padding="5")
        meas_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        self.info_label = ttk.Label(meas_frame, text="Expected Frames: 180 (1° step)", 
                                     font=("Arial", 10, "bold"))
        self.info_label.grid(row=0, column=0, padx=5, sticky=tk.W, columnspan=2)
        
        self.start_btn = ttk.Button(meas_frame, text="Start Measurement", command=self.start_measurement)
        self.start_btn.grid(row=0, column=2, padx=10)
        
        self.stop_btn = ttk.Button(meas_frame, text="Stop", command=self.stop_measurement, state=tk.DISABLED)
        self.stop_btn.grid(row=0, column=3, padx=5)
        
        self.settings_btn = ttk.Checkbutton(meas_frame, text="⚙ Settings", 
                                           variable=self.show_settings, 
                                           command=self.toggle_settings)
        self.settings_btn.grid(row=0, column=4, padx=10)
        
        # Settings frame
        self.settings_frame = ttk.Frame(meas_frame, padding="5")
        
        ttk.Label(self.settings_frame, text="Angle Step (°/frame):").grid(row=0, column=0, padx=5, sticky=tk.W)
        angle_step_combo = ttk.Combobox(self.settings_frame, textvariable=self.angle_step, 
                                        values=[1, 2, 4], width=8, state='readonly')
        angle_step_combo.grid(row=0, column=1, padx=5)
        angle_step_combo.bind('<<ComboboxSelected>>', self.update_expected_frames)
        
        ttk.Label(self.settings_frame, text="(1° = 180 frames, 2° = 90 frames, 4° = 45 frames)").grid(
            row=1, column=0, columnspan=2, padx=5, pady=2, sticky=tk.W)
        
        self.frame_label = ttk.Label(meas_frame, text="Frames Recorded: 0 / 180", font=("Arial", 12, "bold"))
        self.frame_label.grid(row=3, column=0, columnspan=5, pady=10)
        
        # Processing Frame
        proc_frame = ttk.LabelFrame(main_frame, text="Data Processing", padding="5")
        proc_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        self.process_btn = ttk.Button(proc_frame, text="Process Frames", command=self.process_frames)
        self.process_btn.grid(row=0, column=0, padx=5)
        
        self.open_html_btn = ttk.Button(proc_frame, text="Open 3D Plot (HTML)", command=self.open_html, state=tk.DISABLED)
        self.open_html_btn.grid(row=0, column=1, padx=5)
        
        self.open_excel_btn = ttk.Button(proc_frame, text="Open Excel", command=self.open_excel, state=tk.DISABLED)
        self.open_excel_btn.grid(row=0, column=2, padx=5)
        
        self.open_folder_btn = ttk.Button(proc_frame, text="Open Folder", command=self.open_folder, state=tk.DISABLED)
        self.open_folder_btn.grid(row=0, column=3, padx=5)
        
        # Progress Bar
        self.progress = ttk.Progressbar(main_frame, mode='indeterminate')
        self.progress.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        # Log Window
        log_frame = ttk.LabelFrame(main_frame, text="Log", padding="5")
        log_frame.grid(row=4, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=20, state='disabled')
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
    def log(self, message):
        self.log_text.config(state='normal')
        self.log_text.insert(tk.END, f"[{datetime.now().strftime('%H:%M:%S')}] {message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state='disabled')
    
    def update_expected_frames(self, event=None):
        step = self.angle_step.get()
        self.expected_frames = 180 // step
        self.info_label.config(text=f"Expected Frames: {self.expected_frames} ({step}° step)")
        self.frame_label.config(text=f"Frames Recorded: {self.frame_count} / {self.expected_frames}")
    
    def toggle_settings(self):
        if self.show_settings.get():
            self.settings_frame.grid(row=2, column=0, columnspan=5, sticky=(tk.W, tk.E), pady=5)
        else:
            self.settings_frame.grid_forget()
        
    def start_measurement(self):
        try:
            step = self.angle_step.get()
            self.expected_frames = 180 // step
            self.target_angle = self.expected_frames
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.current_measurement_folder = os.path.join(
                MEASUREMENTS_BASE, 
                f"meas_{timestamp}_{self.expected_frames}frames_{step}deg"
            )
            os.makedirs(self.current_measurement_folder, exist_ok=True)
            os.makedirs(os.path.join(self.current_measurement_folder, "frames"), exist_ok=True)
            
            self.log(f"Created measurement folder: {self.current_measurement_folder}")
            self.log(f"Target: {self.expected_frames} frames at {step}° per frame")
            
            self.start_btn.config(state=tk.DISABLED)
            self.stop_btn.config(state=tk.NORMAL)
            self.is_scanning = True
            self.frame_count = 0
            self.update_frame_counter()
            
            threading.Thread(target=self.measurement_worker, daemon=True).start()
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to start measurement: {e}")
            self.log(f"ERROR: {e}")
            
    def measurement_worker(self):
        sample_file = os.path.join(self.current_measurement_folder, "sample_data.txt")
        
        try:
            self.serial_port = pyserial_lib.Serial(port=self.com_port.get(), baudrate=BAUDRATE, timeout=1)
            self.log(f"Opened {self.serial_port.port} @ {BAUDRATE}")
            time.sleep(2)
            
            step = self.angle_step.get()
            self.send_start_command_with_preset(step)
            
            buffer = bytearray()
            
            with open(sample_file, "a") as f:
                while self.is_scanning:
                    if self.serial_port.in_waiting > 0:
                        chunk = self.serial_port.read(self.serial_port.in_waiting or 1)
                        if chunk:
                            buffer.extend(chunk)
                            hex_line = ' '.join(f'0x{b:02X}' for b in chunk) + '\n'
                            f.write(hex_line)
                            
                            if END_MARKER in buffer:
                                if END_CYCLE_MARKER in buffer:
                                    self.log("End of cycle detected!")
                                    self.save_frame(sample_file)
                                    self.stop_measurement()
                                    break
                                else:
                                    self.frame_count += 1
                                    self.root.after(0, self.update_frame_counter)
                                    self.log(f"Frame {self.frame_count} completed")
                                    self.save_frame(sample_file)
                                    
                                    if self.frame_count >= self.target_angle:
                                        self.log(f"Target of {self.target_angle} frames reached")
                                        self.stop_measurement()
                                        break
                                    
                                    time.sleep(2)
                                    self.send_command(CommandFlags.RESUME)
                                    buffer.clear()
                    else:
                        time.sleep(0.01)
                        
        except Exception as e:
            self.log(f"ERROR: {e}")
            messagebox.showerror("Error", str(e))
        finally:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
                self.log("Serial port closed")
    
    def send_start_command_with_preset(self, angle_step):
        cmd_bytes = bytes([CommandFlags.START, angle_step])
        self.log(f"Sending START command with preset: 0x{CommandFlags.START:02X} 0x{angle_step:02X}")
        self.serial_port.write(cmd_bytes)
        
        time.sleep(0.1)
        echo = self.serial_port.read(1)
        
        if echo == bytes([CommandFlags.START]):
            self.log(f"✓ Command confirmed: START received")
            
            if angle_step in [2, 4]:
                time.sleep(0.05)
                preset_echo = self.serial_port.read(1)
                if preset_echo == bytes([angle_step]):
                    self.log(f"✓ Preset confirmed: {angle_step}° per frame")
                else:
                    self.log(f"⚠ Warning: Preset echo mismatch")
        else:
            self.log(f"⚠ Warning: Command echo mismatch")
                
    def send_command(self, command_flag):
        cmd_bytes = bytes([command_flag])
        cmd_name = {0: "STOP", 1: "START", 2: "RESUME"}.get(command_flag, "UNKNOWN")
        self.log(f"Sending command: 0x{command_flag:02X} ({cmd_name})")
        self.serial_port.write(cmd_bytes)
        echo = self.serial_port.read(1)
        if echo == cmd_bytes:
            self.log(f"✓ Command confirmed: {cmd_name}")
            
    def save_frame(self, sample_file):
        with open(sample_file, "r") as f:
            text = f.read()
            
        hex_bytes = [b for b in text.split() if b.startswith("0x")]
        if not hex_bytes:
            self.log("WARNING: No data to save")
            return
            
        data = bytes(int(b, 16) for b in hex_bytes)
        frame_path = os.path.join(self.current_measurement_folder, "frames", f"frame_{self.frame_count:04d}.bin")
        
        with open(frame_path, "wb") as fbin:
            fbin.write(data)
            
        self.log(f"Saved {frame_path} ({len(data)} bytes)")
        open(sample_file, "w").close()
        
    def update_frame_counter(self):
        self.frame_label.config(text=f"Frames Recorded: {self.frame_count} / {self.expected_frames}")
        
    def stop_measurement(self):
        self.is_scanning = False
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.log("Measurement stopped")
        
    def process_frames(self):
        folder = filedialog.askdirectory(initialdir=MEASUREMENTS_BASE, title="Select Measurement Folder")
        if not folder:
            return
            
        frames_folder = os.path.join(folder, "frames")
        if not os.path.exists(frames_folder):
            messagebox.showerror("Error", "No 'frames' folder found")
            return
        
        folder_name = os.path.basename(folder)
        angle_step = 1 # Default
        parts = folder_name.split('_')
        for i, part in enumerate(parts):
            if part.endswith("deg") and i > 0:
                try:
                    # The angle is the part just before 'deg' or the whole part if it's just '4deg'
                    if part.endswith("deg") and len(part) > 3:
                        angle_step = int(part[:-3]) # e.g., '4deg' -> '4'
                    elif parts[i-1].isdigit():
                         angle_step = int(parts[i-1])
                         
                    self.log(f"Detected angle step from folder name: {angle_step}°")
                    break # Exit loop once found
                except ValueError:
                    # Log the failure, but let angle_step remain 1
                    self.log(f"Could not parse angle from part: {part}. Using default 1°.")
                
        self.log(f"Processing frames from: {folder}")
        self.progress.start()
        
        threading.Thread(target=self.process_worker, args=(folder, frames_folder, angle_step), daemon=True).start()
        
    def process_worker(self, measurement_folder, frames_folder, angle_step):
        try:
            self.log("Starting frame processing...")
            self.log(f"Using angle step: {angle_step}° per frame")
            
            all_measurements = []
            
            files = sorted([f for f in os.listdir(frames_folder) if f.lower().endswith(".bin")])
            self.log(f"Found {len(files)} frame files")
            
            for idx, file in enumerate(files):
                filepath = os.path.join(frames_folder, file)
                self.log(f"Processing {file}...")
                
                with open(filepath, 'rb') as f:
                    data = f.read()
                    
                if not data:
                    continue
                    
                start_idx = data.find(b'\xA5\x5A\x05\x00\x00\x40\x81')
                if start_idx == -1:
                    self.log(f"WARNING: No scan response in {file}")
                    continue
                    
                scan_data = data[start_idx + 7:]
                turret_angle = idx * angle_step
                
                measurements = self.decode_measurements(scan_data, turret_angle)
                all_measurements.extend(measurements)
                
            if not all_measurements:
                self.log("ERROR: No measurements found!")
                self.root.after(0, self.progress.stop)
                return
                
            self.log(f"Total measurements: {len(all_measurements)}")
            
            df = pd.DataFrame(all_measurements)
            df_filtered = self.filter_outliers(df)
            
            self.log(f"After filtering: {len(df_filtered)} measurements")
            self.log(f"Turret angle range: {df_filtered['Turret_Angle (deg)'].min()}° to {df_filtered['Turret_Angle (deg)'].max()}°")
            
            excel_path = os.path.join(measurement_folder, "LidarParsed.xlsx")
            with pd.ExcelWriter(excel_path) as writer:
                df.to_excel(writer, sheet_name="Measurements", index=False)
                df_filtered.to_excel(writer, sheet_name="Measurements_Filtered", index=False)
            self.log(f"Saved Excel: {excel_path}")
            
            self.generate_plots(df_filtered, measurement_folder)
            
            self.current_measurement_folder = measurement_folder
            self.root.after(0, lambda: self.open_html_btn.config(state=tk.NORMAL))
            self.root.after(0, lambda: self.open_excel_btn.config(state=tk.NORMAL))
            self.root.after(0, lambda: self.open_folder_btn.config(state=tk.NORMAL))
            
            self.log("Processing complete!")
            self.root.after(0, self.progress.stop)
            self.root.after(0, lambda: messagebox.showinfo("Success", "Frame processing completed!"))
            
        except Exception as e:
            self.log(f"ERROR during processing: {e}")
            self.root.after(0, self.progress.stop)
            self.root.after(0, lambda: messagebox.showerror("Error", str(e)))
            
    def decode_measurements(self, scan_data: bytes, turret_angle_deg: float) -> List[Dict]:
        measurements = []
        
        for i in range(0, len(scan_data), 5):
            if i + 4 >= len(scan_data):
                break
                
            b0, b1, b2, b3, b4 = scan_data[i:i+5]
            check_bit = (b0 >> 2) & 0x01
            s_flag = b0 & 0x01
            not_s_flag = (b0 >> 1) & 0x01
            quality = b0 >> 3
            
            if check_bit != 1 or s_flag == not_s_flag:
                continue
                
            distance_q2 = (b4 << 8) | b3
            distance_mm = distance_q2 / 4.0
            
            if distance_mm == 0 or distance_mm > 12000:
                continue
                
            angle_q6 = (b2 << 7) + (b1 & 0x7F)
            angle_deg = (angle_q6 / 64.0) % 360.0
            
            lidar_angle_rad = math.radians(angle_deg)
            turret_rad = math.radians(turret_angle_deg)
            
            r_in_plane = distance_mm * math.sin(lidar_angle_rad)
            z_global = distance_mm * math.cos(lidar_angle_rad)
            
            x_global = r_in_plane * math.cos(turret_rad)
            y_global = r_in_plane * math.sin(turret_rad)
            
            measurements.append({
                "Quality": quality,
                "LIDAR_Angle (deg)": round(angle_deg, 1),
                "Distance (mm)": int(distance_mm),
                "Turret_Angle (deg)": turret_angle_deg,
                "X (mm)": round(x_global, 2),
                "Y (mm)": round(y_global, 2),
                "Z (mm)": round(z_global, 2)
            })
            
        return measurements
        
    def filter_outliers(self, df):
        for col in ['X (mm)', 'Y (mm)', 'Z (mm)']:
            mean, std = df[col].mean(), df[col].std()
            df = df[(df[col] >= mean - FILTER_STD_FACTOR*std) & 
                   (df[col] <= mean + FILTER_STD_FACTOR*std)]
        return df
        
    def generate_plots(self, df, output_folder):
        self.log("Generating plots...")
        
        self.log(f"X range: {df['X (mm)'].min():.0f} to {df['X (mm)'].max():.0f}")
        self.log(f"Y range: {df['Y (mm)'].min():.0f} to {df['Y (mm)'].max():.0f}")
        self.log(f"Z range: {df['Z (mm)'].min():.0f} to {df['Z (mm)'].max():.0f}")
        
        html_path = os.path.join(output_folder, "Lidar3D_Interactive.html")
        self.create_interactive_plot(df, html_path)
        
        png_path = os.path.join(output_folder, "Lidar3D.png")
        self.create_static_plot(df, png_path)
        
        multiview_path = os.path.join(output_folder, "Lidar_MultiView.png")
        self.create_multiview_plot(df, multiview_path)
        
    def create_interactive_plot(self, df, output_path):
        fig = go.Figure(data=[go.Scatter3d(
            x=df["X (mm)"], y=df["Y (mm)"], z=df["Z (mm)"],
            mode='markers',
            marker=dict(size=2, color=df["Z (mm)"], colorscale='Viridis', opacity=0.7)
        )])
        
        x_min, x_max = df["X (mm)"].min(), df["X (mm)"].max()
        y_min, y_max = df["Y (mm)"].min(), df["Y (mm)"].max()
        z_min, z_max = df["Z (mm)"].min(), df["Z (mm)"].max()
        
        box_edges = [
            ([x_min, x_max], [y_min, y_min], [z_min, z_min]),
            ([x_max, x_max], [y_min, y_max], [z_min, z_min]),
            ([x_max, x_min], [y_max, y_max], [z_min, z_min]),
            ([x_min, x_min], [y_max, y_min], [z_min, z_min]),
            ([x_min, x_max], [y_min, y_min], [z_max, z_max]),
            ([x_max, x_max], [y_min, y_max], [z_max, z_max]),
            ([x_max, x_min], [y_max, y_max], [z_max, z_max]),
            ([x_min, x_min], [y_max, y_min], [z_max, z_max]),
            ([x_min, x_min], [y_min, y_min], [z_min, z_max]),
            ([x_max, x_max], [y_min, y_min], [z_min, z_max]),
            ([x_max, x_max], [y_max, y_max], [z_min, z_max]),
            ([x_min, x_min], [y_max, y_max], [z_min, z_max]),
        ]
        
        for i, (x, y, z) in enumerate(box_edges):
            fig.add_trace(go.Scatter3d(
                x=x, y=y, z=z, mode="lines",
                line=dict(color="red", width=3),
                showlegend=(i == 0), name="Room Box" if i == 0 else None
            ))
        
        fig.add_trace(go.Scatter3d(
            x=[0, 0], y=[0, 0], z=[z_min, z_max],
            mode="lines",
            line=dict(color="red", width=5, dash="dash"),
            name="Origin (0,0)"
        ))
            
        fig.update_layout(
            title="LIDAR 3D Room Scan (Interactive)",
            scene=dict(
                xaxis_title='X (mm)', yaxis_title='Y (mm)', zaxis_title='Z (mm)',
                aspectmode='data'
            ),
            width=1200, height=900
        )
        
        fig.write_html(output_path)
        self.log(f"Saved HTML plot: {output_path}")
        
    def create_static_plot(self, df, output_path):
        fig = plt.figure(figsize=(16, 12))
        ax = fig.add_subplot(111, projection='3d')
        
        sc = ax.scatter(df['X (mm)'], df['Y (mm)'], df['Z (mm)'], 
                       c=df['Z (mm)'], cmap='viridis', s=0.5, alpha=0.6)
        
        z_min, z_max = df['Z (mm)'].min(), df['Z (mm)'].max()
        ax.plot([0, 0], [0, 0], [z_min, z_max],
                color='red', linewidth=2, linestyle='--', label='Origin (0,0)')
        
        ax.set_xlabel('X (mm) - Width')
        ax.set_ylabel('Y (mm) - Depth')
        ax.set_zlabel('Z (mm) - Height')
        ax.set_title('LIDAR 3D Room Scan')
        ax.legend()
        plt.colorbar(sc, label='Height (mm)', shrink=0.5)
        
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()
        self.log(f"Saved PNG plot: {output_path}")
        
    def create_multiview_plot(self, df, output_path):
        fig, axes = plt.subplots(2, 2, figsize=(16, 14))
        
        sc1 = axes[0, 0].scatter(df['X (mm)'], df['Y (mm)'], c=df['Z (mm)'], cmap='viridis', s=1, alpha=0.6)
        axes[0, 0].set_xlabel('X (mm)')
        axes[0, 0].set_ylabel('Y (mm)')
        axes[0, 0].set_title('Top View')
        axes[0, 0].set_aspect('equal')
        plt.colorbar(sc1, ax=axes[0, 0])
        
        sc2 = axes[0, 1].scatter(df['X (mm)'], df['Z (mm)'], c=df['Y (mm)'], cmap='plasma', s=1, alpha=0.6)
        axes[0, 1].set_xlabel('X (mm)')
        axes[0, 1].set_ylabel('Z (mm)')
        axes[0, 1].set_title('Front View')
        axes[0, 1].set_aspect('equal')
        plt.colorbar(sc2, ax=axes[0, 1])
        
        sc3 = axes[1, 0].scatter(df['Y (mm)'], df['Z (mm)'], c=df['X (mm)'], cmap='cool', s=1, alpha=0.6)
        axes[1, 0].set_xlabel('Y (mm)')
        axes[1, 0].set_ylabel('Z (mm)')
        axes[1, 0].set_title('Side View')
        axes[1, 0].set_aspect('equal')
        plt.colorbar(sc3, ax=axes[1, 0])
        
        axes[1, 1].hist(df['Turret_Angle (deg)'], bins=50, color='skyblue', edgecolor='black')
        axes[1, 1].set_xlabel('Turret Angle (degrees)')
        axes[1, 1].set_ylabel('Point Count')
        axes[1, 1].set_title('Turret Coverage')
        
        plt.tight_layout()
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()
        self.log(f"Saved multi-view plot: {output_path}")
        
    def open_html(self):
        if self.current_measurement_folder:
            html_path = os.path.join(self.current_measurement_folder, "Lidar3D_Interactive.html")
            if os.path.exists(html_path):
                webbrowser.open(html_path)
                self.log(f"Opened: {html_path}")
                
    def open_excel(self):
        if self.current_measurement_folder:
            excel_path = os.path.join(self.current_measurement_folder, "LidarParsed.xlsx")
            if os.path.exists(excel_path):
                os.startfile(excel_path)
                self.log(f"Opened: {excel_path}")
                
    def open_folder(self):
        if self.current_measurement_folder:
            os.startfile(self.current_measurement_folder)
            self.log(f"Opened folder: {self.current_measurement_folder}")


if __name__ == "__main__":
    root = tk.Tk()
    app = LIDARScannerGUI(root)
    root.mainloop()