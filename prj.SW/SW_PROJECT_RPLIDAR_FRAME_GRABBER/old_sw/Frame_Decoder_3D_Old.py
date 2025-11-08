import pandas as pd
from typing import List, Dict
import os
import math
import matplotlib.pyplot as plt
import numpy as np

# Set folder path
frames_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "frames")
output_excel = "LidarParsed.xlsx"
output_png = "Lidar3D.png"

ANGLE_PER_FRAME = 1.0  # unghi pe frame (în grade)
FILTER_STD_FACTOR = 2  # filtrează punctele care sunt la mai mult de 2 std dev de media

def read_bin_data(filename: str) -> bytes:
    try:
        return open(filename, 'rb').read()
    except FileNotFoundError:
        print(f"Error: '{filename}' not found!")
        return bytes()

def get_commands_from_sample_data(data: bytes, commands: List[Dict]) -> None:
    for i in range(len(data) - 1):
        b1, b2 = data[i], data[i + 1]
        if b1 == 0xA5:
            cmd = f"A5 {b2:02X}"
            description = ""
            if b2 == 0x25: description = "STOP request"
            elif b2 == 0x20: description = "SCAN request"
            elif b2 == 0x52: description = "GET HEALTH response"
            elif b2 == 0x50: description = "GET INFO request"
            elif b2 == 0x5A: description = "Response descriptor"
            elif b2 == 0x40: description = "RESET request"
            elif b2 == 0x82: description = "EXPRESS_SCAN request"
            if description:
                commands.append({"Index": i, "Command": cmd, "Description": description})

def get_start_idx_for_measurements(data: bytes) -> int:
    START_SCAN_RESPONSE = b'\xA5\x5A\x05\x00\x00\x40\x81'
    return data.find(START_SCAN_RESPONSE)

def get_valid_measurement_from_sample_data(scan_data: bytes, z_angle_deg: float) -> List[Dict]:
    measurements = []
    for i in range(0, len(scan_data), 5):
        if i + 4 >= len(scan_data):
            break
        b0, b1, b2, b3, b4 = scan_data[i:i+5]
        s_flag = b0 & 0x01
        not_s_flag = (b0 >> 1) & 0x01
        check_bit = (b0 >> 2) & 0x01
        quality = b0 >> 3
        if check_bit != 1 or s_flag == not_s_flag:
            continue

        distance_q2 = (b4 << 8) | b3
        distance_mm = distance_q2 / 4.0
        if distance_mm == 0:
            continue

        angle_q6 = (b2 << 7) + (b1 & 0x7F)
        angle_deg = angle_q6 / 64.0
        angle_deg = angle_deg % 360.0
        angle_rad = math.radians(angle_deg)

        coord_x = distance_mm * math.cos(angle_rad)
        coord_y = distance_mm * math.sin(angle_rad)

        # transformare în coordonate globale ținând cont de rotația turelei
        z_rad = math.radians(z_angle_deg)
        x_global = coord_x * math.cos(z_rad) - coord_y * math.sin(z_rad)
        y_global = coord_x * math.sin(z_rad) + coord_y * math.cos(z_rad)

        measurements.append({
            "Index": i,
            "Quality": quality,
            "Angle (deg)": int(angle_deg),
            "Distance (mm)": int(distance_mm),
            "X_local": round(coord_x, 2),
            "Y_local": round(coord_y, 2),
            "Z_angle (deg)": z_angle_deg,
            "X_global": round(x_global, 2),
            "Y_global": round(y_global, 2)
        })
    return measurements

def decode_all_bin_files(frames_folder: str, output_excel: str):
    all_measurements = []
    all_commands = []

    for file in sorted(os.listdir(frames_folder)):
        if file.lower().endswith(".bin"):
            filepath = os.path.join(frames_folder, file)
            print(f"Processing {file}...")
            data = read_bin_data(filepath)
            if not data:
                continue

            commands = []
            get_commands_from_sample_data(data, commands)
            start_idx = get_start_idx_for_measurements(data)
            if start_idx == -1:
                print(f"Start scan response not found in {file}!")
                continue

            scan_data = data[start_idx + 7:]
            frame_no = int(''.join(filter(str.isdigit, file)))
            z_angle = frame_no * ANGLE_PER_FRAME

            measurements = get_valid_measurement_from_sample_data(scan_data, z_angle)
            all_measurements.extend(measurements)
            all_commands.extend(commands)

    df = pd.DataFrame(all_measurements)
    df_commands = pd.DataFrame(all_commands)

    # filter outlier
    mean_x, std_x = df['X_global'].mean(), df['X_global'].std()
    mean_y, std_y = df['Y_global'].mean(), df['Y_global'].std()
    mask = (df['X_global'] >= mean_x - FILTER_STD_FACTOR*std_x) & (df['X_global'] <= mean_x + FILTER_STD_FACTOR*std_x) & \
           (df['Y_global'] >= mean_y - FILTER_STD_FACTOR*std_y) & (df['Y_global'] <= mean_y + FILTER_STD_FACTOR*std_y)
    df_filtered = df[mask]

    # salvare Excel
    with pd.ExcelWriter(output_excel) as writer:
        df_commands.to_excel(writer, sheet_name="Commands", index=False)
        df.to_excel(writer, sheet_name="Measurements", index=False)
        df_filtered.to_excel(writer, sheet_name="Measurements_Filtered", index=False)

    print(f"--> OUTPUT FILE: {output_excel}")
    print(f"--> Total CMD Count: {len(all_commands)} | Total measurements count: {len(all_measurements)} | Filtered count: {len(df_filtered)}")

    # plot 3D
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')

    xs = df_filtered['X_global']
    ys = df_filtered['Y_global']
    zs = df_filtered['Z_angle (deg)']
    sc = ax.scatter(xs, ys, zs, c=df_filtered['Quality'], cmap='viridis', s=10, marker='o')

    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Turret Rotation (deg)')
    ax.set_title('LIDAR 3D Measurements - Circular Layout (Filtered)')
    plt.colorbar(sc, label='Quality')

    # salvare PNG
    plt.savefig(output_png, dpi=300)
    print(f"--> 3D plot saved as: {output_png}")
    plt.show()

if __name__ == "__main__":
    decode_all_bin_files(frames_folder, output_excel)
