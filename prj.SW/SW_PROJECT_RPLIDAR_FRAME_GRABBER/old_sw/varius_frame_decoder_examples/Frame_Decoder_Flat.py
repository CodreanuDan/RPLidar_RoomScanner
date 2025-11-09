import pandas as pd
from typing import List, Dict
import os
import math
import matplotlib.pyplot as plt

# Set folder path
frames_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "frames")
output_excel = "LidarParsed.xlsx"

def read_bin_data(filename: str) -> bytes:
    try:
        with open(filename, 'rb') as f:
            data = f.read()
        return data
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
    start_idx = data.find(START_SCAN_RESPONSE)
    return start_idx

def get_valid_measurement_from_sample_data(scan_data: bytes) -> List[Dict]:
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

        measurements.append({
            "Index": i,
            "StartFlag": s_flag,
            "Quality": quality,
            "Angle (deg)": int(angle_deg),
            "Distance (mm)": int(distance_mm),
            "X (mm)": round(coord_x, 2),
            "Y (mm)": round(coord_y, 2)
        })
    measurements.sort(key=lambda item: item['Angle (deg)'])
    return measurements

def aggregate_measurements(measurements: List[Dict]) -> pd.DataFrame:
    if not measurements:
        return pd.DataFrame()
    df = pd.DataFrame(measurements)
    aggregated_df = df.groupby('Angle (deg)', as_index=False).agg(
        Distance_mm_Avg=('Distance (mm)', 'mean'),
        Quality_Avg=('Quality', 'mean'),
        Count=('Quality', 'count')
    )
    aggregated_df['Angle (rad)'] = aggregated_df['Angle (deg)'].apply(math.radians)
    R_avg = aggregated_df['Distance_mm_Avg']
    Theta_rad = aggregated_df['Angle (rad)']
    aggregated_df['X (mm)_Avg'] = (R_avg * Theta_rad.apply(math.cos)).round(2)
    aggregated_df['Y (mm)_Avg'] = (R_avg * Theta_rad.apply(math.sin)).round(2)
    aggregated_df = aggregated_df.rename(columns={'Distance_mm_Avg': 'Distance (mm)_Avg'})
    aggregated_df = aggregated_df[['Angle (deg)', 'Distance (mm)_Avg', 'X (mm)_Avg', 'Y (mm)_Avg', 'Quality_Avg', 'Count']]
    return aggregated_df

def plot_3d_measurements(df: pd.DataFrame):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    xs = df['X (mm)_Avg']
    ys = df['Y (mm)_Avg']
    zs = 0 
    
    sc = ax.scatter(xs, ys, zs, c=df['Quality_Avg'], cmap='viridis', marker='o')
    
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z = 0')
    ax.set_title('LIDAR 3D Measurements')
    
    ax.set_zlim(0, 1)  
    plt.colorbar(sc, label='Quality')
    plt.show()

def decode_all_bin_files(frames_folder: str, output_excel: str):
    all_measurements = []
    all_commands = []
    
    for file in os.listdir(frames_folder):
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
            measurements = get_valid_measurement_from_sample_data(scan_data)
            all_measurements.extend(measurements)
            all_commands.extend(commands)

    aggregated_df = aggregate_measurements(all_measurements)
    
    with pd.ExcelWriter(output_excel) as writer:
        pd.DataFrame(all_commands).to_excel(writer, sheet_name="Commands", index=False)
        pd.DataFrame(all_measurements).to_excel(writer, sheet_name="Measurements", index=False)
        aggregated_df.to_excel(writer, sheet_name="Aggregated Measurements", index=False)

    print(f"--> OUTPUT FILE: {output_excel}")
    print(f"--> Total CMD Count: {len(all_commands)} | Total measurements count: {len(all_measurements)}")
    print(f"--> Aggregated measurements count: {len(aggregated_df)}")

    plot_3d_measurements(aggregated_df)

if __name__ == "__main__":
    decode_all_bin_files(frames_folder, output_excel)
