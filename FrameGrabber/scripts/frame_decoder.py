import re
import pandas as pd
from typing import List, Dict
import os
import math

os.chdir(os.path.dirname(os.path.abspath(__file__)))

# Open file and extract HEX data
def read_sample_data(filename: str) -> bytes:
    
    try:
        with open(filename, 'r') as f:
            text = f.read()
    except FileNotFoundError:
        print(f"Error: '{filename}' not found !")
        return bytes()
        
    hex_bytes = re.findall(r"[A-Fa-f0-9]{2}", text)
    data = bytes.fromhex(" ".join(hex_bytes))
    
    return data

def get_commands_from_sample_data(data: bytes, commands: List[Dict]) -> None:
    
    for i in range(len(data) - 1):
        
        b1, b2 = data[i], data[i + 1]
        
        if b1 == 0xA5:
            
            cmd = f"A5 {b2:02X}"
            description = ""

            if b2 == 0x25:
                description = "STOP request"
            elif b2 == 0x20:
                description = "SCAN request"
            elif b2 == 0x52:
                description = "GET HEALTH response"
            elif b2 == 0x50:
                description = "GET INFO request"
            elif b2 == 0x5A:
                description = "Response descriptor (check next bytes)"
            elif b2 == 0x40:
                description = "RESET request"
            elif b2 == 0x82:
                description = "EXPRESS_SCAN request"
            if description:
                commands.append({"Index": i, "Command": cmd, "Description": description})
                
def get_start_idx_for_measurements(data: bytes, commands: List[Dict]) -> int:
    
    START_SCAN_RESPONSE = "A5 5A 05 00 00 40 81"
    scan_start_pattern = bytes.fromhex(START_SCAN_RESPONSE)
    start_idx = data.find(scan_start_pattern)
    if start_idx == -1:
        return -1
    else:
        commands.append({"Index": start_idx, "Command": START_SCAN_RESPONSE, "Description": "Start scan response"})
    
    return start_idx

def get_valid_measurement_from_sample_data(scan_data: bytes, measurements: List[Dict]) -> None:
        
        for i in range(0,len(scan_data), 5):
            if i + 4 >= len(scan_data):
                break
            
            b0, b1, b2, b3, b4 = scan_data[i:i + 5]
            
            s_flag = b0 & 0x01
            not_s_flag = (b0 >> 1) & 0x01
            check_bit = (b0 >> 2) & 0x01
            quality = b0 >> 3
            
            
            # validate package 
            if check_bit != 1 or s_flag == not_s_flag:
                continue
            
            # calculate angle_q6 and angle_deg
            # angle_q6 = ((b2 << 8) | b1) > 1
            angle_q6 = (b2 << 7) + (b1 & 0x7F)
            angle_deg = angle_q6 / 64.0
            angle_deg = angle_deg % 360.0 # Angle wrap-around 0-360 degrees
            
            # calculate distance_q2 and distance_mm
            distance_q2 = (b4 << 8) | b3
            distance_mm = distance_q2 / 4.0
            
            if distance_mm == 0:
                continue
            
            # polar coord to vizualize in scatter plot 
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
   
def aggregate_measurements(measurements: List[Dict]) -> pd.DataFrame:
    """
    Calculează media distanței și a calității pentru măsurătorile cu același unghi,
    și recalculează coordonatele carteziene agregate.
    """
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
         
def decode_sample_data(data: bytes, filename: str) -> None:
    
    commands = []
    measurements = []
    
    # 1. Check for commands 
    get_commands_from_sample_data(data, commands)
                
    # 2. Check for command response
    start_idx = get_start_idx_for_measurements(data, commands)
    ''' Real start idx is after the scand data response (7 bytes)'''
    scan_data = data[start_idx + 7:]
    
    # 3. Decode 5 bytes packeges
    get_valid_measurement_from_sample_data(scan_data, measurements)
    
    # 4. Agregate measurements 
    aggregated_df = aggregate_measurements(measurements)
    
    # 5. Save data to excel
    with pd.ExcelWriter(filename) as writer:
        pd.DataFrame(commands).to_excel(writer, sheet_name="Commands", index=False)
        pd.DataFrame(measurements).to_excel(writer, sheet_name="Measurements", index=False)
        aggregated_df.to_excel(writer, sheet_name="Aggregate Measurements", index=False) 
        
        print(f"--> OUTPUT FILE: LIDAR_Parsed.xlsx")
        print(f"--> CMD Count: {len(commands)} | Valid measurements count: {len(measurements)}")
        print(f"--> Aggregated measurements count: {len(aggregated_df)}")
        
if __name__ == "__main__":
    
    input_file = r"d:\FACULTATE\AN_2_MASTER\SISTEME INFORMATICE RECONFIGURABILE\RPLidar_RoomScanner\FrameGrabber\scripts\sample_data.txt"
    output_file = "LidarParsed.xlsx"
    
    # Read sample file
    data = read_sample_data(filename = input_file)
    # Decode sample file and save info to excel
    decode_sample_data(data, filename = output_file)
    
    
        
    
    
        
    
        
        
        