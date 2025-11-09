import pandas as pd
from typing import List, Dict
import os
import math
import matplotlib.pyplot as plt
import numpy as np
import plotly.graph_objects as go

# Set folder path
frames_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "frames")
output_excel = "LidarParsed.xlsx"
output_png = "Lidar3D.png"

ANGLE_PER_FRAME = 1.0  # turret rotation per frame (degrees)
FILTER_STD_FACTOR = 2.5  # filter outliers beyond 2.5 std dev

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

def get_valid_measurement_from_sample_data(scan_data: bytes, turret_angle_deg: float) -> List[Dict]:
    """
    CORRECT Physical Setup:
    - LIDAR mounted VERTICALLY on turret
    - LIDAR spins 360° creating a vertical circular slice
    - Turret rotates 0-180° sweeping this slice horizontally
    
    Coordinate transformation:
    - angle_deg (0-360°): LIDAR's rotation angle in its vertical plane
    - turret_angle_deg (0-180°): horizontal rotation of turret
    
    Result: 
    - When turret = 0°-90°: scans front half of room
    - When turret = 90°-180°: scans back half of room
    - Combined: full 360° room coverage
    """
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
        if distance_mm == 0 or distance_mm > 12000:  # filter unrealistic distances
            continue

        angle_q6 = (b2 << 7) + (b1 & 0x7F)
        angle_deg = angle_q6 / 64.0
        angle_deg = angle_deg % 360.0
        
        # Convertire la radiani (unghiul brut al LIDAR-ului)
        lidar_angle_rad = math.radians(angle_deg)
        turret_rad = math.radians(turret_angle_deg)
        
        # --- IMPLEMENTAREA TRANSPOZIȚIEI LA NIVELUL PROIECȚIEI ---
        # Presupunere: Unghiul 0°/360° al LIDAR-ului este pe Z (vertical).
        
        r_in_plane = distance_mm * math.sin(lidar_angle_rad)  # Componenta orizontală (folosește sin)
        z_global = distance_mm * math.cos(lidar_angle_rad)     # Componenta verticală (folosește cos)
        
        effective_turret_rad = turret_rad 
        
        x_global = r_in_plane * math.cos(effective_turret_rad)
        y_global = r_in_plane * math.sin(effective_turret_rad)
        
        measurements.append({
            "Index": i,
            "Quality": quality,
            "LIDAR_Angle (deg)": round(angle_deg, 1),
            "Distance (mm)": int(distance_mm),
            "Turret_Angle (deg)": turret_angle_deg,
            "X (mm)": round(x_global, 2),
            "Y (mm)": round(y_global, 2),
            "Z (mm)": round(z_global, 2)
        })
    return measurements

def save_interactive_3d_plot(df_filtered: pd.DataFrame, output_html: str):
    print("[INFO] Generating interactive 3D plot...")

    fig = go.Figure(
        data=[
            go.Scatter3d(
                x=df_filtered["X (mm)"],
                y=df_filtered["Y (mm)"],
                z=df_filtered["Z (mm)"],
                mode='markers',
                marker=dict(
                    size=2,
                    color=df_filtered["Z (mm)"],
                    colorscale='Viridis',
                    opacity=0.7
                )
            )
        ]
    )

    fig.update_layout(
        title="LIDAR 3D Room Scan (Interactive)",
        scene=dict(
            xaxis_title='X (mm) - Width',
            yaxis_title='Y (mm) - Depth',
            zaxis_title='Z (mm) - Height',
            aspectmode='data'
        ),
        width=1200,
        height=900
    )

    # Get room boundaries
    x_min, x_max = df_filtered["X (mm)"].min(), df_filtered["X (mm)"].max()
    y_min, y_max = df_filtered["Y (mm)"].min(), df_filtered["Y (mm)"].max()
    z_min, z_max = df_filtered["Z (mm)"].min(), df_filtered["Z (mm)"].max()

    # Draw box outline (12 edges of the rectangular room)
    # Bottom rectangle
    box_edges = [
        # Bottom face
        ([x_min, x_max], [y_min, y_min], [z_min, z_min]),
        ([x_max, x_max], [y_min, y_max], [z_min, z_min]),
        ([x_max, x_min], [y_max, y_max], [z_min, z_min]),
        ([x_min, x_min], [y_max, y_min], [z_min, z_min]),
        # Top face
        ([x_min, x_max], [y_min, y_min], [z_max, z_max]),
        ([x_max, x_max], [y_min, y_max], [z_max, z_max]),
        ([x_max, x_min], [y_max, y_max], [z_max, z_max]),
        ([x_min, x_min], [y_max, y_min], [z_max, z_max]),
        # Vertical edges
        ([x_min, x_min], [y_min, y_min], [z_min, z_max]),
        ([x_max, x_max], [y_min, y_min], [z_min, z_max]),
        ([x_max, x_max], [y_max, y_max], [z_min, z_max]),
        ([x_min, x_min], [y_max, y_max], [z_min, z_max]),
    ]
    
    for i, (x_edge, y_edge, z_edge) in enumerate(box_edges):
        fig.add_trace(go.Scatter3d(
            x=x_edge,
            y=y_edge,
            z=z_edge,
            mode="lines",
            line=dict(color="red", width=3),
            showlegend=(i == 0),
            name="Room Box" if i == 0 else None
        ))

    # Add vertical red line at origin (reference)
    fig.add_trace(go.Scatter3d(
        x=[0, 0],
        y=[0, 0],
        z=[z_min, z_max],
        mode="lines",
        line=dict(color="yellow", width=5, dash="dash"),
        name="Origin (0,0)"
    ))

    fig.write_html(output_html)
    print(f"--> Interactive 3D plot saved as: {output_html}")

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
            turret_angle = frame_no * ANGLE_PER_FRAME

            measurements = get_valid_measurement_from_sample_data(scan_data, turret_angle)
            all_measurements.extend(measurements)
            all_commands.extend(commands)

    if not all_measurements:
        print("No measurements found!")
        return

    df = pd.DataFrame(all_measurements)
    df_commands = pd.DataFrame(all_commands)

    print(f"Total measurements before filtering: {len(df)}")

    # Filter outliers
    mean_x, std_x = df['X (mm)'].mean(), df['X (mm)'].std()
    mean_y, std_y = df['Y (mm)'].mean(), df['Y (mm)'].std()
    mean_z, std_z = df['Z (mm)'].mean(), df['Z (mm)'].std()
    
    mask = (
        (df['X (mm)'] >= mean_x - FILTER_STD_FACTOR*std_x) & 
        (df['X (mm)'] <= mean_x + FILTER_STD_FACTOR*std_x) & 
        (df['Y (mm)'] >= mean_y - FILTER_STD_FACTOR*std_y) & 
        (df['Y (mm)'] <= mean_y + FILTER_STD_FACTOR*std_y) &
        (df['Z (mm)'] >= mean_z - FILTER_STD_FACTOR*std_z) & 
        (df['Z (mm)'] <= mean_z + FILTER_STD_FACTOR*std_z)
    )
    df_filtered = df[mask]
    
    # Swap X and Z axes - X contains the actual height data!
    # df_filtered = df_filtered.rename(columns={
    #     "X (mm)": "Z (mm)",
    #     "Z (mm)": "X (mm)"
    # })

    # Save Excel
    with pd.ExcelWriter(output_excel) as writer:
        df_commands.to_excel(writer, sheet_name="Commands", index=False)
        df.to_excel(writer, sheet_name="Measurements", index=False)
        df_filtered.to_excel(writer, sheet_name="Measurements_Filtered", index=False)

    print(f"--> OUTPUT FILE: {output_excel}")
    print(f"--> Total CMD Count: {len(all_commands)} | Total measurements: {len(all_measurements)} | Filtered: {len(df_filtered)}")

    # Print statistics
    print(f"\nRoom dimensions (mm):")
    print(f"  X range: {df_filtered['X (mm)'].min():.0f} to {df_filtered['X (mm)'].max():.0f} (width: {df_filtered['X (mm)'].max() - df_filtered['X (mm)'].min():.0f})")
    print(f"  Y range: {df_filtered['Y (mm)'].min():.0f} to {df_filtered['Y (mm)'].max():.0f} (depth: {df_filtered['Y (mm)'].max() - df_filtered['Y (mm)'].min():.0f})")
    print(f"  Z range: {df_filtered['Z (mm)'].min():.0f} to {df_filtered['Z (mm)'].max():.0f} (height: {df_filtered['Z (mm)'].max() - df_filtered['Z (mm)'].min():.0f})")

    # Plot 3D - ROOM VIEW
    fig = plt.figure(figsize=(16, 12))
    ax = fig.add_subplot(111, projection='3d')

    xs = df_filtered['X (mm)']
    ys = df_filtered['Y (mm)']
    zs = df_filtered['Z (mm)']
    
    # Color by height (Z) for better room visualization
    sc = ax.scatter(xs, ys, zs, c=zs, cmap='viridis', s=0.5, marker='o', alpha=0.6)
    # --- Draw vertical red bar at Z = 0 reference plane ---
    ax.plot([0, 0], [0, 0], [df_filtered['Z (mm)'].min(), df_filtered['Z (mm)'].max()],
            color='red', linewidth=1.5, linestyle='--', label='Z=0 reference')
    ax.legend()

    ax.set_xlabel('X (mm) - Room Width', fontsize=12)
    ax.set_ylabel('Y (mm) - Room Depth', fontsize=12)
    ax.set_zlabel('Z (mm) - Room Height', fontsize=12)
    ax.set_title('LIDAR 3D Room Scan - Full 360° Coverage\n(Turret 180° × LIDAR 360° = Complete Room)', fontsize=14)
    
    # Set viewing angle for better perspective
    ax.view_init(elev=20, azim=45)
    
    plt.colorbar(sc, label='Height (mm)', shrink=0.5, pad=0.1)

    # Save PNG
    plt.savefig(output_png, dpi=300, bbox_inches='tight')
    print(f"--> 3D plot saved as: {output_png}")
    
    # Create additional views
    fig2, axes = plt.subplots(2, 2, figsize=(16, 14))
    
    # Top view (X-Y plane) - Floor plan
    sc1 = axes[0, 0].scatter(xs, ys, c=zs, cmap='viridis', s=1, alpha=0.6)
    axes[0, 0].set_xlabel('X (mm)', fontsize=11)
    axes[0, 0].set_ylabel('Y (mm)', fontsize=11)
    axes[0, 0].set_title('Top View (Floor Plan) - Colored by Height', fontsize=12)
    axes[0, 0].set_aspect('equal')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].axhline(y=0, color='r', linestyle='--', linewidth=0.5, alpha=0.5)
    axes[0, 0].axvline(x=0, color='r', linestyle='--', linewidth=0.5, alpha=0.5)
    plt.colorbar(sc1, ax=axes[0, 0], label='Height (mm)')
    
    # Front view (X-Z plane)
    sc2 = axes[0, 1].scatter(xs, zs, c=ys, cmap='plasma', s=1, alpha=0.6)
    axes[0, 1].set_xlabel('X (mm)', fontsize=11)
    axes[0, 1].set_ylabel('Z (mm) - Height', fontsize=11)
    axes[0, 1].set_title('Front View - Colored by Depth', fontsize=12)
    axes[0, 1].set_aspect('equal')
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].axhline(y=0, color='r', linestyle='--', linewidth=0.5, alpha=0.5)
    axes[0, 1].axvline(x=0, color='r', linestyle='--', linewidth=0.5, alpha=0.5)
    plt.colorbar(sc2, ax=axes[0, 1], label='Depth (mm)')
    
    # Side view (Y-Z plane)
    sc3 = axes[1, 0].scatter(ys, zs, c=xs, cmap='cool', s=1, alpha=0.6)
    axes[1, 0].set_xlabel('Y (mm)', fontsize=11)
    axes[1, 0].set_ylabel('Z (mm) - Height', fontsize=11)
    axes[1, 0].set_title('Side View - Colored by Width', fontsize=12)
    axes[1, 0].set_aspect('equal')
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].axhline(y=0, color='r', linestyle='--', linewidth=0.5, alpha=0.5)
    axes[1, 0].axvline(x=0, color='r', linestyle='--', linewidth=0.5, alpha=0.5)
    plt.colorbar(sc3, ax=axes[1, 0], label='Width (mm)')
    
    # Turret angle coverage
    axes[1, 1].hist(df_filtered['Turret_Angle (deg)'], bins=180, color='skyblue', edgecolor='black')
    axes[1, 1].set_xlabel('Turret Angle (degrees)', fontsize=11)
    axes[1, 1].set_ylabel('Point Count', fontsize=11)
    axes[1, 1].set_title('Turret Rotation Coverage (should be 0-180°)', fontsize=12)
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].set_xlim(0, 180)
    
    plt.tight_layout()
    plt.savefig('Lidar_MultiView.png', dpi=300, bbox_inches='tight')
    print(f"--> Multi-view plot saved as: Lidar_MultiView.png")
    
    #plt.show()
    return df_filtered

if __name__ == "__main__":
    df_filtered = decode_all_bin_files(frames_folder, output_excel)
    if df_filtered is not None and not df_filtered.empty:
        interactive_html = "Lidar3D_Interactive.html"
        save_interactive_3d_plot(df_filtered, interactive_html)