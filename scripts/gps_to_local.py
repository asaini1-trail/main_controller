#!/usr/bin/env python3
"""
GPS to Local Coordinate Converter
Input: CSV with 'lat', 'lon' columns
Output: CSV with 'x', 'y' columns in meters (ENU frame)
"""

import numpy as np
import csv
import sys

def gps_to_local(lat, lon, lat_ref, lon_ref):
    """Convert GPS to local ENU coordinates (meters)"""
    R = 6371000  # Earth radius in meters
    
    x = R * np.radians(lon - lon_ref) * np.cos(np.radians(lat_ref))  # East
    y = R * np.radians(lat - lat_ref)  # North
    
    return x, y


def convert_file(input_csv, output_csv, lat_col='Lat', lon_col='Lon', time_col='Timestamp'):
    """Read GPS CSV, convert to local, write output"""

    # Read input
    lat_list, lon_list, time_list = [], [], []
    with open(input_csv, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            lat_list.append(float(row[lat_col]))
            lon_list.append(float(row[lon_col]))
            time_list.append(row[time_col])  # Keep timestamp as string

    lat = np.array(lat_list)
    lon = np.array(lon_list)
    time = time_list  # Keep as list of strings
    
    # Use first point as origin
    lat_ref, lon_ref = lat[0], lon[0]
    
    # Convert
    x, y = gps_to_local(lat, lon, lat_ref, lon_ref)

    # Write output
    with open(output_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Timestamp', 'x', 'y'])
        for ti, xi, yi in zip(time, x, y):
            writer.writerow([ti, f'{xi:.4f}', f'{yi:.4f}'])
    
    print(f"Converted {len(x)} points")
    print(f"Origin: ({lat_ref:.6f}, {lon_ref:.6f})")
    print(f"Saved to: {output_csv}")


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python gps_to_local.py input.csv output.csv [lat_col] [lon_col] [time_col]")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]
    lat_col = sys.argv[3] if len(sys.argv) > 3 else 'lat'
    lon_col = sys.argv[4] if len(sys.argv) > 4 else 'lon'
    time_col = sys.argv[5] if len(sys.argv) > 5 else 'timestamp'

    convert_file(input_file, output_file, lat_col, lon_col, time_col)