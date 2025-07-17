import math
import ast
import sys
import os
import numpy as np

def shift_pose(x, y, yaw, do_shift, distance=0.49094):
    if do_shift:
        return (x - distance * math.cos(yaw), y - distance * math.sin(yaw))
    else:
        return (x, y)

def process_file(filename):
    path_types = []
    positions = []
    yaws = []
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('actions:') or not line.startswith('['):
                continue
            arr = ast.literal_eval(line.rstrip(','))
            path_types.append(arr[0])
            positions.append((arr[3], arr[4]))
            yaws.append(arr[5])

    # Exclude empty actions list (fewer than 2 points)
    if len(positions) < 2:
        return None

    total_length = 0.0
    transfer_length = 0.0
    transit_length = 0.0

    for i in range(1, len(positions)):
        type_prev, type_curr = path_types[i-1], path_types[i]
        yaw_prev, yaw_curr = yaws[i-1], yaws[i]
        x_prev, y_prev = positions[i-1]
        x_curr, y_curr = positions[i]

        x0_adj, y0_adj = shift_pose(x_prev, y_prev, yaw_prev, type_prev == 1)
        x1_adj, y1_adj = shift_pose(x_curr, y_curr, yaw_curr, type_curr == 1)

        d = math.hypot(x1_adj - x0_adj, y1_adj - y0_adj)
        total_length += d

        if type_prev == 1 and type_curr == 1:
            transfer_length += d
        else:
            transit_length += d

    return total_length, transfer_length, transit_length

def main():
    if len(sys.argv) < 2:
        print("Usage: python script.py <directory_or_file>")
        return

    path = sys.argv[1]
    files = []

    if os.path.isdir(path):
        files = [os.path.join(path, f) for f in os.listdir(path)
                 if os.path.isfile(os.path.join(path, f)) and f.endswith('.res')]
    elif os.path.isfile(path):
        files = [path]
    else:
        print("Provided path is not a valid file or directory.")
        return

    if not files:
        print("No valid .txt files found in the directory.")
        return

    total_lengths = []
    transfer_lengths = []
    transit_lengths = []
    files_used = []

    for file in sorted(files):
        result = process_file(file)
        if result is None:
            print(f"Skipping empty or invalid actions file: {file}")
            continue
        total, transfer, transit = result
        total_lengths.append(total)
        transfer_lengths.append(transfer)
        transit_lengths.append(transit)
        files_used.append(file)
        print(f"File: {file}")
        print(f"  Total path length:    {total:.3f} m")
        print(f"  Transfer length:      {transfer:.3f} m")
        print(f"  Transit length:       {transit:.3f} m\n")

    print(f"Number of files processed: {len(files_used)} (non-empty actions)")

    if len(files_used) == 0:
        print("No valid files to process for statistics.")
        return

    def stat_str(values):
        mean = np.mean(values)
        std = np.std(values)
        return f"mean = {mean:.3f} m, std = {std:.3f} m"

    print("Summary for all files:")
    print(f"  Total path length:   {stat_str(total_lengths)}")
    print(f"  Transfer length:     {stat_str(transfer_lengths)}")
    print(f"  Transit length:      {stat_str(transit_lengths)}")

if __name__ == "__main__":
    main()
