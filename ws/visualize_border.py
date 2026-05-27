#!/usr/bin/env python3
import os
import glob
import json
import csv
import sys

def get_latest_border_file(extension='csv'):
    directory = os.path.expanduser('~/border_maps')
    search_path = os.path.join(directory, f'*.{extension}')
    files = glob.glob(search_path)
    if not files:
        return None
    return max(files, key=os.path.getmtime)

def plot_border(filepath):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("Error: matplotlib is not installed. Install it with: pip install matplotlib")
        print("\nPrinting points from file:")
        try:
            with open(filepath, 'r') as f:
                print(f.read())
        except Exception as e:
            print(f"Could not read file: {e}")
        return

    x = []
    y = []
    ext = os.path.splitext(filepath)[1].lower()
    
    if ext == '.csv':
        with open(filepath, 'r') as f:
            reader = csv.reader(f)
            next(reader, None)  # Skip header
            for row in reader:
                if len(row) >= 2:
                    try:
                        x.append(float(row[0]))
                        y.append(float(row[1]))
                    except ValueError:
                        continue
    elif ext == '.geojson':
        with open(filepath, 'r') as f:
            try:
                data = json.load(f)
                # Traverse GeoJSON to find coordinates
                for feature in data.get('features', []):
                    geom = feature.get('geometry', {})
                    if geom.get('type') == 'Polygon':
                        coords = geom.get('coordinates', [[]])[0]
                        for pt in coords:
                            if len(pt) >= 2:
                                x.append(pt[0])
                                y.append(pt[1])
                        break
            except Exception as e:
                print(f"Error reading GeoJSON: {e}")
                return
    else:
        print(f"Unsupported file type: {ext}")
        return

    if not x:
        print("No points found to plot.")
        return

    plt.figure(figsize=(8, 8))
    plt.plot(x, y, 'r-o', linewidth=2, label='Border')
    # Draw start/end point
    plt.plot(x[0], y[0], 'go', markersize=10, label='Start/End')
    plt.fill(x, y, alpha=0.1, color='red')
    plt.grid(True, which='both', linestyle='--', alpha=0.5)
    plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    plt.axvline(x=0, color='k', linestyle='-', alpha=0.3)
    plt.title(f"Recorded Border\n{os.path.basename(filepath)}")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.axis('equal')
    plt.legend()
    plt.show()

def main():
    if len(sys.argv) > 1:
        filepath = sys.argv[1]
    else:
        filepath = get_latest_border_file('csv')
        if not filepath:
            filepath = get_latest_border_file('geojson')
            
    if not filepath or not os.path.exists(filepath):
        print("No border maps found in ~/border_maps. Make sure you recorded a border first.")
        sys.exit(1)
        
    print(f"Visualizing: {filepath}")
    plot_border(filepath)

if __name__ == '__main__':
    main()
