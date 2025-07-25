import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read configuration file for RPM limits
try:
    with open('config.txt', 'r') as f:
        lines = f.readlines()

    # Extract first two lines without commas
    config_lines = [line.strip() for line in lines if ',' not in line.strip() and line.strip()]

    if len(config_lines) >= 2:
        min_rpm = int(config_lines[0])
        max_rpm = int(config_lines[1])
        print(f"Using RPM limits from config.txt: {min_rpm} - {max_rpm}")
    else:
        print("Warning: config.txt doesn't have enough valid lines, using auto-scale")
        min_rpm = None
        max_rpm = None
except FileNotFoundError:
    print("Warning: config.txt not found, using auto-scale")
    min_rpm = None
    max_rpm = None
except ValueError:
    print("Warning: Invalid RPM values in config.txt, using auto-scale")
    min_rpm = None
    max_rpm = None

# Read the CSV data
df = pd.read_csv('output.csv')

# Use the Time column from the CSV data
time = df['Time'].values

# Define colors for each gear (more readable palette)
gear_colors = {
    1: '#E74C3C',  # Red
    2: '#F39C12',  # Orange
    3: '#2ECC71',  # Green
    4: '#3498DB',  # Blue
    5: '#9B59B6'   # Purple
}

# Create the plot
fig, ax = plt.subplots(figsize=(12, 8))

# Create arrays to store the line segments
rpm_segments = []
revmatch_segments = []

# Process each data point sequentially
for i in range(len(df)):
    current_gear = df.iloc[i]['CurrentGear']
    current_rpm = df.iloc[i]['RPM']
    current_revmatch = df.iloc[i]['RevMatch']
    current_time = time[i]

    # Add RPM point with color based on current gear
    if i == 0:
        # First point - start new segment
        rpm_segments.append({
            'time': [current_time],
            'rpm': [current_rpm],
            'gear': current_gear
        })
    else:
        # Check if gear changed
        prev_gear = df.iloc[i-1]['CurrentGear']
        if current_gear == prev_gear:
            # Same gear - extend current segment
            rpm_segments[-1]['time'].append(current_time)
            rpm_segments[-1]['rpm'].append(current_rpm)
        else:
            # Gear changed - start new segment
            rpm_segments.append({
                'time': [current_time],
                'rpm': [current_rpm],
                'gear': current_gear
            })

    # Handle RevMatch segments
    if current_gear > 1 and current_revmatch != -1:
        revmatch_gear = current_gear - 1

        # Find if we need to start a new RevMatch segment
        if not revmatch_segments or (i > 0 and (df.iloc[i-1]['CurrentGear'] != current_gear or df.iloc[i-1]['RevMatch'] == -1)):
            # Start new RevMatch segment
            revmatch_segments.append({
                'time': [current_time],
                'revmatch': [current_revmatch],
                'gear': revmatch_gear
            })
        elif revmatch_segments[-1]['gear'] == revmatch_gear:
            revmatch_segments[-1]['time'].append(current_time)
            revmatch_segments[-1]['revmatch'].append(current_revmatch)
        else:
            # Different gear - start new segment
            revmatch_segments.append({
                'time': [current_time],
                'revmatch': [current_revmatch],
                'gear': revmatch_gear
            })

# Plot RPM segments
for segment in rpm_segments:
    ax.plot(segment['time'], segment['rpm'], 
           color=gear_colors.get(segment['gear'], '#000000'), 
           linewidth=2,
           zorder=1)  # Lower z-order for RPM

# Plot RevMatch segments (higher z-order to appear above RPM)
for segment in revmatch_segments:
    ax.plot(segment['time'], segment['revmatch'], 
           color=gear_colors.get(segment['gear'], '#000000'), 
           linewidth=2, 
           linestyle='--',
           alpha=0.8,
           zorder=2)  # Higher z-order for RevMatch

# Create custom legend with 1 row per gear using matplotlib's multi-line legend
from matplotlib.legend_handler import HandlerTuple

legend_elements = []
all_gears = sorted(set([s['gear'] for s in rpm_segments] + [s['gear'] for s in revmatch_segments]))

for gear in all_gears:
    # Create RPM line (solid)
    rpm_line = plt.Line2D([0], [0], color=gear_colors.get(gear, '#000000'), 
                         linewidth=2, linestyle='-')

    if gear in [s['gear'] for s in revmatch_segments]:
        # Create RevMatch line (dashed) - same color as this gear
        revmatch_line = plt.Line2D([0], [0], color=gear_colors.get(gear, '#000000'), 
                                  linewidth=2, linestyle='--', alpha=0.8)
        # Add both lines as a tuple for this gear
        legend_elements.append((rpm_line, revmatch_line))
    else:
        # Only RPM line for this gear
        legend_elements.append((rpm_line,))

# Create labels for each gear
gear_labels = [f'Gear {gear}' for gear in all_gears]

# Create legend with custom handler for tuples
ax.legend(legend_elements, gear_labels, 
         handler_map={tuple: HandlerTuple(ndivide=None, pad=0.3)},
         bbox_to_anchor=(1.05, 1), loc='upper left', ncol=1)

# Customize the plot
ax.set_xlabel('Time', fontsize=12)
ax.set_ylabel('RPM', fontsize=12)
ax.set_title('RPM and RevMatch vs Time', fontsize=14, fontweight='bold')
ax.grid(True, alpha=0.3)

# Hide x-axis tick labels but keep the axis title
ax.tick_params(axis='x', labelbottom=False)

# Set Y-axis limits based on config file (always use config values if available)
if min_rpm is not None and max_rpm is not None:
    ax.set_ylim(min_rpm, max_rpm)
    print(f"Y-axis set to: {min_rpm} - {max_rpm} RPM")
else:
    ax.autoscale(enable=True, axis='y', tight=True)
    print("Y-axis auto-scaled (config.txt not available or invalid)")

# Auto-scale X-axis
ax.autoscale(enable=True, axis='x', tight=True)

# Adjust layout to prevent legend cutoff
plt.tight_layout()

# Display the plot
plt.show()

# Optional: Save the plot
# plt.savefig('rpm_revmatch_plot.png', dpi=300, bbox_inches='tight')

print("Plot generated successfully!")
print(f"Data points: {len(df)}")
print(f"Gears present: {sorted(df['CurrentGear'].unique())}")
print(f"RPM range: {df['RPM'].min()} - {df['RPM'].max()}")
print(f"RevMatch range (excluding -1): {df[df['RevMatch'] != -1]['RevMatch'].min()} - {df[df['RevMatch'] != -1]['RevMatch'].max()}")