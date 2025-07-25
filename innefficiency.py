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
df = pd.read_csv('output copy.csv')
actual = df.drop(['RevMatch', 'Time'], axis=1)
finaldrive = 61/15
gears = {1: 37/11,
2: 41/22,
3: 37/28,
4: 35/34,
5: 32/39}

wheelCircumference = 74.38402
print(actual.dtypes)

actual['RevMatch'] = actual.apply(lambda row:(1056 * row['MPH'] * finaldrive * gears[row['CurrentGear']]) / wheelCircumference, axis=1)
actual = actual.groupby('CurrentGear')

for i in range(len(actual)):
    g = actual.get_group(i+1)
    g.sort_values(by='MPH', inplace=True)
    # g = g.groupby(['MPH']).mean().reset_index()
    print(g)
    # plt.plot(g['RPM'], g['MPH'], color='blue', )
    plt.plot(g['RevMatch'], g['MPH'], color='red')
    plt.plot(np.unique(g['RPM']), np.poly1d(np.polyfit(g['RPM'], g['MPH'], 1))(np.unique(g['RPM'])),color='blue')
    plt.show()