import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load the data
try:
    data = pd.read_csv('imu_data.txt', sep=',')
except Exception as e:
    print(f"Error loading file: {e}")
    exit()

# Print column names to debug
print("Column names:", data.columns.tolist())

# Strip whitespace from column names
data.columns = data.columns.str.strip()

# Verify required columns exist
required_columns = ['Timestamp', 'acc68X', 'acc68Y', 'acc68Z', 'acc69X', 'acc69Y', 'acc69Z']
if not all(col in data.columns for col in required_columns):
    print(f"Missing columns. Available columns: {data.columns.tolist()}")
    exit()

# Calculate magnitudes
data['mag68'] = np.sqrt(data['acc68X']**2 + data['acc68Y']**2 + data['acc68Z']**2)
data['mag69'] = np.sqrt(data['acc69X']**2 + data['acc69Y']**2 + data['acc69Z']**2)

# Create subplots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# Subplot 1: Sensor 68
ax1.plot(data['Timestamp'], data['acc68X'], label='acc68X', color='r')
ax1.plot(data['Timestamp'], data['acc68Y'], label='acc68Y', color='g')
ax1.plot(data['Timestamp'], data['acc68Z'], label='acc68Z', color='b')
ax1.plot(data['Timestamp'], data['mag68'], label='Magnitude', color='k', linestyle='--')
ax1.set_title('Sensor 68 Acceleration')
ax1.set_ylabel('Acceleration (g)')
ax1.legend()
ax1.grid(True)

# Subplot 2: Sensor 69
ax2.plot(data['Timestamp'], data['acc69X'], label='acc69X', color='r')
ax2.plot(data['Timestamp'], data['acc69Y'], label='acc69Y', color='g')
ax2.plot(data['Timestamp'], data['acc69Z'], label='acc69Z', color='b')
ax2.plot(data['Timestamp'], data['mag69'], label='Magnitude', color='k', linestyle='--')
ax2.set_title('Sensor 69 Acceleration')
ax2.set_xlabel('Timestamp (s)')
ax2.set_ylabel('Acceleration (g)')
ax2.legend()
ax2.grid(True)

# Adjust layout and display
plt.tight_layout()
plt.show()