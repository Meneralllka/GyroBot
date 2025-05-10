import matplotlib.pyplot as plt
import pandas as pd

# Read the CSV file, specifying the header row (row 5, 0-based index)
data = pd.read_csv('GyroTest5.csv', header=5)

# Extract position data for Rigid Body 1 and Marker 1
rb1_pos_x = data['X']  # Rigid Body 1 Position X (column 2)
rb1_pos_z = data['Z']  # Rigid Body 1 Position Z (column 4)
marker1_pos_x = data['X.1']  # Marker 1 Position X (column 6)
marker1_pos_z = data['Z.1']  # Marker 1 Position Z (column 8)

# Create figure for Rigid Body 1
plt.figure(figsize=(8, 6))
plt.scatter(rb1_pos_x, rb1_pos_z, color='red', label='Rigid Body 1', s=10)
plt.title('Rigid Body 1: X Position vs Z Position')
plt.xlabel('X Position (m)')
plt.ylabel('Z Position (m)')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig('gyro_x_vs_z_rigid_body1.png')
plt.close()

# Create figure for Marker 1
plt.figure(figsize=(8, 6))
plt.scatter(marker1_pos_x, marker1_pos_z, color='blue', label='Marker 1', s=10)
plt.title('Marker 1: X Position vs Z Position')
plt.xlabel('X Position (m)')
plt.ylabel('Z Position (m)')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig('gyro_x_vs_z_marker1.png')
plt.close()