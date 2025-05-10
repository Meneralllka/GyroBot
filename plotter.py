import matplotlib.pyplot as plt
import pandas as pd

# Read the CSV file
df = pd.read_csv('joystick_log_20250503_202015.txt')

# Convert time to relative time (starting from 0)
df['Time'] = df['Time'] - df['Time'].iloc[0] - 2

# Create a figure with two subplots
plt.figure(figsize=(12, 8))

# Plot Goal Positions
plt.subplot(2, 1, 1)
plt.plot(df['Time'], df['Goal_Position_0']*360/303454, 'b-', label='Goal Position 0')
plt.plot(df['Time'], df['Present_Position_0']*360/303454, 'r-', label='Present  Position 0')
plt.title('Joystick Goal vs Present Positions, Rotational Move')
plt.ylabel('Position')
plt.xlim([0,4])
plt.grid(True)

plt.legend()

# Plot Present Positions
plt.subplot(2, 1, 2)
plt.plot(df['Time'], df['Goal_Position_1']*360/303454, 'b-', label='Goal Position 1')
plt.plot(df['Time'], df['Present_Position_1']*360/303454, 'r-', label='Present Position 1')
plt.xlabel('Time (seconds)')
plt.ylabel('Position')
plt.xlim([0,4])
#plt.ylim([-25, 25])
plt.grid(True)
plt.legend()

# Adjust layout to prevent overlap
plt.tight_layout()

# Save the plot
plt.savefig('joystick_positions_plot2.png')
plt.show()