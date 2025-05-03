import pandas as pd
import matplotlib.pyplot as plt

# Load the data from the .txt file
# Replace 'joystick_log.txt' with the actual name of your file (e.g., joystick_log_20250411_123456.txt)
data = pd.read_csv('joystick_log_20250411_131135.txt', delimiter=',')  # Use delimiter=',' for comma-separated values

# Convert the time to relative time (subtract the first timestamp)
data['Relative_Time'] = data['Time'] - data['Time'].iloc[0]

# Create the plot
plt.figure(figsize=(10, 6))
plt.plot(data['Relative_Time'], data['Axis_0_Value'], label='Axis 0 Value', color='blue')
plt.plot(data['Relative_Time'], data['Axis_1_Value'], label='Axis 1 Value', color='red')

# Add titles and labels
plt.title('Joystick Axis Values Over Time')
plt.xlabel('Relative Time (seconds)')
plt.ylabel('Axis Value')

# Add a legend
plt.legend()

# Display the grid for better readability
plt.grid(True)

# Show the plot
plt.show()