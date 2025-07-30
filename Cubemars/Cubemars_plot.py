import matplotlib
matplotlib.use('Agg')  # Add this before importing pyplot
import pandas as pd
import matplotlib.pyplot as plt

def plot_motor_data(csv_file='motor_data.csv'):
    try:
        data = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: {csv_file} not found.")
        return
    except Exception as e:
        print(f"Error reading CSV: {e}")
        return

    plt.figure(figsize=(10, 8))
    plt.subplot(2, 1, 1)
    plt.plot(data['Timestamp'], data['Motor0_p_in'], label='Motor 0 Reference', color='blue')
    plt.plot(data['Timestamp'], data['Motor0_p_out'], label='Motor 0 Actual', color='red')
    plt.title('Motor 0 Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (rad)')
    plt.grid(True)
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(data['Timestamp'], data['Motor1_p_in'], label='Motor 1 Reference', color='blue')
    plt.plot(data['Timestamp'], data['Motor1_p_out'], label='Motor 1 Actual', color='red')
    plt.title('Motor 1 Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (rad)')
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.savefig('motor_plot.png')
    print("Plot saved as motor_plot.png")

if __name__ == "__main__":
    plot_motor_data()
