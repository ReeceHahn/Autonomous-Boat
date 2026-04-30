import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def load_data(file_path):
    """Load 3D magnetometer data from a tab-separated text file."""
    return np.loadtxt(file_path, delimiter='\t')

def plot_3d_data(data, fig_num, title="Magnetometer Data", color='blue', axis_range=(-0.6, 0.6)):
    """Plot 3D data in a scatter plot with fixed axis limits."""
    fig = plt.figure(fig_num, figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.scatter(data[:, 0], data[:, 1], data[:, 2], c=color, alpha=0.7, s=10)
    ax.set_title(title)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_box_aspect([1, 1, 1])  # Keep cubic aspect ratio

    ax.set_xlim(axis_range)
    ax.set_ylim(axis_range)
    ax.set_zlim(axis_range)

# Example usage
if __name__ == "__main__":
    input_file = "magnetometer_data.txt"
    calibrated_file = "sphere_data.txt"

    try:
        raw_data = load_data(input_file)
        plot_3d_data(raw_data, fig_num=1, title="Raw Magnetometer Data (Ellipsoid)", color='red')
    except Exception as e:
        print(f"⚠️ Couldn't load raw data: {e}")

    try:
        calibrated_data = load_data(calibrated_file)
        plot_3d_data(calibrated_data, fig_num=2, title="Calibrated Magnetometer Data (Sphere)", color='green')
    except Exception as e:
        print(f"⚠️ Couldn't load calibrated data: {e}")

    # Show all plots together at the end
    plt.show()