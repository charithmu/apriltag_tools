import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Function to read data, plot positions in 3D and 2D space, calculate the 3D center, determine errors, and display statistics
def analyze_pose_data(file_path, tag_id):

    # Read the CSV file
    data = pd.read_csv(file_path)
    
    # Strip whitespace from column names if necessary
    data.columns = data.columns.str.strip()
    
    # Filter data based on tag_id
    data = data[data['tag_id'] == int(tag_id)]

    # Convert coordinates to millimeters
    data['x'] *= 1000
    data['y'] *= 1000
    data['z'] *= 1000
    
    # Calculate the 3D center of the points in millimeters
    center_x = data['x'].mean()
    center_y = data['y'].mean()
    center_z = data['z'].mean()
    center_point = (center_x, center_y, center_z)
    
    # Calculate Standard Deviation of the points in millimeters
    std_dev = (data['x'].std(), data['y'].std(), data['z'].std())

    # Calculate Euclidean distance from the center for each point
    ecl_dist = np.sqrt((data['x'] - center_x)**2 + (data['y'] - center_y)**2 + (data['z'] - center_z)**2)
    
    # Determine the maximum Euclidean distance from the center
    max_ecl_dist = ecl_dist.max()
    min_ecl_dist = ecl_dist.min()
    mean_ecl_dist = ecl_dist.mean()
    
    # Calculate Mean Squared Error (MSE) in square millimeters
    # mse_error = np.mean((data[['x', 'y', 'z']] - np.array([center_x, center_y, center_z]))**2)
    
    
    return data, center_point, std_dev, max_ecl_dist, min_ecl_dist, mean_ecl_dist

def plot_data(data, center_x, center_y, center_z, max_distance, msr_error, mse_error):
    # Plotting
    fig = plt.figure(figsize=(16, 10))
    ax1 = fig.add_subplot(221, projection='3d')
    ax1.scatter(data['x'], data['y'], data['z'], c='blue', label='Pose Points')
    ax1.scatter(center_x, center_y, center_z, c='red', label='Center', s=100)  # Plot the center point
    ax1.set_xlabel('X Position (mm)', fontsize=24, labelpad=30)
    ax1.set_ylabel('Y Position (mm)', fontsize=24, labelpad=30)
    ax1.set_zlabel('Z Position (mm)', fontsize=24, labelpad=30)
    ax1.legend(fontsize=24)
    plt.title('3D Position of Pose Points and Center', fontsize=24)

    # 2D plots for x, y, z components
    ax2 = fig.add_subplot(222)
    ax2.plot(data['x'], label='X Position')
    ax2.axhline(y=center_x, color='r', linestyle='--', label='Mean X')
    ax2.set_title('X Component over Index', fontsize=24)
    ax2.set_xlabel('Index', fontsize=24)
    ax2.set_ylabel('X Position (mm)', fontsize=24)
    ax2.legend(fontsize=24)

    ax3 = fig.add_subplot(223)
    ax3.plot(data['y'], label='Y Position')
    ax3.axhline(y=center_y, color='r', linestyle='--', label='Mean Y')
    ax3.set_title('Y Component over Index', fontsize=24)
    ax3.set_xlabel('Index', fontsize=24)
    ax3.set_ylabel('Y Position (mm)', fontsize=24)
    ax3.legend(fontsize=24)

    ax4 = fig.add_subplot(224)
    ax4.plot(data['z'], label='Z Position')
    ax4.axhline(y=center_z, color='r', linestyle='--', label='Mean Z')
    ax4.set_title('Z Component over Index', fontsize=24)
    ax4.set_xlabel('Index', fontsize=24)
    ax4.set_ylabel('Z Position (mm)', fontsize=24)
    ax4.legend(fontsize=24)

    ax1.tick_params(axis='both', which='major', labelsize=20)
    ax2.tick_params(axis='both', which='major', labelsize=20)
    ax3.tick_params(axis='both', which='major', labelsize=20)
    ax4.tick_params(axis='both', which='major', labelsize=20)

    plt.subplots_adjust(left=0.04, right=0.98, bottom=0.04, top=0.98, wspace=0.3, hspace=0.3)

    stats_text = f"Center: ({center_x:.6f}, {center_y:.6f}, {center_z:.6f}) mm\n"
    stats_text += f"Max Distance: {max_distance:.6f} mm\n"
    stats_text += f"MSR Error: {msr_error:.6f} mm²\n"
    stats_text += f"MSE Error: {mse_error.sum():.6f} mm²"
    ax1.text2D(0.5, -0.2, stats_text, transform=ax1.transAxes, fontsize=24,
                verticalalignment='bottom', horizontalalignment='center', bbox=dict(boxstyle='round,pad=0.5', facecolor='wheat', alpha=0.5))
    
    # plt.tight_layout()
    plt.show()

# Call the method

def main():
    if len(sys.argv) != 3:
        print("Usage: python analyse3.py <file_path> <tag_id>")
        sys.exit(1)

    file_path = sys.argv[1]
    tag_id = sys.argv[2]

    data, center, std_dev, max_ecl_dist, min_ecl_dist, mean_ecl_dist = analyze_pose_data(file_path, tag_id)
    center_x, center_y, center_z = center
    print("Center of points:", center)
    print("Maximum Euclidean distance from center:", max_err)
    print("Minimum Euclidean distance from center:", msr)
    print("Mean Euclidean distance from center:", mse)

    plot_data(data, center_x, center_y, center_z, max_err, msr, mse)

if __name__ == "__main__":
    main()

