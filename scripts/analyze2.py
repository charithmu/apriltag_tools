import dis
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Function to read data, plot positions in 3D space, calculate the 3D center, determine errors, and display statistics
def analyze_pose_data(file_path):
    # Read the CSV file
    data = pd.read_csv(file_path)
    
    # Strip whitespace from column names if necessary
    data.columns = data.columns.str.strip()
    
    # Convert coordinates to millimeters
    data['x'] *= 1000
    data['y'] *= 1000
    data['z'] *= 1000
    
    # Calculate the 3D center of the points in millimeters
    center_x = data['x'].mean()
    center_y = data['y'].mean()
    center_z = data['z'].mean()
    center_point = (center_x, center_y, center_z)
    
    # Calculate Euclidean distance from the center for each point
    # distances = np.sqrt((data['x'] - center_x)**2 + (data['y'] - center_y)**2 + (data['z'] - center_z)**2)
    distances = np.sqrt((data['x'] - center_x)**2 + (data['y'] - center_y)**2 + (data['z'] - center_z)**2)
    
    # Determine the maximum Euclidean distance from the center
    max_distance = distances.max()
    
    # Calculate Mean Squared Residual (MSR) error in square millimeters
    msr_error = np.mean(distances**2)
    
    # Calculate Mean Squared Error (MSE) in square millimeters
    mse_error = np.mean((data[['x', 'y', 'z']] - np.array([center_x, center_y, center_z]))**2)
    
    mse2 = np.mean(distances)

    # Plotting
    fig = plt.figure(figsize=(14, 7))
    ax = fig.add_subplot(121, projection='3d')
    scatter = ax.scatter(data['x'], data['y'], data['z'], c='blue', label='Pose Points')
    ax.scatter(center_x, center_y, center_z, c='red', label='Center', s=100)  # Plot the center point
    ax.set_xlabel('X Position (mm)', fontsize=16)
    ax.set_ylabel('Y Position (mm)', fontsize=16)
    ax.set_zlabel('Z Position (mm)', fontsize=16)
    ax.legend(fontsize=16)
    plt.title('3D Position of Pose Points and Center', fontsize=14)
    
    # Adding text box for statistics on the right side of the plot
    stats_text = f"Center: ({center_x:.6f}, {center_y:.6f}, {center_z:.6f}) mm\n"
    stats_text += f"Max Distance: {max_distance:.6f} mm\n"
    stats_text += f"MSR Error: {msr_error:.6f} mm²\n"
    stats_text += f"MSE Error: {mse_error.sum():.6f} mm²"
    ax.text2D(1.05, 0.5, stats_text, transform=ax.transAxes, fontsize=24,
             verticalalignment='center', horizontalalignment='left', bbox=dict(boxstyle='round,pad=0.5', facecolor='wheat', alpha=0.5))
    
    plt.show()
    
    return mse2, center_point, max_distance, msr_error, mse_error.sum()

# Example usage
file_path = 'tag1.txt'
mse2, center, max_err, msr, mse = analyze_pose_data(file_path)
print("Center of points:", center)
print("Maximum Euclidean distance from center:", max_err)
print("Mean Squared Residual error:", msr)
print("Mean Squared Error (MSE):", mse)
print("Mean Squared Error 2 (MSE):", mse2)