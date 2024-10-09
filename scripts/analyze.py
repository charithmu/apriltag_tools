import numpy as np
import sys

import matplotlib.pyplot as plt
class DataAnalyzer:
    def __init__(self, file_path):
        self.file_path = file_path
        self.data = None
        self.x = None
        self.y = None
        self.z = None
        self.ori_w = None
        self.ori_x = None
        self.ori_y = None
        self.ori_z = None

    def load_data(self):
        self.data = np.loadtxt(self.file_path, delimiter=",", skiprows=1)
        self.x = self.data[:, 0]
        self.y = self.data[:, 1]
        self.z = self.data[:, 2]
        self.ori_w = self.data[:, 3]
        self.ori_x = self.data[:, 4]
        self.ori_y = self.data[:, 5]
        self.ori_z = self.data[:, 6]

    def plot_data(self):
        plt.figure()
        plt.subplot(4, 2, 1)
        plt.plot(self.x)
        plt.title("x")
        plt.ylabel(self.data[0, 0])
        plt.subplot(4, 2, 2)
        plt.plot(self.y)
        plt.title("y")
        plt.ylabel(self.data[0, 1])
        plt.subplot(4, 2, 3)
        plt.plot(self.z)
        plt.title("z")
        plt.ylabel(self.data[0, 2])
        plt.subplot(4, 2, 4)
        plt.plot(self.ori_w)
        plt.title("ori_w")
        plt.ylabel(self.data[0, 3])
        plt.subplot(4, 2, 5)
        plt.plot(self.ori_x)
        plt.title("ori_x")
        plt.ylabel(self.data[0, 4])
        plt.subplot(4, 2, 6)
        plt.plot(self.ori_y)
        plt.title("ori_y")
        plt.ylabel(self.data[0, 5])
        plt.subplot(4, 2, 7)
        plt.plot(self.ori_z)
        plt.title("ori_z")
        plt.ylabel(self.data[0, 6])
        plt.show()

    def plot_data_all_in_one(self):
        plt.figure()
        plt.subplot(4, 2, 1)
        plt.plot(self.x, label='x', color='red')
        plt.title("x")
        plt.ylabel(self.data[0, 0])
        plt.subplot(4, 2, 2)
        plt.plot(self.y, label='y', color='blue')
        plt.title("y")
        plt.ylabel(self.data[0, 1])
        plt.subplot(4, 2, 3)
        plt.plot(self.z, label='z', color='green')
        plt.title("z")
        plt.ylabel(self.data[0, 2])
        plt.subplot(4, 2, 4)
        plt.plot(self.ori_w, label='ori_w', color='orange')
        plt.title("ori_w")
        plt.ylabel(self.data[0, 3])
        plt.subplot(4, 2, 5)
        plt.plot(self.ori_x, label='ori_x', color='purple')
        plt.title("ori_x")
        plt.ylabel(self.data[0, 4])
        plt.subplot(4, 2, 6)
        plt.plot(self.ori_y, label='ori_y', color='brown')
        plt.title("ori_y")
        plt.ylabel(self.data[0, 5])
        plt.subplot(4, 2, 7)
        plt.plot(self.ori_z, label='ori_z', color='gray')
        plt.title("ori_z")
        plt.ylabel(self.data[0, 6])

        plt.legend()

        plt.show()

        avg_x, avg_y, avg_z, avg_ori_w, avg_ori_x, avg_ori_y, avg_ori_z, std_x, std_y, std_z, std_ori_w, std_ori_x, std_ori_y, std_ori_z = self.calculate_statistics()

        plt.text(0.5, 0.95, f"Average x: {avg_x:.2f}, Mean x: {std_x:.2f}", horizontalalignment='center', verticalalignment='center', transform=plt.gca().transAxes)
        plt.text(0.5, 0.90, f"Average y: {avg_y:.2f}, Mean y: {std_y:.2f}", horizontalalignment='center', verticalalignment='center', transform=plt.gca().transAxes)
        plt.text(0.5, 0.85, f"Average z: {avg_z:.2f}, Mean z: {std_z:.2f}", horizontalalignment='center', verticalalignment='center', transform=plt.gca().transAxes)
        plt.text(0.5, 0.80, f"Average ori_w: {avg_ori_w:.2f}, Mean ori_w: {std_ori_w:.2f}", horizontalalignment='center', verticalalignment='center', transform=plt.gca().transAxes)
        plt.text(0.5, 0.75, f"Average ori_x: {avg_ori_x:.2f}, Mean ori_x: {std_ori_x:.2f}", horizontalalignment='center', verticalalignment='center', transform=plt.gca().transAxes)
        plt.text(0.5, 0.70, f"Average ori_y: {avg_ori_y:.2f}, Mean ori_y: {std_ori_y:.2f}", horizontalalignment='center', verticalalignment='center', transform=plt.gca().transAxes)
        plt.text(0.5, 0.65, f"Average ori_z: {avg_ori_z:.2f}, Mean ori_z: {std_ori_z:.2f}", horizontalalignment='center', verticalalignment='center', transform=plt.gca().transAxes)

    def calculate_statistics(self):
        avg_x = np.mean(self.x)
        avg_y = np.mean(self.y)
        avg_z = np.mean(self.z)
        avg_ori_w = np.mean(self.ori_w)
        avg_ori_x = np.mean(self.ori_x)
        avg_ori_y = np.mean(self.ori_y)
        avg_ori_z = np.mean(self.ori_z)

        std_x = np.std(self.x)
        std_y = np.std(self.y)
        std_z = np.std(self.z)
        std_ori_w = np.std(self.ori_w)
        std_ori_x = np.std(self.ori_x)
        std_ori_y = np.std(self.ori_y)
        std_ori_z = np.std(self.ori_z)
        return avg_x,avg_y,avg_z,avg_ori_w,avg_ori_x,avg_ori_y,avg_ori_z,std_x,std_y,std_z,std_ori_w,std_ori_x,std_ori_y,std_ori_z
    
    def print_statistics(self):
        avg_x, avg_y, avg_z, avg_ori_w, avg_ori_x, avg_ori_y, avg_ori_z, std_x, std_y, std_z, std_ori_w, std_ori_x, std_ori_y, std_ori_z = self.calculate_statistics()
        print(f"Average x: {avg_x:.2f}, Std. x: {std_x:.2f}")
        print(f"Average y: {avg_y:.2f}, Std. y: {std_y:.2f}")
        print(f"Average z: {avg_z:.2f}, Std. z: {std_z:.2f}")
        print(f"Average ori_w: {avg_ori_w:.2f}, Std. ori_w: {std_ori_w:.2f}")
        print(f"Average ori_x: {avg_ori_x:.2f}, Std. ori_x: {std_ori_x:.2f}")
        print(f"Average ori_y: {avg_ori_y:.2f}, Std. ori_y: {std_ori_y:.2f}")
        print(f"Average ori_z: {avg_ori_z:.2f}, Std. ori_z: {std_ori_z:.2f}")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        file_path = sys.argv[1]
    else:
        file_path = "tag1.txt"

    analyzer = DataAnalyzer(file_path)
    analyzer.load_data()
    analyzer.plot_data_all_in_one()
    analyzer.print_statistics()