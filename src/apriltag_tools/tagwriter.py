#!/usr/bin/env python3

import os
import rospy
from apriltag_tools.detection import Detection
from pynput import keyboard
from apriltag_ros.msg import AprilTagDetectionArray


class TagDetectionListner:
    def __init__(self):
        rospy.init_node("tag_detection_listener", anonymous=True)
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.callback)
        self.file = rospy.get_param("~file", "")
        self.save = False
        self.pause = False
        self.keylistner = keyboard.Listener(on_press=self.on_press)
        self.keylistner.start()

        if self.file != "":
            if os.path.isfile(str(self.file)):
                print("File already exists, appending to it.")
                self.save = True
            else:
                print("File does not exist, creating it.")
                with open(str(self.file), "w", encoding="utf-8") as file:
                    file.write(
                        "time, id, size, x, y, z, orientation_w, orientation_x, orientation_y, orientation_z\n"
                    )
                self.save = True
        else:
            print(
                "No file specified, not saving detections to file. Printing to console instead."
            )

        print("Press space to pause/resume processing.")

        rospy.spin()

    def on_press(self, key):
        if key == keyboard.Key.space:
            self.pause = not self.pause
            print("Processing is now: ", "Paused" if self.pause else "Resumed")

    def callback(self, data):
        if self.pause:
            return

        time = data.header.stamp
        for detection in data.detections:
            detobj = Detection.create_from_tag_detection(time, detection)
            Detection.print_console(detobj)
            if self.save:
                Detection.print_to_file(detobj, self.file)


if __name__ == "__main__":
    listener = TagDetectionListner()
