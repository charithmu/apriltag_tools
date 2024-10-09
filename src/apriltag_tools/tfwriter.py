#!/usr/bin/env python3

import logging
import os
import yaml
import rospy
import tf2_ros
from apriltag_tools.detection import Detection
from pynput import keyboard


class TFWriter:
    def __init__(self):
        rospy.init_node("tf2_listener")
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.frame = rospy.get_param("~frame", "base_link")
        self.file = rospy.get_param("~file", "")
        self.save = False
        self.pause = False
        self.keylistner = keyboard.Listener(on_press=self.on_press)
        self.keylistner.start()
        self.logger = logging.getLogger(__name__)

        print("Transform from frame: ", self.frame)

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

    def on_press(self, key):
        if key == keyboard.Key.space:
            self.pause = not self.pause
            print("Processing is now: ", "Paused" if self.pause else "Resumed")

    def run(self):
        rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():
            if self.pause:
                continue

            frame_list = list(yaml.safe_load(self.tfBuffer.all_frames_as_yaml()).keys())
            tag_frames = [key for key in frame_list if key.startswith("tag_")]
            # print("Tag Frames: ", tag_frames)

            for tag_frame in tag_frames:
                try:
                    transform = self.tfBuffer.lookup_transform(
                        self.frame, tag_frame, rospy.Time()
                    )
                except tf2_ros.TransformException: # type: ignore
                    self.logger.warning(
                        "Failed to get transform: %s to %s", self.frame, tag_frame
                    )

                detobj = Detection.create_from_tf(transform)
                Detection.print_console(detobj)
                if self.save:
                    Detection.print_to_file(detobj, self.file)

            rate.sleep()


if __name__ == "__main__":
    tf_writer = TFWriter()
    tf_writer.run()
