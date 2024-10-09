#!/usr/bin/env python3

class Detection:
    def __init__(self, time, id, size, x, y, z, orientation_w, orientation_x, orientation_y, orientation_z):
        self.time = time
        self.id = id
        self.size = size
        self.x = x
        self.y = y
        self.z = z
        self.orientation_w = orientation_w
        self.orientation_x = orientation_x
        self.orientation_y = orientation_y
        self.orientation_z = orientation_z

    @staticmethod
    def create_from_tag_detection(time, tag_det):
        id = tag_det.id[0]
        size = tag_det.size[0]
        x = tag_det.pose.pose.pose.position.x
        y = tag_det.pose.pose.pose.position.y
        z = tag_det.pose.pose.pose.position.z
        orientation_w = tag_det.pose.pose.pose.orientation.w
        orientation_x = tag_det.pose.pose.pose.orientation.x
        orientation_y = tag_det.pose.pose.pose.orientation.y
        orientation_z = tag_det.pose.pose.pose.orientation.z

        return Detection(time, id, size, x, y, z, orientation_w, orientation_x, orientation_y, orientation_z)

    @staticmethod
    def create_from_tf(tf_stamped_msg):
        time = tf_stamped_msg.header.stamp
        id = tf_stamped_msg.child_frame_id
        x = tf_stamped_msg.transform.translation.x
        y = tf_stamped_msg.transform.translation.y
        z = tf_stamped_msg.transform.translation.z
        orientation_w = tf_stamped_msg.transform.rotation.w
        orientation_x = tf_stamped_msg.transform.rotation.x
        orientation_y = tf_stamped_msg.transform.rotation.y
        orientation_z = tf_stamped_msg.transform.rotation.z
        size = 0

        return Detection(time, id, size, x, y, z, orientation_w, orientation_x, orientation_y, orientation_z)

    def print_console_verbose(self):
        print("Detection Object:")
        print(f"Time: {self.time}")
        print(f"ID: {self.id}")
        print(f"Size: {self.size}")
        print(f"Position: ({self.x}, {self.y}, {self.z})")
        print(
            f"Orientation: ({self.orientation_w}, "
            f"{self.orientation_x}, "
            f"{self.orientation_y}, "
            f"{self.orientation_z})"
        )
        print()

    def print_console(self):
        print(
            f"{self.time}, "
            f"{self.id}, "
            f"{self.size}, "
            f"{self.x}, "
            f"{self.y}, "
            f"{self.z}, "
            f"{self.orientation_w}, "
            f"{self.orientation_x}, "
            f"{self.orientation_y}, "
            f"{self.orientation_z}"
        )

    def print_to_file(self, filename):
        with open(filename, "a", encoding="utf-8") as file:
            file.write(
                f"{self.time}, "
                f"{self.id}, "
                f"{self.size}, "
                f"{self.x}, "
                f"{self.y}, "
                f"{self.z}, "
                f"{self.orientation_w}, "
                f"{self.orientation_x}, "
                f"{self.orientation_y}, "
                f"{self.orientation_z}\n"
            )
