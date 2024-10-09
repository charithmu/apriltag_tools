from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

d = generate_distutils_setup(
    packages=['apriltag_tools'],
    package_dir={'': 'src'},
    requires=['rospy', 'std_msgs', 'geometry_msgs', 'apriltag_ros', 'pynput']
)

setup(**d)