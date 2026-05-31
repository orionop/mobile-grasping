from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# This file is consumed by catkin_python_setup() in CMakeLists.txt.
# It exposes `src/mobile_grasping_ros/` as an importable Python module
# under the name `mobile_grasping_ros` after `catkin_make` + sourcing
# devel/setup.bash.

setup_args = generate_distutils_setup(
    packages=["mobile_grasping_ros"],
    package_dir={"": "src"},
)

setup(**setup_args)
