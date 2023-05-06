# f1tenth_labs_openrepo
Branch of Lab Assignments for the 2022 UPenn F1TENTH Course. All programs are written with Python with the C++ nodes not touched.

Labs 1,2,3, and 4 are complete with SLAM and the Particle Filter running on the robot so lab 6 (Pure Pursuit) and be started.

# Simulator Notes

The F1TENTH Gym is located [here](https://github.com/f1tenth/f1tenth_gym_ros) and it is easier to follow the **Without a NVIDIA GPU** guideslines for setup. The simulator does not really utilize for GPU for calculations but instead uses it to display important vizuilzations tools in ROS (RViz, rqt graphs, and Gazebo if you end up installing it). It might be weird to not install ROS or the simulator directly but that is the use of Docker (an application similar to a virtual machine (but not actually a VM) for packaging applications with all their dependencies together) which is already compliled for us by F1TENTH.
