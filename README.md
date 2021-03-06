# Quadruped-Robot

### A Low Cost Quadruped Robot with Autonomous Capabilities

The aim was to develop a low cost quadruped robot so that it has the capability to traverse autonomously on irregular terrains:

1) The quadruped was designed employing the 5bar - 5R mechanism, allowing us to mount all the actuators on the chassis of the robot, thus minimizing the moment of inertia of the leg. 
2) Carbon Fibre Reinforced Polymer tubes were used for manufacturing the links of the mechanism to keep it feather light. By using appropriate materials and other weight reduction techniques we were able to limit the weight of the robot to 1.15 kg.
3) Stress analysis of the various mechanical components of the robot to prevent failures. Material Optimizations were implemented to reduce weight and strengthen the components.
4) ROS was integrated with the robot, after developing the URDF of the 5bar - 5R closed chain. Then ROS controllers on each of the eight actuators were set up to achieve the joint control. 
5) Stereo Camera and IMU sensors were integrated with the robot in the simulation. Extended Kalman Filtering was used for the fusion of the two sensor data to produce a much accurate odometry.
6) Equations of motion were derived for the robot A base controllers was developed to provide movement to the toe and to generate the various types of motion given the command velocity.
7) Tested using Gazebo based physics simulation for the robot, to validate the equations derived, check robot stability and to test the efficiency of the base controller.

Find more information about the robot [here](https://drive.google.com/file/d/16AhPm6JRH62gSZBvzB-JlDFh6qmGrHIq/view?usp=sharing).

Check out the [videos](https://drive.google.com/drive/folders/1-zKFX-dxUtHMzNQqa7gvRL7YORVA8Zfu?usp=sharing) of the prototype and simulation.

## Includes:

1) Description Files
2) Gazebo Simulation Package
3) ROS Control Package
4) Hardware Drivers
5) Matlab Codes for Trajectory Generation
6) CAD Files


