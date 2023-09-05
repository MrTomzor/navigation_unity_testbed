## HARDNAV - Introduction
Hi! Thank you for showing interest in our simulator! The full work-in-progress publication name is "HARDNAV - Simulator for Benchmarking Robust Navigation and Place Recognition in Large, Confusing and Highly Dynamic Environments". We hope this simulator can help you quickly test your robust perception and spatial intelligence methods against the varying, often extrem conditions that can occur in the real world before doing costly real world experiments. Any feedback is greatly appreciated! 

Quick showcase of the simulator:

[<img src="https://img.youtube.com/vi/AT6wtF-p_fQ/hqdefault.jpg" width="600" height="300"
/>](https://www.youtube.com/embed/AT6wtF-p_fQ)

## Dependencies
ROS Dependencies:
- [ROS-TCP-Endpoint ROS package](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)
  
Unity packages/assets (not needed if you use the executable, only if you want to edit the project in the Unity Editor, add your own features etc.):
- [YamlDotNet](https://assetstore.unity.com/packages/tools/integration/yamldotnet-for-unity-36292) - needed for using YAML to reconfigure the world, project will not compile without it
- [DreamForestTree](https://assetstore.unity.com/packages/3d/vegetation/trees/dream-forest-tree-105297) - contains trees, vegetation and ground textures used in Forest1 and ground texture for ScifiBase1. You might want to add colliders to the trees, as they dont have them by default in the package.
- [Yughues Free Metal Materials](https://assetstore.unity.com/packages/2d/textures-materials/metals/yughues-free-metal-materials-12949) - very nice materials used for most objects in the simulator, and also for reflective and transparent walls

## Getting started
To use the HARDNAV simulator, there are two options:
1) You can download one of the executable builds at the bottom of the readme,
2) Or, to make modifications to the existing worlds, create new ones or add your own features into the worlds, you can download the Unity Hub from https://unity.com/download and open the "navigation_unity_project" folder as a Unity project. This will require importing the assets mentioned in the Dependencies section.

For both of these options, the simulator connects to ROS automatically if there is a [ROS TCP Endpoint node](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) running along with the Unity game, in which there is some ROS publisher/subscriber node. If the icon in the top left corner of the Unity game screen is blue, Unity is connected to ROS. For troubleshooting the ROS-Unity connection, please see the repo of the ROS-TCP-Endpoint.

The world is, after running the game, in some default state (e.g. clouds, position of random objects, dynamic objects, ...), where it was left off in the editor before building. To reset it at runtime, you can run the launch file "default_world.launch" from the "navigation_unity_core" ROS package in this repo. This runs a python node which reads the YAML config file "navigation_unity_core/unity_world_config/default_world.yaml" and sends it through a service call to the Unity world.

To play around with changing the world settings, you can run the "session_cycler.py" node for showcasing changes of 4 world states, as shown in the video above, or run "place_recognition_from_motion_trial.py" for an example of how an active place recognition trial could work (you can test it on willing human subjects - tell them to try recognizing which area they are spawned in!). You can use these as examples to design your own experiments for developing and benchmarking robust vision-based spatial intelligence methods :)

## State of the project
### Sensors and robot models
Available:
- IMU sensor with gaussian noise, any number of monocular cameras with their camera_info published automatically (by default there is 1 camera on each robot, to add more, you for now have to edit the Unity project), "perfect odometry" for debugging and evaluating methods (a transform is published between the robot and its last spawnpoint, and also between the robot and the world coodinate frame origin). There are 2 available robots - "generic_wheeled" robot with wheels and a car-like controller, and a "generic_space" robot which is fully velocity/force controlled and has gravity disabled.
  
In progress:
- URDF robot importing, depth camera, ultrasound rangefinder

### Challenges for robustness
The ones in normal text are available, the ones in italics are planned:
![Alt text](media/challenges.png)

### Known issues
- There is some issue with post-processing when doing manual render calls, in which the output ROS image is blurred too much, if the camera component is not active. However, if it is active, Unity seems to also render the camera on its own along with the manual render calls. Therefore, post-processing is now disabled for the ROS image data.
- Large difference between realtime factor when running the game in the Editor and when running the built version (getting RTF of ~1.0 in the forest when the debug camera is disabled in Editor,but in the build, the RTF does not improve when debug cam is disabled in the built game and stays around 0.6 on a standard laptop). It's likely a problem with some project settings and timestep configuration.

## Builds
Build 1.0 for Linux can be downloaded [here](https://nasmrs.felk.cvut.cz/index.php/s/pb72nPHCTmn9cF2). On startup, it will load up a "generic_space" robot with a monocular camera in the "forest1" scene. To change the scenes, modify the config file "ros_packages/navigation_unity_core/unity_world_config/default.yaml" and then apply it by running "rosrun navigation_unity_core default_world_loader.py". Any feedback is welcome!
