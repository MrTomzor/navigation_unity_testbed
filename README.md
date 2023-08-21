# HARDNAV - Introduction
Hi! Thank you for showing interest in our simulator! The full work-in-progress publication name is "HARDNAV - Simulator for Benchmarking Robust Navigation and Place Recognition in Large, Confusing and Highly Dynamic Environments". We hope this simulator can help you quickly test your perception and spatial intelligence methods against the extreme conditions that occur in the real world before doing costly real world experiments. Any feedback is greatly appreciated! This is still a work-in-progress, so some things are bound to break. If you want a stabler experience, please come back in a few weeks. If you want to dig around in this now, then great! The simulator is mostly built thanks to the Unity-Robotics-Hub (https://github.com/Unity-Technologies/Unity-Robotics-Hub) so that is worth checking out!

Quick showcase of the simulator:

[<img src="https://img.youtube.com/vi/AT6wtF-p_fQ/hqdefault.jpg" width="600" height="300"
/>](https://www.youtube.com/embed/AT6wtF-p_fQ)

# Dependencies
ROS Dependencies:
- [ROS-TCP-Endpoint ROS package](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)
  
Unity packages/assets (not needed if you use the executable, only if you want to play around with the simulator insides):
- [YamlDotNet](https://assetstore.unity.com/packages/tools/integration/yamldotnet-for-unity-36292)
- TODO - currently the terrain assets are broken, so we are looking for a new package for them

# Usage guide
To use the HARDNAV simulator, there are two options:
1) You can download one of the executable builds at the bottom of the readme,
2) Or, to make modifications to the existing worlds, create new ones or add your own features into the worlds, you can download the Unity Hub from https://unity.com/download and open the "navigation_unity_project" folder as a Unity project.

For both of these options, the simulator connects to ROS automatically if there is a ROS TCP Bridge node (see dependencies) running, and if you have set up networking properly. If the icon in the top left corner of the Unity game screen is blue, Unity is connected to ROS. For troubleshooting the ROS-Unity connection, please see the repo of the ROS-TCP-Endpoint.

The world is, after running the game, in some default state (e.g. clouds, position of random objects, dynamic objects, ...), where it was left off in the editor before building. To reset it at runtime, you can run the launch file "default_world.launch" from the "navigation_unity_core" ROS package in this repo. This runs a python node which reads the YAML config file "navigation_unity_core/unity_world_config/default_world.yaml" and sends it through a service call to the Unity world.

To play around with changing the world settings, you can run the "session_cycler.py" node for showcasing changes of 4 world states, as shown in the video above, or run "place_recognition_from_motion_trial.py" for an example of how an active place recognition trial could work (you can test it on willing human subjects - tell them to try recognizing which area they are spawned in!)

# State of the project
TODO

# Builds
TODO
