# neural_controller
This repository aims to ease the deployment of neural network control policies on robot hardware. It runs within the [ros2_control](https://github.com/ros-controls/ros2_control) framework and adopts [RTNeural](https://github.com/jatinchowdhury18/RTNeural) for real-time inference.

## Motivation
Getting a policy up and running on real hardware can be surprisingly tricky. Many opportunities exist for mistakes in processing the observations and actions. 
Furthermore, common neural network libraries such as Torch are not designed to run in a real-time control loop, introducing latency/jitter that increases the difficulty of sim-to-real transfer. 

# Functionality
## Inputs
- Maps hardware_interface states and ROS topics (such as cmd_vel) to the policy obersvation vector
- Performs processing such as quaternion-to-gravity-vector conversion and scaling/normalization on the observations
- Allows for parts of the observation vector to be hardcoded as fixed values

## Outputs
- Performs scaling/normalization on the actions
- Maps the policy action vector to hardware_interface commands

## Convenience and safety features
- On startup, smoothly returns the robot to a predefined starting pose
- Easy loading of policies trained in Isaac Gym (just drag and drop a new .pt file)
- Triggers an emergency stop when safety limits are exceeded (joint velocity, body pitch, etc.)

## How to configure