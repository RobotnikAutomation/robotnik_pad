# robotnik_pad

This package is intended to be used as a standard pad for all Robotnik robots. 

## Dependencies 

The package depends on some Robotnik packages:

- rcomponent [🔗](https://github.com/RobotnikAutomation/rcomponent/)
```bash
git clone https://github.com/RobotnikAutomation/rcomponent
```
- robotnik_msgs [🔗](https://github.com/RobotnikAutomation/robotnik_msgs/)
```bash
git clone https://github.com/RobotnikAutomation/robotnik_msgs
```

This package may depend on other Robotnik or ROS standard packages in function of the handlers that are developed. The standard ROS packages can be installed using the rosdep install command:

```bash
# located in the workspace folder
rosdep install --from-path src --ignore-src -y -r
```
---
## Node Description
The `robotnik_pad_node` loads plugins that specify the desired behaviour. This allows you to load different plugins depending on your needs. Besides, it is possible to create your own plugins.

### How To Launch it
```bash
roslaunch robotnik_pad robotnik_pad.launch
```

This will launch two nodes: 
- joy: This node is in charge of reading from the joystick and publish the information (sensor_msgs/Joy) through a topic
- robotnik_pad_node: This node will load the different plugins included in the config file

### How To Prepare The Config File
This an example of a config file loading a single plugin: 

```yaml
plugins: 
  - TwistMovement

TwistMovement:
  type: robotnik_pad_plugins/Movement
  scale_linear: 1.5
  scale_angular: 3
  cmd_topic_vel: cmd_vel
  config: 
    button_deadman: 5
    axis_linear_x: 1
    axis_linear_y: 0
    axis_angular_z: 2
    button_speed_up: 3
    button_speed_down: 1
    button_kinematic_mode: 7
```

First of all you need to define a list containing the different plugins you want to load. Then, for each of the plugins you want to load, you should specify its parameters.