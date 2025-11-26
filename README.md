# robotnik_pad

This package is intended to be used as a standard pad for all Robotnik robots.


## Installation

To use this package with a PS5 Dualsense pad, it is necessary to install:

- [ps5_pad_driver](https://github.com/RobotnikAutomation/ps5_pad_driver)

To use it with ps4:

- [joy_linux](https://index.ros.org/p/joy_linux/)

This package may depend on other Robotnik or ROS standard packages in function of the handlers that are developed. The standard ROS packages can be installed using the rosdep install command:

```bash
# located in the workspace folder
rosdep install --from-path src --ignore-src -y -r
```

### PS4 Dualshock automatic installation
In order to install the ds4drv and its components, ypu can use the installer:

```bash
sudo ./resources/ds4drv-install.sh
```
Now you your system should be ready.

### PS4 Dualshock manual installation

You need to install the ds4drv pip script:

```bash
sudo pip3 install --force-reinstall six==1.13.0
sudo pip3 install --force-reinstall evdev==0.8.1
sudo pip3 install ds4drv
```

Install PS4 controller config for ds4drv:
```bash
cd /etc && sudo wget https://raw.githubusercontent.com/RobotnikAutomation/robotnik_pad/jazzy-devel/resources/ds4drv.conf
```
```bash
cd /etc/systemd/system && sudo wget https://raw.githubusercontent.com/RobotnikAutomation/robotnik_pad/jazzy-devel/resources/ds4drv.service
```
Add the udev rules for PS4 controller:

```bash
cd /etc/udev/rules.d/ && sudo wget https://raw.githubusercontent.com/RobotnikAutomation/robotnik_pad/jazzy-devel/resources/55-ds4drv.rules
```

Enable the execution of the ds4drv service on boot:
```bash
sudo systemctl daemon-reload
```
```bash
sudo systemctl enable ds4drv.service
```
Now the ds4drv is loaded on boot.

To enable the joystick without rebooting:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo systemctl start ds4drv.service
```

### PS5 Dualsense installation
Add the udev rules for PS5 controller:

```bash
sudo cp resources/55-dualsense.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---
# 1. robotnik_pad node

The `robotnik_pad` node loads plugins that specify the desired behaviour. This allows you to load different plugins depending on your needs. Besides, it is possible to create your own plugins.

Available plugins:
* **Movement**
    * Intended to command the mobile base via velocity references

## 1.1. Parameters
### 1.1.1. Common parameters
* ~**desired_freq** (double, default: 40): Frequency of the node.
* ~**pad.num_of_buttons** (int, default: 0): Number of buttons published by the joy topic. Must be the same to work.
* ~**pad.num_of_axes** (int, default: 0): Number of axes published by the joy topic. Must be the same to work.
* ~**pad.joy_topic** (string, default: joy): Name of the topic the node has to subscribe to.
* ~**pad.joy_timeout** (double, default: 5.0): Max time the node can wait without receiving joy messages.
* ~**plugins** (list of strings, default: []): List of the pluginst to be loaded.

### 1.1.2. Plugin parameters
* ~***Plugin*.type** (string, default: ""): Type of plugin.

#### 1.1.2.1. Movement plugin
* ~**max_linear_speed** (double, default: 1.5):  Maximum linear speed that can be sent to the controller based on the current velocity level (0.1->1) and the current axis_linear_x or axis_linear_y value (0->1).
* ~**max_angular_speed** (double, default: 1.5): Maximum angular speed that can be sent to the controller based on the current velocity level (0.1->1) and the current axis_angular_z value (0->1).
* ~**cmd_topic_vel** (string, default: cmd_vel): Name of topic where the command vel is being published.
* ~**wheel_base** (double, default: 0): Distance between the front axle and the rear axle of the vehicle. If set to 0, ackermann kinematics will not be taken into account when kinematics are changed.
* ~**config/button_deadman** (int, default: 5): Button number to enable any command sent to the controller.
* ~**config/button_speed_up** (int, default: 3): Button number to increase the current velocity level applied to the max_speed params.
* ~**config/button_speed_down** (int, default: 1): Button number to decrease the current velocity level applied to the max_speed params.
* ~**config/button_kinematic_mode** (int, default: 6) Button number to switch between kinematic modes: diff, omni and ackermann.
* ~**config/axis_linear_x** (int, default: 1): Axis number to set the linear x speed.
* ~**config/axis_linear_y** (int, default: 0): Axis number to set the linear y speed.
* ~**config/axis_angular_z** (int, default: 2) Axis number to set the angular speed.
* ~**config/use_accel_watchdog** (bool, default: true): Flag to check if any of the defined watchdog axes is changing its value in order to keep publishing velocity commands.
* ~**config/watchdog_duration** (double, default: 0.5): Time in seconds the node will wait without receiving changes in the watchdog axes before stopping publishing velocity commands.
* ~**config/axis_watchdog** (list of ints, default: [8]): Axes used to monitor that the joy is updating buttons and axes. The defined axes should correspond to the accelerometers of the pad.

### 1.1.3. Configuration example
This an example of a config file loading a single plugin:

```yaml
/**:
  ros__parameters:

    plugins:
      - Movement

    desired_freq: 10.0

    pad:
      num_of_buttons: 14
      num_of_axes: 14
      joy_topic: joy
      joy_timeout: 1.0
    
    Movement:
      type: robotnik_pad_plugins/Movement
      max_linear_speed: 1.0
      max_angular_speed: 1.5
      cmd_topic_vel: pad_teleop/cmd_vel
      config:
        button_deadman: 5
        axis_linear_x: 1
        axis_linear_y: 0
        axis_angular_z: 2
        button_speed_up: 3
        button_speed_down: 1
        button_kinematic_mode: 7
        use_accel_watchdog: true
        axis_watchdog: [6,7,8]
        watchdog_duration: 0.5

```
First of all you need to define a list containing the different plugins you want to load. Then, for each of the plugins you want to load, you should specify its parameters.

## 1.2. Subscribed topics
* **joy** ([sensor_msgs/msg/Joy](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Joy.html)): Gets the buttons and axis status.

## 1.3. Published topics
### 1.3.1. Plugin published topics
#### 1.3.1.1. Movement
* **cmd_topic_vel** ([geometry_msgs/msg/Twist](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html)): Sends the velocity references to defined topic.

## 1.4. Services

None

## 1.5. Services Called

None

## 1.6. Action server

None

## 1.7. Action clients called

None

## 1.8. Required tf Transforms

None

## 1.9. Provided tf Transforms

None

## 1.10. Bringup

```bash
ros2 launch robotnik_pad pad.launch.py
```

This will launch two nodes:
- **joy_node**: This node is in charge of reading from the joystick and publishing the information (sensor_msgs/msg/Joy) through a topic.
- **robotnik_pad**: This node will load the different plugins included in the config file.

### 1.10.1. Launch arguments

* **robot_id** (string, default: $ROBOT_ID (if set), otherwise 'robot'): Namespace of the nodes.
* **log_level** (string, default: $LOG_LEVEL (if set), otherwise: 'info'): Logger level.
* **pad_model** (string, default: $ROBOT_PAD_MODEL (if set), otherwise: 'ps5'): Pad model used.
* **deadzone** (double, default: $ROBOT_PAD_DEADZONE (if set), otherwise: 0.1): Deadzone for the joystick axes.

Arguments loaded by the PS5 joy node:
* **desired_freq** (double, default: 40.0): Frequency of the node.
* **serial_number** (string: default: ''): Serial number of the pad (the node can work without the argument being set).

Arguments loaded by the PS4 joy node:
* **device** (double, default: $ROBOT_PAD_DEV (if set), otherwise: '/dev/input/js_base'): Device of the PS4 pad.
* **autorepeat_rate** (double, default: 0.0): Rate in Hz at which a joystick that has a non-changing state will resend the previously sent message.

### 1.10.2. Loaded config files
* **robotnik_pad/config/pad.yaml**

---

## 2. Plugin Development

The `robotnik_pad` system supports custom plugins. To create a new plugin package, use the provided plugin template generator script.

### 2.1. Creating a New Plugin Package

Use the `plugin_template_generator.sh` script located in the root of the repository. It will ask for a `<plugin_name>` to create the package.

```bash
./plugin_template_generator.sh
```

**Note:** The package will be created in the root of the repository. Consider moving it outside.

Build the new package:

```bash
colcon build --packages-select <plugin_name>_pad_plugins
```

Add the configuration in config/pad.yaml found in the robotik_pad packages

```
ros__parameters:
  plugins:
    - <PluginName>
  
  <PluginName>:
    type: <plugin_name>_pad_plugins/<PluginName>
    max_linear_speed: 1.0
    max_angl...
```
