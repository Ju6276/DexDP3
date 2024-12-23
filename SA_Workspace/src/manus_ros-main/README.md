# Manus ROS2
This is the ROS2 warpper for the manus glove.
- For ROS1 with version 2.3.0.1 SDK checkout v0.2
- For ROS2 with version 2.3.0.1 SDK checkout v0.4
- For ROS2 with version 2.4.0 SDK checkout v0.5 or the main branch
## Setup Manus glove
- Note: current manus-sdk version: 2.4.0
- Documentation: https://docs.manus-meta.com/2.4.0/Plugins/SDK/
- Copy the udev rules from `config/70-manus-hid.rules` to `/etc/udev/rules.d/`
```bash
sudo cp config/70-manus-hid.rules /etc/udev/rules.d/
```

## ROS2 setup
### Run code to get data from data glove
```bash
ros2 run manus_ros_driver full_client
```

### Known issues
- Current `manus_ros_driver` for ROS2 does not support launch from launch file, use `ros2 run manus_ros_driver full_client` to start the manus node
