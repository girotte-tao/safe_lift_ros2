
# safe_lift_ros2

## Overview
`safe_lift_ros2` is a ROS2-based project designed for handling hardware, specifically focusing on processing data from Livox HAP LiDAR sensors. This project contains a package named `hapManager` which is dedicated to managing and subscribing to the data published by the Livox sensors.

## Prerequisites
- ROS2 (latest version recommended)
- Livox HAP LiDAR Hardware
- livox_ros_driver2 https://github.com/Livox-SDK/livox_ros_driver2

## Installation and Setup

1. **Clone the Repository:**
   Navigate to your ROS2 workspace and clone the repository.
   ```bash
   git clone https://github.com/girotte-tao/safe_lift_ros2.git
   ```

2. **Build the Package:**
   Compile the `hapManager` package using `colcon`.
   ```bash
   cd safe_lift_ros2/src
   colcon build --packages-select hapManager
   ```

3. **Source the Setup Script:**
   After building, source the setup script to include the new package in your environment.
   ```bash
   source install/setup.sh
   ```

## Usage

To run the `hapManager` package and start subscribing to messages from the Livox LiDAR:

```bash
ros2 run hapManager livox_subscriber
```

This command will initiate the `livox_subscriber` node and begin handling the LiDAR data.

## Contributing

Feel free to contribute to the development of `safe_lift_ros2`. Please read `CONTRIBUTING.md` for details on our code of conduct, and the process for submitting pull requests.

## License

[//]: # (This project is licensed under the [Your License] - see the `LICENSE.md` file for details.)

## Acknowledgments

[//]: # (- Mention any inspirations, code snippets, etc.)
