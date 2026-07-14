# SLAM_play

**SLAM_play** is a simple 2D SLAM (Simultaneous Localization and Mapping) simulation built in Python using `pygame`. The simulation features a robot that can move around a grid-based environment and map its surroundings using an ultrasonic sensor. The SLAM algorithm processes sensor feedback to detect obstacles and unexplored areas, updating the map in real time.

## Features

- **Real-time Robot Movement**: The robot can be controlled to move forward, backward, and rotate.
- **Ultrasonic Sensor Simulation**: A simulated ultrasonic sensor detects obstacles within a specified range and updates the map.
- **SLAM Algorithm**: The SLAM algorithm keeps track of the robot’s environment by:
  - Marking clear space as the robot moves.
  - Detecting obstacles and marking them on the map.
  - Indicating unexplored "frontier" areas when the sensor does not detect any obstacles.
- **Visual Feedback**: 
  - Clear spaces are marked in grey.
  - Detected obstacles are marked in green.
  - Unexplored frontiers (max sensor range) are marked in dark grey.
