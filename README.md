# IRC BackUp Plan

This repository contains Python scripts designed to manage the control and navigation of a rover.

## Repository Structure

### 1. `coord_follow_arrow.py`
- **Functionality**: Implements a rover control system that follows detected arrows. It processes visual cues to navigate toward the direction indicated by the arrows.
- **Core features**:
  - Reads sensor data to follow arrows.
  - Adjusts the robot's orientation and path based on the detection of visual cues.
  - Uses ROS to handle real-time movement updates and control flow.

### 2. `coord_gps_map.py`
- **Functionality**: Maps the rover's GPS coordinates in real time, allowing visualization of its path.
- **Core features**:
  - Uses GPS data to track and plot the rover's position on a 2D grid.
  - Continuously updates the plot as the rover moves, providing real-time feedback.

### 3. `coord_maincontrol.py`
- **Functionality**: Acts as the main control module, managing the entire system's workflow. This script coordinates searching, detection, and movement processes.
- **Core features**:
  - Detects objects (arrows) in the environment using a CNN-based detection pipeline.
  - Controls navigation decisions: moves towards detected objects and handles 90-degree rotations.
  - Integrates GPS and visual data to make movement decisions.

### 4. `coord_rotate_90.py`
- **Functionality**: Handles precise 90-degree rotations of the rover.
- **Core features**:
  - Rotates the rover in place by 90 degrees after detecting an arrow or completing a movement task.
  - Ensures accurate turning using sensor feedback and ROS.

### 5. `coord_search_arrow.py`
- **Functionality**: Focuses on detecting arrows in the environment and triggering appropriate responses.
- **Core features**:
  - Implements the search algorithm for arrows.
  - Uses a combination of visual detection and movement commands to locate and approach arrows.


