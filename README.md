# Inspection_GDP

Multi-UAV indoor simulation in ROS Melodic + Gazebo 9 for mapping and inspection.

This project simulates two UAVs in a custom Gazebo indoor environment:

* **UAV1 (surveyor):** performs lawnmower mapping with RGB-D + lidar
* **UAV2 (inspector):** flies to a detected feature of interest (FOI)
* RTAB-Map is used for mapping and map export
* Custom Gazebo models and materials are included in this repository

---

## Tested on

* Ubuntu 18.04
* ROS Melodic
* Gazebo 9

---

## Features

* Custom indoor room world
* Two Hector quadrotors
* Custom UAV sensors:

  * 2D lidar
  * RGB-D camera
* RTAB-Map integration
* AprilTag localization pipeline
* Automatic mission metrics logging

---

## Repository structure

```text
inspection_GDP/
├── launch/          # ROS launch files
├── worlds/          # Gazebo world files
├── models/          # custom Gazebo models
├── media/           # Gazebo materials / textures
├── scripts/         # mission logic and helper nodes
├── urdf/            # custom UAV sensor xacro
├── start_sim.sh     # startup helper script
└── README.md
```

---

## Dependencies

Install the required ROS packages:

```bash
sudo apt update
sudo apt install -y \
  ros-melodic-desktop-full \
  ros-melodic-gazebo-ros \
  ros-melodic-gazebo-plugins \
  ros-melodic-gazebo-msgs \
  ros-melodic-hector-quadrotor \
  ros-melodic-hector-quadrotor-description \
  ros-melodic-hector-quadrotor-gazebo \
  ros-melodic-hector-uav-msgs \
  ros-melodic-rtabmap \
  ros-melodic-rtabmap-ros \
  ros-melodic-xacro \
  ros-melodic-tf
```

---

## Installation

### 1. Create a catkin workspace (if needed)

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```

### 2. Source ROS Melodic

```bash
source /opt/ros/melodic/setup.bash
```

### 3. Clone the repository

```bash
cd ~/catkin_ws/src
git clone <YOUR_REPO_URL>
```

### 4. Build the workspace

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## Running the simulation

The easiest way to launch everything is using the helper script:

```bash
cd ~/catkin_ws/src/inspection_GDP
chmod +x start_sim.sh
./start_sim.sh
```

This script will:

* kill old ROS / Gazebo processes
* remove previous RTAB-Map databases
* rebuild the workspace
* source the workspace
* launch the simulation

---

## Manual launch (optional)

If you prefer launching manually:

```bash
cd ~/catkin_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
roslaunch inspection_GDP room.launch
```

---

## Important notes

### Custom Gazebo assets

Custom models and textures are already included in this repository.

The launch file automatically sets:

* `GAZEBO_MODEL_PATH`
* `GAZEBO_RESOURCE_PATH`

so you should **not need to manually copy models into `~/.gazebo/models`**.

### Custom quadrotor model

This project uses a custom UAV sensor setup defined in:

```text
urdf/quadrotor_custom.gazebo.xacro
```

This avoids modifying upstream Hector package files.

### Upstream dependencies still required

This project depends on upstream ROS packages:

* Hector Quadrotor stack
* RTAB-Map

These must be installed through ROS or built from source.

---

## Output files

Simulation results may be written to:

```bash
~/simulation_results/
```

RTAB-Map temporary database:

```bash
~/.ros/rtabmap.db
```

---

## Troubleshooting

### Gazebo cannot find a model

Make sure you launch with:

```bash
roslaunch inspection_GDP room.launch
```

or use:

```bash
./start_sim.sh
```

### Package not found

Rebuild and source the workspace:

```bash
cd ~/catkin_ws
catkin_make
source /opt/ros/melodic/setup.bash
source devel/setup.bash
```

### Python script permission issues

Run:

```bash
chmod +x ~/catkin_ws/src/inspection_GDP/scripts/*.py
```

---

## Acknowledgments

This project builds on:

* Hector Quadrotor packages
* RTAB-Map
* Gazebo / ROS simulation tools

---
