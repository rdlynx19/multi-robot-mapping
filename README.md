# Collaborative Mapping using a Quadrotor and Quadruped
Author: Pushkar Dave\
This repository contains the packages, scripts, and the STL files for my MSR Winter Project\
More details about this project can be found on my website: [Pushkar's Portfolio](pushkardave.com)

## Package Descriptions
- `pi_mapping` : This package runs on the Raspberry Pi to launch the Oak-D camera nodes, the IMU node, and sync the image topics using rtabmap_sync
- `remote_rtabmap`: This package runs on the remote workstation (System76 Linux laptop), and launches all the required RTABMap nodes - rtabmap_slam, rtabmap_odom, rtabmap_viz, and rtabmap_relay
- `rpi_control`: This package runs in a Docker container on the Raspberry Pi on the quadrotor. It has a control node, which has multiple services to arm, disarm, takeoff, land and command the quadrotor to follow a polygonal trajectory.
- `rpi_interfaces`: This package contains all the interface definitions for the rpi_control package.
- `go1_exploration`: This package implements a frontier exploration node, to generate new target goal positions for the quadruped. Currently, this is used for visualization purposes.
- `scripts`: This is a collection of shell scripts to control the quadrotor, and start the mapping session.

