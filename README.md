# README #

This README would normally document whatever steps are necessary to get your application up and running.

### What is this repository for? ###

* Quick summary
* Version
* [Learn Markdown](https://bitbucket.org/tutorials/markdowndemo)

### How do I get set up? ###

* Summary of set up
* Configuration
* Dependencies
* Database configuration
* How to run tests
* Deployment instructions

### Contribution guidelines ###
# ROS 2 Point Cloud Detection

A ROS 2 package for processing 3D point cloud data from a RGB-D camera to detect:

- missing ground (e.g. holes, staircases, drop-offs)
- obstacles within a defined height range

The output is converted into a 2D `LaserScan` like representation for integration with mobile robot navigation pipelines.

## Overview

Traditional 2D laser scanners cannot detect hazards such as missing ground or hanging obstacles outside their scan plane. This package extends perception by analyzing depth point clouds and extracting these features from 3D data.

## Approach

The algorithm works in two stages:

1. **Ground detection**  
   Points within a configurable ground height range are projected into a polar grid in front of the robot. If a grid cell contains fewer than a threshold number of ground points, it is treated as missing ground.

2. **Obstacle detection**  
   If valid ground is present, points above the ground range and below a maximum height are classified as obstacles. These are projected into 2D and returned as scan points.

## Features

- Point cloud transformation into target frame
- Ground / fall detection using polar grid analysis
- Obstacle detection within configurable height limits
- 2D scan output for downstream navigation modules
- ROS 2 launch support

## Tech Stack

- ROS 2 Humble
- Python (`rclpy`)
- `sensor_msgs/PointCloud2`
- `sensor_msgs/LaserScan`
- TF transformations

## Run

```bash
colcon build --packages-select pointcloud_detection
source install/setup.bash
ros2 launch pointcloud_detection pointcloud_object_detection_launch.py
* Writing tests
* Code review
* Other guidelines

### Who do I talk to? ###

* Repo owner or admin
* Other community or team contact