[map]: ./snapshots/map.png
[db]: ./snapshots/map_db_viewer.png
[localization]: ./snapshots/localization.png
[navigation]: ./snapshots/navigation.png

# Home Service Robot
This document enumerates the packages used for Mapping, Localization and Navigation for this project.

## Mapping
**RTAB-Map Package**
RTAB-Map (*Real-Time Appearance-Based Mapping*) is an RGB-D SLAM approach based on a global loop closure detector with real-time constraints. Using this package, we can create a 2D occupancy grid map that can be used for navigating the robot.
![map]
![db]

## Localization
**AMCL Package**
AMCL (*Adaptive Monte Carlo Localization*) is a probabilistic localization system for a robot which implements adaptive Monte Carlo localization approach while using a particle filter to track the pose of the robot against a known map.
![localization]

## Navigation
**ROS Navigation Stack**
Using this package, we can creates a path for the robot based on Dijkstra's algorithm while avoiding obstacle along the way.
![navigation]

