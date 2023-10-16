# Autonomous Indoor Mapping & Target Tracking Drone System


## Introduction
This project pioneers the fusion of Raspberry Pi 4B, YDLIDAR X2L, and Pixhawk drone to implement real-time indoor mapping and target tracking. Integrated with a Raspberry Pi camera, the drone can navigate autonomously within an indoor environment, identify, and track specific targets.

The Indoor Autonomous Driving & Target Tracking Drone is specifically designed to navigate and operate within indoor environments. Using visual data from onboard cameras, the drone can autonomously traverse confined spaces and track designated targets or objects. This system offers potential applications in security, surveillance, and asset tracking, such as identifying unauthorized individuals, locating specific objects, or continuously monitoring a predefined area.

## Features
Real-time Indoor Mapping: Uses YDLIDAR X2L to generate maps of indoor environments dynamically.
Drone Localization: Efficiently tracks the drone's precise position within the generated map.
Autonomous Navigation: Employs Raspberry Pi cameras for smart obstacle navigation.
Target Detection & Tracking: Utilizes computer vision techniques to identify and pursue specific targets.

## Prerequisites
- Hardware
    - Raspberry Pi 4B
    - YDLIDAR X2L
    - Pixhawk Drone
    - Raspberry Pi Camera
    
- Software
    - Ubuntu 20.04 LTS
    - ROS (Robot Operating System)
    - mavlink, pymavlink
    - OpenCV (for image processing) - Preparing

# System Mechanics (Concept)

## Indoor Autonomous Navigation
- Pre-mapping with RTAB-Map: An iPhone's LiDAR sensor can pre-map the indoor structure using the RTAB-Map application, reducing the time required for the drone's onboard LiDAR sensor to measure the environment. Although the drone can also perform direct mapping, this technique can expedite the process.
- LiDAR-based Mapping with ROS: Utilizes the Robot Operating System (ROS) to represent location data measured by sensors in 3D or 2D formats via the SLAM MAPPING algorithm.
- Localization at Launch: Upon takeoff, the drone uses ROS's NAVIGATING algorithm to compare the pre-mapped structure with real-time LiDAR data to precisely determine its location.
- Route Planning: Employs ROS's route planning algorithm for smooth and efficient autonomous indoor navigation.

## Target Tracking Mechanism
- YOLO for Object Detection: Utilizes the YOLO CNN deep learning model to identify and learn about potential tracking targets.
- OpenCV for Real-time Tracking: Uses pretrained models within OpenCV to track learned objects continuously.
- Vector-based Drone Movement: Represents the tracking target as a vector in OpenCV. Raspberry Pi then translates this vector into motor operation commands for the Pixhawk, thereby controlling the drone's speed and direction.




## MAVROS: Bridging ROS & MAVLink for Autonomous Drone Navigation
MAVROS provides a bridge between ROS (Robot Operating System) and MAVLink, packaging MAVLink to allow streamlined ROS communications.


## Overview

- Definition: MAVROS is a packaged version of MAVLink for ROS, enabling direct communication between various MAVLink-equipped systems. It functions as a communication driver, ensuring efficient data exchange between drones or UAVs.
   
- Usability: MAVROS can be deployed for both ground control systems and aerial platforms using UDP communication.
   
- Core Features: MAVROS offers various topics to users, allowing them to transmit and receive essential information based on their system configuration.


# ROS Architecture
- Basics: ROS operates through a master-node architecture where nodes represent executable programs, and the master facilitates inter-node communication, the essence of ROS.

- Master-Node Communication: Initially, the master connects nodes. After establishment, nodes communicate directly, bypassing the master. Nodes utilize XMLRPC, a specification and set of implementations, to call procedures over the internet, employing HTTP for transmission and XML for encoding.

- Information Exchange: Nodes provide the master with four key details: node name, topic/service name, message type, and URI address/port. This exchange typically occurs when nodes are initiated using commands like "rosrun" or "roslaunch".

- Direct Node Communication: After initial setup by the master, subscriber nodes (nodes intending to receive data) connect directly to publisher nodes (nodes broadcasting data). In this, the publisher acts as the TCPROS server while the subscriber functions as the client.


## PX4 Structure
- Bridging Gap: MAVROS acts as a bridge between ROS and PX4, ensuring seamless data flow across numerous topics.

- PX4 Layers: PX4 consists of two primary layers - the PX4 flight stack, responsible for drone flight functions, and the PX4 middleware used in general robotics.

- PX4 Flight Stack: This stack processes sensor data to deduce position and orientation. Data flows into controllers, which then relay commands to the drone's motors via a mixer. The mixer translates high-level force or torque commands into specific motor signals, adjusting for the drone's configuration.

- High-Level Missions: Commands from ROS, regarded as high-level missions, enter the PX4 system through the commander and navigator. They're then interpreted and executed while maintaining constant feedback to ROS via MAVLink.


## Integration with SITL
In the context of Software In The Loop (SITL), MAVROS interacts with two main nodes executed via roslaunch: mavros and pub_setpoints_position. Upon starting these nodes, communication ensues, facilitating data exchange between the PX4 and ROS.
