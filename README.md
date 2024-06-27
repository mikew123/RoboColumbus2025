# RoboColumbus2025
Wheeled autonomus robot for 2025 DPRG RoboColumbus competion

This project was started in June 2024 and is expected to have a competitive robot for the RoboColumbus competition in Fall of 2025. I will drive the robot around the field in The Fall of 2024 and maybe have odometry and mapping show on ROS2 Rviz2.
https://www.dprg.org/robocolumbus-2024/

# RoboColumbus competition
The basic objective for RoboColumbus are:
-Three orange cones, designated as home, target, and challenge, are placed
outdoors approximately 100 yards from each other. The course between the home cone and the
target cone is clear of obstacles. The course between the challenge cone to both the home
cone and the target cone contains at least one obstacle that requires the robot to deviate from a
direct path. The challenge is to touch and stop at each cone.
- If multiple contestants find and touch the same number of cones then the total time is used to select the winner.
- Robots run on a field of grasses and some tree/bushes boxes etc for obsticles to avoid. Some pavement is OK.
- GPS coordinates of the 3 traffic cones in the field are provided. There should be obsticals between some of the cones. The owner can create GPS coordinates to avoid obsticles.
- The cones are orange and about 17" high and 7" diameter.
- Robots are NOT allowed on the field before competition.
- Run times are less than 15 Minutes
- Robots checked for weight(65 lbs), size(48x48x60 in), and safety switch compliance.

(https://www.dprg.org/wp-content/uploads/2024/06/robocolumbusplus-20240611-1.pdf)




# Robot platform
The AXIAL RCX6 1/6 scale Jeep rock crawler platform was selected to be used for the robot. This platform allows generous room for sensors and electronics to be added and can be installed into the interior of the Jeep for protection.
https://www.axialadventure.com/product/1-6-scx6-jeep-jlu-wrangler-4x4-rock-crawler-rtr-green/AXI05000T1.html

The motor drive, steering and shift control signals must be multiplexed between the RC receiver and the computer. A module in the engine compartment will be added to perform this with a serial interface to the computer in the Jeep cabin. The RC receiver is powered from the motor controller (facrory default) and the connections to the computer is optional, the module will power up to use the RC receiver signals.

The main battery in the engine compartment will also be used for sensors and computer power. A DC-DC module would be added in the engine compartment with 2 sets of 5V at 3 Amps power leads to the Jeep cabin. Fusing the power from the main battery is added for protection. Combining the control multiplexor and DC converter is a good idea.

# Connectivity
- Loran for run and kill switch
- RC receiver for manual steering control
- WIFI for development and debug and course configuration

# Sensors
## GPS
GPS receiver with optional RTK capbility
Used for primary odometry
## IMU
IMU to provide tilt compensation for sensors and maybe bearing to help GPS
## Camera
Object detection and depth camera
## LIDAR
Optional rotating LIDAR for far obsticle detection and avoidance
## TOF
Time Of Flight sensors on front, rear and optional sides with mm resolution for close obstical avoidance and close cone sensor

# Computer 
- Raspberry Pi4 (or 5)
- Enbedded PC with Linux or Windows (WSL)
  
# Software
## ROS2
- Humble or Iron requires Linx Ubuntu 20.04 OS
- 

