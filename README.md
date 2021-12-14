> Project: "Depth filter"

> Owner: "Roberto Cappellaro" 

> Date: "2021:10" 

---

# Depth filter

## Description of the project
Ros2 foxy package that filters a depth image based on the minumum and maximum distance from the camera, along an axis normal to the camera palane, and a hight interval, from the camera reference frame. It sets the points outside the filtering volume to infinity.

## Installation procedure
Clone the repository in your colcon workspace and build it. It depends on the ros core package and the sensor_msgs package.

## User Guide
Set the parameters in the launch file.
