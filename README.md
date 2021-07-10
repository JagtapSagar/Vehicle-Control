Vehicle Control
===

[Longitudinal Control](#Longitudinal-Control)/[Lateral Control](#Lateral-Control)/[Running Carla Simulator](#Running-Carla-Simulator)/[Simulation](#Simulation)

This respository includes files from the final project of the [Introduction to Self-Driving Cars](https://www.coursera.org/learn/intro-self-driving-cars) course by University of Toronto. This project involves implementation of vehicle controllers for self-driving cars in CARLA simulator in python 3.6.

In order to implement a fully functional controller module for autonomous car the following controls were implemented:
* Longitudinal Control
* Lateral Control

The table below shows the output varaibles of the controller module and their respective ranges. 

| Description |	Variable Name	| Limits
|---|---|---
Throttle |	throttle_output |	0 to 1 (in percentage)
Steering |	steer_output |	-1.22 to 1.22 (in radians, from left to right)
Brake	| brake_output |	0 to 1 (in percentage)

## Longitudinal Control

A Proportional-Integral-Derivative (PID) controller was used for longitudinal control. The controller takes in the difference between the desired and the actual vehicle speeds as inputs and outputs throttle and/or brake positions needed to minimize that difference. Since the requirement here is of a simple speed controller, a PID based control design would be suffice. This controller calculates the necessary throttle and brake positions needed to acheive desired speed.

The schematic diagram below is the illustration of the PID control loop used.

<p align="center"><img src="https://github.com/JagtapSagar/Vehicle-Control/blob/main/media/longitudinal_control.PNG"></p>

This design can be broken down into a high level and a low level controller. The formulation of the high level PID controller is as follows:

<p align="center"><img src="https://latex.codecogs.com/gif.latex?\bg_white&space;u(t)&space;=&space;K_pe(t)&space;&plus;&space;K_i\int&space;e(t)&space;\text{d}t&space;&plus;&space;K_d\frac{\text{d}e}{\text{d}t}" title="u(t) = K_pe(t) + K_i\int e(t) \text{d}t + K_d\frac{\text{d}e}{\text{d}t}"></p>

Where,
- u(t) - Controller effort (Acceleration output)
- e(t) - Error value
- K<sub>p</sub> - Proportional gain
- K<sub>i</sub> - Integral gain
- K<sub>d</sub> - Derivative gain 

A low level controller converts the acceleration output to throttle and brake commands. In this case, for simplicity positive acceleration will be throttle and negative outputs will correspond to brake.

## Lateral Control

A leteral controller is used by the vehicle to track the vehicles desired path. The two main goals of this controller are: </br>
* Heading path alignment
* Elimination of offset to path

In this project a Stanley Controller was implemented, which is a geometric control design ,i.e., it relies on the geometry and coordinates of the desired path and the kinematic model of the vehicle.

The kinematic model of the vehicle is based on the Bicycle kinematic model which is illustrated below.
<p align="center"><img src="https://github.com/JagtapSagar/Vehicle-Control/blob/main/media/kinematic_model_for_stanley_controller.PNG" width=280 height=250/></p>

The reference positions given here are waypoint positions, therefore heading and crosstrack errors need to be calculated.
- Crosstrack error (e) - Calculated as the perpendicular distance from the line connecting the previous and next waypoints and the front axle of the vehicle.
- Crosstrack steering (δ<sub>crosstrack</sub>) - Contribution of crosstrack error to the steerinf input.

  <p align="center"><img src="https://latex.codecogs.com/gif.latex?\bg_white&space;\delta(t)_{crosstrack}&space;=&space;\tan^{-1}\left&space;(\frac{ke}{v}&space;\right&space;)" title="\delta(t) = \tan^{-1}\left (\frac{ke}{v} \right )"></p>
  
  Where,
  - k - Gain (determined experimentally)
  - v - Current velocity

- Heading error (Ψ): Calculated as the difference between the trajectory line angle and the vehicle yaw.

Combining the above gives the output steering angle (δ) calculated as shown below.

<p align="center"><img src="https://latex.codecogs.com/gif.latex?\bg_white&space;\delta(t)&space;=&space;\psi&space;&plus;&space;\delta&space;_{crosstrack}" title="\delta(t) = \psi + \delta _{crosstrack}"></p>


Combining the longitudinal and lateral controller completes the vehicle control module.

Running Carla Simulator
---
To run this project:
1. Install Carla Simulator
2. Download this directory with all its files into a folder in the PythonClient directory within the Carla simulator directory.
3. Execute following command in the simulator directory to open the simulator.
   * For Ubuntu: `./CarlaUE4.sh /Game/Maps/RaceTrack -windowed -carla-server -benchmark -fps=30`
   * For Windows: `CarlaUE4.exe /Game/Maps/RaceTrack -windowed -carla-server -benchmark -fps=30`
4. Execute module_7.py from project directory.
   * For Ubuntu: `python3 module_7.py`
   * For Windows: `python module_7.py`

NOTE: 
* CARLA requires python version 3.5 or 3.6
* matplotlib version 3.0.0 was used for this project. Newer version might cause errors with the use of deprecated matplotlib.backends.tkagg import.



Simulation
---
The simulation begins with the self-driving car being placed at the start line of a preconfigured racetrack in the CARLA simulator. The controller is feed the list of way points and a velocity profile to follow. PID longitudinal controller outputs the throttle and brake commands to execute given velocity profile. And the Stanley lateral controller outputs steering angles to ensure the vehicle follows the desired path which consists of the given waypoint. 

<p align="center"><img src="https://github.com/JagtapSagar/Vehicle-Control/blob/main/media/simulation_and_control_feedback.gif"><p>

The reference and the actual trajectory profile of the simulation is also illustrated in the gif below.
<p align="center"><img src="https://github.com/JagtapSagar/Vehicle-Control/blob/main/media/vehicle_trajectory.gif"></p>

