# Vehicle Control

This respository includes files from the final project of the [Introduction to Self-Driving Cars](https://www.coursera.org/learn/intro-self-driving-cars) course by University of Toronto. This project involves implementation of vehicle controllers for self-driving cars in CARLA simulator in python 3.6.

In order to implement a fully functional controller module for autonomous car the following controls were implemented:
* Longitudinal Control
* Lateral Control

## Longitudinal Control
u(t) = K<sub>p</sub>e(t) + K<sub>i</sub>\int e(t) \text{d}t + K_d\frac{\text{d}e}{\text{d}t}

<a href="https://www.codecogs.com/eqnedit.php?latex=u(t)&space;=&space;K_pe(t)&space;&plus;&space;K_i\int&space;e(t)&space;\text{d}t&space;&plus;&space;K_d\frac{\text{d}e}{\text{d}t}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?u(t)&space;=&space;K_pe(t)&space;&plus;&space;K_i\int&space;e(t)&space;\text{d}t&space;&plus;&space;K_d\frac{\text{d}e}{\text{d}t}" title="u(t) = K_pe(t) + K_i\int e(t) \text{d}t + K_d\frac{\text{d}e}{\text{d}t}" /></a>

h<sub>&theta;</sub>(x) = &theta;<sub>o</sub> x + &theta;<sub>1</sub>x

## Lateral Control

