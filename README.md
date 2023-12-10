# Autonomous Foosball With a "Twist"
A project for CMPUT 312 - Introduction to Robotics and Mechatronics at the University of Alberta

Created by Cole Dewis, Joey Quinlan, Arshia Kamal

![gif of our robot](img/robotgif.gif)

## Overview

This project is an autonomous system where two robots play a foosball/pong like game against each other, attempting to either score on each other, or pass for as long as possible. 

This project makes use of forward and inverse kinematics, thread programming, computer vision, trajectory estimation, and more.

We use two LEGO Mindstorms EV3 Bricks and one Arduino UNO board for hardware. These interact with LEGO motors and NEMA 17 Stepper motors to move the joints of our robot.

For a more in depth explanation of the project, you can read our [paper!]() TODO: add link

## Individual Contributions
Cole: Robot kinematics (`\kinematics`), embedded communications (`\arduino_client`, `arduino_server.py`, `\brick`, `brick_server.py`, `\messages`), state machine and robot logic (`robot.py`). 

Joey: Computer Vision, Ball Tracking. (`\vision`)

Arshia: Ball Trajectory Analysis. (`\trajectory`)

## Demo Video
A video demo of our project can be seen [here.](https://drive.google.com/file/d/1JVUitq1qnXPh7LTEeeYDJyjFxKlvqjnj/view?usp=sharing)

## Gallery

TODO: add images to repo