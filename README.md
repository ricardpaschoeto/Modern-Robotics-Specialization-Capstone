# Modern-Robotics-Specialization-Capstone

This project is a capstone project from Coursera Specialization Robotics Course. The aim of this project is design a python code for control a robot arm to get and move a cube.

## Motivation

My first point to enroll in this Specialization is to get more confidence in robot control and design. This project mixed everything that I learned in this course and it became very interesting and challenge.

## Buid Status

The project is finalized, you can see the results inside the "results" folder.

## Python version

- Python 3.7
- modern_robotics python library
- Numpy library
- IDE: VSCODE

## Approach

To solve the problem I created a Class called "youBot" where each input from the main functions described in the project instructions became a member variable of the class and the constructor received the parameters defined in the beginning of the project instructons. I tried to build the project exactly that was describe in the instructions and for each Function was create a test to facilitate the debug process. I implemented the sigularity avoidance using the python function "np.linalg.pinv(m, 1e-4)". The joint limit avoidance was created but not tested and in this way I prefer to keep this part of code commented. My results were good and show up like expected.
