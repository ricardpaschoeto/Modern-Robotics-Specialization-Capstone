# -*- coding: utf-8 -*-
"""
Created on Tue Oct 15 08:33:38 2019

@author: paschoeto
"""

import milestone_01 as ml
import numpy as np
import os
import csv

# chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4
robot_config01 = np.array([-0.75959, -0.47352, 0.058167, 0.80405, -0.91639, -0.011436, 0.054333, 0.00535, 1.506, -1.3338, 1.5582, 1.6136])
robot_config02 = np.zeros((12,))
#
## joint arms velocity and wheels velocitity
velocity1 = np.array([0, 0, 0, 0, 0, 10,10,10,10])
velocity2 = np.array([0, 0, 0, 0, 0, -10,10,-10,10])
velocity3 = np.array([0, 0, 0, 0, 0, -10,10,10,-10])

def write_result(results, filename):

    with open(filename, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile, quoting=csv.QUOTE_MINIMAL)
        if(type(results) == list):
            for result in results:
                arr = np.append(result)
                writer.writerow(arr)
        else:
            writer.writerow(results)
            
def test(robot_config, velocity, dt, filename, gripper_state):
    
    results = ml.NextState(robot_config, velocity, dt, 10)
    results = np.append(results, gripper_state)    
    write_result(results,filename)

if os.path.isfile('body.csv'):
    os.remove('body.csv')
        
for ii in range(0, 101):
    test(robot_config02, velocity3, ii*0.01, 'body.csv', int(0))