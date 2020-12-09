# -*- coding: utf-8 -*-
"""
Created on Mon Oct  7 19:07:27 2019

@author: Home
"""
import numpy as np
import modern_robotics as mr
import csv
import os 

import milestone_01 as ml01
import milestone_03 as ml03

M0e = np.array([[1, 0, 0,  0.033],
                [0, 1, 0,    0  ],
                [0, 0, 1, 0.6546],
                [0, 0, 0,    1  ]])

Tb0 = np.array([[1, 0, 0, 0.1662],
                [0, 1, 0,      0],
                [0, 0, 1, 0.0026],
                [0, 0, 0,      1]])

Blist = np.array([[0,           0,       0,       0, 0],
                  [0,          -1,      -1,      -1, 0],
                  [1,           0,       0,       0, 1],
                  [0,     -0.5076, -0.3526, -0.2176, 0],
                  [0.033,       0,       0,       0, 0],
                  [    0,       0,       0,       0, 0]])

Tsc_initial = np.array([[1, 0, 0,   1  ],
                        [0, 1, 0,    0  ],
                        [0, 0, 1,  0.025],
                        [0, 0, 0,    1  ]])

Tsc_goal = np.array([[0,  1, 0,    0  ],
                      [-1, 0, 0,  -1  ],
                      [0,  0, 1, 0.025],
                      [0,  0, 0,   1  ]])

#================
X = np.array([[0,  0,  1,     0],
              [0,  1,  0,     0],
              [-1, 0,  0,   0.5],
              [0,  0,  0,   1  ]])

r_config = np.array([0,0,0,0,0,0.2,-1.6,0,0,0,0,0])

kp1 = np.zeros((6,6))
kp2 = np.eye(6)
ki = np.zeros((6,6))

dt = 0.01

def mountX(row):
    
    Tse = np.array([[row[0], row[1], row[2], row[9]],
                  [row[3], row[4], row[5],  row[10]],
                  [row[6], row[7], row[8],  row[11]],
                  [     0,      0,      0,       1]])
    
    return Tse

def write_result(results, filename, gripper_state):
    with open(filename, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile, quoting=csv.QUOTE_MINIMAL)
        if(type(results) == list):
            for result in results:
                arr = np.append(result, gripper_state)
                writer.writerow(arr)
        else:
            arr = np.append(results, gripper_state)
            writer.writerow(arr)
        
def youBot(filename, fnameerror, X, Blist, r_config, M0e, Tb0, kp, ki, dt):
    
    if os.path.isfile(filename):
        os.remove(filename)
    if os.path.isfile(fnameerror):
        os.remove(fnameerror)
    with open('endeffector.csv') as trajectories:
        lines = list(csv.reader(trajectories, delimiter=','))
        for ii in range(0, len(lines) - 1):
            row = [float(st) for st in lines[ii][0:12]]
            Xd = mountX(row)
            row_next = [float(st) for st in lines[ii+1][0:12]]
            Xd_next = mountX(row_next)
            
            Xerr, u_tetadot = ml03.FeedbackControl(X, Xd, Xd_next, kp, ki, dt, Blist, r_config, M0e, Tb0)

            result = ml01.NextState(r_config, u_tetadot, dt, 10)
            r_config = np.asarray(result)
            X = mr.FKinBody(M0e, Blist, r_config[3:8])
            write_result(result, filename, lines[ii][-1])
            write_result(Xerr, fnameerror, 0)
            
youBot('body.csv', 'error.csv', X, Blist, r_config, M0e, Tb0, kp1, ki, dt)

#print(np.round(mr.FKinBody(M0e, Blist, r_config[3:8]),decimals=3))



