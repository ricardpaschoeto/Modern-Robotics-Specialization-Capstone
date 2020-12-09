# -*- coding: utf-8 -*-
"""
Created on Tue Oct 15 09:27:16 2019

@author: paschoeto
"""

import milestone_02 as ml
import numpy as np
import modern_robotics as mr

M0e = np.array([[1, 0, 0,  0.033],
                [0, 1, 0,    0  ],
                [0, 0, 1, 0.6546],
                [0, 0, 0,    1  ]])

Blist = np.array([[0,           0,       0,       0, 0],
                  [0,          -1,      -1,      -1, 0],
                  [1,           0,       0,       0, 1],
                  [0,     -0.5076, -0.3526, -0.2176, 0],
                  [0.033,       0,       0,       0, 0],
                  [    0,       0,       0,       0, 0]])

Tsc_initial = np.array([[1, 0, 0,    1  ],
                        [0, 1, 0,    0  ],
                        [0, 0, 1,  0.025],
                        [0, 0, 0,    1  ]])

Tsc_goal = np.array([[0,  1, 0,    0  ],
                      [-1, 0, 0,  -1  ],
                      [0,  0, 1, 0.025],
                      [0,  0, 0,   1  ]])

Tse_initial = np.array([[0,  0,  1,     0],
                        [0,  1,  0,     0],
                        [-1, 0,  0,   0.5],
                        [0,  0,  0,   1  ]])

#================

teta = 3*np.pi/4


R = np.array([[ np.cos(teta), 0, np.sin(teta)],
               [            0, 1,            0],
               [-np.sin(teta), 0, np.cos(teta)]])

p1 = np.array([0, 0, 0.2])
T1 = mr.RpToTrans(R,p1)

Tce_standoff = Tsc_initial @ T1

p2 = np.array([0, 0, 0])
T2 = mr.RpToTrans(R,p2)

Tce_grasp = Tsc_initial @ T2

ml.TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_goal, Tce_grasp, Tce_standoff)