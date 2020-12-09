# -*- coding: utf-8 -*-
"""
Created on Tue Oct 15 09:51:40 2019

@author: paschoeto
"""

import milestone_03 as ml
import numpy as np

Blist = np.array([[0,           0,       0,       0, 0],
                  [0,          -1,      -1,      -1, 0],
                  [1,           0,       0,       0, 1],
                  [0,     -0.5076, -0.3526, -0.2176, 0],
                  [0.033,       0,       0,       0, 0],
                  [    0,       0,       0,       0, 0]])

M0e = np.array([[1, 0, 0,  0.033],
                [0, 1, 0,    0  ],
                [0, 0, 1, 0.6546],
                [0, 0, 0,    1  ]])

Tb0 = np.array([[1, 0, 0, 0.1662],
                [0, 1, 0,      0],
                [0, 0, 1, 0.0026],
                [0, 0, 0,      1]])

#======================= TEST INPUTS

r_config = np.array([0,0,0,0,0,0.2,-1.6,0])

Xd = np.array([[0, 0, 1,   0.5],
               [0,  1, 0,    0],
               [-1, 0, 0,  0.5],
               [ 0, 0, 0,    1]])

Xd_next = np.array([[0, 0, 1,   0.6],
                    [0,  1, 0,    0],
                    [-1, 0, 0,  0.3],
                    [ 0, 0, 0,    1]])

X = np.array([[ 0.170,  0, 0.985,   0.387],
              [     0,  1,     0,       0],
              [-0.985,  0, 0.170,   0.570],
              [     0,  0,     0,       1]])

kp1 = np.zeros((6,6))
kp2 = np.eye(6)
ki = np.zeros((6,6))

dt = 0.01

def printdata(Vd, Ad, V, xerr, Je):
    print('* Vd\n')
    print(Vd)
    print('\n* Ad x Vd\n')
    print(np.matmul(Ad,Vd))
    print('\n* V\n')
    print(V)
    print('\n* Xerr\n' )
    print(xerr)
    print('\n* Je\n')
    print(np.round(Je, decimals=3))    

def matrices(x, xd, xd_next, kp, ki, dt, Blist, r_config):
    Ad = ml.calculateAd(x, xd)
    Vd = ml.calculateVd(xd, xd_next, dt)
    xerr = ml.calculateXerr(x,xd)
    
    V = Ad @ Vd + kp @ xerr + ki @ xerr*dt
    
    Je = ml.calculateJe(Blist, r_config, M0e, Tb0)
    
    printdata(Vd, Ad, V, xerr, Je)


matrices(X, Xd, Xd_next, kp2, ki, dt, Blist, r_config)

print('\n* u teta_p\n')
Xerr, udot = ml.FeedbackControl(X, Xd, Xd_next, kp1, ki, dt, Blist, r_config, M0e, Tb0)
print(udot)