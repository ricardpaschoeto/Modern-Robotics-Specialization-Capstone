"""
Created on Wed Apr 24 10:04:59 2019

@author: paschoeto
"""

import numpy as np
import modern_robotics as mr

def q_step(robot_config, speeds, dt, speed_limit):
    
    l = 0.235
    w = 0.15
    r = 0.0475
    
    H0 = (1/r)*np.array([[-l-w, 1, -1],
                         [ l+w, 1,  1],
                         [ l+w, 1, -1],
                         [-l-w, 1,  1]])
    
    H0_pinverse = np.linalg.pinv(H0)

    Tsb = np.array([[np.cos(robot_config[0]), -np.sin(robot_config[0]),    0,       robot_config[1]],
                    [np.sin(robot_config[0]),  np.cos(robot_config[0]),    0,       robot_config[2]],
                    [0,                                              0,    1,                0.0963],
                    [0,                                              0,    0,                     1]])
    
    # for ii,speed in enumerate(speeds):
    #     if speed > speed_limit:
    #         speeds[ii] = speed_limit
    #     elif speed < - speed_limit:
    #         speeds[ii] = -speed_limit
    np.clip(speeds, -speed_limit, speed_limit)
            
    Vb = H0_pinverse @ speeds[5:]*dt
    Vb6 = np.array([0, 0, Vb[0], Vb[1], Vb[2], 0])
    skew_matrix = mr.VecTose3(Vb6)
    T = mr.MatrixExp6(skew_matrix)
    Tsb = np.matmul(Tsb,T)
    
    if Vb[0] == 0.0:        
        deltaqb = np.array([0,Vb[1],Vb[2]])
    else:
        deltaqb = np.array([Vb[0], (Vb[1]*np.sin(Vb[0]) + Vb[2]*(np.cos(Vb[0])-1))/Vb[0],
                            (Vb[2]*np.sin(Vb[0]) + Vb[1]*(1-np.cos(Vb[0])))/Vb[0]])
    phi = np.arccos(Tsb[0][0])
    deltaq = np.array([[1,0,0],[0,np.cos(phi),-np.sin(phi)],[0,np.sin(phi),np.cos(phi)]]) @ deltaqb
    
    return deltaq

def NextState(robot_config, speeds, dt, speed_limit):
    
    deltaq = q_step(robot_config, speeds, dt, speed_limit)
    
    q0 = robot_config[0:3]
    J0 = robot_config[3:8]
    W0 = robot_config[-4:]
    
    q = q0 + deltaq
    q0 = q
    J = J0 + speeds[:5]*dt
    J0 = J
    W = W0 + speeds[5:]*dt
    W0 = W
    line = np.round([q[0],q[1],q[2],J[0],J[1],J[2],J[3],J[4],W[0],W[1],W[2],W[3]],decimals=3)
    
    return line

      
        

    
    
     
    

