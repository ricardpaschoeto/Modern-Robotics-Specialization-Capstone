"""
Created on Wed Apr 24 10:04:59 2019

@author: paschoeto
"""

import numpy as np
import modern_robotics as mr
import csv
import os

def endEffectorFrame(Tse_initial, Blist, thetalist):
    
    T = mr.FKinBody(Tse_initial, Blist, thetalist)
    
    return T
    

def write_result(results):
    if os.path.isfile('endeffector.csv'):
        os.remove('endeffector.csv')
    with open('endeffector.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, quoting=csv.QUOTE_MINIMAL)
        for result in results:
            writer.writerow(result)

def vector(matrix_set, line, gripper_state):
    
    ixgrot = np.ix_([0,1,2], [0,1,2])
    ixtrans = np.ix_([0,1,2], [3])

    for ii, matrix in enumerate(matrix_set):
        linerot = np.reshape(matrix[ixgrot], (1,9))[0]
        linetrans = np.reshape(matrix[ixtrans], (1,3))[0]
        line.append([linerot[0],linerot[1],linerot[2],linerot[3],linerot[4],linerot[5],linerot[6],linerot[7],linerot[8], linetrans[0], linetrans[1], linetrans[2], gripper_state])
    

def TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_goal, Tce_grasp, Tce_standoff, k = 1):

    teta = 3*np.pi/4
    R = np.array([[ np.cos(teta), 0, np.sin(teta)],
                  [            0, 1,            0],
                  [-np.sin(teta), 0, np.cos(teta)]])
    
    N = (k/0.01) + 1;
    Tf = 10
    Tf_open_close = 1
    line = []
    
    traj_1 = mr.ScrewTrajectory(Tse_initial, Tce_standoff, Tf, N, 3)
    vector(traj_1, line, 0)
    
    traj_2 = mr.ScrewTrajectory(Tce_standoff, Tce_grasp, Tf, N, 3)
    vector(traj_2, line, 0)

    Traj_3 = mr.ScrewTrajectory(traj_2[-1], traj_2[-1], Tf_open_close, N, 3)
    vector(Traj_3, line, 1)
    
    traj_4 = mr.ScrewTrajectory(Traj_3[-1], traj_1[-1], Tf, N, 3)
    vector(traj_4, line, 1)
    
    p1 = np.array([0, 0, 0.2])
    T1 = mr.RpToTrans(R,p1)
    traj_5 = mr.ScrewTrajectory(traj_4[-1], Tsc_goal @ T1, Tf, N, 3)
    vector(traj_5, line, 1)
    
    p2 = np.array([0, 0, 0])
    T2 = mr.RpToTrans(R,p2)
    traj_6 = mr.ScrewTrajectory(traj_5[-1], Tsc_goal @ T2, Tf, N, 3)
    vector(traj_6, line, 1)
    
    traj_7 = mr.ScrewTrajectory(traj_6[-1], traj_6[-1], Tf_open_close, N, 3)
    vector(traj_7, line, 0)

    traj_8 = mr.ScrewTrajectory(traj_7[-1], traj_5[-1], Tf, N, 3)
    vector(traj_8, line, 0)
    
    write_result(line)

   
   

