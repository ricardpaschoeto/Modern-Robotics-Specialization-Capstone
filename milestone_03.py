"""
Created on Wed Oct 02 14:06:59 2019

@author: paschoeto
"""
import numpy as np
import modern_robotics as mr

def calculateXerr(x, xd):
    
    x_inv = np.linalg.inv(x)
    xerr = x_inv @ xd
    
    se3mat = mr.MatrixLog6(xerr)
    
    Xerr = mr.se3ToVec(se3mat) 
    
    return np.round(Xerr, decimals=3)

def calculateVd(xd, xd_next, dt):
    
    xd_inv = np.linalg.inv(xd)
    Vd = xd_inv @ xd_next
    
    se3mat = (1/dt)*mr.MatrixLog6(Vd)
    
    Vd = mr.se3ToVec(se3mat)

    return Vd

def calculateAd(x, xd):
    
    x_inv = np.linalg.inv(x)
    Adx_inv = mr.Adjoint(x_inv)
    Adxd = mr.Adjoint(xd)
    
    Ad = Adx_inv @ Adxd
    
    return Ad

def calculateJbase(r, l, w, M0e, Tb0, blist, r_config):
    
    thetalist = r_config[3:8]
    H0 = (1/r)*np.array([[-l-w, 1, -1],
                         [ l+w, 1,  1],
                         [ l+w, 1, -1],
                         [-l-w, 1,  1]])
    
    F = np.linalg.pinv(H0)
    
  
    F6 = np.array([[     0,       0,      0,      0],
                   [     0,       0,      0,      0],
                   [F[0,0],  F[0,1], F[0,2], F[0,3]],
                   [F[1,0],  F[1,1], F[1,2], F[1,3]],
                   [F[2,0],  F[2,1], F[2,2], F[2,3]],
                   [     0,       0,      0,     0]])
    
    T0e = mr.FKinBody(M0e, blist, thetalist)
    Ad1 = mr.Adjoint(np.linalg.inv(T0e))
    Ad2 = mr.Adjoint(np.linalg.inv(Tb0))
    
    Ad = Ad1 @ Ad2
    
    Jbase = Ad @ F6
    
    return Jbase

def calculateJe(blist, r_config, M0e, Tb0):
    
    thetalist = r_config[3:8]
    Jarm = mr.JacobianBody(blist, thetalist)
    Jbase = calculateJbase(0.0475, 0.235, 0.15,M0e, Tb0, blist, r_config)
    
    Je = np.c_[Jbase, Jarm]    
   
    return Je

def FeedbackControl(x, xd, xd_next, kp, ki, dt, Blist, r_config, M0e, Tb0):
    
    Ad = calculateAd(x, xd)
    Vd = calculateVd(xd, xd_next, dt)
    xerr = calculateXerr(x,xd)
    
    V = Ad @ Vd + kp @ xerr + ki @ (xerr*dt)
    
    Je = calculateJe(Blist, r_config, M0e, Tb0)
   
    Je_pinv = np.linalg.pinv(Je)
    ut = Je_pinv @ V
    
    u_tetadot = np.array([ut[0], ut[1], ut[2], ut[3], ut[4], ut[5], ut[6], ut[7], ut[8]])
    
    return xerr, np.round(u_tetadot, decimals=1)
    
   
 



    

