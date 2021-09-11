# -*- coding: utf-8 -*-
"""
Created on Wed Dec  9 15:15:26 2020

@author: paschoeto
"""

from modern_robotics.core import TransToRp
import numpy as np
import modern_robotics as mr
import csv
import os

from numpy.core.numeric import indices
import matplotlib.pyplot as plt

class youBot:
    
    def __init__(self, init_cube_config=[1.,0.,0.025, 0], desired_cube_config=[0., -1., 0.025, -np.pi/2], initial_youBot_conf=[], ref_init_youBot=[]):
        
        ## youBot chassi
        self.l = 0.235 # distance between the wheels
        self.w = 0.15 # side-to-side distance between wheels
        self.r = 0.0475 # The radius of each wheel
        self.z = 0.0963 # the height of the {b} frame above the floor
        self.integral = 0.
        self.robot_config = initial_youBot_conf
        self.desired_cube_config = desired_cube_config
        self.init_cube_config = init_cube_config
         
        self.robot_configs = []
        self.errors = []
        # The fixed offset from the chassis frame {b} to the base frame of the arm {0}
        self.Tb0 = np.array([[1, 0, 0, 0.1662],
                             [0, 1, 0, 0     ],
                             [0, 0, 1, 0.0026],
                             [0, 0, 0, 1     ]])
        # When the arm is at its home configuration (all joint angles zero, as shown in the figure), the end-effector frame {e}
        # relative to the arm base frame {0}
        self.M0e = np.array([[1, 0, 0, 0.033 ],
                             [0, 1, 0, 0     ],
                             [0, 0, 1, 0.6546],
                             [0, 0, 0, 1     ]])
    
      
        # When the arm is at its home configuration, the screw axes
        # for the five joints are expressed in the end-effector frame {e}
        self.B1 = [0,0,1,0,0.033,0]
        self.B2 = [0,-1,0,-0.5076,0,0]
        self.B3 = [0,-1,0,-0.3526,0,0]
        self.B4 = [0,-1,0,-0.2176,0,0] 
        self.B5 = [0,0,1,0,0,0]
        self.blist = np.array([self.B1,self.B2,self.B3,self.B4,self.B5]).T
        
        ## youBot GRIPPER
        self.d1min = 2 # The minimum opening distance of the gripper
        self.d1max = 7 # the maximum opening distance
        self.d2 = 3.5 # the interior length of the fingers
        self.d3 = 4.3 # the distance from the base of the fingers to the frame {e}
        
        ## Cube
       
        # These are written in SE(3) matrix
        self.Tsc_initial = np.array([[1,  0,    0,  init_cube_config[0]],
                                     [0,  1,    0,  init_cube_config[1]],
                                     [0,  0,    1,  init_cube_config[2]],
                                     [0,  0,    0,  1                  ]])
        
        self.Tsc_goal = np.array([[0,  1,    0,  desired_cube_config[0]],
                                  [-1, 0,    0,  desired_cube_config[1]],
                                  [0,  0,    1,  desired_cube_config[2]],
                                  [0,  0,    0,  1                   ]])
        


    def write_result(self, results, filename):
        with open(filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile, quoting=csv.QUOTE_MINIMAL)
            writer.writerow(results)

    def write_control(self, results, filename):
        if os.path.isfile(filename):
            os.remove(filename)
        with open(filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile, quoting=csv.QUOTE_MINIMAL)
            for result in results:
                writer.writerow(result)
                
    def write_result_end_effector(self, results):
        if os.path.isfile('endeffector.csv'):
            os.remove('endeffector.csv')
        with open('endeffector.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile, quoting=csv.QUOTE_MINIMAL)
            for result in results:
                writer.writerow(result)
                
    ##
    ## BODY MOVEMENT
    ##                
    def q_step(self, robot_config, speeds, dt):

        # Tsb = np.array([[np.cos(robot_config[0]), -np.sin(robot_config[0]),    0,  robot_config[1]],
        #                 [np.sin(robot_config[0]),  np.cos(robot_config[0]),    0,  robot_config[2]],
        #                 [                      0,                        0,    1,           self.z],
        #                 [                      0,                        0,    0,                1]])

        Vb = (self.r/4)*np.array([[-1/(self.l+self.w), 1/(self.l+self.w), 1/(self.l+self.w), -1/(self.l+self.w)],
                                  [                 1,                 1,                 1,                  1],
                                  [                -1,                 1,                -1,                  1]]) @ (speeds[5:]*dt)

        # Vb6 = np.array([0, 0, Vb[0], Vb[1], Vb[2], 0])
        # se_matrix = mr.VecTose3(Vb6) 
        # Tbb = mr.MatrixExp6(se_matrix)
        # Tsb_adv = Tsb @ Tbb

        if Vb[0] == 0.0:        
            deltaqb = np.array([0,Vb[1],Vb[2]])
        else:
            deltaqb = np.array([Vb[0], (Vb[1]*np.sin(Vb[0]) + Vb[2]*(np.cos(Vb[0])-1))/Vb[0],
                                (Vb[2]*np.sin(Vb[0]) + Vb[1]*(1-np.cos(Vb[0])))/Vb[0]])

        phi = robot_config[0]
        deltaq = np.array([[1,0,0],[0,np.cos(phi),-np.sin(phi)],[0,np.sin(phi),np.cos(phi)]]) @ deltaqb

        return deltaq
    
    def NextState(self, robot_config, speeds, dt, speed_limit, gripper=0):

        speeds = np.clip(speeds, speed_limit[0], speed_limit[1])
        deltaq = self.q_step(robot_config, speeds, dt)
        
        q = robot_config[0:3]
        J = robot_config[3:8]
        W = robot_config[8:12]
        
        q = q + deltaq
        J = J + speeds[:5]*dt
        W = W + speeds[5:]*dt
        
        new_robot_config = np.concatenate((q,J,W, np.array([gripper])))
        self.write_result(new_robot_config, 'body.csv')

        return list(new_robot_config)

    ##
    ## END EFFECTOR MOVEMENT
    ##

    def vector(self, matrix_set, line, gripper_state):
        
        for Tse in matrix_set:
            rot = np.reshape(Tse[0:3, 0:3], (9,))  
            trans = np.reshape(Tse[0:3,-1], (3,))
            arr = np.hstack((rot,trans, [gripper_state]))
            line.append(arr)

    def TrajectoryGenerator(self, dt, k = 1):
    
        N = k/dt
        Tf = 10
        Tf_open_close = 0.625
        line = []
        teta = 0

        Tse_initial = mr.FKinBody(self.M0e, self.blist, self.robot_config[3:8])
        # Tse_initial = np.array([[ 0,  0,  1,    0],
        #                         [ 0,  1,  0,    0],
        #                         [-1, 0,   0,  0.5],
        #                         [ 0,  0,  0,    1]])
        Rtse,_ = mr.TransToRp(Tse_initial)
        pstd = np.array([self.init_cube_config[0], self.init_cube_config[1], self.init_cube_config[2] + 0.2])

        Rstd = np.array([[np.cos(teta), 0,  np.sin(teta)],
                        [            0, 1,             0],
                        [-np.sin(teta), 0,  np.cos(teta)]])
                      
        Rstd_final = Rstd @ Rtse
        Tce_standoff = mr.RpToTrans(Rstd_final,pstd)

        Tce_grasp = np.copy(Tce_standoff)
        Tce_grasp[2][3] = self.init_cube_config[2] - 0.07

        Rstd2 = np.array([[ np.cos(self.desired_cube_config[3]), -np.sin(self.desired_cube_config[3]), 0],
                          [ np.sin(self.desired_cube_config[3]),  np.cos(self.desired_cube_config[3]), 0],
                          [                                   0,                                    0, 1]])

        p = np.array([self.desired_cube_config[0], self.desired_cube_config[1], self.desired_cube_config[2] + 0.2])
        R = Rstd2 @ Rstd_final
        Tce_standoff2 = mr.RpToTrans(R, p)

        Tce_grasp2 = np.copy(Tce_standoff2)
        Tce_grasp2[2][3] = self.desired_cube_config[2] - 0.07

        # A trajectory to move the gripper from its initial configuration to a "standoff"
        # configuration a few cm above the block.
        traj_1 = mr.ScrewTrajectory(Tse_initial, Tce_standoff, Tf, 2*N, 3)
        self.vector(traj_1, line, 0)
        
        # A trajectory to move the gripper down to the grasp position.
        traj_2 = mr.ScrewTrajectory(Tce_standoff, Tce_grasp, Tf, 2*N, 3)
        self.vector(traj_2, line, 0)
        
        # Closing of the gripper.
        Traj_3 = mr.ScrewTrajectory(Tce_grasp, Tce_grasp, Tf_open_close, 0.6*N, 3)
        self.vector(Traj_3, line, 1)
        
        # A trajectory to move the gripper back up to the "standoff" configuration.
        traj_4 = mr.ScrewTrajectory(Tce_grasp, Tce_standoff, Tf, 2*N, 3)
        self.vector(traj_4, line, 1)
        
        # A trajectory to move the gripper to a "standoff" configuration above the final configuration.
        traj_5 = mr.ScrewTrajectory(Tce_standoff, Tce_standoff2, Tf, 8*N, 3)
        self.vector(traj_5, line, 1)
        
        # A trajectory to move the gripper to the final configuration of the object.
        traj_6 = mr.ScrewTrajectory(Tce_standoff2, Tce_grasp2, Tf, 2*N, 3)
        self.vector(traj_6, line, 1)
        
        # Opening of the gripper.
        traj_7 = mr.ScrewTrajectory(Tce_grasp2, Tce_grasp2, Tf_open_close, 0.6*N, 3)
        self.vector(traj_7, line, 0)
        
        # A trajectory to move the gripper back to the "standoff" configuration.
        traj_8 = mr.ScrewTrajectory(Tce_grasp2, Tce_standoff2, Tf, 2*N, 3)
        self.vector(traj_8, line, 0)
        
        self.write_result_end_effector(line)

        return line
    
    ##
    ## CONTROL ALGORITHM
    ##                    
    def calculateXerr(self,x, xd):
        
        x_inv = np.linalg.inv(x)
        xerr = x_inv @ xd
        
        se3mat = mr.MatrixLog6(xerr)
        
        Xerr = mr.se3ToVec(se3mat) 
        
        return Xerr
    
    def calculateVd(self,xd, xd_next, dt):
        
        xd_inv = np.linalg.inv(xd)
        Vd = xd_inv @ xd_next
        
        se3mat = (1/dt)*mr.MatrixLog6(Vd)
        
        Vd = mr.se3ToVec(se3mat)
    
        return Vd
    
    def calculateAd(self,x, xd):
        
        x_inv = np.linalg.inv(x)
        Adx_inv = mr.Adjoint(x_inv)
        Adxd = mr.Adjoint(xd)
        
        Ad = Adx_inv @ Adxd
        
        return Ad
    
    def calculateJbase(self, robot_config):
        
        thetalist = robot_config[3:8]
        H0 = (1/self.r)*np.array([[-self.l-self.w, 1, -1],
                             [ self.l+self.w, 1,  1],
                             [ self.l+self.w, 1, -1],
                             [-self.l-self.w, 1,  1]])
        
        F = np.linalg.pinv(H0, 1e-4)
        
      
        F6 = np.array([[     0,       0,      0,      0],
                       [     0,       0,      0,      0],
                       [F[0,0],  F[0,1], F[0,2], F[0,3]],
                       [F[1,0],  F[1,1], F[1,2], F[1,3]],
                       [F[2,0],  F[2,1], F[2,2], F[2,3]],
                       [     0,       0,      0,     0]])
        
        T0e = mr.FKinBody(self.M0e, self.blist, thetalist)
        Ad1 = mr.Adjoint(np.linalg.inv(T0e))
        Ad2 = mr.Adjoint(np.linalg.inv(self.Tb0))
        
        Ad = Ad1 @ Ad2
        
        Jbase = Ad @ F6
        
        return Jbase
    
    def calculateJe(self, robot_config):
        
        thetalist = robot_config[3:8]
        Jarm = mr.JacobianBody(self.blist, thetalist)
        Jbase = self.calculateJbase(robot_config)
        
        Je = np.hstack((Jarm, Jbase))
       
        return Je
    
    def FeedbackControl(self,robot_config, x, xd, xd_next, kp, ki, dt):
        
        Ad = self.calculateAd(x, xd)
        Vd = self.calculateVd(xd, xd_next, dt)
        Adxxd = Ad @ Vd
        xerr = self.calculateXerr(x,xd)
        
        #self.integral += (xerr*dt)
        V = Adxxd + kp @ xerr + ki @ (xerr*dt)
        
        Je = self.calculateJe(robot_config)
       
        Je_pinv = np.linalg.pinv(Je, 1e-4)
        ut = Je_pinv @ V

        # Je, violeted = self.testJoinLimits(Je, ut, dt)
        # if violeted:
        #     Je_pinv = np.linalg.pinv(Je, 1e-4)
        #     ut = np.round(Je_pinv @ V, decimals=1)            
        
        return xerr, V, ut
    
    def testJoinLimits(self, Je, ut, dt):
        tetha = ut[:5]*dt
        violeted = False
        if tetha[2] > -0.2:
            Je[:,2] = np.zeros((6,))
            violeted = True
        if tetha[3] > -0.2:
            Je[:,3] = np.zeros((6,))
            violeted = True
        
        return Je, violeted

    def feedforward(self, Kp, Ki, dt):
        if os.path.isfile('body.csv'):
            os.remove('body.csv')
        trajectory = self.TrajectoryGenerator(dt)
        new_robot_config = self.robot_config
        for ii in range(len(trajectory)-1):
            Tsb = np.array([[np.cos(new_robot_config[0]), -np.sin(new_robot_config[0]),    0,  new_robot_config[1]],
                            [np.sin(new_robot_config[0]),  np.cos(new_robot_config[0]),    0,  new_robot_config[2]],
                            [                      0,                        0,    1,           self.z],
                            [                      0,                        0,    0,                1]])
            X = Tsb @ self.Tb0 @ mr.FKinBody(self.M0e, self.blist, new_robot_config[3:8])

            Xd = np.array([[trajectory[ii][0], trajectory[ii][1], trajectory[ii][2], trajectory[ii][9]],
                           [trajectory[ii][3], trajectory[ii][4], trajectory[ii][5], trajectory[ii][10]],
                           [trajectory[ii][6], trajectory[ii][7], trajectory[ii][8], trajectory[ii][11]],
                           [0,0,0,1]])

            Xd_next = np.array([[trajectory[ii+1][0], trajectory[ii+1][1], trajectory[ii+1][2], trajectory[ii+1][9]],
                           [trajectory[ii+1][3], trajectory[ii+1][4], trajectory[ii+1][5], trajectory[ii+1][10]],
                           [trajectory[ii+1][6], trajectory[ii+1][7], trajectory[ii+1][8], trajectory[ii+1][11]],
                           [0,0,0,1]])

            gripper = trajectory[ii][-1]

            Xerr, V, ut = self.FeedbackControl(new_robot_config, X,Xd,Xd_next,Kp,Ki,dt)
            new_robot_config = self.NextState(new_robot_config, ut, dt=dt, gripper=gripper, speed_limit=[-5.,5.])

            self.robot_configs.append(new_robot_config)
            self.errors.append(Xerr)

        self.write_control(self.robot_configs, 'control.csv')

def test_next_state():
    if os.path.isfile('body.csv'):
        os.remove('body.csv')
    # chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4
    robot_config01 = np.array([-0.75959, -0.47352, 0.058167, 0.80405, -0.91639, -0.011436, 0.054333, 0.00535, -np.pi/4, np.pi/4, -np.pi/4, np.pi/4])
    robot_config02 = np.zeros((12,))
    ## joint arms velocity and wheels velocitity
    velocity1 = np.array([0, 0, 0, 0, 0, 10,10,10,10])
    velocity2 = np.array([0, 0, 0, 0, 0, -10,10,-10,10])
    velocity3 = np.array([0, 0, 0, 0, 0, -10,10,10,-10])
    robot = youBot(initial_youBot_conf=robot_config02)
    sim = 100
    for _ in range(sim):
        robot_config02 = robot.NextState(robot_config= robot_config02, speeds=velocity3, speed_limit=[-15.,15.], dt=0.01)

def test_trajectory_generator():
    # chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4
    robot_config01 = np.array([0, 0, 0, 0, 0, 0, 0, 0, -np.pi/4, np.pi/4, -np.pi/4, np.pi/4])
    robot_config02 = np.zeros((12,))

    robot = youBot(initial_youBot_conf=robot_config01)

    robot.TrajectoryGenerator()

def test_feedbackcontrol():
    robot_config = np.array([0,0,0,0,0,0.2,-1.6,0, -np.pi/4, np.pi/4, -np.pi/4, np.pi/4])
    #robot_config = np.zeros((12,))

    robot = youBot(initial_youBot_conf=robot_config)

    Xd = np.array([[0, 0, 1,   0.5],
                [0,  1, 0,    0],
                [-1, 0, 0,  0.5],
                [ 0, 0, 0,    1]])

    Xd_next = np.array([[0, 0, 1,   0.6],
                        [0,  1, 0,    0],
                        [-1, 0, 0,  0.3],
                        [ 0, 0, 0,    1]])

    Tb0 = np.array([[1, 0, 0, 0.1662],
                    [0, 1, 0, 0     ],
                    [0, 0, 1, 0.0026],
                    [0, 0, 0, 1     ]])

    Tsb = np.array([[np.cos(robot_config[0]), -np.sin(robot_config[0]),    0,  robot_config[1]],
                    [np.sin(robot_config[0]),  np.cos(robot_config[0]),    0,  robot_config[2]],
                    [                      0,                        0,    1,           0.0963],
                    [                      0,                        0,    0,                1]])

    X = Tsb @ Tb0 @ mr.FKinBody(robot.M0e, robot.blist,robot_config[3:8])
    kp1 = np.zeros((6,6))
    kp2 = np.eye(6)
    ki = np.zeros((6,6))

    dt = 0.01
    Ad = robot.calculateAd(X, Xd)
    Vd = robot.calculateVd(Xd, Xd_next, dt)
    xerr = robot.calculateXerr(X,Xd)
    
    V = Ad @ Vd + kp1 @ xerr + ki @ xerr*dt
    
    Je = robot.calculateJe(robot_config)
    
    printdata(Vd, Ad, V, xerr, Je)

    print('\n* u teta_p\n')
    xerr, V, ut = robot.FeedbackControl(robot_config, X, Xd, Xd_next, kp1, ki, dt)
    print(ut)

def test_feedforward(dt):
    robot_config01 = np.array([0,0,0,0,0,0,-np.pi/2,0, -np.pi/4, np.pi/4, -np.pi/4, np.pi/4])
    robot_config02 = np.zeros((12,))

    robot = youBot(initial_youBot_conf=robot_config01)  
    kp1 = np.zeros((6,6))
    kp2 = 0.01*np.eye(6)
    ki = np.zeros((6,6))
    ki2 = 0.0025*np.eye(6)

    robot.feedforward(kp1,ki, dt)

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

def print_Xerr (Xerror):

    v_errors = np.asarray(Xerror)
    plt.plot(range(len(v_errors[:,0])), v_errors)
    plt.show()

#test_feedforward(dt=0.01)
init_cube_config = [1.0, 0.0, 0.0]
desired_cube_config=[0., -1., 0.025, -np.pi/2] 
initial_youBot_conf=np.array([0,0,0,0,0,0,-np.pi/2,0, -np.pi/4, np.pi/4, -np.pi/4, np.pi/4])

kp1 = np.zeros((6,6))
kp2 = 1.0*np.eye(6)
ki = np.zeros((6,6))
ki2 = 0.0025*np.eye(6)

robot = youBot(init_cube_config=init_cube_config, desired_cube_config=desired_cube_config,initial_youBot_conf=initial_youBot_conf)
robot.feedforward(kp1, ki, 0.01)
print_Xerr(robot.errors)