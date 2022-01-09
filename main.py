# -*- coding: utf-8 -*-


import numpy as np
import pinocchio as pin
from time import sleep
import pybullet as p

from bullet_utils.env import BulletEnvWithGround
from robot_properties_solo.solo8wrapper import Solo8Robot, Solo8Config

from centroidal_controller import RobotCentroidalController
from impedance_controller import RobotImpedanceController

NB_EE = 4

def run():
    
    env = BulletEnvWithGround()
    env.set_floor_frictions()
    
    K_des_lean_ID = p.addUserDebugParameter("des_lean", 0, 0.5, .1)
    K_des_height_ID = p.addUserDebugParameter("des_height", 0, 0.5, .25)    
    K_des_xp_ID = p.addUserDebugParameter("des_xp", 0, 0.5, 0.0)
    K_des_xv_ID = p.addUserDebugParameter("des_xv", 0, 0.1, 0.0)
    
    robot = Solo8Robot(useFixedBase=False)
    robot = env.add_robot(robot)
    robot_config = Solo8Config()
    mu = 0.2
    kc = [200, 200, 200]
    dc = [5, 5, 5]
    kb = [200, 200, 200]
    db = [1.0, 1.0, 1.0]
    qp_penalty_lin = 3 * [1e6]
    qp_penalty_ang = 3 * [1e6]


    q,dq = robot.get_state()
    
    
    g = robot.pin_robot.gravity(q)  
    # inertia matrix
    M = robot.pin_robot.mass(q)  
    
    config_file = "./solo_impedance.yaml"
    

    robot_leg_ctrl = RobotImpedanceController(robot, config_file)
        
    # Initialize control
    tau = np.zeros(robot.nb_dof)

    # Reset the robot to some initial state.
    q0 = np.matrix(robot_config.initial_configuration).T
    dq0 = np.matrix(robot_config.initial_velocity).T
    robot.reset_state(q0, dq0)
    
    # Impedance controller gains
    kp = NB_EE * [200, 200, 200]
    kd = NB_EE * [10.0, 10.0, 10.0]

    # Desired leg length.
    x_des = NB_EE * [0.1, 0.0 -0.25]
    
    print(x_des)
    xd_des = NB_EE * [0.0, 0.0, 0.0]
    
    # Desired center of mass position and velocity.
    x_com = [0.0, 0.0, 0.18]
    xd_com = [0.0, 0.0,  0.0]
    # The base should be flat.
    x_ori = [0.0, 0.0, 0.0, 1.0]
    x_angvel = [0.0, 0.0, 0.0]
    # Alle end-effectors are in contact.
    cnt_array = 4 * [1]

    # distributing forces to the active end-effectors
    f = np.zeros(NB_EE * 3)
    f = NB_EE * [0.0,(robot_config.mass * 9.8) / 4]
        
        
    try: 
        # Run the simulator for 100 steps
        while True:
            # Step the simulator.
            env.step(
                sleep=True
            )  # You can sleep here if you want to slow down the replay
            # Read the final state and forces after the stepping.
            q, dq = robot.get_state()
            
        
            x_des = NB_EE * [p.readUserDebugParameter(K_des_lean_ID), 0.0, -p.readUserDebugParameter(K_des_height_ID)]

            # passing forces to the impedance controller
            tau = robot_leg_ctrl.return_joint_torques(
                q, dq, kp, kd, x_des, xd_des, 12 * [0]
            )
            
            
            # passing torques to the robot
            robot.send_joint_command(tau)
    except Exception as e:
        pass
        
    finally:
        
        # close pybullet environment
        p.disconnect()


if __name__ == "__main__":
    run()