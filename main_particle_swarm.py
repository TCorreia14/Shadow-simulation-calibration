import rospy
import numpy as np
import time
import random
import matplotlib.pyplot as plt
import run_optimisation_trajectories_node as RosNode
import parameter_attainment as ObtainParam
import csv_reader
from std_msgs.msg import Bool
from sklearn.metrics import mean_squared_error

joint_list = ['FFJ1', 'FFJ2', 'FFJ3', 'FFJ4', 'LFJ1', 'LFJ2', 'LFJ3', 'LFJ4', 'LFJ5', 'MFJ1', 'MFJ2', 'MFJ3', 'MFJ4', 'RFJ1', 'RFJ2', 'RFJ3', 'RFJ4', 'THJ1', 'THJ2', 'THJ3', 'THJ4', 'THJ5', 'WRJ1', 'WRJ2']

#CONSTANT DEFINITIONS 
JOINT_TYPE = 'FFJ1'
PARAM_PERCENT = 0.7
MEDIAN_ANGLE = 1.5708
TRAJECTORY_NAME = 'hand_closing'

_reload_verification = False #Global variable to control the update load in the xml file

def findClosestPoints(point, array):
    timestamps = np.array([p[0] for p in array])
    index = np.argmin(np.abs(timestamps - point[0]))
    return array[index]

def filterArray(array_sim, array_real):
    matched_points = []
    for point in array_real:
        matched_point = findClosestPoints(point, array_sim)
        matched_points.append(matched_point)
    return matched_points

def ReloadCallback(data):
    global _reload_verification
    if not data.data:
        _reload_verification = False

def getIndexesFromJointList(joint):
    indexes = []
    for i in range(len(joint_list)):
        if joint in joint_list[i]:
            indexes.append(i)
    return indexes

def getMSEfromJoint(positions_sim, timestamp_column, positions_real, real_timestamp_column, joint):
    total_mse = 0
    indexes = getIndexesFromJointList(joint) #TODO acrescentar em erro em caso de coloaçao errada do nome do joint

    for i in indexes:
        #Simulated points
        position_column = [row[i] for row in positions_sim]
        points_sim = np.array([(x, y) for x, y in zip(timestamp_column, position_column)])

        #Real Points
        real_position_column = [row[i] for row in positions_real]
        points_real = np.array([(x, y) for x, y in zip(real_timestamp_column, real_position_column)])

        #Filter the simulation array -> get similar timestamps from real hand
        result_array = np.array(filterArray(points_sim, points_real))

        total_mse += mean_squared_error(result_array[:, 1],points_real[:, 1])
    
    median_mse = total_mse / len(indexes)
    return median_mse

def getGlobalMSE(positions_sim, timestamp_column, positions_real, real_timestamp_column):
    total_mse = 0

    for i in range(len(joint_list)):
        position_column = [row[i] for row in positions_sim]
        points_sim = np.array([(x, y) for x, y in zip(timestamp_column, position_column)])

        #Real Points
        real_position_column = [row[i] for row in positions_real]
        points_real = np.array([(x, y) for x, y in zip(real_timestamp_column, real_position_column)])

        #Filter the simulation array -> get similar timestamps from real hand
        result_array = np.array(filterArray(points_sim, points_real))

        total_mse += mean_squared_error(result_array[:, 1],points_real[:, 1])
    
    median_mse = total_mse / len(joint_list)
    return median_mse

def getBounds(numerics_params, PARAM_PERCENT):
    bounds = []
    for param in numerics_params:
        if isinstance(param, (int, float)):
            lower_bound = param * (1 - PARAM_PERCENT)
            upper_bound = param * (1 + PARAM_PERCENT)
            bounds.append((lower_bound, upper_bound))
        elif isinstance(param, list):
            inner_bounds = getBounds(param, PARAM_PERCENT)
            bounds.append(inner_bounds)
    return bounds

def getRandomParams(bounds):
    params = []
    for bound in bounds:
        if isinstance(bound, tuple):
            param = random.uniform(bound[0], bound[1])
            params.append(param)
        elif isinstance(bound, list):
            inner_params = getRandomParams(bound)
            params.append(inner_params)
    return params

def writeOptiResults(open_mode, curr_particle ,iter,  curr_mse, best_mse, curr_global_mse, best_global_mse): #Write/update file with algorithm results (interation, current mse, best mse)
    with open("/home/user/projects/shadow_robot/base/optimisationDTparams/particle_swarm_results.txt", open_mode) as f:
        f.write(f'{curr_particle}, {iter}, {curr_mse}, {best_mse}, {curr_global_mse}, {best_global_mse}\n')
    f.close()

if __name__ == "__main__":

    all_pos = [] #array for all the position for certain joint
    all_timestamps = [] #array for all the timestamps corresponding each position for certain joint

    #For mujoco model reload
    reload_pub = rospy.Publisher('mujoco_reload_verify', Bool, queue_size=10)

    rospy.Subscriber('mujoco_reload_verify', Bool, ReloadCallback)

    reload_msg = Bool() 
    reload_msg.data = True

    str = RosNode.initNode()
    print('\n-> Starting!\n ')
    start_time = time.time() #timer

    RosNode.executeTrajectory(str, 'hand_stretched')

    real_positions, n_ints, real_timestamps=csv_reader.csvReader('_'+ TRAJECTORY_NAME +'_real_movement.csv','r')
    real_timestamp_column = [row-real_timestamps[0] for row in real_timestamps]
    real_position_column = [row[joint_list.index(JOINT_TYPE)] for row in real_positions]
    points_real = np.array([(x, y) for x, y in zip(real_timestamp_column, real_position_column)])

    # Define bounds for the parameters
    numerics_params = []
    numerics_params += ObtainParam.getSingleParam_shared_options('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', 'FFJ1')
    numerics_params += ObtainParam.getSingleParam_shared_options('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', 'FFJ2')
    numerics_params += ObtainParam.getSingleParam_shared_options('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', 'FFJ3')

    bounds = getBounds(numerics_params, PARAM_PERCENT)

    # Define swarm parameters
    num_particles = 10
    num_dimensions = len(numerics_params)
    max_iter = 100
    c1 = 0.5
    c2 = 0.3
    w = 0.9

    # Initialize the swarm
    swarm = np.zeros((num_particles, num_dimensions))
    velocity = np.zeros((num_particles, num_dimensions))
    best_swarm_position = np.zeros((num_particles, num_dimensions))
    best_swarm_error = np.ones(num_particles) * np.inf
    best_particle_position = np.zeros(num_dimensions)
    best_particle_error = np.inf
    global_error = np.inf
    best_global_error = np.inf

    lower_bound_range = MEDIAN_ANGLE - 0.05 * MEDIAN_ANGLE
    upper_bound_range = MEDIAN_ANGLE + 0.05 * MEDIAN_ANGLE

    writeOptiResults('w',-1, -1, best_particle_error, best_particle_error, global_error, best_global_error)

    #swarm[0] = numerics_params
    best_swarm_position[0] = swarm[0]
    # Initialize particles with random positions
    for i in range(num_particles):
        if i == 1:
            swarm[i] = numerics_params
        else:
            swarm[i] = getRandomParams(bounds)
            #Comment dor opti without range
            swarm[i][5]= random.uniform(lower_bound_range, upper_bound_range)
            swarm[i][11]= random.uniform(lower_bound_range, upper_bound_range)
            swarm[i][17]= random.uniform(lower_bound_range, upper_bound_range)
        best_swarm_position[i] = swarm[i]

    # print('Best swarm position: ')
    # print(best_swarm_position)
    #print(swarm)

    # Run the PSO algorithm
    iteration = 0

    while iteration < max_iter:
        # Evaluate the objective function for each particle
        for i in range(num_particles):
            '''if(swarm[i] == best_particle_position).all():
                continue'''
            '''else:'''
            #Used for inly FFJ1 & FFJ2 without range
            #fj1_parameters = swarm[i][:4]
            #fj2_parameters = swarm[i][4:]
            
            #Used for inly FFJ1 & FFJ2 & FFJ3 with range
            parameters_1 = swarm[i][:6] 
            parameters_2 = swarm[i][6:12]
            parameters_3 = swarm[i][12:]

            # ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', fj1_parameters, 'FFJ1')
            # ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', fj2_parameters, 'FFJ2')
            
            # ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', fj1_parameters, 'LFJ1')
            # ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', fj2_parameters, 'LFJ2')
            
            # ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', fj1_parameters, 'MFJ1')
            # ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', fj2_parameters, 'MFJ2')

            # ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', fj1_parameters, 'RFJ1')
            # ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', fj2_parameters, 'RFJ2')


            ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_1, 'FFJ1')
            ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_2, 'FFJ2')
            ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_3, 'FFJ3')

            ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_1, 'LFJ1')
            ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_2, 'LFJ2')
            ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_3, 'LFJ3')

            ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_1, 'MFJ1')
            ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_2, 'MFJ2')
            ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_3, 'MFJ3')

            ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_1, 'RFJ1')
            ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_2, 'RFJ2')
            ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_3, 'RFJ3')
            
            #Publish the signal to upodate mujoco model
            _reload_verification=True
            reload_pub.publish(reload_msg)

            while(_reload_verification): #Pensar em colocar em threads, mais giro (maybe)
                1

            # Simulate the system with the current particle's position
            positions, n_iterations, timestamps = RosNode.simulateTrajectory(str, TRAJECTORY_NAME, iteration)
        
            timestamp_column = [row-timestamps[0] for row in timestamps]
            all_timestamps.append(timestamp_column)
            position_column = [row[joint_list.index(JOINT_TYPE)] for row in positions]
            all_pos.append(position_column)

            points_sim = np.array([(x, y) for x, y in zip(timestamp_column, position_column)])

            result_array = np.array(filterArray(points_sim, points_real))

            mse = getMSEfromJoint(positions, timestamp_column, real_positions, real_timestamp_column, JOINT_TYPE[:-1])
            global_error = getGlobalMSE(positions, timestamp_column, real_positions, real_timestamp_column)

            RosNode.executeTrajectory(str, 'hand_stretched')

            if mse < best_swarm_error[i]:
                best_swarm_position[i] = swarm[i]
                best_swarm_error[i] = mse

            # Update the best positions
            if global_error < best_global_error: # mse < best_particle_error:
                best_particle_position = swarm[i]
                best_particle_error = mse
                best_global_error = global_error
                

            writeOptiResults('a',iteration , i, mse, best_particle_error, global_error, best_global_error)

            # print('MSE: ')
            # print(mse)
            # print('present warm:')
            # print(swarm[i])
            # """print('sWARM VALUES')
            # print(swarm)"""
            # print('best mse:')
            # print(best_particle_error)
            # print('best params: ')
            # print(best_particle_position)

        # Update the particles' velocities and positions
        for i in range(num_particles):
            r1 = np.random.random(num_dimensions)
            r2 = np.random.random(num_dimensions)

            # Ensure particles are within bounds
            
            velocity[i] = (w * velocity[i] + c1 * r1 * (best_swarm_position[i] - swarm[i]) + c2 * r2 * (best_particle_position - swarm[i]))
            swarm[i] += velocity[i]

            # for j in range(num_dimensions):
            #     if swarm[i][j] < bounds[j][0]:
            #         swarm[i][j] = bounds[j][0]
            #     elif swarm[i][j] > bounds[j][1]:
            #         swarm[i][j] = bounds[j][1]

        print(f'\nIterarion swarm: {iteration}\n')
        iteration += 1

    print(best_particle_error)

    #Used for inly FFJ1 & FFJ2 without range
    # j1_parameters = best_particle_position[:4]
    # j2_parameters = best_particle_position[4:]
    
    #Used for inly FFJ1 & FFJ2 & FFJ3 with range
    parameters_1 = best_particle_position[:6] 
    parameters_2 = best_particle_position[6:12]
    parameters_3 = best_particle_position[12:]

    # ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', fj1_parameters, 'FFJ1')
    # ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', fj2_parameters, 'FFJ2')

    # ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', fj1_parameters, 'FFJ1')
    # ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', fj2_parameters, 'FFJ2')
    
    # ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', fj1_parameters, 'LFJ1')
    # ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', fj2_parameters, 'LFJ2')
    
    # ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', fj1_parameters, 'MFJ1')
    # ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', fj2_parameters, 'MFJ2')

    # ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', fj1_parameters, 'RFJ1')
    # ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', fj2_parameters, 'RFJ2')

    ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_1, 'FFJ1')
    ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_2, 'FFJ2')
    ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_3, 'FFJ3')

    ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_1, 'LFJ1')
    ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_2, 'LFJ2')
    ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_3, 'LFJ3')

    ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_1, 'MFJ1')
    ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_2, 'MFJ2')
    ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_3, 'MFJ3')

    ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_1, 'RFJ1')
    ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_2, 'RFJ2')
    ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_3, 'RFJ3')

    #Send signal to Update the xml file with the best params
    _reload_verification=True
    reload_pub.publish(reload_msg)

    #Why not (onjetivo de dar reset á posição da mao com os melhores parametros)
    RosNode.executeTrajectory(str, 'hand_stretched')

    print(best_particle_position)

    end_time = time.time()
    print("\nTime taken:", end_time - start_time, "seconds")