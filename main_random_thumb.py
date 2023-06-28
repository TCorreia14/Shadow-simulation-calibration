import re
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
from scipy.optimize import curve_fit

joint_list = ['FFJ1', 'FFJ2', 'FFJ3', 'FFJ4', 'LFJ1', 'LFJ2', 'LFJ3', 'LFJ4', 'LFJ5', 'MFJ1', 'MFJ2', 'MFJ3', 'MFJ4', 'RFJ1', 'RFJ2', 'RFJ3', 'RFJ4', 'THJ1', 'THJ2', 'THJ3', 'THJ4', 'THJ5', 'WRJ1', 'WRJ2']

#CONSTANT DEFINITIONS
JOINT_TYPE = 'THJ5'
NUM_ITERACTIONS = 0   #if == 0 -> Check the mse from joint / finger / global hand
MAO_REAL_PRESENT = True
PARAM_PERCENT = 0.7
TRAJECTORY_NAME = 'thumb_calibration'
MEDIAN_ANGLE = 1.5708

_reload_verification = False #Global variable to control the update load in the xml file

def poly_func(x, i, h, g, a, b, c, d, e, f):
    return i*x**8 + h*x**7 + g*x**6 + a*x**5 + b*x**4 + c*x**3 + d*x**2 + e*x + f

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
    indexes = getIndexesFromJointList(joint) 

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

def writeOptiResults(open_mode, iter, curr_mse, best_mse, curr_global_mse, best_global_mse): #Write/update file with algorithm results (interation, current mse, best mse)
    with open("/home/user/projects/shadow_robot/base/optimisationDTparams/random_results.txt", open_mode) as f:
        f.write(f'{iter}, {curr_mse}, {best_mse}, {curr_global_mse}, {best_global_mse}\n')
    f.close()

if __name__ == "__main__":

    index = 1 #keep track of n iterations
    all_pos = [] #array for all the position for certain joint
    all_timestamps = [] #array for all the timestamps corresponding each position for certain joint

    #For mujoco model reload
    reload_pub = rospy.Publisher('mujoco_reload_verify', Bool, queue_size=10)

    rospy.Subscriber('mujoco_reload_verify', Bool, ReloadCallback)

    reload_msg = Bool() 
    reload_msg.data = True

    str = RosNode.initNode()
    print('\n-> Starting!')
    start_time = time.time() #timer

    RosNode.executeTrajectory(str, 'hand_stretched')
    
    numerics_params = [] 
    numerics_params += ObtainParam.getSingleParam_shared_options('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', 'THJ1')
    numerics_params += ObtainParam.getSingleParam_shared_options('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', 'THJ2')
    numerics_params += ObtainParam.getSingleParam_shared_options('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', 'THJ3')
    numerics_params += ObtainParam.getSingleParam_shared_options('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', 'THJ4')
    numerics_params += ObtainParam.getSingleParam_shared_options('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', 'THJ5')

    #print(numerics_params)
    best_parameters = numerics_params

    #First iteraiton for 1 error/best parameters initialization
    positions, n_iterations, timestamps = RosNode.simulateTrajectory(str, TRAJECTORY_NAME, index)
    
    timestamp_column = [row-timestamps[0] for row in timestamps]
    all_timestamps.append(timestamp_column)
    position_column = [row[joint_list.index(JOINT_TYPE)] for row in positions]
    all_pos.append(position_column)

    points_sim = np.array([(x, y) for x, y in zip(timestamp_column, position_column)])

    #Reset hand position
    RosNode.executeTrajectory(str, 'hand_stretched')

    real_positions, n_ints, real_timestamps=csv_reader.csvReader('_'+ TRAJECTORY_NAME +'_real_movement.csv','r')
    real_timestamp_column = [row-real_timestamps[0] for row in real_timestamps]
    real_position_column = [row[joint_list.index(JOINT_TYPE)] for row in real_positions]
    points_real = np.array([(x, y) for x, y in zip(real_timestamp_column, real_position_column)])

    #Filter the simulation array -> get similar timestamps from real hand
    result_array = np.array(filterArray(points_sim, points_real))
    
    #Calculate error
    joint_error = mean_squared_error(result_array[:, 1],points_real[:, 1])
    print(f'\nInitial Joint Error: {joint_error}')

    best_error = getMSEfromJoint(positions, timestamp_column, real_positions, real_timestamp_column, JOINT_TYPE[:-1])

    best_global_error = getGlobalMSE(positions, timestamp_column, real_positions, real_timestamp_column)

    #Define the bounds for the params optimisation
    bounds = getBounds(numerics_params, PARAM_PERCENT)

    lower_bound_range = MEDIAN_ANGLE - 0.10 * MEDIAN_ANGLE
    upper_bound_range = MEDIAN_ANGLE + 0.10 * MEDIAN_ANGLE

    parameter_ranges = list(zip(*bounds))

    #Create file with iteracions
    writeOptiResults('w', -1, best_error, best_error, best_global_error, best_global_error)

    for i in range(NUM_ITERACTIONS):
        result_array = []

        points_sim = []
        # Generate a set of random parameters
        parameters = getRandomParams(bounds)

        #Used for inly FFJ1 & FFJ2 without range
        #parameters_1 = parameters[:4] 
        #parameters_2 = parameters[4:]

        #Used for inly FFJ1 & FFJ2 & FFJ3 with range
        parameters_1 = parameters[:6] 
        parameters_2 = parameters[6:12]
        parameters_3 = parameters[12:18]
        parameters_4 = parameters[18:24] 
        parameters_5 = parameters[24:]

        #Update only range because of the more narrow range of values (with actual significance)
        parameters_1[5] =random.uniform(lower_bound_range, upper_bound_range)
        parameters[5] = parameters_1[5]
        parameters_2[5] = random.uniform(lower_bound_range, upper_bound_range)
        parameters[11] = parameters_2[5]
        parameters_3[5] = random.uniform(lower_bound_range, upper_bound_range)
        parameters[17] = parameters_3[5]

        print('AQUIIIII')
        print(parameters_1[5])
        print(f'COISAS:{parameters}')
        print(parameters_3[5])

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
        #ObtainParam.setParam_shared_options('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters)
        
        #Publish the signal to upodate mujoco model
        _reload_verification=True
        reload_pub.publish(reload_msg)

        while(_reload_verification): #Pensar em colocar em threads, mais giro (maybe)
            1
  
        #Simulate the trajectory the trajectory
        positions, n_iterations, timestamps = RosNode.simulateTrajectory(str, TRAJECTORY_NAME, index)
        
        if len(positions) == 0:
            i-=1
            print(f'ERROR OCURRED ON: {i} and index number: {index}')
            continue
        
        #print(f'Concluded movement on main program with: {positions} positions')
        position_column = [row[joint_list.index(JOINT_TYPE)] for row in positions]
        all_pos.append(position_column)

        timestamp_column = [row-timestamps[0] for row in timestamps]
        all_timestamps.append(timestamp_column)

        points_sim = np.array([(x, y) for x, y in zip(timestamp_column, position_column)])
        
        #Filter the simulation array -> get similar timestamps from real hand
        result_array = np.array(filterArray(points_sim, points_real))
        
        # Evaluate the error for the current set of parameters
        error = getMSEfromJoint(positions, timestamp_column, real_positions, real_timestamp_column, JOINT_TYPE[:-1])
        print(f'\nError: {error}')
        
        global_error = getGlobalMSE(positions, timestamp_column, real_positions, real_timestamp_column)
        print(f'\nGlobal error: {global_error}')

        #Reset hand position
        RosNode.executeTrajectory(str, 'hand_stretched')
        
        # Update the best set of parameters seen so far
        if error < best_error:
            best_parameters = parameters
            best_error = error
            best_global_error = global_error

        if (index-1)%10 == 0:
            print(best_error)
        
        writeOptiResults('a',index, error, best_error, global_error, best_global_error)
        print('\nInteraction number: {}\n'.format(index))
        index+=1

    #Set the param in the xml file with the best params

    #Used for inly FFJ1 & FFJ2 without range
    #parameters_1 = best_parameters[:4]
    #parameters_2 = best_parameters[4:]
    
    #Used for inly FFJ1 & FFJ2 & FFJ3 with range
    parameters_1 = best_parameters[:6] 
    parameters_2 = best_parameters[6:12]
    parameters_3 = best_parameters[12:]
     
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
    #ObtainParam.setParam_shared_options('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', best_parameters)
    
    #Send signal to Update the xml file with the best params
    _reload_verification=True
    reload_pub.publish(reload_msg)

    #Why not (onjetivo de dar reset á posição da mao com os melhores parametros)
    RosNode.executeTrajectory(str, 'hand_stretched')

    print(f'Best median mse finger error: {best_error}')
    print(f'Global hand median mse error: {getGlobalMSE(positions, timestamp_column, real_positions, real_timestamp_column)}')
    
    #plot interactions
    for i in range(index):
        plt.scatter(all_timestamps[i-1], all_pos[i-1], s=15)
        
    plt.xlabel('Time')
    plt.ylabel('Joint angle')
    plt.title('Plot Position')

    if MAO_REAL_PRESENT:
        real_positions, n_ints, real_timestamps=csv_reader.csvReader('_' + TRAJECTORY_NAME + '_real_movement.csv','r')
        real_timestamp_column = [row-real_timestamps[0] for row in real_timestamps]
        real_position_column = [row[joint_list.index(JOINT_TYPE)] for row in real_positions]

        plt.scatter(real_timestamp_column, real_position_column, s=10, color='red')

    plt.scatter( result_array[:, 0], result_array[:, 1], s=10, color = 'green')

    end_time = time.time()
    print("\nTime taken:", end_time - start_time, "seconds")
    plt.show()

""" Curve_fit (8º gradient)
    if MAO_REAL_PRESENT:
        real_positions, n_ints, real_timestamps=csv_reader.csvReader('_index_closing_real_movement.csv','r')
        real_timestamp_column = [row-real_timestamps[0] for row in real_timestamps]
        real_position_column = [row[joint_list.index(JOINT_TYPE)] for row in real_positions]

        if False:
            print('\nTimestamp: ') 
            print(real_timestamp_column)
            print('\nPosição: ')
            print(real_position_column)

            print('\n\n\n Timestamp simulação: ')
            print(all_timestamps[0])
            print('\n\n\n posição simulação: ')
            print(all_pos[0])

        plt.scatter(real_timestamp_column, real_position_column, s=4, color='red')

        coeff, _ = curve_fit(poly_func, all_timestamps[0], all_pos[0])
        print(coeff)
        x_vals = np.linspace(0, all_timestamps[0][-1], 10000)
        y_vals = poly_func(x_vals, *coeff)
    
        plt.plot(x_vals, y_vals)
"""