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
from skopt import gp_minimize

joint_list = ['FFJ1', 'FFJ2', 'FFJ3', 'FFJ4', 'LFJ1', 'LFJ2', 'LFJ3', 'LFJ4', 'LFJ5', 'MFJ1', 'MFJ2', 'MFJ3', 'MFJ4', 'RFJ1', 'RFJ2', 'RFJ3', 'RFJ4', 'THJ1', 'THJ2', 'THJ3', 'THJ4', 'THJ5', 'WRJ1', 'WRJ2']

#CONSTANT DEFINITIONS 
JOINT_TYPE = 'THJ1'
MAO_REAL_PRESENT = True
PARAM_PERCENT = 2
TRAJECTORY_NAME = 'thumb_calibration'
MEDIAN_ANGLE = 1.5708
ANGLE_DEV = 0.01

reload_pub = rospy.Publisher('mujoco_reload_verify', Bool, queue_size=10)

_reload_verification = False #Global variable to control the update load in the xml file

i=0
real_positions = []
real_timestamp_column = []

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

def getBounds(numerics_params, param_percent):
    bounds = []
    for param in numerics_params:
        if isinstance(param, (int, float)):
            lower_bound = param * (1 - param_percent)
            upper_bound = param * (1 + param_percent)
            if lower_bound > upper_bound:
                lower_bound, upper_bound = upper_bound, lower_bound
            bounds.append([lower_bound, upper_bound])
        elif isinstance(param, list):
            inner_bounds = getBounds(param, param_percent)
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

def writeOptiResults(open_mode, iter,  curr_mse, curr_global_mse): #Write/update file with algorithm results (interation, current mse, best mse)
    with open("/home/user/projects/shadow_robot/base/optimisationDTparams/bayesian_search_results.txt", open_mode) as f:
        f.write(f'{iter}, {curr_mse}, {curr_global_mse}\n')
    f.close()

def execute_simulation(params):  # run simulation with parameters
   
    parameters_1 = params[:2] 
    parameters_2 = params[2:4]
    parameters_3 = params[4:6]
    parameters_4 = params[6:8] 
    parameters_5 = params[8:]

    ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_1, 'THJ1', True)
    ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_2, 'THJ2', True)
    ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_3, 'THJ3', True)
    ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_4, 'THJ4', True)
    ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_5, 'THJ5', True)
    
    global _reload_verification
    _reload_verification = True
    reload_pub.publish(reload_msg)

    while(_reload_verification): #Pensar em colocar em threads, mais giro (maybe)
        1

    # Simulate the system with the current particle position
    positions, n_iterations, timestamps = RosNode.simulateTrajectory(str, TRAJECTORY_NAME)
    
    '''real_positions, n_ints, real_timestamps=csv_reader.csvReader('_'+ TRAJECTORY_NAME +'_real_movement.csv','r')
    real_timestamp_column = [row-real_timestamps[0] for row in real_timestamps]'''

    timestamp_column = [row-timestamps[0] for row in timestamps]
    all_timestamps.append(timestamp_column)
    position_column = [row[joint_list.index(JOINT_TYPE)] for row in positions]
    all_pos.append(position_column)

    RosNode.executeTrajectory(str, 'hand_stretched')

    mse = getMSEfromJoint(positions, timestamp_column, real_positions, real_timestamp_column, JOINT_TYPE[:-1])
    curr_global_error = getGlobalMSE(positions, timestamp_column, real_positions, real_timestamp_column)

    global i
    writeOptiResults('a', i, mse, curr_global_error)
    i+=1

    # return mse
    return mse

if __name__ == "__main__":

    all_pos = [] #array for all the position for certain joint
    all_timestamps = [] #array for all the timestamps corresponding each position for certain joint

    #For mujoco model reload
    rospy.Subscriber('mujoco_reload_verify', Bool, ReloadCallback)

    reload_msg = Bool() 
    reload_msg.data = True

    str = RosNode.initNode()
    print('\n-> Starting!')
    start_time = time.time() #timer

    RosNode.executeTrajectory(str, 'hand_stretched')

    positions, n_iterations, timestamps = RosNode.simulateTrajectory(str, TRAJECTORY_NAME)
    
    real_positions, n_ints, real_timestamps=csv_reader.csvReader('_'+ TRAJECTORY_NAME +'_real_movement.csv','r')
    real_timestamp_column = [row-real_timestamps[0] for row in real_timestamps]

    timestamp_column = [row-timestamps[0] for row in timestamps]
    all_timestamps.append(timestamp_column)
    position_column = [row[joint_list.index(JOINT_TYPE)] for row in positions]
    all_pos.append(position_column)

    mse = getMSEfromJoint(positions, timestamp_column, real_positions, real_timestamp_column, JOINT_TYPE[:-1])
    best_global_error = getGlobalMSE(positions, timestamp_column, real_positions, real_timestamp_column)

    print(f'\n MSE inicial (Before simulation): {mse}\n') #Print inicial MSE values

    writeOptiResults('w',-1, mse, best_global_error)

    # Define bounds for the parameters
    numerics_params = [] 
    numerics_params += ObtainParam.getSingleParam_shared_options('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', 'THJ1', True)
    numerics_params += ObtainParam.getSingleParam_shared_options('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', 'THJ2', True)
    numerics_params += ObtainParam.getSingleParam_shared_options('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', 'THJ3', True)
    numerics_params += ObtainParam.getSingleParam_shared_options('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', 'THJ4', True)
    numerics_params += ObtainParam.getSingleParam_shared_options('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', 'THJ5', True)

    bounds = getBounds(numerics_params, PARAM_PERCENT)

    bounds = [tuple(sublist) for sublist in bounds]
    start_time = time.time() #timer
    
    result = gp_minimize(execute_simulation, bounds, n_calls=500)
    
    parameters_1 = result.x[:2] 
    parameters_2 = result.x[2:4]
    parameters_3 = result.x[4:6]
    parameters_4 = result.x[6:8] 
    parameters_5 = result.x[8:]

    ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_1, 'THJ1', True)
    ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_2, 'THJ2', True)
    ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_3, 'THJ3', True)
    ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_4, 'THJ4', True)
    ObtainParam.setSingleParam_shared_options_joint_parameters('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', parameters_5, 'THJ5', True)

    writeOptiResults('a',-1, result.fun, -1)
    print(result.x)
    #Send signal to Update the xml file with the best params
    _reload_verification = True
    reload_pub.publish(reload_msg)

    #Duration in sec
    end_time = time.time()
    print('\n-> Finished in', round(end_time-start_time,2), 'seconds')
    
    #Why not (onjetivo de dar reset á posição da mao com os melhores parametros)
    RosNode.executeTrajectory(str, 'hand_stretched')
