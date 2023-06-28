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
from sklearn.model_selection import RandomizedSearchCV
from scipy.stats import randint

joint_list = ['FFJ1', 'FFJ2', 'FFJ3', 'FFJ4', 'LFJ1', 'LFJ2', 'LFJ3', 'LFJ4', 'LFJ5', 'MFJ1', 'MFJ2', 'MFJ3', 'MFJ4', 'RFJ1', 'RFJ2', 'RFJ3', 'RFJ4', 'THJ1', 'THJ2', 'THJ3', 'THJ4', 'THJ5', 'WRJ1', 'WRJ2']

#CONSTANT DEFINITIONS 
JOINT_TYPE = 'FFJ1' 
NUM_ITERACTIONS = 0
MAO_REAL_PRESENT = True
PARAM_PERCENT = 0.7
TRAJECTORY_NAME = 'hand_closing'

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

    numerics_params = ObtainParam.getParam_shared_options('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml')
    best_parameters = numerics_params

    #First iteraiton for 1 error/best parameters initialization
    positions, n_iterations, timestamps = RosNode.simulateTrajectory(str, TRAJECTORY_NAME, index)
    
    timestamp_column = [row-timestamps[0] for row in timestamps]
    all_timestamps.append(timestamp_column)
    position_column = [row[joint_list.index(JOINT_TYPE)] for row in positions]
    all_pos.append(position_column)
    
    #Loop for optimization
    for i in range(NUM_ITERACTIONS):
        print('\n-> Iteration n°', i+1)

        #Define bounds for random search
        param_grid = {
            'i': randint(0, 1000),
            'h': randint(0, 1000),
            'g': randint(0, 1000),
            'a': randint(0, 1000),
            'b': randint(0, 1000),
            'c': randint(0, 1000),
            'd': randint(0, 1000),
            'e': randint(0, 1000),
            'f': randint(0, 1000),
        }

        # Define the search algorithm
        random_search = RandomizedSearchCV(estimator=None, param_distributions=param_grid, n_iter=10, scoring=getGlobalMSE, cv=3, n_jobs=-1, verbose=1)

        # Perform the search
        random_search.fit(all_pos, all_timestamps)

        print('Best parameters found:', random_search.best_params_)
        print('MSE: ', random_search.best_score_)

        #Get new parameters
        best_parameters['i'] = random_search.best_params_['i']
        best_parameters['h'] = random_search.best_params_['h']
        best_parameters['g'] = random_search.best_params_['g']
        best_parameters['a'] = random_search.best_params_['a']
        best_parameters['b'] = random_search.best_params_['b']
        best_parameters['c'] = random_search.best_params_['c']
        best_parameters['d'] = random_search.best_params_['d']
        best_parameters['e'] = random_search.best_params_['e']
        best_parameters['f'] = random_search.best_params_['f']

        #Load the parameters to the XML file
        ObtainParam.setParam_shared_options('projects/shadow_robot/base/src/sr_common/sr_description/mujoco_models/shared_options.xml', best_parameters, )

        #Execute trajectory to simulate a new movement with new parameters
        positions, n_iterations, timestamps = RosNode.simulateTrajectory(str, TRAJECTORY_NAME, index)

        timestamp_column = [row-timestamps[0] for row in timestamps]
        all_timestamps.append(timestamp_column)
        position_column = [row[joint_list.index(JOINT_TYPE)] for row in positions]
        all_pos.append(position_column)

        #Publish to reload the mujoco model
        reload_pub.publish(reload_msg)
        time.sleep(1)

        index += 1 #increment iteration index

    end_time = time.time()
    print('\n-> Finished in', round(end_time-start_time,2), 'seconds')
    print('-> Best parameters found:', best_parameters)
    print('-> Best MSE:', random_search.best_score_)
