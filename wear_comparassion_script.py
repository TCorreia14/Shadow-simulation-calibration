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
JOINT_TYPE = 'FFJ1'
JOINT_CAPTION = 'FJ1'
MAO_REAL_PRESENT = True
TRAJECTORY_NAME = 'hand_closing' #  hand_closing  or  thumb_calibration

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

    real_positions, n_ints, real_timestamps=csv_reader.csvReader('_'+ TRAJECTORY_NAME +'_real_movement.csv','r') #NORMAL CASE
    real_timestamp_column = [row-real_timestamps[0] for row in real_timestamps]
    real_position_column = [row[joint_list.index(JOINT_TYPE)] for row in real_positions]
    points_real = np.array([(x, y) for x, y in zip(real_timestamp_column, real_position_column)])


    real_positions_2, n_ints, real_timestamps_2=csv_reader.csvReader('_used_'+ TRAJECTORY_NAME +'_real_movement.csv','r') #wear CASE
    real_timestamp_column_2 = [row-real_timestamps_2[0] for row in real_timestamps_2]
    real_position_column_2 = [row[joint_list.index(JOINT_TYPE)] for row in real_positions_2]
    points_real_2 = np.array([(x, y) for x, y in zip(real_timestamp_column_2, real_position_column_2)])

        
    plt.xlabel('Time (ns)', fontsize=16)
    plt.ylabel('Angle (rad)', fontsize=16)
    plt.title(JOINT_CAPTION, fontsize=16)
    plt.xticks(fontsize=13)
    plt.yticks(fontsize=13)

    if MAO_REAL_PRESENT:
        plt.scatter(real_timestamp_column, real_position_column, s=6, color='red')
        plt.scatter(real_timestamp_column_2, real_position_column_2, s=6, color='blue')

    plt.ylim(-0.1,1.74)
    plt.show()
