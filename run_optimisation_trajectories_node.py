import subprocess
import rospy
import rospkg
from run_optimisation_trajectories import SrRunTrajectories
import csv_reader
import rosbag_converter

CSV_FILENAME = 'conversionToCSV.csv'
TOPIC_NAME = '/joint_states'
ROSBAG_FILENAME = 'rosbag_record_positions' 

def initNode():
    #Init node
    rospy.init_node('run_trajectories_node')
    #trajectories_file_path = rospkg.RosPack().get_path('sr_run_trajectories') + '/config/example_close_hand_trajectory.yaml'
    trajectories_file_path = '/home/user/projects/shadow_robot/base/optimisationDTparams/test_trajectory/trajectories.yaml'
    srt = SrRunTrajectories(trajectories_file_path, arm=False, hand=True)
    return srt

def executeTrajectory(srt, trajectory_name):
    #Run Trajectory definied
    srt.run_trajectory('hand', trajectory_name)

def simulateTrajectory(srt, trajectory_name, index = None):
    #Initialize rosbag record (and wait the end of the command execution)   
    subprocess.Popen(['rosbag record -O ~/projects/shadow_robot/base/optimisationDTparams/rosbag_files/{} {}'.format(ROSBAG_FILENAME, TOPIC_NAME)], shell=True)
    while(1):
        rosnode_list = subprocess.Popen(['rosnode', 'list'], stdout=subprocess.PIPE)
        output, error = rosnode_list.communicate()
        if 'record_' in output.decode('utf-8'):
            #print("Rosbag recording started successfully")
            break

    #Run Trajectory definied
    executeTrajectory(srt, trajectory_name)

    #Finish record o rosbag (and wait the end of the command execution)
    subprocess.Popen(['rosnode list | grep record_ | while read line; do rosnode kill $line; done'], shell=True)

    while(1):
        rosnode_list = subprocess.Popen(['rosnode', 'list'], stdout=subprocess.PIPE)
        output, error = rosnode_list.communicate()

        if not 'record_' in output.decode('utf-8'):
            #print("Rosbag recording stoped successfully")
            break

    #Convert rosbag to csv file 

    #csv_filename = f'conversionToCSV_{index}.csv'
    #rosbag_converter.ConvertRosbagtoCsv(TOPIC_NAME, ROSBAG_FILENAME, csv_filename)
    #csv_filename2 = f'__conversionToCSV_{index}.csv'
    rosbag_converter.ConvertRosbagtoCsvFiltred(TOPIC_NAME, ROSBAG_FILENAME, CSV_FILENAME)

    #Analyse csv file
    positions, n_interactions, timestamps = csv_reader.csvReader(CSV_FILENAME, 'r')
    #print(f'\nNumber of interations:  {n_interactions}\n')
    return positions, n_interactions, timestamps

if __name__ == "__main__":
    str=initNode()
    print('Executing the hand traj')
    executeTrajectory(str, 'index_closing')