import rosbag
import pandas as pd


def ConvertRosbagtoCsv(topic_name, rosbag_filename, csv_filename):
    
    COLUMNS_NAME = ['name', 'position', 'velocity', 'effort']

    bag = rosbag.Bag('projects/shadow_robot/base/optimisationDTparams/rosbag_files/{}.bag'.format(rosbag_filename))
    df = pd.DataFrame(columns=COLUMNS_NAME)
    #aux=0

    for topic, msg, t in bag.read_messages(topics=topic_name):
        name = msg.name
        position = msg.position
        velocity = msg.velocity
        #velocity_check_movemet = float(velocity[0])
        effort = msg.effort

        df = pd.concat([df, pd.DataFrame([[name, position, velocity, effort]], columns=COLUMNS_NAME)], ignore_index=True)
    df.to_csv('projects/shadow_robot/base/optimisationDTparams/rosbag_files/{}'.format(csv_filename), index=False)

def RealConvertRosbagtoCsvFiltred(topic_name, rosbag_filename, csv_filename): #index closing trajectory (real movement)

    COLUMNS_NAME1 = ['name', 'position', 'velocity', 'effort', 'timestamp_nsecs']
    NUM_ITERATIONS = 800

    bag = rosbag.Bag('projects/shadow_robot/base/optimisationDTparams/rosbag_files/{}.bag'.format(rosbag_filename))
    df = pd.DataFrame(columns=COLUMNS_NAME1)
    aux=0
    check_movement=False

    for topic, msg, t in bag.read_messages(topics=topic_name):
        name = msg.name
        position = msg.position
        velocity = msg.velocity
        velocity_check_movement = float(velocity[0])
        #velocity_check_movement = float(velocity[18]) #Used for thumb
        effort = msg.effort
        timestamp_sec= msg.header.stamp.secs
        timestamp_nsec= msg.header.stamp.nsecs

        if(check_movement and aux < NUM_ITERATIONS):
            df = pd.concat([df, pd.DataFrame([[name, position, velocity, effort, timestamp_sec * 1000000000 + timestamp_nsec]], columns=COLUMNS_NAME1)], ignore_index=True)
            aux+=1

        elif((not check_movement) and aux < 1):
            last_name = name
            last_position = position
            last_velocity = velocity
            last_effort = effort
            last_timestamp_sec = timestamp_sec
            last_timestamp_nsec = timestamp_nsec
            if (velocity_check_movement>0.01):
                df = pd.concat([df, pd.DataFrame([[last_name, last_position, last_velocity, last_effort, last_timestamp_sec * 1000000000 + last_timestamp_nsec]], columns=COLUMNS_NAME1)], ignore_index=True)
                check_movement=True
        
    df.to_csv('projects/shadow_robot/base/optimisationDTparams/rosbag_files/{}'.format(csv_filename), index=False)


def RealCloseHandConvertRosbagtoCsvFiltred(topic_name, rosbag_filename, csv_filename): #hand closing trajectory (real movement)

    COLUMNS_NAME1 = ['name', 'position', 'velocity', 'effort', 'timestamp_nsecs']
    NUM_ITERATIONS = 800 #used for finger/index

    bag = rosbag.Bag('projects/shadow_robot/base/optimisationDTparams/rosbag_files/{}.bag'.format(rosbag_filename))
    df = pd.DataFrame(columns=COLUMNS_NAME1)
    aux=0
    aux2=0
    check_movement=False

    for topic, msg, t in bag.read_messages(topics=topic_name):
        name = msg.name
        position = msg.position
        velocity = msg.velocity
        velocity_check_movement = float(velocity[1]) 
        effort = msg.effort
        timestamp_sec= msg.header.stamp.secs
        timestamp_nsec= msg.header.stamp.nsecs
        aux2+=1

        if(check_movement and aux < NUM_ITERATIONS):
            df = pd.concat([df, pd.DataFrame([[name, position, velocity, effort, timestamp_sec * 1000000000 + timestamp_nsec]], columns=COLUMNS_NAME1)], ignore_index=True)
            aux+=1

        elif((not check_movement) and aux < 1 and aux2 > 2200):
            last_name = name
            last_position = position
            last_velocity = velocity
            last_effort = effort
            last_timestamp_sec = timestamp_sec
            last_timestamp_nsec = timestamp_nsec
            if (velocity_check_movement>0.01):
                df = pd.concat([df, pd.DataFrame([[last_name, last_position, last_velocity, last_effort, last_timestamp_sec * 1000000000 + last_timestamp_nsec]], columns=COLUMNS_NAME1)], ignore_index=True)
                check_movement=True
        
    df.to_csv('projects/shadow_robot/base/optimisationDTparams/rosbag_files/{}'.format(csv_filename), index=False)


def ConvertRosbagtoCsvFiltred(topic_name, rosbag_filename, csv_filename):
    #NUM_ITERATIONS = 1050 #For index_closing
    NUM_ITERATIONS = 1650 #For 4 fingers closing
    #NUM_ITERATIONS = 1800 #For thumb 

    COLUMNS_NAME = ['name', 'position', 'velocity', 'effort', 'timestamp_nsecs']

    bag = rosbag.Bag('projects/shadow_robot/base/optimisationDTparams/rosbag_files/{}.bag'.format(rosbag_filename))
    df = pd.DataFrame(columns=COLUMNS_NAME)
    aux=0
    check_movement=False

    for topic, msg, t in bag.read_messages(topics=topic_name):
        name = msg.name
        position = msg.position
        velocity = msg.velocity
        velocity_check_movement = float(velocity[1]) #for index/hand calibration
        #velocity_check_movement = float(velocity[18]) #for thumb calibration
        
        effort = msg.effort
        timestamp_sec= msg.header.stamp.secs
        timestamp_nsec= msg.header.stamp.nsecs

        if(check_movement and aux < NUM_ITERATIONS):
            df = pd.concat([df, pd.DataFrame([[name, position, velocity, effort, timestamp_sec * 1000000000 + timestamp_nsec]], columns=COLUMNS_NAME)], ignore_index=True)
            aux+=1

        elif((not check_movement) and aux < 1):
            last_name = name
            last_position = position
            last_velocity = velocity
            last_effort = effort
            last_timestamp_sec = timestamp_sec
            last_timestamp_nsec = timestamp_nsec
            if (velocity_check_movement>0.01): #0.005 for thumb - 0.01 for index/hand
                df = pd.concat([df, pd.DataFrame([[last_name, last_position, last_velocity, last_effort, last_timestamp_sec * 1000000000 + last_timestamp_nsec]], columns=COLUMNS_NAME)], ignore_index=True)
                check_movement=True
        
    df.to_csv('projects/shadow_robot/base/optimisationDTparams/rosbag_files/{}'.format(csv_filename), index=False)

if __name__ == "__main__":
    TOPIC_NAME = '/joint_states'
    
    CSV_FILENAME = '_close_index_hand_real_movement.csv'
    CSV_FILENAME1 = '_hand_closing_real_movement.csv'
    CSV_FILENAME2 = 'conversionToCSV.csv'
    CSV_FILENAME3 = '_thumb_calibration_real_movement.csv'
    CSV_FILENAME4 = '_used_hand_closing_real_movement.csv'
    

    #ROSBAG_FILENAME = 'trajectories_tiago_2023-03-16-19-04-25' 
    #ROSBAG_FILENAME = 'rosbag_record_positions'  
    #ROSBAG_FILENAME = 'trajectory_thumb'  
    ROSBAG_FILENAME = 'trajectory_wear_close_hand'  #trajectory_close_hand_used
    
    
    
    #ConvertRosbagtoCsvFiltred(TOPIC_NAME, ROSBAG_FILENAME, CSV_FILENAME2)
    #ConvertRosbagtoCsv(TOPIC_NAME,ROSBAG_FILENAME,CSV_FILENAME)
    #RealCloseHandConvertRosbagtoCsvFiltred(TOPIC_NAME, ROSBAG_FILENAME, CSV_FILENAME1)
    # --->> RealConvertRosbagtoCsvFiltred(TOPIC_NAME,ROSBAG_FILENAME,CSV_FILENAME3)
    RealConvertRosbagtoCsvFiltred(TOPIC_NAME,ROSBAG_FILENAME,CSV_FILENAME4)

"""
CSV_FILENAME = 'conversionToCSV.csv'
TOPIC_NAME = '/joint_states'
ROSBAG_FILENAME = 'rosbag_record_positions' 

bag = rosbag.Bag('projects/shadow_robot/base/optimisationDTparams/rosbag_files/{}.bag'.format(ROSBAG_FILENAME))
df = pd.DataFrame(columns=COLUMNS_NAME)

for topic, msg, t in bag.read_messages(topics=TOPIC_NAME):
    name = msg.name
    position = msg.position
    velocity = msg.velocity
    effort = msg.effort

    df = pd.concat([df, pd.DataFrame([[name, position, velocity, effort]], columns=COLUMNS_NAME)], ignore_index=True)

df.to_csv('projects/shadow_robot/base/optimisationDTparams/rosbag_files/{}'.format(CSV_FILENAME), index=False)

"""