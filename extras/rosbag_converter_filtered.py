import rosbag
import pandas as pd

def ConvertRosbagtoCsv(topic_name, rosbag_filename, csv_filename):

    NUM_INTERATIONS = 850
    COLUMNS_NAME = ['name', 'position', 'velocity', 'effort', 'timestamp_nsecs']

    bag = rosbag.Bag('projects/shadow_robot/base/optimisationDTparams/rosbag_files/{}.bag'.format(rosbag_filename))
    df = pd.DataFrame(columns=COLUMNS_NAME)
    aux=0
    check_movement=False

    for topic, msg, t in bag.read_messages(topics=topic_name):
        name = msg.name
        position = msg.position
        velocity = msg.velocity
        velocity_check_movement = float(velocity[0])
        effort = msg.effort
        timestamp_sec= msg.header.stamp.secs
        timestamp_nsec= msg.header.stamp.nsecs

        if(check_movement and aux < NUM_INTERATIONS):
            df = pd.concat([df, pd.DataFrame([[name, position, velocity, effort, timestamp_sec * 1000000000 + timestamp_nsec]], columns=COLUMNS_NAME)], ignore_index=True)
            aux+=1

        elif((not check_movement) and aux < 1):
            last_name = name
            last_position = position
            last_velocity = velocity
            last_effort = effort
            last_timestamp_sec = timestamp_sec
            last_timestamp_nsec = timestamp_nsec
            if (velocity_check_movement>0.001):
                df = pd.concat([df, pd.DataFrame([[last_name, last_position, last_velocity, last_effort, last_timestamp_sec * 1000000000 + last_timestamp_nsec]], columns=COLUMNS_NAME)], ignore_index=True)
                check_movement=True
        
    df.to_csv('projects/shadow_robot/base/optimisationDTparams/rosbag_files/{}'.format(csv_filename), index=False)

if __name__ == "__main__":
    CSV_FILENAME = 'tentativ1.csv'
    TOPIC_NAME = '/joint_states'
    ROSBAG_FILENAME = 'trajectories_tiago_2023-03-16-19-04-25' 
    ConvertRosbagtoCsv(TOPIC_NAME,ROSBAG_FILENAME,CSV_FILENAME)
