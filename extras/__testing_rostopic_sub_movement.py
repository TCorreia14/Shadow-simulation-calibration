import rosbag
import pandas as pd

COLUMNS_NAME = ['name', 'position', 'velocity', 'effort', 'goal_status']

def ConvertRosbagtoCsv(topic_names, rosbag_filename, csv_filename):
    
    bag = rosbag.Bag('projects/shadow_robot/base/optimisationDTparams/rosbag_files/{}.bag'.format(rosbag_filename))
    df = pd.DataFrame(columns=COLUMNS_NAME)
    goal_id=0
    
    for topic, msg, t in bag.read_messages(topics=topic_names):
        
        if topic == '/execute_trajectory/status':
            for status in msg.status_list:
                goal_id = status.status
        
        if topic == '/joint_states':
            name = msg.name
            position = msg.position
            velocity = msg.velocity
            effort = msg.effort

            df = pd.concat([df, pd.DataFrame([[name, position, velocity, effort, goal_id]], columns=COLUMNS_NAME)], ignore_index=True)
            
    df.to_csv('projects/shadow_robot/base/optimisationDTparams/rosbag_files/{}'.format(csv_filename), index=False)

if __name__ == "__main__":
    CSV_FILENAME = 'tentativ1.csv'
    TOPIC_NAMES = ['/joint_states', '/execute_trajectory/status']
    ROSBAG_FILENAME = 'test1' 
    ConvertRosbagtoCsv(TOPIC_NAMES, ROSBAG_FILENAME, CSV_FILENAME)



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