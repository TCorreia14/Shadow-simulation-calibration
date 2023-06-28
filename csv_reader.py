import csv

def csvReader(filename, open_mode):

    positions = []
    timestamps = []
    with open('projects/shadow_robot/base/optimisationDTparams/rosbag_files/{}'.format(filename), mode=open_mode) as csv_file:
        csv_reader = csv.DictReader(csv_file)
        line_count = 0
        for row in csv_reader:
            if line_count == 0:
                #print(f'Column names are {", ".join(row)}')
                line_count += 1
            string_list = row["position"].split(',')
            float_array = [float(x.strip().replace('(', '').replace(')', '')) for x in string_list]
            positions.append(float_array)
            timestamp_list = float(row["timestamp_nsecs"])
            timestamps.append(timestamp_list)
            line_count += 1
            #print(positions)
        #print(f'Processed {line_count} lines.')
        return positions, line_count, timestamps

if __name__ == "__main__":
    #csv_filename='conversionToCSV_1.csv'
    csv_filename='_thumb_calibration_real_movement.csv'
    
    positions, n_interactions, timestamps=csvReader(csv_filename, 'r')
    print(f'\nNumber of interations:  {positions[18]}\n')

#Testing fase         
"""
####Prints::
            #print(f'\tNome: {row["position"]} \nPosition of each joint: {row["position"][3]} \nVelocity of each joint: {row["velocity"]} \nEffort of each joint: {row["effort"]}.')
            #print(f'\t\nPosition of each joint: {row["position"][1]}\n\n .')
            #positions.append(row["position"])

------------------------------------------------------------------------------------------------------------------

        prev_position=None
        check_movement = False

        for row in csv_reader:
            if line_count == 0:
                #print(f'Column names are {", ".join(row)}')
                line_count += 1
            else:
                #print(f'\tNome: {row["name"]} \nPosition of each joint: {row["position"]} \nVelocity of each joint: {row["velocity"]} \nEffort of each joint: {row["effort"]}.')
                print(f'\t\nPosition of each joint: {row["position"][0]}\n\n .')
                #positions.append(row["position"])
                #line_count += 1
                curr_position = row["position"][0]
                print(row["position"][0])

                if (prev_position is not None and abs((curr_position) - (prev_position)) / (prev_position) >= 100000000 ) or check_movement:
                    positions.append(curr_position)
                    check_movement=True

                prev_position = curr_position
                line_count += 1
        print(f'Processed {line_count} lines.')
        print(positions)
        return positions, line_count




import csv
positions = []
with open('coisas.csv', mode='r') as csv_file:

#with open(filename, mode=open_mode) as csv_file:
    csv_reader = csv.DictReader(csv_file)
    line_count = 0
    for row in csv_reader:
        if line_count == 0:
            #print(f'Column names are {", ".join(row)}')
            line_count += 1
        #print(f'\tNome: {row["name"]} \nPosition of each joint: {row["position"]} \nVelocity of each joint: {row["velocity"]} \nEffort of each joint: {row["effort"]}.')
        print(f'\t\nPosition of each joint: {row["position"]}\n\n .')
        positions.append(row["position"])
        line_count += 1
        if line_count>60:
            break

    print(f'Processed {line_count} lines.')
    print(positions[2])
    #return positions, line_count

"""