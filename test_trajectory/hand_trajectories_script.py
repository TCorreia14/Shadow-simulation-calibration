import rospy
from run_optimisation_trajectories import SrRunTrajectories
import time
if __name__ == "__main__":
    rospy.init_node('run_trajectories_node')

    trajectories_file_path = 'trajectories.yaml'
    srt = SrRunTrajectories(trajectories_file_path)
    srt.run_trajectory('hand', 'hand_stretched') #init
    time.sleep(3)

    #Mov
    srt.run_trajectory('hand', 'coisas')
    time.sleep(10)
    srt.run_trajectory('hand', 'coisas2')
    time.sleep(10)
    
