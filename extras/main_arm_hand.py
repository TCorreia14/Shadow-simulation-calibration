import rospy
from run_optimisation_trajectories import SrRunTrajectories

if __name__ == "__main__":
    rospy.init_node('run_trajectories_node')
    #trajectories_file_path = rospkg.RosPack().get_path('sr_run_trajectories') + '/config/example_close_hand_trajectory.yaml'
    trajectories_file_path = '/home/user/projects/shadow_robot/base/optimisationDTparams/test_trajectory/trajectories.yaml'
    srt = SrRunTrajectories(trajectories_file_path, arm=True, hand=True)

    srt.run_trajectory('arm_and_hand', 'test_trajectory')