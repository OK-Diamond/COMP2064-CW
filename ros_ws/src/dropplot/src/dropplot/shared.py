import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped

# these are from the initial position of the simulated robot
sim_init_pos = (-0.018831152468919754, -0.03515131399035454, 0.0)
sim_init_orient = (0.0, 0.0, 0.8590163727429395, 0.5119481139330071)

sim_init_pose= PoseWithCovarianceStamped()
sim_init_pose.header.frame_id= "map"
sim_init_pose.pose.pose.position.x= sim_init_pos[0]
sim_init_pose.pose.pose.position.y= sim_init_pos[1]
sim_init_pose.pose.pose.position.z= sim_init_pos[2]

sim_init_pose.pose.pose.orientation.x= sim_init_orient[0]
sim_init_pose.pose.pose.orientation.y= sim_init_orient[1]
sim_init_pose.pose.pose.orientation.z= sim_init_orient[2]
sim_init_pose.pose.pose.orientation.w= sim_init_orient[3]

# not sure how much this certainty will affect it finding the complete position, the covariance is quite high
sim_init_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]

def get_sim_init_pose():
    sim_init_pose.header.stamp = rospy.Time.now()
    return sim_init_pose

status_strs = [""] * (GoalStatus.LOST + 1)
status_strs[GoalStatus.PENDING] = "PENDING"
status_strs[GoalStatus.ACTIVE] = "ACTIVE"
status_strs[GoalStatus.PREEMPTED] = "PREEMPTED"
status_strs[GoalStatus.SUCCEEDED] = "SUCCEEDED"
status_strs[GoalStatus.ABORTED] = "ABORTED"
status_strs[GoalStatus.REJECTED] = "REJECTED"
status_strs[GoalStatus.PREEMPTING] = "PREEMPTING"
status_strs[GoalStatus.RECALLING] = "RECALLING"
status_strs[GoalStatus.RECALLED] = "RECALLED"
status_strs[GoalStatus.LOST] = "LOST"

