import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

sim_init_pos = (6.819190502166748, -7.2531280517578125, 0.0)
sim_init_orient = (0.0, 0.0, 0.02128992297961797, 0.9997733439032679)

sim_init_pose= PoseWithCovarianceStamped()
sim_init_pose.header.frame_id= "map"
sim_init_pose.pose.pose.position.x= sim_init_pos[0]
sim_init_pose.pose.pose.position.y= sim_init_pos[1]
sim_init_pose.pose.pose.position.z= sim_init_pos[2]

sim_init_pose.pose.pose.orientation.x= sim_init_orient[0]
sim_init_pose.pose.pose.orientation.y= sim_init_orient[1]
sim_init_pose.pose.pose.orientation.z= sim_init_orient[2]
sim_init_pose.pose.pose.orientation.w= sim_init_orient[3]

sim_init_pose.pose.covariance = [0 for _ in range(36)]

def get_sim_init_pose():
    sim_init_pose.header.stamp = rospy.Time.now()
    return sim_init_pose
