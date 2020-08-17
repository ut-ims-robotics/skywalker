from geometry_msgs.msg import Pose
import tf.transformations as tf
from urdf_parser_py.urdf import Robot
from pykdl_utils.kdl_kinematics import KDLKinematics
from pykdl_utils.joint_kinematics import JointKinematics
#import rospy
import numpy as np
robot = Robot.from_parameter_server()
import random
#base_link = robot.get_root()
#end_link = robot.link_map.keys()[random.randint(0, len(robot.link_map)-1)]
base_link = "base_link"
end_link = "tool0"
print "Root link: %s; end link: %s" % (base_link, end_link)
#js_kin = JointKinematics(robot, base_link, end_link)
#print "Joint velocities:", js_kin.get_joint_velocities()
kdl_kin = KDLKinematics(robot, base_link, end_link)
q = np.array([0,0,0,0,0,0])
#q = kdl_kin.random_joint_angles()
#q = Kdl_kin.
#print "Random angles:", q
pose = kdl_kin.forward(q)
print (type(pose))

def np2ros(np_pose):
	    """Transform pose from np.array format to ROS Pose format.

	    Args:
		np_pose: A pose in np.array format (type: np.array)

	    Returns:
		An HTM (type: Pose).
	    """

	    # ROS pose
	    ros_pose = Pose()

	    # ROS position
	    ros_pose.position.x = np_pose[0, 3]
	    ros_pose.position.y = np_pose[1, 3]
	    ros_pose.position.z = np_pose[2, 3]

	    # ROS orientation 
	    np_q = tf.quaternion_from_matrix(np_pose)
	    ros_pose.orientation.x = np_q[0]
	    ros_pose.orientation.y = np_q[1]
	    ros_pose.orientation.z = np_q[2]
	    ros_pose.orientation.w = np_q[3]

	    return ros_pose


#pose  = np.array([[-0.58373237, -0.62130173, -0.52272428,  0.44829319],
# [-0.12450737, -0.56768288,  0.81377765,  0.71879111],
# [-0.80234309,  0.54011138,  0.25401823,  0.49950773],
# [ 0.,          0.,          0.,          1.        ]])

#pose_ros = np2ros(pose)
#print pose_ros
def ros2np(ros_pose):
    """Transform pose from ROS Pose format to np.array format.
    Args:
        ros_pose: A pose in ROS Pose format (type: Pose)
    Returns:
        An HTM (type: np.array).
    """

    # orientation
    np_pose = tf.quaternion_matrix([ros_pose.orientation.x, ros_pose.orientation.y, ros_pose.orientation.z, ros_pose.orientation.w])
    
    # position
    np_pose[0][3] = ros_pose.position.x
    np_pose[1][3] = ros_pose.position.y
    np_pose[2][3] = ros_pose.position.z

    return np_pose
#np_pose = ros2np(pose_ros)
#print np_pose
print "FK:", pose
q_new = kdl_kin.inverse(pose)
print "IK (not necessarily the same):", q_new
if q_new is not None:
    pose_new = kdl_kin.forward(q_new)
    print "FK on IK:", pose_new
    print "Error:", np.linalg.norm(pose_new * pose**-1 - np.mat(np.eye(4)))
else:
    print "IK failure"
J = kdl_kin.jacobian(q_new)
print "Jacobian:", J
#M = kdl_kin.inertia(q)
#print "Inertia matrix:", M
#if False:
#    M_cart = kdl_kin.cart_inertia(q)
#    print "Cartesian inertia matrix:", M_cart
