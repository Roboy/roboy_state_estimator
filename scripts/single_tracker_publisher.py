import triad_openvr
import time
import sys
import rospy
import tf
import numpy as np
import math
from pyquaternion import Quaternion
import std_msgs, sensor_msgs

tracker_name = "tracker_2"

rospy.init_node('tracker_tf_broadcaster')
br = tf.TransformBroadcaster()
li = tf.TransformListener()

v = triad_openvr.triad_openvr()
v.print_discovered_objects()

interval = 1/10

initial_pose1 = v.devices[tracker_name].get_pose_quaternion()
q_init1 = Quaternion(initial_pose1[6],initial_pose1[3],initial_pose1[4],initial_pose1[5])

joint_state = rospy.Publisher('/joint_states', sensor_msgs.msg.JointState , queue_size=1)

X0 = np.array([1,0,0])
X1 = np.array([0,1,0])
X2 = np.array([0,0,1])
trans_top = np.array([0,0,0])

align_to_world = Quaternion([0,0,0,1])

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
def rotationMatrixToEulerAngles(R) :

    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        z = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        x = math.atan2(R[1,0], R[0,0])
    else :
        z = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        x = 0

    return np.array([x, y, z])

while not rospy.is_shutdown():
    start = time.time()
    try:
        pose = v.devices["tracker_2"].get_pose_quaternion()
        # pose = v.devices["tracker_2"].get_pose_quaternion()
    except:
        continue

    w = pose[6]
    x = pose[3]
    y = pose[4]
    z = pose[5]
    q_tracker_1 = Quaternion(w,x,y,z)*q_init1.inverse

    pos_tracker_1 = np.array([pose[0]-initial_pose1[0],pose[1]-initial_pose1[1],pose[2]-initial_pose1[2]])

    br.sendTransform([pos_tracker_1[0],pos_tracker_1[1],pos_tracker_1[2]],
                     q_tracker_1,
                     rospy.Time.now(),
                     "tracker_2",
                     "world")

    euler = rotationMatrixToEulerAngles(q_tracker_1.rotation_matrix)

    msg = sensor_msgs.msg.JointState()
    msg.header = std_msgs.msg.Header()
    msg.header.stamp = rospy.Time.now()
    msg.name = ['axis0', 'axis1', 'axis2', 'axis3', 'axis4', 'axis5']
    msg.position = [-euler[0], -euler[1], euler[2], pos_tracker_1[2], -pos_tracker_1[0], pos_tracker_1[1]]
    msg.velocity = [0,0,0]
    msg.effort = [0,0,0]
    # joint_state.publish(msg)

    rospy.loginfo_throttle(5,euler)
