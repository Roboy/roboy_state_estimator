import triad_openvr
import time
import sys
import rospy
import tf
import numpy as np
import math
from pyquaternion import Quaternion
import std_msgs, sensor_msgs
rospy.init_node('roboy_ik_target')

br = tf.TransformBroadcaster()
li = tf.TransformListener()

v = triad_openvr.triad_openvr()
v.print_discovered_objects()

initial_pose1 = v.devices["tracker_1"].get_pose_quaternion()
initial_pose2 = v.devices["tracker_2"].get_pose_quaternion()
p_init1 = inital_pose1[0:3]
p_init2 = inital_pose2[0:3]
q_init1 = Quaternion(initial_pose1[6],initial_pose1[3],initial_pose1[4],initial_pose1[5])
q_init2 = Quaternion(initial_pose2[6],initial_pose2[3],initial_pose2[4],initial_pose2[5])

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

pos_tracker_1_prev = np.array([0,0,0])
pos_tracker_2_prev = np.array([0,0,0])

while not rospy.is_shutdown():
    start = time.time()
    try:
        pose = v.devices["tracker_1"].get_pose_quaternion()
    except:
        continue

    w = pose[6]
    x = pose[3]
    y = pose[4]
    z = pose[5]
    q_tracker_1 = Quaternion(w,x,y,z)*q_init1.inverse

    pos_tracker_1 = np.array([pose[0]-p_init1[0],pose[1]-p_init1[1],pose[2]-p_init1[2]])

    br.sendTransform([pos_tracker_1[0],pos_tracker_1[1],pos_tracker_1[2]],
                     q_tracker_1,
                     rospy.Time.now(),
                     "tracker_1",
                     "world")

    if (pos_tracker_1-pos_tracker_1_prev).norm()>0.05:
        rospy.loginfo("pose tracker_1 changed")
        pos_tracker_2_prev = pos_tracker_2

    try:
        pose = v.devices["tracker_2"].get_pose_quaternion()
    except:
        continue
    w = pose[6]
    x = pose[3]
    y = pose[4]
    z = pose[5]
    q_tracker_2 = Quaternion(w,x,y,z)*q_init2.inverse

    pos_tracker_2 = np.array([pose[0]-p_init2[0],pose[1]-p_init2[1],pose[2]-p_init2[2]])

    br.sendTransform([pos_tracker_2[0],pos_tracker_2[1],pos_tracker_2[2]],
                     q_tracker_2,
                     rospy.Time.now(),
                     "tracker_2",
                     "world")
    if (pos_tracker_2-pos_tracker_2_prev).norm()>0.05:
        rospy.loginfo("pose tracker_2 changed")
        pos_tracker_2_prev = pos_tracker_2
