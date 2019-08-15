import triad_openvr
import time
import sys
import rospy
import tf
import numpy as np
import math
from pyquaternion import Quaternion
import std_msgs, sensor_msgs

rospy.init_node('tracker_tf_broadcaster')

# use these to change publishing behaviour
publish_robot_state = True
publish_robot_state_for_training = True
head = False
shoulder_left = True

br = tf.TransformBroadcaster()
li = tf.TransformListener()

v = triad_openvr.triad_openvr()
v.print_discovered_objects()

# import pdb; pdb.set_trace()

interval = 1/10

right = "tracker_2"
left = "tracker_3"
torso = "tracker_1"

track_left = True
track_right = True

initial_pose_torso = v.devices[torso].get_pose_quaternion()
q_init_torso = Quaternion(initial_pose_torso[6],initial_pose_torso[3],initial_pose_torso[4],initial_pose_torso[5])

if track_left:
    initial_pose_left = v.devices[left].get_pose_quaternion()
    q_init_left = Quaternion(initial_pose_left[6],initial_pose_left[3],initial_pose_left[4],initial_pose_left[5])

if track_right:
    initial_pose_right = v.devices[right].get_pose_quaternion()
    q_init_right = Quaternion(initial_pose_right[6],initial_pose_right[3],initial_pose_right[4],initial_pose_right[5])





joint_state = rospy.Publisher('/joint_targets', sensor_msgs.msg.JointState , queue_size=1)
joint_state_training = rospy.Publisher('/joint_states_training', sensor_msgs.msg.JointState , queue_size=1)

X0 = np.array([1,0,0])
X1 = np.array([0,1,0])
X2 = np.array([0,0,1])
trans_top = np.array([0,0,0])

align_to_world = Quaternion([0,0,0,1])

try:
    pose = v.devices[torso].get_pose_quaternion()
except:
    rospy.loginfo("could not find transform world->torso, initialization might be wrong")

q_torso = Quaternion(pose[6],pose[3],pose[4],pose[5])                 #*q_init1.inverse

pos_torso = np.array([pose[0]-initial_pose_torso[0],pose[1]-initial_pose_torso[1],pose[2]-initial_pose_torso[2]])

br.sendTransform([pos_torso[0],pos_torso[1],pos_torso[2]],
                 q_torso,
                 rospy.Time.now(),
                 torso,
                 "world")

if track_right:
    try:
        pose = v.devices[right].get_pose_quaternion()
    except:

        rospy.loginfo("could not find transform world->right, initialization might be wrong")

    q_right = Quaternion(pose[6],pose[3],pose[4],pose[5])                 #*q_init2.inverse

    pos_right = np.array([pose[0]-initial_pose_right[0],pose[1]-initial_pose_right[1],pose[2]-initial_pose_right[2]])

    br.sendTransform([pos_right[0],pos_right[1],pos_right[2]],
                     q_right,
                     rospy.Time.now(),
                     right,
                     "world")

    q_tracker_diff = q_right*q_torso.inverse

    tracker_diff = q_tracker_diff.rotation_matrix

    Y0 = np.array(tracker_diff[0][:])
    Y1 = np.array(tracker_diff[1][:])
    Y2 = np.array(tracker_diff[2][:])

    rot_align = np.array([[X0.dot(Y0),X0.dot(Y1),X0.dot(Y2)],[X1.dot(Y0),X1.dot(Y1),X1.dot(Y2)],[X2.dot(Y0),X2.dot(Y1),X2.dot(Y2)]])

    q_align = Quaternion(matrix=rot_align)


if track_left:
    try:
        pose = v.devices[left].get_pose_quaternion()
    except:

        rospy.loginfo("could not find transform world->left, initialization might be wrong")

    q_left = Quaternion(pose[6],pose[3],pose[4],pose[5])                 #*q_init2.inverse

    pos_left = np.array([pose[0]-initial_pose_left[0],pose[1]-initial_pose_left[1],pose[2]-initial_pose_left[2]])

    br.sendTransform([pos_left[0],pos_left[1],pos_left[2]],
                     q_left,
                     rospy.Time.now(),
                     left,
                     "world")

    q_tracker_diff = q_left*q_torso.inverse

    tracker_diff = q_tracker_diff.rotation_matrix

    Y0 = np.array(tracker_diff[0][:])
    Y1 = np.array(tracker_diff[1][:])
    Y2 = np.array(tracker_diff[2][:])

    rot_align = np.array([[X0.dot(Y0),X0.dot(Y1),X0.dot(Y2)],[X1.dot(Y0),X1.dot(Y1),X1.dot(Y2)],[X2.dot(Y0),X2.dot(Y1),X2.dot(Y2)]])

    q_align = Quaternion(matrix=rot_align)


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


def calculateEuler(base, link, initial_pose_base, initial_pose_link):
    try:
        pose = v.devices[base].get_pose_quaternion()
    except:
        rospy.logerr("Could not find device %s"%base)

    q_init = Quaternion(pose[6],pose[3],pose[4],pose[5])

    w = pose[6]
    x = pose[3]
    y = pose[4]
    z = pose[5]
    q_base = Quaternion(w,x,y,z)*q_init.inverse

    pos_base = np.array([pose[0]-initial_pose_base[0],pose[1]-initial_pose_base[1],pose[2]-initial_pose_base[2]])

    br.sendTransform([pos_base[0],pos_base[1],pos_base[2]],
                     q_base,
                     rospy.Time.now(),
                     base,
                     "world")
    try:
        pose = v.devices[link].get_pose_quaternion()
    except:
        rospy.logerr("Could not find device %s"%link)

    w = pose[6]
    x = pose[3]
    y = pose[4]
    z = pose[5]
    q_link_init = Quaternion(initial_pose_link[6],initial_pose_link[3],initial_pose_link[4],initial_pose_link[5])
    q_link = Quaternion(w,x,y,z)*q_link_init.inverse*q_base.inverse
    q_link = q_link.normalised

    pos_link = np.array([pose[0]-initial_pose_link[0],pose[1]-initial_pose_link[1],pose[2]-initial_pose_link[2]])
    # pos_link = np.array([0,0,0])

    br.sendTransform([pos_link[0],pos_link[1],pos_link[2]],
                     q_link,
                     rospy.Time.now(),
                     link,
                     "world")

    return rotationMatrixToEulerAngles(q_link.rotation_matrix)


    # q_top_estimate = q_link#*q_init.inverse
    # rospy.loginfo_throttle(1,q_top_estimate)

    # br.sendTransform(trans_top,
    #                  [q_top_estimate[3],q_top_estimate[2],q_top_estimate[1],q_top_estimate[0]],
    #                  rospy.Time.now(),
    #                  "top_estimate_"+link,
    #                  "world")

    # euler = rotationMatrixToEulerAngles(q_top_estimate.rotation_matrix)
    # return [0,0,0]


while not rospy.is_shutdown():
    start = time.time()
    
    if track_left:
        left_euler = calculateEuler(torso, left, initial_pose_torso, initial_pose_left)
    if track_right:
        right_euler = calculateEuler(torso, right, initial_pose_torso, initial_pose_right)

    if publish_robot_state:
        msg = sensor_msgs.msg.JointState()
        msg.header = std_msgs.msg.Header()
        msg.header.stamp = rospy.Time.now()
        msg.name = ["shoulder_left_axis0","shoulder_left_axis1","shoulder_left_axis2","shoulder_right_axis0","shoulder_right_axis1","shoulder_right_axis2"]
        msg.velocity = [0,0,0,0,0,0]
        msg.effort = [0,0,0,0,0,0]
        msg.position = [left_euler[0], -left_euler[2], left_euler[1], right_euler[0], -right_euler[2], right_euler[1]]        
        joint_state.publish(msg)
        if publish_robot_state_for_training:
            joint_state_training.publish(msg)

