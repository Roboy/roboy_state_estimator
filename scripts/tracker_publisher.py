import triad_openvr
import time
import sys
import rospy
import tf
import numpy as np
from pyquaternion import Quaternion

br = tf.TransformBroadcaster()
li = tf.TransformListener()

rospy.init_node('tracker_tf_broadcaster')

v = triad_openvr.triad_openvr()
v.print_discovered_objects()

interval = 1/10

initial_pose1 = v.devices["tracker_1"].get_pose_quaternion()
initial_pose2 = v.devices["tracker_2"].get_pose_quaternion()
q_init1 = Quaternion(initial_pose1[6],initial_pose1[3],initial_pose1[4],initial_pose1[5])
q_init2 = Quaternion(initial_pose2[6],initial_pose2[3],initial_pose2[4],initial_pose2[5])

try:
    (trans_top,rot) = li.lookupTransform('/world', '/top', rospy.Time(0))
    rot_top = Quaternion(rot)
    top = rot_top.rotation_matrix
    X0 = np.array(top[0][:])
    X1 = np.array(top[1][:])
    X2 = np.array(top[2][:])
except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    rospy.loginfo("could not find transform world->top, initialization might be wrong")

try:
    pose = v.devices["tracker_1"].get_pose_quaternion()
except:
    rospy.loginfo("could not find transform world->tracker_1, initialization might be wrong")

q_tracker_1 = Quaternion(pose[6],pose[3],pose[4],pose[5])                 #*q_init1.inverse

pos_tracker_1 = np.array([pose[0]-initial_pose1[0],pose[1]-initial_pose1[1],pose[2]-initial_pose1[2]])

br.sendTransform([pos_tracker_1[0],pos_tracker_1[1],pos_tracker_1[2]],
                 q_tracker_1,
                 rospy.Time.now(),
                 "tracker_1",
                 "world")
try:
    pose = v.devices["tracker_2"].get_pose_quaternion()
except:
    rospy.loginfo("could not find transform world->tracker_2, initialization might be wrong")

q_tracker_2 = Quaternion(pose[6],pose[3],pose[4],pose[5])                 #*q_init2.inverse

pos_tracker_2 = np.array([pose[0]-initial_pose2[0],pose[1]-initial_pose2[1],pose[2]-initial_pose2[2]])

br.sendTransform([pos_tracker_2[0],pos_tracker_2[1],pos_tracker_2[2]],
                 q_tracker_2,
                 rospy.Time.now(),
                 "tracker_2",
                 "world")

q_tracker_diff = q_tracker_2*q_tracker_1.inverse

tracker_diff = q_tracker_diff.rotation_matrix

Y0 = np.array(tracker_diff[0][:])
Y1 = np.array(tracker_diff[1][:])
Y2 = np.array(tracker_diff[2][:])

rot_align = np.array([[X0.dot(Y0),X0.dot(Y1),X0.dot(Y2)],[X1.dot(Y0),X1.dot(Y1),X1.dot(Y2)],[X2.dot(Y0),X2.dot(Y1),X2.dot(Y2)]])

q_align = Quaternion(matrix=rot_align)

while not rospy.is_shutdown():
    start = time.time()
    try:
        pose = v.devices["tracker_1"].get_pose_quaternion()
    except:
        continue

    q_tracker_1 = Quaternion(pose[6],pose[3],pose[4],pose[5])                 #*q_init1.inverse

    pos_tracker_1 = np.array([pose[0]-initial_pose1[0],pose[1]-initial_pose1[1],pose[2]-initial_pose1[2]])

    br.sendTransform([pos_tracker_1[0],pos_tracker_1[1],pos_tracker_1[2]],
                     q_tracker_1,
                     rospy.Time.now(),
                     "tracker_1",
                     "world")
    try:
        pose = v.devices["tracker_2"].get_pose_quaternion()
    except:
        continue

    q_tracker_2 = Quaternion(pose[6],pose[3],pose[4],pose[5])                 #*q_init2.inverse

    pos_tracker_2 = np.array([pose[0]-initial_pose2[0],pose[1]-initial_pose2[1],pose[2]-initial_pose2[2]])

    br.sendTransform([pos_tracker_2[0],pos_tracker_2[1],pos_tracker_2[2]],
                     q_tracker_2,
                     rospy.Time.now(),
                     "tracker_2",
                     "world")

    q_tracker_diff = q_tracker_2*q_tracker_1.inverse

    q_top_estimate = q_align*q_tracker_diff

    br.sendTransform(trans_top,
                     q_top_estimate,
                     rospy.Time.now(),
                     "top_estimate",
                     "world")