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

while not rospy.is_shutdown():
    start = time.time()
    txt = ""
    try:
        pose = v.devices["tracker_1"].get_pose_quaternion()
    except:
        continue

    q_current = Quaternion(pose[6],pose[3],pose[4],pose[5])

    q = q_current*q_init1.inverse

    pos = np.array([pose[0]-initial_pose1[0],pose[1]-initial_pose1[1],pose[2]-initial_pose1[2]])

    br.sendTransform([pos[0],pos[1],-pos[2]],
                     (q[1],q[2],q[3],q[0]),
                     rospy.Time.now(),
                     "tracker_1",
                     "world")
    try:
        pose = v.devices["tracker_2"].get_pose_quaternion()
    except:
        continue

    q_current = Quaternion(pose[6],pose[3],pose[4],pose[5])

    q = q_current*q_init2.inverse

    pos = np.array([pose[0]-initial_pose2[0],pose[1]-initial_pose2[1],pose[2]-initial_pose2[2]])

    br.sendTransform([pos[0],pos[1],-pos[2]],
                     (q[1],q[2],q[3],q[0]),
                     rospy.Time.now(),
                     "tracker_2",
                     "world")
