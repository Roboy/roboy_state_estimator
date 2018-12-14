import triad_openvr
import time
import sys
import rospy
import tf
from tf import transformations
import numpy as np
from pyquaternion import Quaternion
br = tf.TransformBroadcaster()
li = tf.TransformListener()

rospy.init_node('tracker_tf_broadcaster')

v = triad_openvr.triad_openvr()
v.print_discovered_objects()

interval = 1/10

while not rospy.is_shutdown():
    start = time.time()
    txt = ""
    try:
        pose = v.devices["tracking_reference_1"].get_pose()
    except:
        continue

    rotX = np.array([[-1.0,0.0,0.0,0.0],[0.0,-1.0,0.0,0.0],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,1.0]])
    pose = np.array([pose[0][:4],pose[1][:4],pose[2][:4],[0.0,0.0,0.0,1.0]])
    pose_corrected = np.matmul(pose,rotX)
    q = tf.transformations.quaternion_from_matrix(pose_corrected)

    br.sendTransform([pose_corrected[0][3],pose_corrected[1][3],pose_corrected[2][3]],
                     q,
                     rospy.Time.now(),
                     "lighthouse1_corrected",
                     "world")
    br.sendTransform([pose[0][3],pose[1][3],pose[2][3]],
                     tf.transformations.quaternion_from_matrix(pose),
                     rospy.Time.now(),
                     "lighthouse1_vive",
                     "world")
    try:
        pose = v.devices["tracking_reference_2"].get_pose()
    except:
        continue

    rotX = np.array([[-1.0,0.0,0.0,0.0],[0.0,-1.0,0.0,0.0],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,1.0]])
    pose = np.array([pose[0][:4],pose[1][:4],pose[2][:4],[0.0,0.0,0.0,1.0]])
    pose_corrected = np.matmul(pose,rotX)
    q = tf.transformations.quaternion_from_matrix(pose_corrected)

    br.sendTransform([pose_corrected[0][3],pose_corrected[1][3],pose_corrected[2][3]],
                     q,
                     rospy.Time.now(),
                     "lighthouse2_corrected",
                     "world")
    br.sendTransform([pose[0][3],pose[1][3],pose[2][3]],
                     tf.transformations.quaternion_from_matrix(pose),
                     rospy.Time.now(),
                     "lighthouse2_vive",
                     "world")
