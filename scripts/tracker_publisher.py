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
publish_robot_target = False
publish_robot_state_for_training = True
head = False
shoulder_left = True

br = tf.TransformBroadcaster()
li = tf.TransformListener()

v = triad_openvr.triad_openvr()
v.print_discovered_objects()

interval = 1/10

initial_pose1 = v.devices["tracker_1"].get_pose_quaternion()
initial_pose2 = v.devices["tracker_2"].get_pose_quaternion()
q_init1 = Quaternion(initial_pose1[6],initial_pose1[3],initial_pose1[4],initial_pose1[5])
q_init2 = Quaternion(initial_pose2[6],initial_pose2[3],initial_pose2[4],initial_pose2[5])

sphere_axis0 = rospy.Publisher('/sphere_axis0/sphere_axis0/target', std_msgs.msg.Float32 , queue_size=1)
sphere_axis1 = rospy.Publisher('/sphere_axis1/sphere_axis1/target', std_msgs.msg.Float32 , queue_size=1)
sphere_axis2 = rospy.Publisher('/sphere_axis2/sphere_axis2/target', std_msgs.msg.Float32 , queue_size=1)

joint_state = rospy.Publisher('/joint_states', sensor_msgs.msg.JointState , queue_size=1)
joint_state_training = rospy.Publisher('/joint_states_training', sensor_msgs.msg.JointState , queue_size=1)

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

#try:
#    (trans_top,rot) = li.lookupTransform('/world', '/top', rospy.Time(0))
#    rot_top = Quaternion(rot)
#    top = rot_top.rotation_matrix
#    X0 = np.array(top[0][:])
#    X1 = np.array(top[1][:])
#    X2 = np.array(top[2][:])
#except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#    rospy.loginfo("could not find transform world->top, initialization might be wrong")

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

    w = pose[6]
    x = pose[3]
    y = pose[4]
    z = pose[5]
    q_tracker_1 = Quaternion(w,x,y,z)                   #*q_init1.inverse

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
    w = pose[6]
    x = pose[3]
    y = pose[4]
    z = pose[5]
    q_tracker_2 = Quaternion(w,x,y,z)                 #*q_init2.inverse

    pos_tracker_2 = np.array([pose[0]-initial_pose2[0],pose[1]-initial_pose2[1],pose[2]-initial_pose2[2]])

    br.sendTransform([pos_tracker_2[0],pos_tracker_2[1],pos_tracker_2[2]],
                     q_tracker_2,
                     rospy.Time.now(),
                     "tracker_2",
                     "world")


    q_top_estimate = q_tracker_2*q_tracker_1.inverse
    rospy.loginfo_throttle(1,q_top_estimate)

    br.sendTransform(trans_top,
                     [q_top_estimate[3],q_top_estimate[2],q_top_estimate[1],q_top_estimate[0]],
                     rospy.Time.now(),
                     "top_estimate",
                     "world")

    euler = rotationMatrixToEulerAngles(q_top_estimate.rotation_matrix)

    if publish_robot_state:
        msg = sensor_msgs.msg.JointState()
        msg.header = std_msgs.msg.Header()
        msg.header.stamp = rospy.Time.now()
        msg.name = ['sphere_axis0', 'sphere_axis1', 'sphere_axis2']
        if head:
            msg.position = [-euler[0], -euler[1], euler[2]]
        if shoulder_left:
            msg.position = [euler[0], euler[1], -euler[2]]
        msg.velocity = [0,0,0]
        msg.effort = [0,0,0]
        joint_state.publish(msg)
    if publish_robot_target:
       # use this for joint targets
        msg = std_msgs.msg.Float32(euler[0])
        sphere_axis0.publish(msg)
        msg = std_msgs.msg.Float32(euler[1])
        sphere_axis1.publish(msg)
        msg = std_msgs.msg.Float32(euler[2])
        sphere_axis2.publish(msg) 
    if publish_robot_state_for_training:
        msg = sensor_msgs.msg.JointState()
        msg.header = std_msgs.msg.Header()
        msg.header.stamp = rospy.Time.now()
        msg.name = ['sphere_axis0', 'sphere_axis1', 'sphere_axis2']
        if head:
            msg.position = [-euler[0], -euler[1], euler[2]]
        if shoulder_left:
            msg.position = [euler[0], euler[1], -euler[2]]
        msg.velocity = [0,0,0]
        msg.effort = [0,0,0]
        joint_state_training.publish(msg)
    

    rospy.loginfo_throttle(5,euler)
