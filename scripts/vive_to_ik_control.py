#!/usr/bin/python3

import triad_openvr
import time
import rospy
import numpy as np
import math
import std_msgs
import yaml
import os.path
import sys
import tf
import pyquaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose, Quaternion
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header, ColorRGBA

from roboy_middleware_msgs.msg import MotorCommand
from roboy_control_msgs.srv import GetLinkPose
import roboy_middleware_msgs.srv
from roboy_middleware_msgs.srv import InverseKinematics, InverseKinematicsRequest
from visualization_msgs.msg import InteractiveMarkerUpdate, InteractiveMarkerPose, Marker

import time
import csv
import pickle
from scipy.spatial.transform import Rotation as R


#Getting triad_openvr object with necessary methods to other functions and checks whether vive controller is connected
def _get_triadVR_object(controller_name):
    v = triad_openvr.triad_openvr()
    while controller_name not in v.devices:
        print("Controller not connected or turned off, waiting 5 seconds to reconnect ")
        print(v.devices)
        time.sleep(5)
    return v

#Getting 3d coordinates of the vive controller with respect to the lighthouses
def _get_tracker_coordinates(controller_name, triad_openvr_object):
    pose_matrix = triad_openvr_object.devices[controller_name].get_pose_matrix()
    # print(pose_matrix)
    while pose_matrix is None:
        print("Controller is out of lighthouses reach, waiting for controller ")
        time.sleep(1)
        pose_matrix = triad_openvr_object.devices[controller_name].get_pose_matrix()
    x = pose_matrix[0][3]
    y = pose_matrix[1][3]
    z = pose_matrix[2][3]
    return np.array([x,y,z])

def _get_tracker_pose(controller_name, triad_openvr_object):
    pose_matrix = triad_openvr_object.devices[controller_name].get_pose_matrix()
    # print(pose_matrix)
    while pose_matrix is None:
        print("Controller is out of lighthouses reach, waiting for controller ")
        time.sleep(1)
        pose_matrix = triad_openvr_object.devices[controller_name].get_pose_matrix()
    return np.array(pose_matrix)    

#Detects the usage of the vertical axis of the trackpad controller
def _get_trackpad_movement(controller_name, triad_openvr_object):
    return triad_openvr_object.devices[controller_name].get_controller_inputs()['trackpad_y']

#Detects the press of the trigger
def _if_trigger_pressed(controller_name, triad_openvr_object):
    return triad_openvr_object.devices[controller_name].get_controller_inputs()['trigger'] > 0.5

#Return the curren angle of a particular robot joint
def _get_single_joint_angle(joint_name, topic_name):
    msg = rospy.wait_for_message(topic_name, JointState)
    i = msg.name.index(joint_name)
    return msg.position[i]

#Returns the coordinates of a particular robot link
def _get_link_coordinats(link_name, topic_name):
    link_coordinates = np.zeros(3)

    def pose_callback(p):
        nonlocal link_coordinates
        if p.header.frame_id == endeffector_name:
            link_coordinates[1] = p.pose.position.y
            link_coordinates[2] = p.pose.position.z
            link_coordinates[0] = p.pose.position.x

    subscriber = rospy.Subscriber(topic_name, PoseStamped, callback=pose_callback)

    while np.allclose(link_coordinates, np.zeros(3)):
        rospy.Rate(100).sleep()
    subscriber.unregister()
    return link_coordinates

#Creates and returns a JointState object, which includes names, positions. velocities and efforts necessary for every joint
#Adds wrist separately since it's controlled directly
def _get_joint_state(names, angles, wrist_name):
    joint_state = JointState(Header(),[],[],[],[])
    for i in range(len(names)):
        if names[i] != wrist_name:
            print(wrist_name, names[i])
            joint_state.name.append(names[i])
            joint_state.position.append(angles[i])
            joint_state.velocity.append(0)
            joint_state.effort.append(0)
    print(joint_state)
    return joint_state

#Returns a default robot JointState object, for a robot with a right arm bent in elbow
def _get_default_joint_state(topic_name, wrist_name):
    msg = rospy.wait_for_message(topic_name, JointState)
    angles = [1.57 if i == "elbow_right" else 0 for i in msg.name]

    return _get_joint_state(msg.name, angles, wrist_name)

def joint_state_filtering(alpha, joint_state, prev_joint_state, wrist_name):
    name_list = []
    angle_list = []
    for i in range(len(joint_state.name)):
        if joint_state.name[i] in prev_joint_state.name:
            i_prev = prev_joint_state.name.index(joint_state.name[i])
            name_list.append(joint_state.name[i])
            angle_list.append(  alpha * prev_joint_state.position[i_prev] +
                                (1-alpha) * joint_state.position[i])
        else:
            name_list.append(joint_state.name[i])
            angle_list.append(joint_state.position[i])
    return _get_joint_state(name_list, angle_list, wrist_name)

def state_cb(msg):
    if msg.header.frame_id ==  "hand_left":
        hand_left_pose = msg.pose

def _get_link_pose(link_name):
    s = rospy.ServiceProxy("/get_link_pose", GetLinkPose)
    return s(link_name).pose
    # msg = rospy.wait_for_message('robot_state', PoseStamped)
    # print(msg.header.frame_id)
    # if msg.header.frame_id == link_name:
    #     return msg.pose
    # else:
    #     _get_link_pose(link_name)




if __name__ == "__main__":

    # Assignment of config variables from yaml config for next with default cases for everything for the next ~70 lines
    # Just scroll through it, it's boring
    # Or look at their detailed description in config directory

    if len(sys.argv) > 1:
        config_name = sys.argv[1]
    else:
        config_name = "vive_control.yaml"

    current_path = os.path.abspath(os.path.dirname(__file__))
    config_path = os.path.join(current_path, "../config/", config_name)

    with open(config_path) as config_file:
        config_dict = yaml.load(config_file, Loader=yaml.FullLoader)



    if "controller_name" in config_dict:
        controller_name = config_dict["controller_name"]
    else:
        controller_name = "controller_1"


    # if "endeffector_name" in config_dict:
    #     endeffector_name = config_dict[endeffector_name]
    # else:
    #     endeffector_name = "scooper"

    # if "inverse_kinematics_service" in config_dict:
    #     inverse_kinematics_service = config_dict[inverse_kinematics_service]
    # else:
    #     inverse_kinematics_service = "ik"

    # if "robot_state_topic" in config_dict:
    #     robot_state_topic = config_dict[robot_state_topic]
    # else:
    #     robot_state_topic = "robot_state"

    # if "target_joint_angle_topic" in config_dict:
    #     target_joint_angle_topic = config_dict[target_joint_angle_topic]
    # else:
    #     target_joint_angle_topic = "joint_targets"


    if "sensitivity" in config_dict:
        sensitivity = config_dict["sensitivity"]
    else:
        sensitivity = 1

    rospy.init_node('vive_to_ik_publisher')

    pos_pub = rospy.Publisher('/hand_left_cartesian_motion_controller/target_frame', PoseStamped, queue_size=1)
    marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)
    # roboy_state_sub = rospy.Subscriber('/robot_state', PoseStamped, robot_state_cb)
    ik = rospy.ServiceProxy('/execute_ik', InverseKinematics)

    br = tf.TransformBroadcaster()
    li = tf.TransformListener()
    transformer = tf.TransformerROS()
    # wrist_publisher = rospy.Publisher(wrist_motor_topic, MotorCommand)
    # angle_publisher = rospy.Publisher(target_joint_angle_topic, JointState, queue_size=5)

    v = _get_triadVR_object(controller_name)

    # joint_state = _get_default_joint_state(joint_state_topic_name, wrist_axis_name)
    # angle_publisher.publish(joint_state)

    initial_pose_controller = v.devices[controller_name].get_pose_quaternion()
    # initial_position_scooper = _get_link_coordinats(endeffector_name, robot_state_topic)

    # joint_state = _get_default_joint_state(joint_state_topic_name, wrist_axis_name)
    # prev_joint_state = joint_state
    # t = v.devices['tracker_1'].get_pose_matrix()
    # tracker_matrix = np.array([[t[0][0], t[0][1], t[0][2], t[0][3]], 
    #                             [t[1][0], t[1][1], t[1][2], t[1][3]],
    #                             [t[2][0], t[2][1], t[2][2], t[2][3]],
    #                             [0,0,0,1]])

    # world_tracker_transform = np.transpose(np.linalg.inv(tracker_matrix))

    t = v.devices['tracker_1'].get_pose_quaternion()
    br.sendTransform([t[0], t[1], t[2]], 
        [t[6], t[3], t[4], t[5]], 
        rospy.Time.now(),
        "tracker_1",
        "world") 


    # hand_left_pose = _get_link_pose("hand_left")
    # rospy.loginfo("Got hand left pose")
    # offset = [hand_left_pose.position.x,hand_left_pose.position.y, hand_left_pose.position.z]
    # t = tf2.getLatestCommonTime('torso', 'hand_left')
    # (trans,rot) = tfBuf.lookupTransform('torso', 'hand_left', rospy.Time())
    j = 0
    first = True
    while not rospy.is_shutdown():
            # time.sleep(0.1)
            
        # if _if_trigger_pressed(controller_name, v):
            # joint_state = _get_default_joint_state(joint_state_topic_name, wrist_axis_name)
            # prev_joint_state = joint_state
            # initial_position_controller = _get_tracker_coordinates(controller_name, v)
            # initial_position_scooper = _get_link_coordinats(endeffector_name, robot_state_topic)

        # else:

        # rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0.0 1.5707 world vive_world 100

            start = rospy.Time(0)
            
            t = v.devices['tracking_reference_1'].get_pose_quaternion()
            w = t[6]
            x = t[3]
            y = t[4]
            z = t[5]
            q_tracker_1 = pyquaternion.Quaternion(w,x,y,-z).unit 
            br.sendTransform([t[0], t[1], t[2]], 
                (t[3], t[4], t[5], t[6]),  
                rospy.Time.now(),
                "lighthouse_1",
                "vive_world") 

            
            t = v.devices['tracking_reference_2'].get_pose_quaternion()
            w = t[6]
            x = t[3]
            y = t[4]
            z = t[5]
            q_tracker_1 = pyquaternion.Quaternion(w,x,y,-z).unit 
            br.sendTransform([t[0], t[1], t[2]], 
                (t[3], t[4], t[5], t[6]), 
                rospy.Time.now(),
                "lighthouse_2",
                "vive_world") 


            t = v.devices['tracking_reference_3'].get_pose_quaternion()
            w = t[6]
            x = t[3]
            y = t[4]
            z = t[5]
            q_tracker_1 = pyquaternion.Quaternion(w,x,y,-z).unit 
            br.sendTransform([t[0], t[1], t[2]], 
                (t[3], t[4], t[5], t[6]), 
                rospy.Time.now(),
                "lighthouse_3",
                "vive_world") 
            
            t = v.devices['tracker_1'].get_pose_quaternion()
            [x, y, z, qx, qy, qz, qw] = t

             # Rotate Vive Trackers 180, so Z+ comes out of the top of the Tracker
            [qx, qy, qz, qw] = tf.transformations.quaternion_multiply([qx, qy, qz, qw], tf.transformations.quaternion_from_euler(math.pi, 0.0, -math.pi/2.0))


            w = t[6]
            x = t[3]
            y = t[4]
            z = t[5]
            q_tracker_1 = pyquaternion.Quaternion(w,x,y,-z).unit 
            br.sendTransform([t[0], t[1], t[2]], 
                [qx, qy, qz, qw],
                # (t[3], t[4], -t[5], t[6]), 
                rospy.Time.now(),
                "tracker_1",
                "vive_world") 


            t = v.devices[controller_name].get_pose_quaternion()
            [x, y, z, qx, qy, qz, qw] = t
            # Rotate Vive Trackers 180, so Z+ comes out of the top of the Tracker
            [qx, qy, qz, qw] = tf.transformations.quaternion_multiply([qx, qy, qz, qw], tf.transformations.quaternion_from_euler(-math.pi, 0.0, 0.0))
            w = t[6]
            x = t[3]
            y = t[4]
            z = t[5]
            q_controller = pyquaternion.Quaternion(w,x,y,-z).unit 
            br.sendTransform((t[0], t[1], t[2]), 
                [qx, qy, qz, qw],
                #(t[3], t[4], -t[5], t[6]), 
                rospy.Time.now(),
                controller_name,
                "vive_world") 

            # for i in range(3):
            #     t[i] = t[i] -  initial_pose_controller[i]

            pose = PoseStamped()
            pose.pose = Pose(position=Point(t[0],t[2],t[1]), orientation=Quaternion(0,0,0,1))    
            pose.header.stamp = start
            pose.header.frame_id = controller_name
            # print(j)
            if not first:
                # print("here")
                # (trans,rot) = li.lookupTransform(controller_name, '/world', rospy.Time(0))
                # print(trans)
                transformed_pose_controller = li.transformPose("/tracker_1", pose)
            #print(transformed_pose_controller)
              
            # current_pose_controller = _get_tracker_pose(controller_name, v)
            # current_position_controller = _get_tracker_coordinates(controller_name, v)
            # import pdb; pdb.set_trace()
            # # delta_pose_controller = current_pose_controller  - initial_pose_controller
            # transfromed_position_controller = np.matmul(world_tracker_transform,current_pose_controller)
            
            #Rewriting the coordinates from OpenVR's to ROS's coordinate frames
            
                '''
                delta_position_scooper = sensitivity * np.array([
                    transformed_pose_controller.pose.position.x,#[0][3],
                    transformed_pose_controller.pose.position.y,#[2][3],
                    transformed_pose_controller.pose.position.z#[1][3]
                ])

                # delta_position_scooper = delta_position_scooper + offset

                # modified_position_scooper = initial_position_scooper + delta_position_scooper


                pos = Point(delta_position_scooper[0], delta_position_scooper[1], delta_position_scooper[2])

                requested_pose = Pose(position = pos, orientation=hand_left_pose.orientation)
                # requested_pose.orientation.w = 1
                msg = PoseStamped()
                msg.pose = requested_pose
                msg.header.frame_id = "world"
                pos_pub.publish(msg)

                marker_msg = Marker()
                marker_msg.header.frame_id = "controller_1"
                marker_msg.id = 102
                marker_msg.type = 1
                marker_msg.action = 0
                marker_msg.text = "tracker"
                marker_msg.scale = Vector3(0.1,0.1,0.1)
                marker_msg.color = ColorRGBA(0.0,  255.0, 0.0, 40.0)
                marker_msg.pose = requested_pose
                marker_pub.publish(marker_msg)

                # ik_request = InverseKinematicsRequest(endeffector="hand_left", target_frame="hand_left", pose=requested_pose, type=1)
                # ik(ik_request)
                
                '''
            if first: first = False   

            # input("Press..")
            # ik_request = roboy_middleware_msgs.srv.InverseKinematicsRequest(endeffector_name, 1, endeffector_name, requested_pose)

            # try:
            #     get_ik = rospy.ServiceProxy(inverse_kinematics_service, roboy_middleware_msgs.srv.InverseKinematics)
            #     ik = get_ik(ik_request)
            #     new_joint_state = _get_joint_state(ik.joint_names, ik.angles, wrist_axis_name)
            #     joint_state = joint_state_filtering(filtering_alpha, joint_state, prev_joint_state, wrist_axis_name)
            #     prev_joint_state = new_joint_state
            # except rospy.ServiceException as exc:
            #     print("Inverse Kinematics calculation wasn't successfull\n: " + str(exc))

        # wrist_angle = _get_single_joint_angle(wrist_axis_name, joint_state_topic_name) +  _get_trackpad_movement(controller_name, v)
        # if(wrist_angle<wrist_min_angle):
        #     wrist_angle = wrist_min_angle
        # if(wrist_angle>wrist_max_angle):
        #     wrist_angle = wrist_max_angle

        # if wrist_simulated:
        #     joint_state.name.append(wrist_axis_name)
        #     joint_state.position.append(wrist_angle)
        #     joint_state.velocity.append(0)
        #     joint_state.effort.append(0)
        # if wrist_direct_motor_control:
        #     wrist_publisher.publish(MotorCommand(6, [2], [wrist_angle]))

        # angle_publisher.publish(joint_state)
