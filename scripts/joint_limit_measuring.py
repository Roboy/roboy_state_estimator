import time
import sys
import yaml
import rospy
import sensor_msgs.msg
rospy.init_node('joint_limit_measuring')

file = None
euler0 = []
euler1 = []
name0 = "axis0"
name1 = "axis1"
initial = True

def callback(data):
    global initial
    global name0
    global name1
    if initial:
        name0 = data.name[0]
        name1 = data.name[1]
        initial = False
    euler0.append(data.position[0])
    euler1.append(data.position[1])
    rospy.sleep(0.5)

with open('joint_limits.yaml', 'w') as file:

    robot_state = rospy.Subscriber('/external_joint_states', sensor_msgs.msg.JointState , callback, queue_size=1)

    while not rospy.is_shutdown():
        rospy.loginfo_throttle(1,"recording external_joint_states")

    rospy.loginfo("writing to yaml");
    euler = {name0: euler0, name1: euler1}
    yaml.dump(euler,file)

rospy.loginfo("done")