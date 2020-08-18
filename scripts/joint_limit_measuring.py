import time
import sys
import yaml
import rospy
import sensor_msgs.msg
rospy.init_node('joint_limit_measuring')

model_name = "upper_body"
file = None
euler0 = []
euler1 = []
euler2 = []
eulers = [euler0, euler1, euler2]
name0 = "shoulder_left_axis0"
name1 = "shoulder_left_axis1"
name2 = "shoulder_left_axis2"
global joint_names
joint_names = [name0, name1, name2]
initial = True

def callback(data):
    global joint_names, eulers
    position = [0,0,0]
    for i in range(3):
        # import pdb; pdb.set_trace()
        if joint_names[i] in data.name:
            idx = data.name.index(joint_names[i])
            # position[i] = data.position[idx]
            eulers[i].append(data.position[idx])
        else:
            continue

    # global initial
    # global name0
    # global name1
    # if initial:
    #     name0 = data.name[0]
    #     name1 = data.name[1]
    #     name2 = data.name
    #     initial = False
    # euler0.append(data.position[0])
    # euler1.append(data.position[1])
    rospy.sleep(0.1)

with open("/home/roboy/workspace/roboy3/src/robots/"+model_name+"/joint_limits.yaml", 'w') as file:

    robot_state = rospy.Subscriber('/external_joint_states', sensor_msgs.msg.JointState , callback, queue_size=1)

    while not rospy.is_shutdown():
        rospy.loginfo_throttle(1,"recording external_joint_states")

    rospy.loginfo("writing to yaml");
    euler = {name0: eulers[0], name1: eulers[1], name2: eulers[2]}
    yaml.dump(euler,file)

rospy.loginfo("done")