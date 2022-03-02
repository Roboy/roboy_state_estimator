import rospy
import yaml
from sensor_msgs.msg import JointState
import time
import numpy as np
import matplotlib.pyplot as plt
from progressbar import progressbar

import argparse
parser = argparse.ArgumentParser(description='Record a joint position trajectory')
parser.add_argument('name', default="recorded_joints",
                    help='enter the name of the file')

args = parser.parse_args()

rospy.init_node("joint_player")
pub = rospy.Publisher('/roboy/oxford/simulation/joint_targets', JointState, queue_size=1)

with open(args.name+".yml", 'r') as stream:
    try:
         joints = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

noise = {}
noisy_targets = {}
msg = JointState()
length = 0
for j in joints:
    msg.name.append(j)
    length = len(joints[j])
    noise[j] = np.random.normal(0,0.4,length)

    noisy_targets[j] = [x+y for (x,y) in zip(joints[j],noise[j])]
    # plt.plot(joints[j])
    # plt.plot(noisy_targets[j])
    # plt.show()
stds = [0.0,0.1,0.2,0.4,0.5]
for k in range(5):
    rospy.loginfo("adding noise " + str(stds[k]))
    for j in joints:
        noise[j] = np.random.normal(0,stds[k],length)
    for i in progressbar(range(length)):
        if rospy.is_shutdown():
            break
        for j in joints:
            # print(str(joints[j][i]) + " -> " + str(joints[j][i]+noise[j][i]) )
            # msg.position.append(joints[j][i])
            msg.position.append(joints[j][i]+noise[j][i])
            msg.velocity.append(0)
            msg.effort.append(0)
        # rospy.loginfo(msg.position)
        pub.publish(msg)
        msg.position = []
        msg.effort = []
        msg.velocity = []
        time.sleep(0.1)
