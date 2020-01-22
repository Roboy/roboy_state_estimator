import rospy
import yaml
from sensor_msgs.msg import JointState
import time

joints = {"shoulder_right_axis0": [], "shoulder_right_axis1": [], "shoulder_right_axis2": [],}
def cb(data):
    for i in range(len(data.name)):

        # if joints[data.name[i]] is None:
        #     joints[data.name[i]] = []
        joints[data.name[i]].append(data.position[i])
    time.sleep(0.1)

rospy.init_node("joint_recorder")
s = rospy.Subscriber('/external_joint_states', JointState, cb, queue_size=1)

while not rospy.is_shutdown():
    rospy.loginfo_throttle(1,"recording external_joint_states")
# raw_input("Press Enter to start recording...")
# while()

with open('recorded_joints.yml', 'w') as outfile:
    yaml.dump(joints, outfile, default_flow_style=False)