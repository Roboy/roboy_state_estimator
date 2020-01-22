import rospy
import yaml
from sensor_msgs.msg import JointState
import time

rospy.init_node("joint_player")
pub = rospy.Publisher('/joint_targets', JointState, queue_size=1)

with open("recorded_joints.yml", 'r') as stream:
    try:
         joints = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

msg = JointState()
length = 0
for j in joints:
    msg.name.append(j)
    length = len(joints[j])

for i in range(length):
    if rospy.is_shutdown():
        break
    for j in joints:
        msg.position.append(joints[j][i])
        msg.velocity.append(0)
        msg.effort.append(0)
    rospy.loginfo(msg.position)
    pub.publish(msg)
    msg.position = []
    msg.effort = []
    msg.velocity = []
    time.sleep(0.1)