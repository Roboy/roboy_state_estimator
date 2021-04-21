import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

from sensor_msgs.msg import JointState
import rospy
import matplotlib.pyplot as plt
import math
from scipy import signal

my_filter = KalmanFilter(dim_x=1, dim_z=1)

my_filter.x = np.array([[1.]])       # initial state (location and velocity)

my_filter.F = np.array([[1.]])    # state transition matrix

my_filter.H = np.array([[1.]])    # Measurement function
my_filter.P *= 1000.                 # covariance matrix
my_filter.R = 5                      # state uncertainty
my_filter.Q = Q_discrete_white_noise(2, 0.001, .1) # process uncertainty

rospy.init_node("joint_filter")
x = 0
y = 0
v=0
data = []
b = signal.firwin(10, 0.004)
z = signal.lfilter_zi(b, 1)
msg1 = JointState()
def cb(msg):
    global y, x, b,z,v

    for i in range(len(msg.name)):
        name = msg.name[i]
        if "elbow_left_axis0" in name:
            
            # v = math.degrees(msg.position[i])
            v = msg.position[i]
            ret, z = signal.lfilter(b, 1, [v], zi=z)
            y = ret[0]
            x += 1
            msg1.position = [y]
            msg1.name = ["elbow_left_axis0"]
            pub.publish(msg1)
            data.append(v)
            # my_filter.predict()
            # my_filter.update(v)
            # y = my_filter.x
            rospy.loginfo_throttle(1, y)


# sub = rospy.Subscriber("/joints", JointState, vis.cb)
sub = rospy.Subscriber("/external_joint_states", JointState, cb)
pub = rospy.Publisher("/joints", JointState)

rospy.spin()
# rate = rospy.Rate(10)
# for i in range(100):
# 	rate.sleep()

# while not rospy.is_shutdown():
# 	plt.scatter(x, y)
# 	# plt.scatter(x,v)
# 	plt.pause(0.01)


# # b, a = signal.butter(3, 0.05)
# # filt = signal.medfilt(data)
# # ff = signal.filtfilt(b,a,data)
# # lf = signal.lfilter(b,a,data)
# # plt.plot(filt, "r--")
# # plt.plot(lf, "g--")
# # plt.plot(data, "b--")



# plt.show()