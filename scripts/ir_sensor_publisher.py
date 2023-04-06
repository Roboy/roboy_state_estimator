#!/usr/bin/python3
import Jetson.GPIO as GPIO
import time
from collections import deque

import rospy
from std_msgs.msg import Int16MultiArray

SENSOR_PINS = [18,19,21,22,23,24]
WINDOW_SIZE = 10
filtered_data_queues = [deque(maxlen=WINDOW_SIZE) for _ in SENSOR_PINS]
data_buffers = [deque(maxlen=WINDOW_SIZE) for _ in SENSOR_PINS]

# SENSOR_PINS = [7,11,12,13,15,16,18,19,21,22,23,24]
TOPIC_NAME = "/roboy/pinky/sensing/distance"

def filter_data(value, data_buffer):
    data_buffer.append(value)
    return 1 if 1 in data_buffer else 0

def moving_average(value, data_queue):
    data_queue.append(value)

    filtered_value = round(sum(data_queue) / len(data_queue))
    return filtered_value

def setup_gpios():
    GPIO.setmode(GPIO.BOARD)
    for pin in SENSOR_PINS:
        GPIO.setup(pin, GPIO.IN)

def publish_data(pub):
    msg = Int16MultiArray()

    for i, pin in enumerate(SENSOR_PINS):
        raw_value = GPIO.input(pin)
        # filtered_value = moving_average(raw_value, filtered_data_queues[i])
        filtered_value = filter_data(raw_value, data_buffers[i])
        msg.data.append(filtered_value)

    pub.publish(msg)

def main():
    setup_gpios()

    rospy.init_node("ir_sensor_publisher")
    pub = rospy.Publisher(TOPIC_NAME, Int16MultiArray, queue_size=1)
    rate = rospy.Rate(50)
    rospy.loginfo("Infrared distance node is setup. Publishing data on " + TOPIC_NAME)
    while not rospy.is_shutdown():
        publish_data(pub)
        rate.sleep()                     

if __name__ == '__main__':
   main()
