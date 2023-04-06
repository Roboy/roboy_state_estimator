#!/usr/bin/python3

import rospy
from std_msgs.msg import Int16MultiArray
import requests
import simpleaudio as sa
import time


# Global variable to store the hug state
SENSOR1_ID = -1
SENSOR2_ID = -2

MIN_HUG_DURATION = 7

IN_PROGRESS_SOUND = "/home/roboy/heartbeat_short.wav"
COUNT_SOUND = "/home/roboy/sparkle.wav"

hug_in_progress = False
counted = False
hug_start_time = None
play_obj = None


def play_sound(filepath, interrupt=True):
    global play_obj
    wave_obj = sa.WaveObject.from_wave_file(filepath)

    if interrupt and play_obj is not None:
        play_obj.stop()

    play_obj = wave_obj.play()


def stop_sound():
    global play_obj
    play_obj.stop()


def distance_callback(msg):
    global hug_in_progress, hug_start_time, counted, SENSOR1_ID, SENSOR2_ID, MIN_HUG_DURATION

    sensor_1 = msg.data[SENSOR1_ID]
    sensor_2 = msg.data[SENSOR2_ID]

    if sensor_1 == 0 and sensor_2 == 0 and not hug_in_progress:
        hug_in_progress = True
        counted = False
        hug_start_time = time.time()
        play_sound(IN_PROGRESS_SOUND)
        requests.get("https://tools.dvnt.ro/roboy/hug_started")
        rospy.loginfo("Hug started")

    elif sensor_1 == 0 and sensor_2 == 0 and hug_in_progress:
        hug_duration = time.time() - hug_start_time
        if hug_duration > MIN_HUG_DURATION and not counted:
            play_sound(COUNT_SOUND)
            rospy.loginfo("Hug counted")
            counted = True

    elif (sensor_1 == 1 or sensor_2 == 1) and hug_in_progress:
        hug_in_progress = False
        stop_sound()
        requests.get("https://tools.dvnt.ro/roboy/hug_ended")


def main():
    rospy.init_node('hug_detector', anonymous=True)
    rospy.Subscriber('/roboy/pinky/sensing/distance',
                     Int16MultiArray, distance_callback)
    rospy.spin()


if __name__ == '__main__':
    main()
