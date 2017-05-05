#!/usr/bin/env python

import math
import os
import subprocess
import signal
from time import sleep

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


def bound(n, lower, upper):
    if n < lower:
        return lower
    if n > upper:
        return upper
    return n


class JoyController(object):
    def __init__(self, rate=1):
        self.control = {'l/r': 3, 'f/r': 1}
        self.switch = {'start_record': 0, 'stop_record': 1}

        rospy.init_node('joy_control')
        rospy.Subscriber('joy', Joy, self.joy_cb)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=0)

        self.recording = False
        self.joy_msg = Joy()
        self.rate = rate
        self.bagsRecorded = 0

        self.base = rospy.get_param('~base_name', 'recording')

        self.bag_recorder = None

        print "Initialized"

    def get_recording_dir(self):
        pkg_dir, _ = subprocess.Popen('rospack find line_follower', shell=True, stdout=subprocess.PIPE).communicate()
        pkg_dir = pkg_dir.strip()

        for i in range(1, 1000):
            recording_dir = '{}/data/{}_{}'.format(pkg_dir, self.base, i)

            if not os.path.exists(recording_dir):
                self.bagsRecorded = i - 1
                return recording_dir

    def start_recording(self):
        recording_dir = self.get_recording_dir()
        os.makedirs(recording_dir)

        print recording_dir

        command = 'rosbag record -a -x ".*image_raw$" -O {}/source.bag'.format(recording_dir)
        self.bag_recorder = subprocess.Popen(command, shell=True, stdin=subprocess.PIPE)

        self.recording = True
        print "Recording #{} started".format(self.bagsRecorded + 1)

    def stop_recording(self):
        print "interrupting process"
        # self.bag_recorder.communicate(' ')
        self.bag_recorder.send_signal(signal.SIGINT)
        sleep(0.2)
        self.bag_recorder.send_signal(signal.SIGTERM)
        self.bag_recorder.send_signal(signal.SIGKILL)
        self.bag_recorder.wait()

        subprocess.Popen('pkill rosbag', shell=True).wait()
        print "Process has finished"

    def joy_cb(self, msg):
        self.joy_msg = msg
        start_cmd = msg.buttons[self.switch['start_record']]
        stop_cmd = msg.buttons[self.switch['stop_record']]

        if start_cmd and not self.recording:
            self.start_recording()

        if stop_cmd and self.recording:
            self.recording = False
            self.stop_recording()
            self.bagsRecorded += 1
            print "Recording #{} finished".format(self.bagsRecorded)

        if not self.recording or self.rate == 0:
            self.use_joy(msg)

    def use_joy(self, msg):
        turn_cmd = msg.axes[self.control['l/r']] * 1.5
        forward_cmd = msg.axes[self.control['f/r']] * 0.3

        if self.recording:
            forward_cmd = 0.05
            turn_cmd *= 0.2

        self.cmd_pub.publish(
            Twist(
                linear=Vector3(x=forward_cmd),
                angular=Vector3(z=turn_cmd)))

    def run(self):
        if self.rate == 0:
            rospy.spin()
        else:
            r = rospy.Rate(self.rate)
            while not rospy.is_shutdown():
                if self.recording:
                    self.use_joy(self.joy_msg)
                r.sleep()


if __name__ == '__main__':
    joy = JoyController()
    joy.run()
