#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/19
# @author:aiden
# 手势控制
import sys
import cv2
import math
import time
import rospy
import signal
import numpy as np
from std_msgs.msg import String
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from hiwonder_servo_controllers.bus_servo_control import set_servos
sys.path.append('/home/hiwonder/software/arm_pc')
from action_group_controller import ActionGroupController

class HandGestureControlNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.image = None
        self.running = True
        self.gesture = "none"
        self.one_count = 0
        self.two_count = 0
        self.three_count = 0
        self.four_count = 0
        self.five_count = 0
        signal.signal(signal.SIGINT, self.shutdown)
        rospy.Subscriber('/hand_gesture_detect/gesture', String, self.get_hand_points_callback)
        self.controller = ActionGroupController(use_ros=True)
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1) 
        rospy.sleep(1)
        # set_servos(self.joints_pub, 1.5, ((10, 300), (5, 500), (4, 600), (3, 0), (2, 750), (1, 500)))
        self.hand_gesture_control()
    
    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def get_hand_points_callback(self, msg):
        self.gesture = msg.data

    def hand_gesture_control(self):
        while self.running:
            if self.gesture != 'none':
                if self.gesture == 'one':
                    self.one_count += 1
                    self.two_count = 0
                    self.three_count = 0
                    self.four_count = 0
                    self.five_count = 0
                    if self.one_count > 5:
                        self.one_count = 0
                        time.sleep(0.3)
                        self.controller.runAction('hand_control_pick')
                elif self.gesture == 'two':
                    self.two_count += 1
                    self.one_count = 0
                    self.three_count = 0
                    self.four_count = 0
                    self.five_count = 0
                    if self.two_count > 5:
                        self.two_count = 0
                        time.sleep(0.3)
                        self.controller.runAction('hand_control_place')
                elif self.gesture == 'three':
                    self.three_count += 1
                    self.one_count = 0
                    self.two_count = 0
                    self.four_count = 0
                    self.five_count = 0
                    if self.three_count > 5:
                        self.three_count = 0
                        time.sleep(0.3)
                        self.controller.runAction('place_right')
                elif self.gesture == 'four':
                    self.four_count += 1
                    self.one_count = 0
                    self.two_count = 0
                    self.three_count = 0
                    self.five_count = 0
                    if self.four_count > 5:
                        self.four_count = 0
                        time.sleep(0.3)
                        self.controller.runAction('place_left')
                elif self.gesture == 'five':
                    self.five_count += 1
                    self.one_count = 0
                    self.two_count = 0
                    self.three_count = 0
                    self.four_count = 0
                    if self.five_count > 5:
                        self.five_count = 0
                        time.sleep(0.3)
                        self.controller.runAction('voice_give')
            else:
                rospy.sleep(0.01)

        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    HandGestureControlNode('hand_gesture_control')

