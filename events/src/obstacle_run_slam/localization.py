from time import sleep

import roslib.packages as rospkg
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
import numpy as np
import math

class Localization(object):
    def __init__(self, control_module):
        self.control_module = control_module
        self.x_steps = 0
        self.y_steps = 0
        self.x_distance = 0
        self.y_distance = 0
        self.angle_steps = 0
        self.angle = 0

        self.current_direction = 'stop'

        self.field_width, self.field_height = 180, 360
        self.avg_x_distance = 5.4 # distance of a step measured from 10 steps forward

        self.step_done_sub = rospy.Subscriber('/robotis/walking/is_full_step_done', Bool,
                                                      self.count_steps)

        self.__robot_pos_pub = rospy.Publisher('/robot_pos', Pose, queue_size=0)
        self.__robot_pos_sub = rospy.Subscriber('/robot_pos_map', Pose, self.update_curr_pos)

    def count_steps(self, msg):
        if msg.data == True:
            if self.current_direction == 'forward':
                self.y_steps += 1
                if self.y_steps % 2 == 0:
                    self.y_distance += 1*math.cos(math.radians(self.angle))
                    self.x_distance += 1*math.sin(math.radians(self.angle))
            elif self.current_direction == 'left':
                self.x_steps -= 1
                if self.x_steps % 4 == 0:
                    self.y_distance += 1*math.cos(math.radians(self.angle))
                    self.x_distance -= 1*math.sin(math.radians(self.angle))
            elif self.current_direction == 'right':
                self.x_steps += 1
                if self.x_steps % 4 == 0:
                    self.y_distance += 1*math.cos(math.radians(self.angle))
                    self.x_distance += 1*math.sin(math.radians(self.angle))
            elif self.current_direction == 'turn_left':
                self.angle_steps -= 1
                self.angle -= 15
            elif self.current_direction == 'turn_right':
                self.angle_steps += 1
                self.angle += 15
            
            self.update_pos()
            

    def walk(self, time):
        self.control_module.walking().walk(self.current_direction)
        sleep(time)
    

    def forward(self, time):
        self.current_direction = 'forward'
        self.walk(time)
    
    def right(self, time):
        self.current_direction = 'right'
        self.walk(time)
    
    def left(self, time):
        self.current_direction = 'left'
        self.walk(time)

    def turn_right(self, time):
        self.current_direction = 'turn_right'
        self.walk(time)
    
    def turn_left(self, time):
        self.current_direction = 'turn_left'
        self.walk(time)
    
    def march(self, time):
        self.current_direction = 'march'
        self.walk(time)

    def update_pos(self):
        msg = Pose()
        msg.position.x = self.x_distance
        msg.position.y = self.y_distance
        msg.position.z = self.angle
        self.__robot_pos_pub.publish(msg)

    def update_curr_pos(self, msg):
        self.x_distance = msg.position.x
        self.y_distance = msg.position.y
        self.angle = msg.position.y

