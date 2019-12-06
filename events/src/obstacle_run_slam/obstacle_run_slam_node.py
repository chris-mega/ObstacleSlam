#!/usr/bin/env python

from enum import Enum
from time import sleep

import rosnode
import rospy
from std_msgs.msg import String
import random

from control_modules.control_module import ControlModule, AdvancedControlModule
from obstacles import Obstacles
from iros_vision.msg import ObjectCoords
from localization import Localization
from collections import OrderedDict

import roslib.packages as rospkg
import os
import json


class RobotMode(Enum):
    READY = 0
    RUNNING = 1


class Movements(Enum):
    MARCH = 0
    FORWARD = 1
    BACKWARD = 2
    TURN_LEFT = 3
    TURN_RIGHT = 4
    ACTION = 5
    UNDEFINE = 6


def button_callack(msg):
    global robot_mode, mode_change

    if msg.data == 'start':
        # Start button pressed
        # Send robot to RUNNING mode
        if robot_mode == RobotMode.READY:
            robot_mode = RobotMode.RUNNING
            mode_change = True
        elif robot_mode == RobotMode.RUNNING:
            rospy.logerr('Robot already in running mode.')

    elif msg.data == 'mode':
        # Mode button pressed
        # Send robot to READY mode
        robot_mode = RobotMode.READY
        mode_change = True


def wait_for_node(node_name):
    # Wait for node to start
    while node_name not in rosnode.get_node_names():
        rospy.logwarn('Node is not running: {0}'.format(node_name))
        sleep(1)

    rospy.loginfo('Node is running: {0}'.format(node_name))



def avoid_obstacles():
    global mode_change, control_module, obstacles, rate, head_pan, head_tilt, yellow_line_pub

    focal_length = 261.3


    while not rospy.is_shutdown():
        yellow_pos = {'up_left':0, 'left':0, 'up_right':0, 'right':0, 'center':0, 'up':0}
        cup_pos = {'up_left':0, 'left':0, 'up_right':0, 'right':0, 'center':0, 'up':0}

        for c in obstacles.cup.position:
            cup_pos[c] += 1
        
        for y in obstacles.yellow.position:
            yellow_pos[y] += 1

        in_left = cup_pos['left']+yellow_pos['left']+cup_pos['up_left']+yellow_pos['up_left']
        in_right = cup_pos['right']+yellow_pos['right']+cup_pos['up_right']+yellow_pos['up_right']
        in_center = cup_pos['center']+yellow_pos['center']+cup_pos['up']+yellow_pos['up']

        if len(obstacles.yellow.x) > 0:
            yellow_line_pub.publish('react')

            obst_near = False
            
            if yellow_pos['center'] > 0 or yellow_pos['up'] > 0:
                if localization.angle > 0:
                    localization.turn_left(time=0.4) 
                elif localization.angle < 0:
                    localization.turn_right(time=0.4)
            else:
                if yellow_pos['left'] > 0 or yellow_pos['up_left']:
                    localization.turn_right(time=0.2)
                elif yellow_pos['right'] > 0 or yellow_pos['up_right'] > 0:
                    localization.turn_left(time=0.2)

            for c in range(len(obstacles.cup.x)):
                cup_x = obstacles.cup.x[c] + obstacles.cup.width[c]/2 
                cup_y = obstacles.cup.y[c] + obstacles.cup.height[c]/2
                if len(obstacles.yellow.x) > 0:
                    dist = ((obstacles.yellow.x[0] - cup_x)**2 + (obstacles.yellow.y[0] - cup_y)**2)**0.5
                    obst_near = dist < 200
                    if obst_near:
                        break

            if obst_near:
                if localization.angle > 0:
                    localization.turn_left(time=0.6)
                else:
                    localization.turn_right(time=0.6)
            
                
            continue
        
        
        if in_left < in_right:
            localization.turn_left(time=0.2)
            continue
        elif in_left > in_right:
            localization.turn_right(time=0.2)
            continue

        if in_center:
            rand  = random.randint(0, 1)
            if rand == 0:
                localization.turn_left(time=0.3)
            else:
                localization.turn_right(time=0.3)
            continue
        
        localization.forward(time=0.5)

        rate.sleep()


    # control_module.walking().stop()
    

def set_variables():
    global head_pan, head_tilt, yellow_min_top, blue_min_top
    config_variables_file = os.path.join(rospkg.get_pkg_dir('events'), 'config', event,
                                         'variables_{0}.json'.format(robot_size))

    vars = None
    with open(config_variables_file, 'r') as fp:
        vars = json.load(fp)

    head_pan = vars['head_pan']
    head_tilt = vars['head_tilt']
    # replace with coordinates where arrow is aimed
    yellow_min_top = vars['yellow_min_top']
    blue_min_top = vars['blue_min_top']


if __name__ == '__main__':

    # Initalize global variables
    robot_mode = RobotMode.READY
    mode_change = False
    obstacles = Obstacles()
    SPIN_RATE = 0.5
    event = 'obstacle_run_slam'

    robot_size = rospy.get_param('obstacle_run_slam_node/size')

    # Wait for other nodes to launch
    wait_for_node('/op3_manager')
    wait_for_node('/piano_detector_node')

    # Initialze ROS node
    rospy.init_node('obstacle_run_slam_node')
    rate = rospy.Rate(SPIN_RATE)
    set_variables()

    # Initialize OP3 ROS generic subscriber
    button_sub = rospy.Subscriber('/robotis/open_cr/button', String, button_callack)
    start_pub = rospy.Publisher('/event_start', String, queue_size=1)
    yellow_line_pub = rospy.Publisher('/yellow_line', String, queue_size=1)

    # Initialze vision subscribers
    cup_sub = rospy.Subscriber('/iros_vision_node/cup', ObjectCoords,
                               lambda msg: obstacles.cup.update(msg.x, msg.y, msg.width, msg.height, msg.area, msg.angle, msg.position))
    blue_sub = rospy.Subscriber('/iros_vision_node/blue', ObjectCoords,
                               lambda msg: obstacles.blue.update(msg.x, msg.y, msg.width, msg.height, msg.area, msg.angle, msg.position))
    yellow_sub = rospy.Subscriber('/iros_vision_node/yellow', ObjectCoords,
                               lambda msg: obstacles.yellow.update(msg.x, msg.y, msg.width, msg.height, msg.area, msg.angle, msg.position))

    # Initializing modules
    control_module = ControlModule(event,robot_size)
    localization = Localization(control_module)

    # Enabling walking module
    sleep(10)
    
    control_module.action().play_action('walk_ready')
    sleep(1)
    control_module.head().move_head(pan_position=head_pan, tilt_position=head_tilt)

    if robot_size == 'adult':
        robot_mode = RobotMode.RUNNING
        control_module.head().move_head(pan_position=head_pan, tilt_position=head_tilt)

        raw_input('enter tab to start')
        mode_change = True
    elif robot_size == 'kid':
        rospy.loginfo('Press start button to begin.')

    while not rospy.is_shutdown():
        if mode_change:
            mode_change = False

            if robot_mode == RobotMode.READY:
                rospy.loginfo('Resetting OP3. Press start button to begin.')

                control_module.walking().stop()
                control_module.head().move_head(pan_position=head_pan, tilt_position=head_tilt)

            elif robot_mode == RobotMode.RUNNING:
                start_pub.publish('start')
                rospy.loginfo('Starting obstacle run event.')

                avoid_obstacles()

        rate.sleep()


