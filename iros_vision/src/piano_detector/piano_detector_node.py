#!/usr/bin/env python

# Author: Christian Melendez

import roslib.packages as rospkg
import rosnode
import rospy
import os
import json
import math
import cv2 as cv
import numpy as np
import random

from vision import Vision, MAIN_WINDOW_NAME, DEBUG
from object_detection import piano
from time import sleep, time
from yaml_parser import Parser
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from iros_vision.msg import ObjectCoords
import heapq

class Particle(object):
    def __init__(self, x, y, angle, weight):
        self.x = x
        self.y = y
        self.angle = angle
        self.weight = weight

class ObstacleMap(object):
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.robot_x = 0
        self.robot_y = 0
        self.robot_angle = 0
        self.obstacles = []
        self.yellow_angle = 0

        self.direction = 'forward'

        self.allow_draw_obstacles = False

        self.__robot_pos_sub = rospy.Subscriber('/robot_pos', Pose, self.update_robot_pos)
        self.__robot_pos_pub = rospy.Publisher('/robot_pos_map', Pose, queue_size=0)
        self.__start_drawing_sub = rospy.Subscriber('/event_start', String, self.set_start_drawing)
        self.__yellow_line_sub = rospy.Subscriber('/yellow_line', String, self.yellow_react_callback)
        self.__yellow_sub = rospy.Subscriber('/iros_vision_node/yellow', ObjectCoords, self.yellow_callback)
        

        self.empty_map = np.zeros((self.height, self.width, 3), np.uint8)
        self.empty_map[:] = [0, 150, 0]
        for i in range(self.width/20):
            for j in range(self.height/20):
                cv.rectangle(self.empty_map, (i*20,j*20), (i*20+20, j*20+20), (0,0,0), 1)

        self.particles = []
        self.n_particles = 10
        for i in range(self.n_particles):
            new_particle = Particle(self.robot_x, self.robot_y, self.robot_angle, 1./self.n_particles)
            self.particles.append(new_particle)
        
        cv.namedWindow('map')


    def set_start_drawing(self, msg):
        self.allow_draw_obstacles = msg.data == 'start'


    def yellow_react_callback(self, msg):        
        if msg.data == 'react':
            estimated_angle = 0
            if self.yellow_angle[0] > 90:
                estimated_angle = 180 - self.yellow_angle[0]
            else:
                estimated_angle = -self.yellow_angle[0]

            self.robot_angle = estimated_angle
            
            self.update_particles(True)
            self.publish_coord()

    def yellow_callback(self, msg):
        self.yellow_x = msg.x
        self.yellow_y = msg.y
        self.yellow_width = msg.width
        self.yellow_height = msg.height
        self.yellow_angle = msg.angle

    def publish_coord(self):
        msg = Pose()
        msg.position.x = self.robot_x
        msg.position.y = self.robot_y
        msg.position.z = self.robot_angle
        self.__robot_pos_pub.publish(msg)
    

    def update_robot_pos(self, msg):
        new_x = int(msg.position.x)
        new_y = int(-msg.position.y)
        new_angle = int(msg.position.z)

        if self.robot_x != new_x:
            self.direction = 'forward'
        elif new_angle < self.robot_angle:
            self.direction = 'left'
        elif new_angle > self.robot_angle:
            self.direction = 'right'

        self.robot_x = new_x
        self.robot_y = new_y
        self.robot_angle = new_angle

        self.update_particles()
        

    def draw_robot(self, x, y, robo_angle, weight=1):
        center_x = self.width/2. + (x*10.)
        center_y = self.height - 10. + (y*10.)
        angle = math.radians(robo_angle)
        x1, y1 = center_x-7., center_y+10.
        x2, y2 = center_x+7., center_y+10.
        x3, y3 = center_x, center_y-10.

        x1r = (x1-center_x)*math.cos(angle) - (y1-center_y)*math.sin(angle) + center_x
        y1r = (x1-center_x)*math.sin(angle) + (y1-center_y)*math.cos(angle) + center_y
        pt1 = [x1r, y1r]
        
        x2r = (x2-center_x)*math.cos(angle) - (y2-center_y)*math.sin(angle) + center_x
        y2r = (x2-center_x)*math.sin(angle) + (y2-center_y)*math.cos(angle) + center_y
        pt2 = [x2r, y2r]

        x3r = (x3-center_x)*math.cos(angle) - (y3-center_y)*math.sin(angle) + center_x
        y3r = (x3-center_x)*math.sin(angle) + (y3-center_y)*math.cos(angle) + center_y 
        pt3 = [x3r, y3r]

        tr_cnt = np.array([pt1, pt2, pt3], np.int32)

        colour = (0,0,int(255*weight))
        if weight == 1:
            colour = (255,0,0)
        
        cv.drawContours(self.frame, [tr_cnt], 0, colour, -1)

    def show_map(self):
        self.frame = self.empty_map.copy()

        for p in self.particles:
            self.draw_robot(p.x, p.y, p.angle, p.weight)


        self.draw_robot(self.robot_x, self.robot_y, self.robot_angle)

        self.draw_obstacles()
        
        cv.imshow('map', self.frame)

    def get_surrounding_obs(self):
        surr_obs = [0]*6
        x = self.width/2 + (self.robot_x*10)
        y = self.height - 10 + (self.robot_y*10)
        for obs in self.obstacles:
            if obs is not None and len(obs) != 0:
                if obs[0] == x-20 and obs[1] == y-40:
                    surr_obs[0] = 'up-left'
                elif obs[0] == x and obs[1] == y-40:
                    surr_obs[1] = 'up'
                elif obs[0] == x+20 and obs[1] == y-40:
                    surr_obs[2] = 'up-right'
                elif obs[0] == x-20 and (obs[1] == y-20 or obs[1] == y):
                    surr_obs[3] = 'left'
                elif obs[0] == x and obs[1] == y-20:
                    surr_obs[4] = 'center'
                elif obs[0] == x+20 and (obs[1] == y-20 or obs[1] == y):
                    surr_obs[5] = 'right'
        return surr_obs

    def add_obstacles(self, pos):
        if self.allow_draw_obstacles:
            surr_obs = self.get_surrounding_obs()
            for p in pos:
                if not (p in surr_obs):
                    coord = None
                    x = self.width/2 + (self.robot_x*10)
                    y = self.height - 10 + (self.robot_y*10)
                    if p == 'up-left':
                        coord = [x-20, y-40]
                    elif p == 'up':
                        coord = [x, y-40]
                    elif p == 'up_right':
                        coord = [x+20, y-40]
                    elif p == 'left':
                        coord = [x-20, y-20]
                    elif p == 'center':
                        coord = [x, y-20]
                    elif p == 'right':
                        coord = [x+20, y-20]
                    self.obstacles.append(coord)


    def update_particles(self, reset=False):
        heap = []
        sum_exp = 0
        for p in self.particles:
            x, y, angle = -1, -1, -1
            if reset:
                x, y, angle = self.get_random(self.robot_x, self.robot_y, self.robot_angle)
            else:    
                x, y, angle = self.get_random(p.x, p.y, p.angle)
            
            if self.width/2 +x*10< 0 or self.width/2 +x*10 > self.width:
                x = self.robot_x
            if self.height - 10 +y*10 < 0 or self.height - 10 +y*10 > self.height:
                y = self.robot_y

            # p.x, p.y, p.angle = x*math.sin(math.radians(angle)), y*math.cos(math.radians(angle)), angle
            p.x, p.y, p.angle = x, y, angle

            distance = ((self.robot_x-p.x)**2+(self.robot_y-p.y)**2)**0.5
            exp = np.exp(distance)
            sum_exp += exp

            heapq.heappush(heap, (exp, p))

        # softmax
        for h in range(self.n_particles):
            pair = heapq.heappop(heap)
            new_weight = pair[0]/sum_exp
            pair[1].weight = new_weight


    def get_random(self, pos_x, pos_y, pos_angle):
        x, y, angle = None, None, None
        if self.direction == 'forward':
            angle = np.random.normal(pos_angle, 2, 1)[0]
            x = np.random.normal(0, 1, 1)[0]
            y = pos_y - random.random()
        elif self.direction == 'left':
            x = pos_x - random.random()
            y = pos_y - random.random()
            angle = pos_angle - random.random()*4.
        elif self.direction == 'right':
            x = pos_x + random.random()
            y = pos_y - random.random()
            angle = pos_angle + random.random()*4.

        return x, y, angle

    def draw_obstacles(self):
        if self.allow_draw_obstacles:
            # print(self.obstacles)
            for obs in self.obstacles:
                if obs is not None and len(obs) != 0:
                    cv.circle(self.frame, (obs[0], obs[1]), 10, (0,0,255), -1)

     
        

if __name__ == '__main__':
    SPIN_RATE = 10
    rospy.init_node('piano_detector_node')
    rate = rospy.Rate(SPIN_RATE)

    configuration_directory = rospy.get_param(
        'piano_detector_node/configuration_directory')
    
    camera_index = rospy.get_param(
        'piano_detector_node/cam_index')

    file_path = os.path.join(configuration_directory, 'piano_config.yaml')

    with_cascade = True
    
    vision = Vision(file_path, camera_index)
    ball_cascade = None

    folder = os.getcwd()
    folder = os.path.join(folder, 'src')

    obs_map = ObstacleMap(180, 360)
  
    
    while not rospy.is_shutdown():
        vision.read_camera()
        vision.create_hsv()

        for objs in vision.color_objects:
            x, y, area, angle, position = piano(vision.frame, objs, vision.width, vision.height)
            if len(position) != 0 and objs.name != 'yellow':
                obs_map.add_obstacles(position)


        cv.putText(vision.frame, 'selected: {}'.format(vision.selected_object.name),
            (10, 20), cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

        cv.imshow(MAIN_WINDOW_NAME, vision.frame)

        cv.imshow(DEBUG, vision.selected_object.debug)

        obs_map.show_map()

        if cv.waitKey(1) == ord('q'):
            break

    vision.cap.release()
    cv.destroyAllWindows()

