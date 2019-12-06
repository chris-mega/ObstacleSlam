#! /usr/bin/env python
import cv2 as cv
import numpy as np
import math
import rospy
from yaml_parser import Parser
from iros_vision.msg import ObjectCoords

MAIN_WINDOW_NAME = 'camera'
DEBUG = 'debug'

class Color_object:
    def __init__(self, config):
        self.debug = None
        self.config = config
        self.name = self.config['name']
        self.min_size = self.config['min_size']
        self.max_size = self.config['max_size']
        self.multiple_objs = self.config['multiple']
        self.color = self.config['colour']
        self.hsv = None
        self.hsv_vals = {}

        self.pub = rospy.Publisher(
            '/iros_vision_node/' + self.name, ObjectCoords, queue_size=1)

    def publish_coords(self, x, y, width, height, area, angle, pos):
        message = ObjectCoords()
        message.x = x
        message.y = y
        message.width = width
        message.height = height
        message.area = area
        message.angle = angle
        message.position = pos
        self.pub.publish(message)
        # print('{0}: x = {1}\ty = {2}\tarea = {3}\tangle = {4}'.format(self.name, x, y, area, angle))


class Vision:
    def __init__(self, file_path, cam_index, width=320, height=240):
        self.parser = Parser(file_path)
        data = self.parser.data

        self.color_objects = []
        for d in data['object']:
            new_color = Color_object(d)
            
            col_lb = new_color.config['lower_bound']
            col_ub = new_color.config['upper_bound']

            new_color.hsv_vals = {'HL': col_lb[0], 'HU': col_ub[0], 'SL': col_lb[1], 'SU': col_ub[1], 'VL': col_lb[2], 'VU': col_ub[2]}
            
            self.color_objects.append(new_color)
        
        self.selected_object = self.color_objects[0]

        self.frame = None

        self.width = width
        self.height = height

        self.cap = cv.VideoCapture(cam_index)

        cv.namedWindow(MAIN_WINDOW_NAME)
        cv.namedWindow(DEBUG)
     
        if len(self.color_objects) > 1:
            cv.createTrackbar('Key', DEBUG, 0, len(self.color_objects)-1, self.pick_object)
        cv.createTrackbar('H (lower)', DEBUG, 0, 255, lambda val: self.update_from_trackbars(val, 'HL'))
        cv.createTrackbar('H (upper)', DEBUG, 0, 255, lambda val: self.update_from_trackbars(val, 'HU'))
        cv.createTrackbar('S (lower)', DEBUG, 0, 255, lambda val: self.update_from_trackbars(val, 'SL'))
        cv.createTrackbar('S (upper)', DEBUG, 0, 255, lambda val: self.update_from_trackbars(val, 'SU'))
        cv.createTrackbar('V (lower)', DEBUG, 0, 255, lambda val: self.update_from_trackbars(val, 'VL'))
        cv.createTrackbar('V (upper)', DEBUG, 0, 255, lambda val: self.update_from_trackbars(val, 'VU'))

        col_lb = self.selected_object.config['lower_bound']
        col_ub = self.selected_object.config['upper_bound']

        self.update_trackbars(col_lb[0], col_ub[0], \
            col_lb[1], col_ub[1], \
                col_lb[2], col_ub[2])

        cv.setMouseCallback(MAIN_WINDOW_NAME, self.mouse_cb)


    def pick_object(self, val):
        self.selected_object = self.color_objects[val]
        lower_b = self.selected_object.config['lower_bound']
        upper_b = self.selected_object.config['upper_bound']

        self.update_trackbars(lower_b[0], upper_b[0], lower_b[1], upper_b[1], lower_b[2], upper_b[2])


    def update_from_trackbars(self, val, var):
        self.selected_object.hsv_vals[var] = val

        bound = None
        index = -1
        if var[0] == 'H':
            index = 0
        elif var[0] == 'S':
            index = 1
        elif var[0] == 'V':
            index = 2

        if var[1] == 'U':
            bound = 'upper_bound'
        elif var[1] == 'L':
            bound = 'lower_bound'

        self.parser.write_values(self.selected_object.config, bound, index, val)


    def update_trackbars(self, hl, hu, sl, su, vl, vu):
        cv.setTrackbarPos('H (lower)', DEBUG, hl)
        cv.setTrackbarPos('H (upper)', DEBUG, hu)

        cv.setTrackbarPos('S (lower)', DEBUG, sl)
        cv.setTrackbarPos('S (upper)', DEBUG, su)

        cv.setTrackbarPos('V (lower)', DEBUG, vl)
        cv.setTrackbarPos('V (upper)', DEBUG, vu)
        

    def create_hsv(self):
        blurred = cv.GaussianBlur(self.frame, (11,11),0)
        for color_obj in self.color_objects:
            color_obj.hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)

            lower_bound = (color_obj.hsv_vals['HL'], color_obj.hsv_vals['SL'], color_obj.hsv_vals['VL'])
            upper_bound = (color_obj.hsv_vals['HU'], color_obj.hsv_vals['SU'], color_obj.hsv_vals['VU'])

            color_obj.debug = cv.inRange(color_obj.hsv, lower_bound, upper_bound) # get binary
            # color_obj.debug = cv.erode(color_obj.debug, None, iterations=2)
            # color_obj.debug = cv.dilate(color_obj.debug, None, iterations=2)
            kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE,(10,10))
            color_obj.debug = cv.morphologyEx(color_obj.debug, cv.MORPH_CLOSE, kernel)


    def read_camera(self):
        _, self.frame = self.cap.read()
        if not self.cap.isOpened():
            print('Cannot open camera')
            exit()

        self.frame = cv.resize(self.frame, (self.width, self.height))


    def mouse_cb(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            self.click_reg = [(x,y)]
        elif event == cv.EVENT_LBUTTONUP:
            self.click_reg.append((x,y))
            mean = cv.mean(self.selected_object.hsv[self.click_reg[0][1]:y, self.click_reg[0][0]:x])
            
            h_mean = int(math.floor(mean[0]))
            s_mean = int(math.floor(mean[1]))
            v_mean = int(math.floor(mean[2]))

            init_bound = 20

            self.update_trackbars(h_mean - init_bound, h_mean + init_bound, \
                s_mean - init_bound, s_mean + init_bound, \
                    v_mean - init_bound, v_mean + init_bound)
                    