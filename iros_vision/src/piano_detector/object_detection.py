import cv2 as cv
import numpy as np
import os
import math

def piano(frame, chosen_one, width, height):
    edged = chosen_one.debug
    # gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # gray = cv.medianBlur(gray, 5)
    # thresh = cv.adaptiveThreshold(gray, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 11, 5)

    # edged = cv.Canny(gray, 30, 200)

    _, contours, hier = cv.findContours(edged, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    obj_x, obj_y, obj_area, obj_angle, obj_w, obj_h, obj_pos = [], [], [], [], [], [], []


    if contours:
        # look for squares
        max_cnt = -1
        rect = None
        for cnt in contours:
            epsilon = 0.01 * cv.arcLength(cnt, True)
            approx = cv.approxPolyDP(cnt, epsilon, True)

            if not chosen_one.multiple_objs and len(approx) > max_cnt:
                rect = cnt
            elif chosen_one.multiple_objs:
                x, y, w, h, area, angle = draw_and_values(frame, cnt, chosen_one)
                if w > -1:
                    obj_x.append(x)
                    obj_y.append(y)
                    obj_w.append(w)
                    obj_h.append(h)
                    obj_area.append(area)
                    obj_angle.append(angle)
                    obj_pos.append(judge_position(x, y, w, h))

        if not chosen_one.multiple_objs:
            x, y, w, h, area, angle = draw_and_values(frame, rect, chosen_one)
            if w > -1:
                obj_x.append(x)
                obj_y.append(y)
                obj_w.append(w)
                obj_h.append(h)
                obj_area.append(area)
                obj_angle.append(angle)
                obj_pos.append(judge_position(x, y, w, h))

        # cv.imshow('thresh', thresh)
        # cv.imshow('edged', edged)
    chosen_one.publish_coords(obj_x, obj_y, obj_w, obj_h, obj_area, obj_angle, obj_pos)

    return obj_x, obj_y, obj_area, obj_angle, obj_pos


def judge_position(x, y, w, h):
    ranges_x = 320/3
    ranges_y = 240/3

    x_limit = [0, ranges_x, ranges_x*2, 320]
    y_limit = [0, ranges_y, ranges_y*2, 240]
    center_x = x+w/2.
    center_y = y+h/2.

    # Left

    if center_x < x_limit[1]:
        if center_y < y_limit[1] and y+h < ranges_y:
            return 'up_left'
        else:
            return 'left'
    elif center_x > x_limit[2]:
        if center_y < y_limit[1] and y+h < ranges_y:
            return 'up_right'
        else:
            return 'right'
    else:
        if center_y < y_limit[1]:
            return 'up'
        else:
            return 'center'
        

def draw_and_values(frame, rect, chosen_one):
    x, y, w, h, area, angle = -1, -1, -1, -1, -1, -1
    if cv.contourArea(rect) > chosen_one.min_size:
        marker = cv.minAreaRect(rect)
        box = cv.boxPoints(marker)
        box = np.int0(box)
        frame = cv.drawContours(frame, [box], 0, chosen_one.color, 2)

        angle = marker[2]
        # taken from https://stackoverflow.com/questions/15956124/minarearect-angles-unsure-about-the-angle-returned
        if marker[1][0] < marker[1][1]: # convert to negative and positive
            angle = 180 + angle
        else:
            angle = 90 + angle

        x,y,w,h = cv.boundingRect(rect)
        area = w*h

    return x, y, w, h, area, angle


def ball_tracker_cascade(frame, cascade, min_size=200):
    # apply the trained haar cascade and compare it with the current frame

    inp = frame.copy()

    gray = cv.cvtColor(inp, cv.COLOR_BGR2GRAY)

    if cascade is not None:
        balls = cascade.detectMultiScale(gray)

        x_circle, y_circle, w_circle, h_circle = -1, -1, -1 , -1

        biggest = min_size

        if balls is not None:
            for (x, y, w, h) in balls:
                if w * h > biggest:
                    x_circle, y_circle, w_circle, h_circle = x, y, w, h
                    biggest = w * h

    return x_circle, y_circle, w_circle, h_circle
        


def ball_tracker(frame, debug, min_size=200):
    # detect the ball with color

    _, thresh = cv.threshold(debug.copy(), 127, 255, 0)
    _, contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    center = None

    x, y, radius = -1, -1, -1

    if len(contours) > 0:
        
        max_contour = -1
        poly = None
        for i, c in enumerate(contours):
            if max_contour < cv.contourArea(contours[i]):
                poly = cv.approxPolyDP(contours[i], 3, True)
                ((x,y), radius) = cv.minEnclosingCircle(poly)
                max_contour = cv.contourArea(contours[i])

        M = cv.moments(poly)

        c_size = M['m00']

        if c_size > 0:
            center = (int(M['m10'] / c_size), int(M['m01'] / c_size))

            pix_to_cm = (min_size/math.pi)**0.5
            if radius <= pix_to_cm:
                x, y, radius = -1, -1, -1
        else:
            x, y, radius = -1, -1, -1

    return x, y, radius


def marker_detect(frame, forward, left, right):
    # detect the arrows
    arrow_name = 'None'
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    gray = cv.blur(gray, (3,3))
    _, thresh = cv.threshold(gray, 150, 255, cv.THRESH_BINARY)

    edged = cv.Canny(gray, 30, 200)

    img, contours, hier = cv.findContours(edged, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    posible_arrow = None
    area = -1

    # look for squares
    for cnt in contours:
        epsilon = 0.01 * cv.arcLength(cnt, True)
        approx = cv.approxPolyDP(cnt, epsilon, True)

        if len(approx) == 4:
            this_area = cv.contourArea(cnt)
            if area < this_area:
                area = this_area
                posible_arrow = approx.copy()

    # decide for arrow
    if posible_arrow is not None and area > 1000:
        # sort list of points
        pts1 = np.zeros(shape=(4,2))

        for i, p in enumerate(posible_arrow):
            pts1[i] = p[0]

        pts1.view('i8,i8').sort(order=['f1'], axis=0)

        change_to = np.zeros(shape=(4,2))

        if pts1[0][0] < pts1[1][0]:
            change_to[0] = [0,0]
            change_to[1] = [400,0]
        else:
            change_to[0] = [400,0]
            change_to[1] = [0,0]
        
        if pts1[2][0] < pts1[3][0]:
            change_to[2] = [0,400]
            change_to[3] = [400,400]
        else:
            change_to[2] = [400,400]
            change_to[3] = [0,400]

        # flat the image (no perspective angles)
        M = cv.getPerspectiveTransform(np.float32(pts1), np.float32(change_to))

        dst = cv.warpPerspective(frame, M, (400, 400))

        # get lines in arrow
        gray_dst = cv.cvtColor(dst, cv.COLOR_BGR2GRAY)
        cann = cv.Canny(gray_dst, 30, 200, apertureSize=3)
        lines = cv.HoughLines(cann, 1, np.pi/180, 100)
        arrow = []
        if lines is not None:
            for line in lines:
                rho,theta = line[0]
                
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
                # only diagonal lines
                if (theta >= math.pi/4-math.pi/8 and theta <= math.pi/4+math.pi/8) or (theta >= math.pi/2+math.pi/4-math.pi/8 and theta <= math.pi/2+math.pi/4+math.pi/8):
                    arrow.append([x1, y1, x2, y2])
                    cv.line(dst,(x1,y1),(x2,y2),(0,0,255),2)

        # we have a pair of diagonal lines!
        if len(arrow) == 2:
            # arrow = [x1, y1, x2, y2], [x3, y3, x4, y4]
            # line-line intersection: ta = ((x4-x3)(y1-y3)-(y4-y3)(x1-x3)) / ((y4-y3)(x2-x1)-(x4-x3)(y2-y1))
            #                         tb = ((x2-x1)(y1-y3)-(y2-y1)(x1-x3)) / ((y4-y3)(x2-x1)-(x4-x3)(y2-y1))
            x1, y1, x2, y2 = arrow[0][0], arrow[0][1], arrow[0][2], arrow[0][3]
            x3, y3, x4, y4 = arrow[1][0], arrow[1][1], arrow[1][2], arrow[1][3]

            ta = float((x4 - x3)*(y1 - y3) - (y4 - y3)*(x1 - x3)) / float((y4 - y3)*(x2 - x1) - (x4 - x3)*(y2 - y1))
            tb = float((x2 - x1)*(y1 - y3) - (y2 - y1)*(x1 - x3)) / float((y4 - y3)*(x2 - x1) - (x4 - x3)*(y2 - y1))

            # see if lines touch
            if ta != 0 and tb != 0: 
                xa = x1 + ta*(x2 - x1)
                ya = y1 + ta*(y2 - y1)
                xb = x3 + tb*(x4 - x3)
                yb = y3 + tb*(y4 - y3)

                # decide which arrow depending of the intersection point of lines
                if ya < 50:
                    arrow_name = 'forward'
                elif xa < 100:
                    arrow_name = 'left'
                elif xa > 300:
                    arrow_name = 'right'
                print('[ARROW] {}'.format(arrow_name))
        else:
            arrow_name = 'None'

        cv.putText(dst, arrow_name, (10,400), cv.FONT_HERSHEY_SIMPLEX,1, (0,0,255),2)
        
        cv.imshow('dst', dst)

        cv.drawContours(frame, [posible_arrow], 0, (0,255,0), 2)

    return arrow_name

