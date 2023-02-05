#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np
import cv2, math
import rospy, rospkg, time
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped

from cv_bridge import CvBridge, CvBridgeError
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os
import random

import time


class Line_Detect:
    def __init__(self):
        self.image = cv2.imread('31.png', cv2.IMREAD_COLOR)
        #self.image = np.empty(shape=[0])
        self.Height, self.Width, self.Channel = self.image.shape

        self.ROI_row_percent = 0.4 # 하단 x0%
        self.ROI_col_percent = 0.8 # 중앙 80% 를 보고 싶다.

        self.Row_ROI_from = self.Height - int(self.Height*self.ROI_row_percent)
        self.Row_ROI_to = self.Height
        self.Col_ROI_from = int(self.Width * (1-self.ROI_col_percent)*(0.5))
        self.Col_ROI_to = int(self.Width-((1-self.ROI_col_percent)*(0.5)*self.Width))
        
        self.estimated_center = 0.0
        self.Center_pos = int(self.Width * 0.5)
        
        self.ROI_light_row_percent = 0.8 # 상단 50%
        self.ROI_light_col_percent = 1 # 중앙 50%
        self.Row_light_ROI_from = 0
        self.Row_light_ROI_to = int(self.Height*self.ROI_light_row_percent)
        
        self.Col_light_ROI_from = int(self.Width * (1-self.ROI_light_col_percent)*(0.5))
        self.Col_light_ROI_to = int(self.Width-((1-self.ROI_light_col_percent)*(0.5)*self.Width))
        
        self.Col_force_right_from = int(self.Width * 0)
        self.Col_force_right_to = int(self.Width)
        
        ### for test real
        self.ROW_ROI_real_under_ignore_percent = 0.1   # 하단 5%는 안 보련다
        self.ROW_ROI_real_percent = 0.25         # 하단 
        
        self.Height_with_ignored = self.Height*(1-self.ROW_ROI_real_under_ignore_percent)
        
        self.Row_ROI_from_real = int(self.Height_with_ignored - int(self.Height_with_ignored*self.ROW_ROI_real_percent))
        self.Row_ROI_to_real = int(self.Height_with_ignored)
        
        self.ROI_col_percent_real = 0.90 # 중앙 80% 를 보고 싶다.
        self.Col_ROI_from_real = int(self.Width * (1-self.ROI_col_percent_real)*(0.5))
        self.Col_ROI_to_real = int(self.Width-((1-self.ROI_col_percent_real)*(0.5)*self.Width))
        
        
        self.estimated_center_left = 0.0
        self.estimated_center_right = 0.0
        
        clear = lambda : os.system('clear')
        clear()
        print ("----- Xycar TEST -----")        
        self.start()
        

    """
    def average_slope_intercept(self, image, lines):
        left_fit = []
        right_fit = []
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            parameter = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameter[0]
            intercept = parameter[1]
            if slope < 0:
                left_fit.append((slope, intercept))
            else:
                right_fit.append((slope, intercept))
        left_fit_average =np.average(left_fit, axis=0)
        right_fit_average = np.average(right_fit, axis =0)
        left_line = self.make_coordinates(image, left_fit_average)
        right_line = self.make_coordinates(image, right_fit_average)

        return np.array([[left_line, right_line]])
    
    def show_lines(self, image, lines) : 
        lines_image = np.zeros_like(image)
        if lines is not None :
            for i in range(len(lines)):
                for x1,y1,x2,y2 in lines[i]:
                    cv2.line(lines_image,(x1,y1),(x2,y2),(255,0,0), 10 )
        return lines_image
    
    def make_coordinates(self, image, line_parameters):
        slope, intercept = line_parameters
        y1 = image.shape[0]
        y2 = int(y1*(3/5))
        x1 = int((y1- intercept)/slope)
        qqx2 = int((y2 - intercept)/slope)
        return np.array([x1, y1, x2, y2])
    """
    """
        안쪽    x, y, w, h : 305, 3, 172, 141
        
        바깥쪽  x, y, w, h : 201, 3, 392, 141
        
    """
    
    def wrapping(self, image):
        (h, w) = (self.Height, self.Width)

        #source = np.float32([[w // 2 - 30, h * 0.53], [w // 2 + 60, h * 0.53], [w * 0.3, h], [w, h]])
        #destination = np.float32([[0, 0], [w-350, 0], [400, h], [w-150, h]])
        
        source = np.float32([[290, 3], [200,3+141], [304+190,3], [201+420,3+150]])
        destination = np.float32([[0, 0], [0, 1000], [1000, 0], [1000, 1000]])
        

        transform_matrix = cv2.getPerspectiveTransform(source, destination)
        minv = cv2.getPerspectiveTransform(destination, source)
        _image = cv2.warpPerspective(image, transform_matrix, (w, h))

        return _image, minv
    
    
    def crosswalk_image_create(self, image, current, window=0):
        margin = 100
        window_height = 50
        w = window

        if(window != 0):
            y_low = image.shape[0] - window_height
            y_high = y_low + window_height
            if(y_high > image.shape[0]):
                y_high = image.shape[0]
        else:
            y_low = w * window_height
            y_high = (w + 1) * window_height
            
        x_low = current - margin
        x_high = current + margin

        crosswalk_image = image[y_low:y_high, x_low:x_high]
        
        return crosswalk_image
    
    
    def crosswalk_histogram(self, image, ratio, verbose=False):
        histogram = np.sum(image[image.shape[0]//2:, :], axis=0)
        histogram_nonzero = np.asarray(np.nonzero(histogram))
        
        histogram_size = (float)(len(histogram))
        histogram_nonzero_size = (float)(histogram_nonzero.size)
        nonzero_ratio = (histogram_nonzero_size/histogram_size)
        
        if(verbose):
            print("         crosswalk ratio: {}".format(nonzero_ratio))
        
        if(nonzero_ratio > ratio): 
            return True
        else:
            return False

    
    def plothalfhistogram(self, image):
        histogram = np.sum(image[image.shape[0]//2:, :], axis=0)
        midpoint = np.int(histogram.shape[0]/2)

        base = np.argmax(histogram[:])

        print("half base {}".format(base))
        return base

    def plothistogram(self, image):
        histogram = np.sum(image[image.shape[0]//2:, :], axis=0)
        midpoint = np.int(histogram.shape[0]/2)
        
        leftbase_min = np.int(histogram.shape[0]*0.15)
        rightbase_min = np.int(histogram.shape[0]*0.98)
        
        leftbase = np.argmax(histogram[leftbase_min:midpoint]) + leftbase_min
        rightbase = np.argmax(histogram[midpoint:rightbase_min]) + midpoint
        
        histogram_nonzero = np.asarray(np.nonzero(histogram))
        histogram_size = (float)(len(histogram))
        histogram_nonzero_size = (float)(histogram_nonzero.size)
        ratio = (histogram_nonzero_size/histogram_size)

        print("leftbase rightbase {}, {}".format(image.shape, rightbase))
        return leftbase, rightbase, ratio

    def slide_window_search(self, binary_warped, left_current, right_current):
        out_img = np.dstack((binary_warped, binary_warped, binary_warped))

        nwindows = 4
        window_height = np.int(binary_warped.shape[0] / nwindows)
        nonzero = binary_warped.nonzero()  # 선이 있는 부분의 인덱스만 저장 
        #print("nonzero: {}".format(nonzero))
        nonzero_y = np.array(nonzero[0])  # 선이 있는 부분 y의 인덱스 값
        nonzero_x = np.array(nonzero[1])  # 선이 있는 부분 x의 인덱스 값 
        margin = 100
        minpix = 50
        left_lane = []
        right_lane = []
        color = [0, 255, 0]
        thickness = 2

        # I made here
        
        left_x = []
        left_y = []
        right_x = []
        right_y = []
        #left_slope = []
        #right_slope = []

        left_x_second_wid = []
        left_y_second_wid = []
        right_x_second_wid = []
        right_y_second_wid = []
        
        h_line_x = []
        for w in range(nwindows):
            win_y_low = binary_warped.shape[0] - (w + 1) * window_height  # window 윗부분
            win_y_high = binary_warped.shape[0] - w * window_height  # window 아랫 부분
            win_xleft_low = left_current - margin  # 왼쪽 window 왼쪽 위
            win_xleft_high = left_current + margin  # 왼쪽 window 오른쪽 아래
            win_xright_low = right_current - margin  # 오른쪽 window 왼쪽 위 
            win_xright_high = right_current + margin  # 오른쪽 window 오른쪽 아래

            # 초록 윈도우 그리는 부분
            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), color, thickness)
            cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), color, thickness)
        
            good_left = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & (nonzero_x >= win_xleft_low) & (nonzero_x < win_xleft_high)).nonzero()[0]
            good_right = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & (nonzero_x >= win_xright_low) & (nonzero_x < win_xright_high)).nonzero()[0]
            left_lane.append(good_left)
            right_lane.append(good_right)
            # cv2.imshow("oo", out_img)

            if len(good_left) > minpix:
                left_current = np.int(np.mean(nonzero_x[good_left]))
            if len(good_right) > minpix:
                right_current = np.int(np.mean(nonzero_x[good_right]))
                
            ### I customized here
            # current left를 이용한 line 그어보기
            cv2.line(out_img, (left_current, win_y_low), (left_current, win_y_high), (0, 0, 255), 1)
            
            # left x y 그려보기
            leftx_test = nonzero_x[np.concatenate(left_lane)]
            lefty_test = nonzero_y[np.concatenate(left_lane)]
            rightx_test = nonzero_x[np.concatenate(right_lane)]
            righty_test = nonzero_y[np.concatenate(right_lane)]
            #print("leftx test size, lefty test size: {} {}".format(len(leftx_test), len(lefty_test)))
            
            try:
                lx_1, lx_2, ly_1, ly_2 = leftx_test[0], leftx_test[len(leftx_test)-1], lefty_test[0], lefty_test[len(lefty_test)-1];
                rx_1, rx_2, ry_1, ry_2 = rightx_test[0], rightx_test[len(rightx_test)-1], righty_test[0], righty_test[len(righty_test)-1];
            except:
                return -1
            
            cv2.line(out_img, (lx_1, ly_1), (lx_2, ly_2), (0, 0, 255), 3)
            cv2.line(out_img, (rx_1, ry_1), (rx_2, ry_2), (0, 0, 255), 3)
            
            # 각 윈도우에서 대표적인 값들을 뽑아 전체의 라인을 그려보자
            
            
            if(w == 2):
                left_x_second_wid.append(lx_1)
                left_x_second_wid.append(lx_2)
                
                left_y_second_wid.append(ly_1)
                left_y_second_wid.append(ly_2)
                
                right_x_second_wid.append(rx_1)
                right_x_second_wid.append(rx_2)
                
                right_y_second_wid.append(ry_1)
                right_y_second_wid.append(ry_2)
            
            if(w == 1 or w == 3):
                left_x.append(lx_2)
                left_y.append(ly_2)
                right_x.append(rx_2)
                right_y.append(ry_2)
            
                
            # 값이 한 윈도우만이라도 튀면 제대로 계산되지 않는다.
            
        
        # I made here
        # 이정도면 쓸만한 라인을 검출하는 것 같다(ROBUST) - 라인들이 구석에서 인식되면 튄다.
        cv2.line(out_img, (left_x[0], left_y[0]), (left_x[1], left_y[1]), (0, 255, 255), 5) # yellow
        cv2.line(out_img, (right_x[0], right_y[0]), (right_x[1], right_y[1]), (0, 255, 255), 5)
        
        # 다시, 하단 2번째 윈도우를 기준으로
        cv2.line(out_img, (left_x_second_wid[0], left_y_second_wid[0]), (left_x_second_wid[1], left_y_second_wid[1]), (255, 255, 0), 5)
        cv2.line(out_img, (right_x_second_wid[0], right_y_second_wid[0]), (right_x_second_wid[1], right_y_second_wid[1]), (255, 255, 0), 5)
        

        #_slope = float(left_y[1] - left_y[0]) / float(left_x[1] - left_x[0])
        
        
        #print(">>>>>>>>sLOPE <<<<<<<<<<", _slope) 
        # 3.jpg: -0.5813953488372093
        # 1.jpg: -1.6
        
        ## 끝 부분의 중앙과 시작 부분의 중앙의 차이점으로 각도를 구하자
        #start_mid = int((left_x[0] + right_x[0]) / 2)
        #end_mid = int((left_x[1] + right_x[1]) / 2)
        start_mid = int((left_x_second_wid[0] + right_x_second_wid[0]) / 2)
        end_mid = int((left_x_second_wid[1] + right_x_second_wid[1]) / 2)
        
        
        cv2.circle(out_img, (start_mid, int((left_y_second_wid[0]+right_y_second_wid[0])/2)), 5, (255, 255, 0), -1)   #car center rectangle
        cv2.circle(out_img, (end_mid, int((left_y_second_wid[1]+right_y_second_wid[1])/2)), 5, (255, 255, 0), -1)   #car center rectangle

        start_end_interval = (end_mid - start_mid)
        #print(">>>>>>>>>>>>diff: ", start_end_interval) # 1.jpg = 4, 3.jpg = 95
        # 양수 커질수록 = 오른쪽으로 회전 필요함
        #start_end_interval = left_x[1] - left_x[0]
        
        self.estimated_center = start_end_interval

  
        left_lane = np.concatenate(left_lane)  # np.concatenate() -> array를 1차원으로 합침
        right_lane = np.concatenate(right_lane)

        out_img[nonzero_y[left_lane], nonzero_x[left_lane]] = [255, 0, 0]
        out_img[nonzero_y[right_lane], nonzero_x[right_lane]] = [0, 0, 255]
        
        cv2.imshow("out_img", out_img)

        return

    def draw_lane_lines(self, original_image, warped_image, Minv, draw_info):
        left_fitx = draw_info['left_fitx']
        right_fitx = draw_info['right_fitx']
        ploty = draw_info['ploty']

        warp_zero = np.zeros_like(warped_image).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        mean_x = np.mean((left_fitx, right_fitx), axis=0)
        pts_mean = np.array([np.flipud(np.transpose(np.vstack([mean_x, ploty])))])

        cv2.fillPoly(color_warp, np.int_([pts]), (216, 168, 74))
        cv2.fillPoly(color_warp, np.int_([pts_mean]), (216, 168, 74))

        newwarp = cv2.warpPerspective(color_warp, Minv, (original_image.shape[1], original_image.shape[0]))
        cv2.imshow("newwarp", newwarp)
        result = 0
        #result = cv2.addWeighted(original_image, 1, newwarp, 0.4, 0)

        return pts_mean, result
    
    ####################################################################################
    
    def move_with_only_with_one_line(self, binary_warped, current, name='none'):
        out_img = np.dstack((binary_warped, binary_warped, binary_warped))

        nwindows = 4
        window_height = np.int(binary_warped.shape[0] / nwindows)
        nonzero = binary_warped.nonzero()  # 선이 있는 부분의 인덱스만 저장 
        #print("nonzero: {}".format(nonzero))
        nonzero_y = np.array(nonzero[0])  # 선이 있는 부분 y의 인덱스 값
        nonzero_x = np.array(nonzero[1])  # 선이 있는 부분 x의 인덱스 값 
        margin = 200
        minpix = 50
        oneline_lane = []
        #right_lane = []
        color = [0, 255, 0]
        thickness = 2
        #print("current: {}".format(current))
        # I made here
        
        oneline_x = []
        oneline_y = []
        h_line_x =[]
        for w in range(nwindows):
            win_y_low = binary_warped.shape[0] - (w + 1) * window_height  # window 윗부분
            win_y_high = binary_warped.shape[0] - w * window_height  # window 아랫 부분
            win_x_low = current - margin  # 왼쪽 window 왼쪽 위
            win_x_high = current + margin  # 왼쪽 window 오른쪽 아래

            # 초록 윈도우 그리는 부분
            cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), color, thickness)

            good_line = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) & (nonzero_x >= win_x_low) & (nonzero_x < win_x_high)).nonzero()[0]
            #print("good line {}".format(good_line))
            oneline_lane.append(good_line)
            # cv2.imshow("oo", out_img)

            if len(good_line) > minpix:
                current = np.int(np.mean(nonzero_x[good_line]))
            
                
            ### I customized here
            # current left를 이용한 line 그어보기
            cv2.line(out_img, (current, win_y_low), (current, win_y_high), (155, 155, 155), 2)
            
            # left x y 그려보기
            onelinex_test = nonzero_x[np.concatenate(oneline_lane)]
            oneliney_test = nonzero_y[np.concatenate(oneline_lane)]

            ox_1, ox_2, oy_1, oy_2 = onelinex_test[0], onelinex_test[len(onelinex_test)-1], oneliney_test[0], oneliney_test[len(oneliney_test)-1];
            
            cv2.line(out_img, (ox_1, oy_1), (ox_2, oy_2), (0, 0, 255), 3)

            if(w == 2):
                oneline_x.append(ox_1)
                oneline_x.append(ox_2)
                
                oneline_y.append(oy_1)
                oneline_y.append(oy_2)

            """### 노란 가로선 인식하기
            if(w == 0):
                # win_x 의 중앙, current로부터 오른쪽의 점들로 선을 그려서 기울기를 보자.
                h_line_x.append([nonzero_x > current])
                #h_line_y.append([nonzero_y])
                
            cv2.line(out_img, (h_line_x[0], int(window_height*0.5)), (h_line_x[len(h_line_x)-1], int(window_height*0.5)), (0, 0, 255), 3)    
            """
            
            # 값이 한 윈도우만이라도 튀면 제대로 계산되지 않는다.
            
            #print("good_left good_right: {}, {}".format(good_left, good_right))
        #print('\n\n')
        
        # I made here
        # 이정도면 쓸만한 라인을 검출하는 것 같다(ROBUST) - 라인들이 구석에서 인식되면 튄다.
        # 다시, 하단 2번째 윈도우를 기준으로
        cv2.line(out_img, (oneline_x[0], oneline_y[0]), (oneline_x[1], oneline_y[1]), (255, 255, 0), 5)
        
        if name == 'none': 
            cv2.imshow("out_img", out_img)
        else:
            cv2.imshow(name, out_img)
            
        # 끝 부분의 중앙과 시작 부분의 중앙의 차이점으로 각도를 구하자
        start_mid = int(oneline_x[0])
        end_mid = int(oneline_x[1])

        start_end_abs = (end_mid - start_mid)
        #print(">>>>>>>>>>>>diff: ", start_end_abs) # 1.jpg = 4, 3.jpg = 95
        # 양수 커질수록 = 오른쪽으로 회전 필요함
        
        start_end_interval = (end_mid - start_mid)
        
        self.estimated_center = start_end_interval
        
        return
    
    def detecting_traffic_light(self, edge_img):
        light_roi = edge_img[self.Row_light_ROI_from:self.Row_light_ROI_to, self.Col_light_ROI_from:self.Col_light_ROI_to]
        

        _light_roi_h, _light_roi_w = light_roi.shape
        
        _lights = cv2.HoughLinesP(light_roi, 1, math.pi/180,30,10,10)
        
        _, thresh = cv2.threshold(light_roi, 75, 110, cv2.THRESH_BINARY)

        #left_base, right_base, crosswalk_ratio = self.plothistogram(thresh)
        
        
        
        cv2.imshow("lights", thresh)

        return
    
    
    def detect_stopline_contour(self, cal_image, x_l, y_l, x_r, y_r):
        #blur = cv2.GaussianBlur(cal_image, (5, 5), 0)
        #_, _, B = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2LAB))
        #_, lane = cv2.threshold(B, low_threshold_value, 255, cv2.THRESH_BINARY)
        # save_img = cal_image.copy()
        
        _w = y_r - y_l
        _h = x_r - x_l
        
        contours, _ = cv2.findContours(cal_image, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        self.x, self.y, self.w, self.h = [], [], [], []
        self.stop = False
        cnt = 0
        for cont in contours:
            length = cv2.arcLength(cont, True)
            
            area = cv2.contourArea(cont)
            print("length, area : {}, {}".format(length, area))
            (x, y, w, h) = cv2.boundingRect(cont)
            center = (x + int(w/2), y + int(h/2))
            width, height = cal_image.shape
            
            #print("x, y, w, h:{}, {}, {}, {}".format(x, y, w, h))
            #print("img shape: {}, {}".format(cal_image.shape[0], cal_image.shape[1]))
            
            # 여기서 x는 가로 축이다
            if(x > 0.52*cal_image.shape[1] and y < 0.19*cal_image.shape[0] and self.stop == False):
                cnt += 1
                if(cnt > 5):
                    print("stop")
                    self.stop = True
                    
                #cv2.rectangle(cal_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                self.x.append(int(x))
                self.y.append(int(y))
                self.w.append(int(w))
                self.h.append(int(h))
            
            if not ((area > 1000) and (length > 400)):
                continue
            if len(cv2.approxPolyDP(cont, length*0.02, True)) < 2:
                continue
        """            

            if 200 <= center[0] <= (width - 200):
                cv2.rectangle(cal_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                print("stopline")
        
        print("number of cnt: ", cnt)      
        cv2.imshow("save_img", save_img)   
        """
                
    def calibrate_image(frame, mtx, dist, cal_mtx, cal_roi):
        height, width, _ = frame.shape
        tf_image = cv2.undistort(frame, mtx, dist, None, cal_mtx)
        x, y, w, h = cal_roi
        tf_image = tf_image[y:y+h, x:x+w]
        
        return cv2.resize(tf_image, (width, height))
    #################################################################33
        
    def start(self):
        
        
        img = self.image.copy()
        #img = img[self.Row_ROI_from:self.Row_ROI_to, self.Col_ROI_from:self.Col_ROI_to]
        
        blur = cv2.GaussianBlur(img, (5, 5), 0)
        ## ver 1
        H, L, S = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HLS))
        _, _L = cv2.threshold(L, 75, 110, cv2.THRESH_BINARY)
        edge_img = cv2.Canny(np.uint8(_L), 60, 70)
        
        ## ver 2
        edge_img_g = cv2.Canny(np.uint8(blur), 60, 70)
        
        ## ver 3
        _, L = cv2.threshold(L, 120, 225, cv2.THRESH_BINARY)
        edge_img_white = L
        edge_img_white_canny = cv2.Canny(np.uint8(L), 60, 70)
        
        ## ver 4 yellow.py
        yL, yA, yB = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2LAB))
        
        #_, yL = cv2.threshold(yL, 140, 255, cv2.THRESH_BINARY)
        _, yB = cv2.threshold(yB, 140, 255, cv2.THRESH_BINARY)
        #yL_canny = cv2.Canny(np.uint8(yB), 60, 70)
        yB_canny = cv2.Canny(np.uint8(yB), 60, 70) # 140,255 B 로 갑시다
        
        ### img test show
        
        image_compare1 = np.hstack((edge_img_g, edge_img))
        image_compare1 = cv2.resize(image_compare1, (2080, 580))
        image_compare2 = np.hstack((edge_img_white, edge_img_white_canny))
        image_compare2 = cv2.resize(image_compare1, (2080, 580))
        
        cv2.imshow("white from HLS(left), white from HLS with canny(right)", image_compare2)
        
        
        '''
        _name = "gaussian(left) and L from HLS(right)"
        _path = "/home/ubuntu/2022-2_lectures/xycar_ws/src/RobotVision_xycar/assignment1/src/image/real_image_compare/" + _name + ".jpg"
        cv2.imwrite(_path, image_compare1)
        cv2.imshow("gaussian(left) and L from HLS(right)", image_compare1) # with gaussian, HLS 비교
        
        image_compare2 = np.hstack((edge_img_white, edge_img_white_canny))
        image_compare2 = cv2.resize(image_compare1, (2080, 580))
        _name = "white from HLS(left), white from HLS with canny(right)"
        cv2.imwrite("/home/ubuntu/2022-2_lectures/xycar_ws/src/RobotVision_xycar/assignment1/src/image/real_image_compare/" + _name + ".jpg", image_compare2)
        
        cv2.imshow("white from HLS(left), white from HLS with canny(right)", image_compare2)'''
        
        ### 
        # Col_ROI_from
        _line_roi = edge_img[self.Row_ROI_from_real:self.Row_ROI_to_real, self.Col_ROI_from_real:self.Col_ROI_to_real]
        #_line_roi_yL = yL_canny[self.Row_ROI_from_real:self.Row_ROI_to_real, self.Col_ROI_from_real:self.Col_ROI_to_real]
        _line_roi_yB = yB_canny[self.Row_ROI_from_real:self.Row_ROI_to_real, self.Col_ROI_from_real:self.Col_ROI_to_real]
        
        yB = yB[self.Row_ROI_from_real:self.Row_ROI_to_real, self.Col_ROI_from_real:self.Col_ROI_to_real]
        edge_img_white = edge_img_white[self.Row_ROI_from_real:self.Row_ROI_to_real, self.Col_ROI_from_real:self.Col_ROI_to_real]
       
        
        cv2.imshow("_line_roi", _line_roi)
        #cv2.imshow("_line_roi_yL", _line_roi_yL)
        cv2.imshow("_line_roi_yB", _line_roi_yB)
        
        
        #### riding test
        
        #line_roi = edge_img[self.Row_ROI_from:self.Row_ROI_to, self.Col_ROI_from:self.Col_ROI_to]
        line_roi = edge_img[self.Row_ROI_from:self.Row_ROI_to, self.Col_force_right_from:self.Col_force_right_to]
        
        _line_roi_h, _line_roi_w = line_roi.shape
        all_lines = cv2.HoughLinesP(line_roi, 1, math.pi/180,30,10,10)
        
        # warpping
        #_gray = cv2.cvtColor(line_roi, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(line_roi, 75, 110, cv2.THRESH_BINARY)
        #cv2.imshow("thresh", thresh)
        
        #wrapped_img, _back = self.wrapping(line_roi)
        #cv2.imshow("wrapped_img", wrapped_img)
        
        # histogram of white line
        clear = lambda : os.system('clear')
        clear()
        _left_base, _right_base, _ = self.plothistogram(_line_roi_yB)
        
        for i in range(10):
            print(">>>>>>>>>> left_base, right_base: {} {}".format(_left_base, _right_base))

        #self.detecting_traffic_light(edge_img)
        
        return_value = self.slide_window_search(_line_roi_yB, _left_base, _right_base)
        
        
        ### half base - left
        _line_roi_lefthalf = yB[:, :int(yB.shape[1]*0.85)]  # row, col
        _lbase = self.plothalfhistogram(_line_roi_lefthalf)
        
                
        ### half base - right
        _line_roi_righthalf = edge_img_white[:, int(edge_img_white.shape[1]*0.4):int(edge_img_white.shape[1])]  # row, col
        _rbase = self.plothalfhistogram(_line_roi_righthalf)
        
        cv2.imshow("_line_roi_lefthalf", _line_roi_lefthalf)
        print("half l test, base:{}".format(_lbase))
        
        cv2.imshow("_line_roi_righthalf", _line_roi_righthalf)
        print("half r test, base:{}".format(_rbase))
        
        try:
            self.move_with_only_with_one_line(_line_roi_lefthalf, _lbase, name='left')
            self.estimated_center_left = self.estimated_center
        except:
            print("left line not detected")
        
        try:
            self.move_with_only_with_one_line(_line_roi_righthalf, _rbase, name='right')
            self.estimated_center_right = self.estimated_center
        except:
            print("right line not detected")          
        
        print("center left, right: {}, {}".format(self.estimated_center_left, self.estimated_center_right))
        self.angled = (self.estimated_center_left + self.estimated_center_right) * 0.15
        print("angle: ", self.angled)
        
        H, L, S = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HLS))
        _, _L = cv2.threshold(L, 120, 225, cv2.THRESH_BINARY)
        line_roi = _L[self.Row_ROI_from_real:self.Row_ROI_to_real , self.Col_ROI_from_real:self.Col_ROI_to_real]
        #### crosswalk check
        
        crosswalk_img = self.crosswalk_image_create(line_roi, int(line_roi.shape[1]*0.85), window=0)
        cv2.imshow("crosswalk checker", crosswalk_img)
                
        self.crosswalk_histogram(crosswalk_img, 0.7, verbose=True)
        
        
        
        """lb, rb = self.plothistogram(_line_roi_yB)
        self.move_with_only_with_one_line(yB, rb)"""
        
        
        """### test for decting stop line
        
        self.detect_stopline_contour(L.copy(), 150)
        
        for i in (range(len(self.x) - 1)):
            cv2.rectangle(img, (self.x[i], self.y[i]), (self.x[i] + self.w[i]-1, self.y[i] + self.h[i]-1), (0, 0, 200), 3)
        cv2.imshow("img", img)
        """
        
        
        
        """
        if(abs(self.estimated_center) > 25):
            try:
                self.move_with_only_with_one_line(thresh, left_base)
            except:
                self.move_with_only_with_one_line(thresh, right_base)
                
            self.angled = -self.estimated_center*0.16
            
        elif(self.estimated_center < -25):
            try:
                self.move_with_only_with_one_line(thresh, right_base)
            except:
                self.move_with_only_with_one_line(thresh, left_base)
                
            self.angled = self.estimated_center*0.16
        else:
            return_value = self.slide_window_search(thresh, left_base, right_base)
            if(return_value == -1):
                print("estimated go forward but not detected two lines")
                try:
                    self.move_with_only_with_one_line(thresh, right_base)
                except:
                    self.move_with_only_with_one_line(thresh, left_base)
                    
            self.angled = -self.estimated_center*0.1
            
        if(self.angled > 50):
            self.angled = 30
        elif(self.angled < -50):
            self.angled = -30    
        
        print("self.angled: {}".format(self.angled))    """
        
        """
        긁어온 함수 테스트 - cv2.poly 안되서 일단 보류중
        #averaged_lines = self.average_slope_intercept(img, all_lines)
        #lines_image = self.show_lines(img, averaged_lines)
        
        #combine_image = cv2.addWeighted(img, 0.8, lines_image, 1, 1)
        """
        #####################################################################################3
        """
        # draw lines in ROI area
        line_img = edge_img.copy()
        
        for line in all_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_roi, (x1, y1), (x2, y2), (255, 255, 255), 3)
            # 라인을 그리는 함수.
        
        # 11.17
        
            
        for line in all_lines:
        # calculate slope and do filtering
        # new lines에는 검출된 차선이 기울어진 경우에만 xstart~yend 데이터가 들어간다.
            slopes = []
            new_lines = []
            for line in all_lines:
                x1, y1, x2, y2 = line[0]
                # xstart xend가 같으면 기울어지지 않은 것이다.
                if (x2 - x1) == 0:
                    slope = 0
                else:
                    slope = float(y2-y1) / float(x2-x1)
                if 0.1 < abs(slope) < 10:
                    slopes.append(slope)
                    new_lines.append(line[0])
                    #cv2.line(line_roi, (x2, 0), (x1, _line_roi_h), (255, 255, 255), 3) # 
                    
                if abs(slope) >= 3:
                    print("detected line's slope has too large angle")
                    #cv2.line(self.image, (x1, 0+240), (x2, 240+240), (0, 0, 255), 3) # RED
                    #cv2.line(line_roi, (x1, 0+240), (x2, 240+240), (0, 0, 255), 3) # RED

            # divide lines left and right
            left_lines = []
            right_lines = []
            for j in range(len(slopes)):
                # 저장된 직선과 기울기 정보를 다시 load
                Line = new_lines[j]
                slope = slopes[j]
                x1, y1, x2, y2 = Line
                # 여기까진 위에 코드랑 같다.
                
                # 왼쪽 상단 x,y가 0,0이다.
                # 픽셀단위 기준으로 left, right 구분 - 기울기가 음수면 왼쪽. 배열느낌이므로...
                # 아래로 내려갈수록 y가 커지고, 오른쪽으로 갈 수록 x가 커진다.
                if (slope < 0) and (x2 < 320):
                    left_lines.append([Line.tolist()])
                    cv2.line(line_roi, (x1, y1), (x2, y2), (255, 255, 255), 20)
                elif (slope > 0) and (x1 > 320):
                    right_lines.append([Line.tolist()])
                    cv2.line(line_roi, (x1, y1), (x2, y2), (255, 255, 255), 20)

            # draw right&left lines in different color
            # color = BGR이다
            #line_img = line_roi.copy()
            for line in left_lines:
                x1, y1, x2, y2 = line[0]
                #cv2.line(self.image, (x1+160, y1+240), (x2+160, y2+240), (0, 0, 255), 2) # RED
                #cv2.line(edge_img, (x1+160, y1+240), (x2+160, y2+240), (0, 0, 255), 2) # RED
                cv2.line(line_roi, (x1, y1), (x1, y2), (255, 255, 255), 3) # 
                
                
            for line in right_lines:
                x1, y1, x2, y2 = line[0]
                # 240보다 줄어들면 라인이 왼쪽 위로 올라가게된다.
                cv2.line(self.image, (x1, y1+240), (x2, y2+240), (0, 255, 0), 2) # G
                cv2.line(edge_img, (x1, y1+240), (x2, y2+240), (0, 255, 0), 2) # G
            if right_lines is None:
                print("Right line not detected")
                
            # get average left-line
            x_sum, y_sum, m_sum = 0.0, 0.0, 0.0
            size = len(left_lines)
            b_left, m_left = 0, 0.1
            for line in left_lines:
                # 맨 왼쪽 흰색 선도 잡혀서 그런가?
                x1, y1, x2, y2 = line[0]
                #print("x1, y1, x2, y2: {}, {}, {}, {}".format(x1, y1, x2, y2))
                #if(x1 > 50):
                    
                x_sum += x1 + x2 + 160 + 160
                y_sum += y1 + y2
                m_sum += float(y2 - y1) / float(x2 - x1)
                x_avg = x_sum / (size * 2)                  # x 좌표의 평균
                y_avg = y_sum / (size * 2)
                m_left = m_sum / size                       # 기울기 값의 평균
                b_left = y_avg - m_left * x_avg             # 직선 그래프의 절편 평균 값
                # y = mx + b, b = y - mx
                
            x1, x2 = 0, 0
            x1 = int((0.0 - b_left) / m_left)           # x = (y-b)/m 인데
            x2 = int((240.0 - b_left) / m_left)            # 왜 0과 240으로 했을까? rmsid dladmlfh
            #print("x avg, y avg, b_left: ({}, {}, {})".format(x_avg, y_avg, b_left))
            print("left x 1 , x 2 ({}, {})".format(x1, x2))
            
            left_center = int((x1+x2)/2)
            cv2.line(self.image, (x1, 0+240), (x2, 240+240), (255, 255, 0), 2)
            # 평균적인 좌측 선을 (255,0,0)으로 나타낸 것
            #print("left line start ({},{}), end ({},{})".format(x1, 0+240, x2, 240+240))
            #cv2.line(self.image, (0,0), (100, 500), (0,255,0), 2)
            
            x_sum, y_sum, m_sum = 0.0, 0.0, 0.0
            size = len(right_lines)
            b_right, m_right = 0, 0.1
            for line in right_lines:
                
                x1, y1, x2, y2 = line[0]
                slope = float(y2-y1) / float(x2-x1)
                #print("right slope: {}".format(slope))
                #print("right x1, y1, x2, y2: {}, {}, {}, {}".format(x1, y1, x2, y2))
                x_sum += x1 + x2
                y_sum += y1 + y2
                m_sum += float(y2 - y1) / float(x2 - x1)
                x_avg = x_sum / (size * 2)
                y_avg = y_sum / (size * 2)
                m_right = m_sum / size
                b_right = y_avg - m_right * x_avg
            
            x1, x2 = 0, 0
            x1 = int((0.0 - b_right) / m_right)
            x2 = int((240.0 - b_right) / m_right)
            right_center = int((x1+x2)/2)
            cv2.line(self.image, (x1, 0+240), (x2, 240+240), (255, 255, 0), 2)
            #print("right x 1 , x 2 ({}, {})".format(x1, x2))
            # 평균적인 우측 선을 (255,0,0) 으로 나타낸 것
            
            line_center = int((left_center + right_center)/2)
            #print("estimated line center {}, rectangle pos: {}".format(line_center, int((325+315)/2)))
            # 계산된 line center가 현재 약 240정도로 320보다 상당히 좌측이다. 초록색으로 박스 그려진게...
            #Car center
            cv2.line(self.image, (0, 360), (640, 360), (0, 255, 255), 1) # 가로선
            cv2.line(self.image, (self.Center_pos, 0), (self.Center_pos, self.Width), (0, 255, 255), 1) # 세로선_1
            cv2.line(self.image, (320, 0), (320, 480), (0, 255, 255), 1) # 세로선_2
            
            cv2.rectangle(self.image, (315, 355), (325, 365), (0, 0, 255), 2)
            # line과 동일하게 시작점 좌표, 종료지점 좌표로 나타냄.
                
            #_angle = line_center - 320
            #self.angled = 
            
            
            #_x, _y, _w, _h = cv2.selectROI("location", line_roi, False)
            #print("x, y, w, h : {}, {}, {}, {}".format(_x, _y, _w, _h) )
        """    
        #cv2.imshow("line_img", line_img)
        #cv2.imshow("self_image", self.image)
        #cv2.imshow("line_roi", line_roi)    
        if cv2.waitKey() == ord('q'):
            cv2.destroyAllWindows()
        """
        안쪽    x, y, w, h : 305, 3, 172, 141
        바깥쪽  x, y, w, h : 201, 3, 392, 141
        
        """
        
if __name__ == '__main__':
    TEST = Line_Detect()