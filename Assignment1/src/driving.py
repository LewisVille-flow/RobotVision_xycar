#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np
import cv2, math
import rospy, rospkg, time
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt

from cv_bridge import CvBridge, CvBridgeError
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os
import random

import time


def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


class LowPassFilter:
    def __init__(self, cutoff_freq, ts):
        self.ts = ts
        self.cutoff_freq = cutoff_freq
        self.pre_out = 0.0
        self.tau = self.filter_coef() 
        
    def filter_coef(self):
        w_cut = 2*np.pi*self.cutoff_freq
        return 1/w_cut
        
    def filter(self, data):
        out = (self.tau * self.pre_out + self.ts * data) / (self.tau + self.ts)
        self.pre_out = out
        return out
    
    
class Robotvisionsystem:
    def __init__(self):
        self.image = np.empty(shape=[0])
        self.realimage = np.empty(shape=[0]) 
        self.bridge = CvBridge() 
        self.motor = None 
        self.angled = 0
        self.speed = 0.1
        self.stop = 0
        self.estimated_center = 0

        self.CAM_FPS = 30
        self.WIDTH, self.HEIGHT = 640, 480
        
        #self.crosswalk_flag = False
        self.crosswalk_cross_flag = False
        self.stage_num = 1
        
        self.lpf = LowPassFilter(cutoff_freq=0.5, ts = 0.03)
        self.center_value_data = []
        self.filtered_value_data = []
        rospy.init_node('driving')
        
        #### For Real World driving!( real xycar )
        #self.motor = rospy.Publisher('/xycar_motor', xycar_motor, queue_size=1)
        #self.real_image = rospy.Subscriber('/usb_cam/image_raw/compressed',CompressedImage, self.realimg_callback)

        #### For Simulator driving! (simul xycar)
        self.unitymotor = rospy.Publisher('/unitymotor', PoseStamped, queue_size=1)
        self.unity_img = rospy.Subscriber('/unitycamera', CompressedImage , self.img_callback)

        clear = lambda : os.system('clear')
        clear()
        
        print ("----- Xycar self driving -----")
        self.start()    

    def img_callback(self, data):
        # print data
        try:
            self.image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8") # mono8, mono16, bgr8, rgb8, bgra8, rgba8, passthrough
            
            self.Height, self.Width, self.Channel = self.image.shape

            self.ROI_row_percent = 0.30 # 하단 30%
            self.ROI_col_percent = 0.84 # 중앙 84%
            self.ROI_col_left_percent = 0.35
            self.ROI_col_right_percent = 0.47    # 좌측 35, 우측 45% 를 보고 싶다.
            
            self.Row_ROI_from = self.Height - int(self.Height*self.ROI_row_percent)
            self.Row_ROI_to = self.Height
            self.Col_ROI_from = int(self.Width * (1-self.ROI_col_percent)*(0.5))
            self.Col_ROI_to = int(self.Width-((1-self.ROI_col_percent)*(0.5)*self.Width))
            
            ## col 좌측 40 우측 50을 봐보자
            self.Col_ROI_40_from = int(self.Width * (0.5-self.ROI_col_left_percent))
            self.Col_ROI_40_to = int(self.Width - (self.Width*(0.5-self.ROI_col_right_percent)))
            
            
            ### stop sign
            self.ROI_stop_row_percent = 0.3 # 하단 50%
            self.ROI_stop_col_percent = 0.8 # 중앙 x%
            
            self.Row_stop_ROI_from = self.Height - int(self.Height*self.ROI_stop_row_percent)
            self.Row_stop_ROI_to = int(self.Height*self.ROI_stop_col_percent)
            
            self.Col_stop_ROI_from = int(self.Width * (1-self.ROI_stop_col_percent)*(0.5))
            self.Col_stop_ROI_to = int(self.Width-((1-self.ROI_stop_col_percent)*(0.5)*self.Width))
            
            ###
            self.Col_force_right_from = int(self.Width * 0.4)
            self.Col_force_right_to = int(self.Width)
        
        
        except CvBridgeError as e:
            print("___Error___")
            print(e)
        
        # np_arr = np.formstring(data, np.unit8)
        # self.image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    
    def realimg_callback(self, data):
        # print data
        try:
            self.realimage = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8") # mono8, mono16, bgr8, rgb8, bgra8, rgba8, passthrough
        except CvBridgeError as e:
            print("___Error___")
            print(e)
        
        # np_arr = np.formstring(data, np.unit8)
        # self.image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    
    ##################################################################################
    
    def nothing(self):
        pass
    
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
    
    def plothistogram(self, image):
        histogram = np.sum(image[image.shape[0]//2:, :], axis=0)
        midpoint = np.int(histogram.shape[0]/2)
        
        leftbase = np.argmax(histogram[:midpoint])
        rightbase = np.argmax(histogram[midpoint:]) + midpoint
        
        histogram_nonzero = np.asarray(np.nonzero(histogram))
        histogram_size = (float)(len(histogram))
        histogram_nonzero_size = (float)(histogram_nonzero.size)
        ratio = (histogram_nonzero_size/histogram_size)

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
        
        self.estimated_center = self.lpf.filter(start_end_interval)
        
        self.center_value_data.append(self.estimated_center)
        self.filtered_value_data.append(self.lpf.filter(self.estimated_center))
        
  
        left_lane = np.concatenate(left_lane)  # np.concatenate() -> array를 1차원으로 합침
        right_lane = np.concatenate(right_lane)

        out_img[nonzero_y[left_lane], nonzero_x[left_lane]] = [255, 0, 0]
        out_img[nonzero_y[right_lane], nonzero_x[right_lane]] = [0, 0, 255]
        
        cv2.imshow("out_img", out_img)

        return
    
    
    ####################################################################################    
    
    def move_with_only_with_one_line(self, binary_warped, current):
        out_img = np.dstack((binary_warped, binary_warped, binary_warped))

        nwindows = 4
        window_height = np.int(binary_warped.shape[0] / nwindows)
        nonzero = binary_warped.nonzero()  # 선이 있는 부분의 인덱스만 저장 
        #print("nonzero: {}".format(nonzero))
        nonzero_y = np.array(nonzero[0])  # 선이 있는 부분 y의 인덱스 값
        nonzero_x = np.array(nonzero[1])  # 선이 있는 부분 x의 인덱스 값 
        margin = 100
        minpix = 50
        oneline_lane = []
        #right_lane = []
        color = [0, 255, 0]
        thickness = 2
        #print("current: {}".format(current))
        # I made here
        
        oneline_x = []
        oneline_y = []
        h_line_x = []
        h_line_y = []
        return_value = 0
        
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
            
            if len(good_line) > minpix:
                current = np.int(np.mean(nonzero_x[good_line]))
            
                
            ### I customized here
            # current left를 이용한 line 그어보기
            cv2.line(out_img, (current, win_y_low), (current, win_y_high), (155, 155, 155), 2)
            
            # left x y 그려보기
            onelinex_test = nonzero_x[np.concatenate(oneline_lane)]
            oneliney_test = nonzero_y[np.concatenate(oneline_lane)]
            
            
            
            try:
                ox_1, ox_2, oy_1, oy_2 = onelinex_test[0], onelinex_test[len(onelinex_test)-1], oneliney_test[0], oneliney_test[len(oneliney_test)-1];
                
                cv2.line(out_img, (ox_1, oy_1), (ox_2, oy_2), (0, 0, 255), 3)
                if(w == 2):
                    oneline_x.append(ox_1)
                    oneline_x.append(ox_2)
                    
                    oneline_y.append(oy_1)
                    oneline_y.append(oy_2)
            except:
                print(" :::: one line exception :::: ")
                return_value = 1
            
            
            # 값이 한 윈도우만이라도 튀면 제대로 계산되지 않는다.
            
            #print("good_left good_right: {}, {}".format(good_left, good_right))
        #print('\n\n')
        
        # I made here
        # 이정도면 쓸만한 라인을 검출하는 것 같다(ROBUST) - 라인들이 구석에서 인식되면 튄다.
        # 다시, 하단 2번째 윈도우를 기준으로
        cv2.line(out_img, (oneline_x[0], oneline_y[0]), (oneline_x[1], oneline_y[1]), (255, 255, 0), 5)
        cv2.imshow("out_img", out_img)
        # 끝 부분의 중앙과 시작 부분의 중앙의 차이점으로 각도를 구하자
        start_mid = int(oneline_x[0])
        end_mid = int(oneline_x[1])

        
        cv2.circle(out_img, (start_mid, int(oneline_y[0]+margin+5)), 5, (255, 255, 0), -1)   #car center rectangle
        cv2.circle(out_img, (end_mid, int(oneline_y[1]+margin+5)), 5, (255, 255, 0), -1)   #car center rectangle

        #print(">>>>>>>>>>>>diff: ", start_end_abs) # 1.jpg = 4, 3.jpg = 95
        # 양수 커질수록 = 오른쪽으로 회전 필요함
        
        start_end_interval = (end_mid - start_mid)
        
        self.estimated_center = self.lpf.filter(start_end_interval)
        
        self.center_value_data.append(self.estimated_center)
        self.filtered_value_data.append(self.lpf.filter(self.estimated_center))
        
        return return_value
    
    def detecting_traffic_light(self, edge_img):
        light_roi = edge_img[self.Row_light_ROI_from:self.Row_light_ROI_to, self.Col_light_ROI_from:self.Col_light_ROI_to]
        

        _light_roi_h, _light_roi_w = light_roi.shape
        
        _lights = cv2.HoughLinesP(light_roi, 1, math.pi/180,30,10,10)
        
        _, thresh = cv2.threshold(light_roi, 75, 110, cv2.THRESH_BINARY)

        #left_base, right_base, crosswalk_ratio = self.plothistogram(thresh)

        cv2.imshow("lights", thresh)

        return
    
    def detecting_right_line_only(self, img):
        
        _line_roi = img[self.Row_ROI_from:self.Row_ROI_to, self.Col_force_right_from:self.Col_force_right_to]
            
        left_base, right_base, crosswalk_ratio = self.plothistogram(_line_roi)
               
        return_value = self.move_with_only_with_one_line(_line_roi, right_base)

        return return_value
    
    def detect_stopline_contour(self):
        
        img = self.image.copy()
            
        blur = cv2.GaussianBlur(img, (5, 5), 0)
        H, L, S = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HLS))
        _, L = cv2.threshold(L, 75, 110, cv2.THRESH_BINARY)
        
        edge_img = cv2.Canny(np.uint8(L), 60, 70)  
        stop_roi = edge_img[self.Row_stop_ROI_from:self.Row_stop_ROI_to, self.Col_stop_ROI_from:self.Col_stop_ROI_to]
        img = img[self.Row_stop_ROI_from:self.Row_stop_ROI_to, self.Col_stop_ROI_from:self.Col_stop_ROI_to]
          
        contours, _ = cv2.findContours(stop_roi, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        #self.x, self.y, self.w, self.h = [], [], [], []
        self.horizontal_line_x, self.horizontal_line_y = [], []
        self.stop = False
        cnt = 0
        
        for i, cont in enumerate(contours):
            length = cv2.arcLength(cont, True)
            
            area = cv2.contourArea(cont)
            print("length, area : {}, {}".format(length, area))
            (x, y, w, h) = cv2.boundingRect(cont)
            center = (x + int(w/2), y + int(h/2))
            width, height = stop_roi.shape
            
            self.horizontal_line_x.append(center[0])
            self.horizontal_line_y.append(center[1])
            
            cv2.circle(img, (center[0], center[1]), 2, (0, 0, 255), -1)
            if i > 1:
                cv2.line(img, (self.horizontal_line_x[i-1], self.horizontal_line_y[i-1]), (self.horizontal_line_x[i], self.horizontal_line_y[i]), (0, 122, 122), 2)
            
            #print("x, y, w, h:{}, {}, {}, {}".format(x, y, w, h))
            #print("img shape: {}, {}".format(cal_image.shape[0], cal_image.shape[1]))
            print("len something: ", len(cv2.approxPolyDP(cont, length*0.02, True)))
            if not ((area > 20) and (length > 40)):
                continue
            if len(cv2.approxPolyDP(cont, length*0.02, True)) < 3:
                continue
            
            # 여기서 x는 가로 축이다
            #if(x > 0.52*stop_roi.shape[1] and y < 0.19*stop_roi.shape[0] and self.stop == False):
            if(y < 0.19*stop_roi.shape[0] and self.stop == False):
                cnt += 1
                print("CNT:",cnt)
                if(cnt > 20):
                    print("stop")
                    self.stop = True
                    return 1
                    
                
                """self.x.append(int(x))
                self.y.append(int(y))
                self.w.append(int(w))
                self.h.append(int(h))"""
            
            
            
        cv2.imshow("stop_roi", img)
        """            

            if 200 <= center[0] <= (width - 200):
                cv2.rectangle(stop_roi, (x, y), (x + w, y + h), (0, 255, 0), 2)
                print("stopline")
        
        print("number of cnt: ", cnt)      
        cv2.imshow("save_img", save_img)   
        """

        return 0
    """
    def check_crosswalk(self, ratio, target_value, num, time, next_stage):
        _cnt = 0
        if(ratio > target_value):
            _cnt += 1
            
            if(_cnt > num):
                self.past_time = self.current_time
                print("Crosswalk detected, {} sec stop", time)
                
                while self.current_time - self.past_time <= time:
                    self.current_time = rospy.get_time()
                    #self.angled = 0
                    self.speed = 0
                    self.unitydrive(self.angled, self.speed)                    
                    
                self.stage_num = 9
                stage_cnt = 0
                print("################## STAGE 9 ##################")
            else:
                self.angled = -30
                self.speed = 0.3
                stage_cnt = 0
    
        return False
    
        return True
    """
    ##################################################################################
    def trackbar(self):
        img = self.image
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        cv2.namedWindow("TrackBar Windows", cv2.WINDOW_AUTOSIZE)
        cv2.createTrackbar("L-High", "TrackBar Windows", 0, 255, self.nothing)
        cv2.createTrackbar("L-Low", "TrackBar Windows", 0, 255, self.nothing)
        cv2.setTrackbarPos("L-High","TrackBar Windows", 255)
        cv2.setTrackbarPos("L-Low" ,"TrackBar Windows", 0)
        print(":::::")
        while cv2.waitKey(1) != ord('q'):
            L_h = cv2.getTrackbarPos("L-High", "TrackBar Windows")
            L_l = cv2.getTrackbarPos("L-Low" , "TrackBar Windows")
            HLS = cv2.inRange(hls, (0, L_l, 0), (255, L_h, 255))
            hls_out = cv2.bitwise_and(hls, hls, mask = HLS)
            result = cv2.cvtColor(hls_out, cv2.COLOR_HSV2BGR)
            cv2.imshow("TrackBar Windows", result)
        
        cv2.destoryAllWindows()

    def drive(self, angle, speed):
        
        motor_msg = xycar_motor()
        motor_msg.angle = angle
        motor_msg.speed = speed

        self.motor.publish(motor_msg)

    def unitydrive(self, angle, speed):

        unitymotor_msg = PoseStamped()
        unitymotor_msg.pose.position.x = speed
        unitymotor_msg.pose.orientation.x = angle

        self.unitymotor.publish(unitymotor_msg) 

    def start(self):
        while not self.image.size == (self.WIDTH * self.HEIGHT * 3):
            # print("==> Complete Load Image ")
            continue
        
        #self.trackbar()
        
        while not rospy.is_shutdown(): # Main Loop
            
            # HLS Ckeck
            # self.trackbar()
            
            # Task1 : White, Yellow line detection
            # Task2 : Traffic light -> Stop or Turn Left
            # Task3 : 90 degree line
            # Task4 : Finish line

            
            self.current_time = rospy.get_time()
            
            
            ##################### 1. BASIC - line detecting
            img = self.image.copy()
            
            blur = cv2.GaussianBlur(img, (5, 5), 0)
            H, L, S = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HLS))
            _, L = cv2.threshold(L, 75, 110, cv2.THRESH_BINARY)
            edge_img = cv2.Canny(np.uint8(L), 60, 70)
            
            line_roi = edge_img[self.Row_ROI_from:self.Row_ROI_to, self.Col_ROI_from:self.Col_ROI_to]
            line_img = line_roi.copy()
            _line_roi_h, _line_roi_w = line_roi.shape
            all_lines = cv2.HoughLinesP(line_roi, 1, math.pi/180,30,10,10)
            
            _, thresh = cv2.threshold(line_roi, 75, 110, cv2.THRESH_BINARY)

            left_base, right_base, crosswalk_ratio = self.plothistogram(thresh)
            #print("left right base and diff: {}, {}, {}".format(left_base, right_base, abs(left_base-right_base)))
            
            # test """
            """
            if(left_base < 10):
                # MOVE with olny right
                print('right base called\n')
                self.move_with_only_with_one_line(thresh, right_base)
                
            elif(right_base >= int(self.Col_ROI_to*0.8)):
                # move with only left    
                print('left base called\n')
                self.move_with_only_with_one_line(thresh, left_base)
                
            else:    
                draw_info = self.slide_window_search(thresh, left_base, right_base)
            """
            '''
            crosswalk_img = self.crosswalk_image_create(thresh, int(thresh.shape[1]/2)+20, window=3)
            
            print("{}, {}".format(crosswalk_img.shape[0], crosswalk_img.shape[1]))
            cv2.imshow("crosswalk checker", crosswalk_img)
                
            '''
            ##################### 2. stage start
            ######### 2-1. stage 1. go forward and move according to the lines
            if(self.stage_num == 1):
                if(crosswalk_ratio < 0.68):
                    if(abs(self.estimated_center) > 30):
                        print("one line detecting")
                        try:
                            self.move_with_only_with_one_line(thresh, left_base)
                        except:
                            self.move_with_only_with_one_line(thresh, right_base)
                            
                        self.angled = -self.estimated_center*0.14
                    
                    elif(self.estimated_center < -30):
                        print("one line detecting")
                        try:
                            self.move_with_only_with_one_line(thresh, right_base)
                        except:
                            self.move_with_only_with_one_line(thresh, left_base)
                            
                        self.angled = self.estimated_center*0.14
                    else:
                        return_value = self.slide_window_search(thresh, left_base, right_base)
                        if(return_value == -1):
                            print("wanna detect two lines but not possible")
                            try:
                                self.move_with_only_with_one_line(thresh, right_base)
                            except:
                                self.move_with_only_with_one_line(thresh, left_base)
                            self.speed = 0.4
                            self.angled = -self.estimated_center*0.1
                            continue    
                                
                        self.angled = -self.estimated_center*0.1
                        
                    if(self.angled > 20):
                        self.angled = 20
                    elif(self.angled < -20):
                        self.angled = -20 
                        
                    self.speed = 0.15
                    
                else:
                    #self.crosswalk_cross_flag = False
                    self.stage_num = 98
                    self.past_time = self.current_time
                    self.unitydrive(self.angled, 0)
                    print("################## STAGE 1-1 ##################")
            
            
            ######### 1-1. stage 1.1 stop for a while
            elif(self.stage_num == 98):
                print ("Stop")
                
                while self.current_time - self.past_time <= 5:
                    self.current_time = rospy.get_time()
                    #self.angled = 0
                    self.speed = 0
                    self.unitydrive(self.angled, self.speed)                    
                    
                self.stage_num = 2
                print("################## STAGE 2 ##################")
            
            
            ######### 2-2. stage 2. cross the crosswalk(go forward)
            # left_base, right_base, crosswalk_ratio = self.plothistogram(thresh) 를 참고.
            elif(self.stage_num == 2):
                cv2.imshow("out_img", thresh)
                print("crossing crosswalk, ratio: ", crosswalk_ratio)
                self.speed = 0.15
                    
                if(crosswalk_ratio < 0.145):
                    self.stage_num = 3
                    stage_cnt = 0
                    print("################## STAGE 3 ##################")
                   
                   
            ######### 2-3. stage 3. go forward until detecting the line       
            elif(self.stage_num == 3):
                cv2.imshow("out_img", thresh)
                print("not lines yet, ratio: ", crosswalk_ratio)
                
                #self.detecting_traffic_light(edge_img)
                
                self.speed = 0.15
                #self.angled = -2.6
                if(abs(self.angled) < 2.0):
                    self.angled = self.angled
                    self.stage_num = 99
                    stage_cnt = 0
                    print("################## STAGE 3-1 ##################")
                    
                    
                else:
                    self.angled = self.angled*0.971
                    
            
            ######### 2-31. stage 3-1. go forward adjusting         
            elif(self.stage_num == 99):
                cv2.imshow("out_img", thresh)
                print("not lines yet, ratio: ", crosswalk_ratio)
                self.speed = 0.18
                self.estimated_center = 0
                if(crosswalk_ratio >= 0.5):
                    stage_cnt += 1
                    
                    if(stage_cnt > 5):
                        self.stage_num = 4
                        stage_cnt = 0
                        print("################## STAGE 4 ##################")
                        print(":::::: LINE DETECTING BACK ::::::")

                else:
                    stage_cnt = 0
                
                if(crosswalk_ratio < 0.4):
                    if(left_base > (self.Col_ROI_to-self.Col_ROI_from) * 0.3):
                        print("over 30 percent of col")
                        self.angled = -10
                    elif(left_base > (self.Col_ROI_to-self.Col_ROI_from) * 0.22):
                        print("over 22 percent of col")
                        self.angled = -8
                    elif(left_base > (self.Col_ROI_to-self.Col_ROI_from) * 0.10):
                        print("over 15 percent of col")
                        self.angled = -6

                    elif(right_base > (self.Col_ROI_to-self.Col_ROI_from) *0.6):
                        print("over 40 percent of right")
                        self.angled = 1.7
                    

            ######### 2-4. stage 4. cross the cross walk again        
            elif(self.stage_num == 4):
                cv2.imshow("out_img", thresh)
                print("stage 4, crossing crosswalk, ratio: ", crosswalk_ratio)
                self.speed = 0.2
                #self.angled = -6.0
                self.angled = 0.0
                
                if(crosswalk_ratio < 0.50):
                    stage_cnt += 1
                    
                    if(stage_cnt > 6):
                        self.stage_num = 5
                        print("################## STAGE 5 ##################")
                        
                        for i in range(10):
                            self.estimated_center = self.lpf.filter(-60)
                        
                        self.angle = 0.0
                        stage_cnt = 0
                        stage_turn_flag = False
                        
                        
                else:
                    stage_cnt = 0
            
            
            ######### 2-5. stage 5. go forward and move according to the lines(likes stage 1.)
            elif(self.stage_num == 5):
                print("stage 5, center, angle {}, {} ".format(self.estimated_center, self.angled))
                detect_roi = edge_img[self.Row_ROI_from:self.Row_ROI_to, self.Col_ROI_40_from:self.Width]
                _, _thresh = cv2.threshold(detect_roi, 75, 110, cv2.THRESH_BINARY)
                _left_base, _right_base, crosswalk_ratio = self.plothistogram(_thresh)
                
  
                
                crosswalk_img = self.crosswalk_image_create(_thresh, int(_thresh.shape[1]/2)+40)
                cv2.imshow("crosswalk checker", crosswalk_img)
                
                if(self.crosswalk_histogram(crosswalk_img, 0.655, verbose=True)):
                    print("... crosswalk estimated ... check ...")
                    stage_cnt += 1
                    self.speed = 0.05
                    
                    if(stage_cnt > 2):
                        self.past_time = self.current_time
                        print ("Stop")
                        
                        while self.current_time - self.past_time <= 1.1:
                            self.current_time = rospy.get_time()
                            #self.angled = 0
                            self.speed = 0
                            self.unitydrive(self.angled, self.speed)                    
                            
                        self.stage_num = 7
                        stage_cnt = 0
                        print("################## STAGE 7 ##################")
                        
                    
                    else:
                        self.speed = 0.15
                        self.angled = self.angled
                    
                else:
                    stage_cnt = 0
                    
                    if(abs(self.estimated_center) > 30):
                        print("one line detecting")
                        print("                     left base, right base: {}, {}".format(_left_base, _right_base))
                        
                        if(self.move_with_only_with_one_line(_thresh, _right_base) == 1):
                            if(self.move_with_only_with_one_line(_thresh, _left_base) == 1):
                                print("nothing is detected, going slowly")
                                self.angled = self.angled
                                self.speed = 0.1
                        else:
                            self.angled = -self.estimated_center*0.3
                            self.speed = 0.1


                    else:
                        return_value = self.slide_window_search(_thresh, _left_base, _right_base)
                        if(return_value == -1):
                            print("wanna detect two lines but not possible")
                            print("left base, right base: {}, {}".format(_left_base, _right_base))
                            
                            if(self.move_with_only_with_one_line(_thresh, _right_base) == 1):
                                if(self.move_with_only_with_one_line(_thresh, _left_base) == 1):
                                    print("nothing is detected, going slowly")
                                    self.angled = self.angled
                                    self.speed = 0.1
                            else:
                                self.angled = -self.estimated_center*0.24
                                self.speed = 0.15
                        else:
                            self.angled = -self.estimated_center*0.24
                            self.speed = 0.15

                    if self.angled > 0:
                        self.angled = -self.angled   


                    
            ######### 2-7. stage 7. turn right !!!!        
            elif(self.stage_num == 7):  
                detect_roi = edge_img[self.Row_ROI_from:self.Row_ROI_to, self.Col_ROI_40_from:self.Col_ROI_40_to]
                _, _thresh = cv2.threshold(detect_roi, 75, 110, cv2.THRESH_BINARY)
                _left_base, _right_base, crosswalk_ratio = self.plothistogram(_thresh)
                
                return_value = self.slide_window_search(_thresh, _left_base, _right_base)
                
                print("7, left base, right base: {}, {}".format(_left_base, _right_base))
                if(return_value != -1 and _left_base > 200 and _left_base < int(_thresh.shape[1]/2) and _right_base > int(_thresh.shape[1]*0.55) and abs(_right_base -_left_base) > 150):
                    stage_cnt += 1
                    
                    if(stage_cnt >3):
                        print("turn right done")
                        self.stage_num = 8
                        stage_cnt = 0
                        print("################## STAGE 8 ##################")
                    
                else:
                    stage_cnt = 0
                    self.speed = 0.05
                    self.angled = -30
            
            
            ######### 2-8. stage 8. follow the line                
            elif(self.stage_num == 8):
                print("stage 8, following the line, left base, center: {}, {}".format(left_base, self.estimated_center))
                self.move_with_only_with_one_line(thresh, left_base)
                
                if(left_base < 150):
                    left_base = int(thresh.shape[1]*0.4)
                    self.angled = -self.estimated_center * 0.14
                elif(left_base > int(thresh.shape[1]*0.4)):
                    left_base = int(thresh.shape[1]*0.4)
                    self.angled = -self.estimated_center * 0.14
                else:
                    self.angled = -self.estimated_center * 0.015
                self.speed = 0.15
                
                if(abs(self.estimated_center) > 60):
                    self.angled = -self.estimated_center * 0.14
                
                crosswalk_img = self.crosswalk_image_create(thresh, int(thresh.shape[1]/2), window=3)
                cv2.imshow("crosswalk checker", crosswalk_img)
                
                if(self.crosswalk_histogram(crosswalk_img, 0.4, verbose=True)):
                    print("... crosswalk estimated ... check ...")
                    stage_cnt += 1
                    self.speed = 0.05
                    
                    if(stage_cnt > 3):
                        self.past_time = self.current_time
                        print ("Stop")
                        
                        while self.current_time - self.past_time <= 4:
                            self.current_time = rospy.get_time()
                            #self.angled = 0
                            self.speed = 0
                            self.unitydrive(self.angled, self.speed)                    
                            
                        self.stage_num = 9
                        stage_cnt = 0
                        print("################## STAGE 9 ##################")
    
    
            
            ## cross the crosswalk
            elif(self.stage_num == 9):
                cv2.imshow("out_img", thresh)
                print("crossing crosswalk, ratio: ", crosswalk_ratio)
                self.speed = 0.1
                self.angled = self.angled*0.7
                
                crosswalk_img = self.crosswalk_image_create(thresh, int(thresh.shape[1]/2), window=3)
                cv2.imshow("crosswalk checker", crosswalk_img)
                
                    
                if(not self.crosswalk_histogram(crosswalk_img, 0.15, verbose=True)):
                    stage_cnt += 1
                    
                    if(stage_cnt > 10):
                        self.stage_num = 10
                        stage_cnt = 0
                        print("################## STAGE 10 ##################")
                else:
                    stage_cnt = 0
                
                   
            ## turn left
            elif(self.stage_num == 10):
                cv2.imshow("out_img", thresh)
                self.angled = 16.154
                self.speed = 0.08
                
                crosswalk_img = self.crosswalk_image_create(thresh, int(thresh.shape[1]/2), window=3)
                cv2.imshow("crosswalk checker", crosswalk_img)
                                
                if(self.crosswalk_histogram(crosswalk_img, 0.45, verbose=True)):
                    stage_cnt += 1
                    
                    if(stage_cnt > 3):
                        self.stage_num = 11
                        print("################## STAGE 11 ##################")
                    
                else:
                    stage_cnt = 0
                    

            ## cross the crosswalk
            elif(self.stage_num == 11):
                cv2.imshow("out_img", thresh)
                print("crossing crosswalk, ratio: ", crosswalk_ratio)
                self.speed = 0.15
                self.angled = self.angled *1.025
                    
                crosswalk_img = self.crosswalk_image_create(thresh, int(thresh.shape[1]/2), window=3)
                cv2.imshow("crosswalk checker", crosswalk_img)
                
                if(not self.crosswalk_histogram(crosswalk_img, 0.18, verbose=True)):
                    stage_cnt += 1
                    
                    if(stage_cnt > 5):
                        self.stage_num = 12
                        stage_cnt = 0
                        print("################## STAGE 12 ##################")
                else:
                    stage_cnt = 0
                    
            
            ### go according to the line
            elif(self.stage_num == 12):
                return_value = self.slide_window_search(thresh, left_base, right_base)
                print("left base, right base {}, {}".format(left_base, right_base))
                if(left_base < int(thresh.shape[1]*0.22)):
                    print("             need to turn left strongly ...")
                    self.move_with_only_with_one_line(thresh, right_base)
                    self.angled = -self.estimated_center*0.72
                    self.speed = 0.1
                    
                else:
                    if(return_value == -1):
                        print("one line detecting")
                        print("                     left base, right base: {}, {}".format(left_base, right_base))
                        
                        if(self.move_with_only_with_one_line(thresh, right_base) == 1):
                            if(self.move_with_only_with_one_line(thresh, left_base) == 1):
                                print("nothing is detected, going slowly")
                                self.angled = self.angled
                                self.speed = 0.1
                        else:
                            self.angled = -self.estimated_center*0.15
                            self.speed = 0.1
                    else:
                        self.angled = -self.estimated_center*0.20
                
                if(abs(self.angled) < 2):
                    
                    stage_cnt += 1
                    if(stage_cnt > 20):
                        self.past_time = self.current_time
                        print("################## mission completed ##################")
                        print("                         Stop")
                        
                        while self.current_time - self.past_time <= 4:
                            self.current_time = rospy.get_time()
                            #self.angled = 0
                            self.speed = 0
                            self.unitydrive(self.angled, self.speed)
                            
                        self.speed = 0
                        self.angled = 0
                        break
                else:
                    stage_cnt = 0

            '''
            ## White line ##########################################
            if self.stop == 0:
                # print("Flag stop == 0")
                line_roi = edge_img[360:480, 80:560]

                # HoughLinesP
                # 이미지로부터 직선을 찾는 함수
                # return = A vector that will store the parameters 
                # (xstart,ystart,xend,yend) of the detected lines
                all_lines = cv2.HoughLinesP(line_roi, 1, math.pi/180,30,30,10)
                
                # draw lines in ROI area
                line_img = line_roi.copy()
                for line in all_lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(line_img, (x1, y1+240), (x2, y2+240), (0, 255, 0), 2)
                    # 라인을 그리는 함수. line_img 에다 그린다. self.img가 아님에 주의.
                    
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
                    if 0.6 < abs(slope) < 10:
                        slopes.append(slope)
                        new_lines.append(line[0])
                    if abs(slope) >= 3:
                        print("detected line's slope has too large angle")
                        cv2.line(self.image, (x1, 0+240), (x2, 240+240), (0, 0, 255), 3) # RED

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
                    elif (slope > 0) and (x1 > 320):
                        right_lines.append([Line.tolist()])

                # draw right&left lines in different color
                # color = BGR이다
                line_img = line_roi.copy()
                for line in left_lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(self.image, (x1+160, y1+240), (x2+160, y2+240), (0, 0, 255), 2) # RED
                    cv2.line(edge_img, (x1+160, y1+240), (x2+160, y2+240), (0, 0, 255), 2) # RED

                    
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
                
                for line in right_lines:
                    
                    x1, y1, x2, y2 = line[0]
                    slope = float(y2-y1) / float(x2-x1)
                    print("right slope: {}".format(slope))
                    print("right x1, y1, x2, y2: {}, {}, {}, {}".format(x1, y1, x2, y2))
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
                print("right x 1 , x 2 ({}, {})".format(x1, x2))
                # 평균적인 우측 선을 (255,0,0) 으로 나타낸 것
                
                line_center = int((left_center + right_center)/2)
                #print("estimated line center {}, rectangle pos: {}".format(line_center, int((325+315)/2)))
                # 계산된 line center가 현재 약 240정도로 320보다 상당히 좌측이다. 초록색으로 박스 그려진게...
                #Car center
                cv2.line(self.image, (0, 360), (640, 360), (0, 255, 255), 1) # 가로선
                cv2.line(self.image, (240, 0), (240, 480), (0, 255, 255), 1) # 세로선_1
                cv2.line(self.image, (320, 0), (320, 480), (0, 255, 255), 1) # 세로선_2
                
                cv2.rectangle(self.image, (315, 355), (325, 365), (0, 0, 255), 2)
                # line과 동일하게 시작점 좌표, 종료지점 좌표로 나타냄.
                
                # Draw Rectangle
                #cv2.rectangle(self.image, (left_center-5, 355), (left_center+5, 365), (0, 255, 40), 2)   #left line center rectangle
                #cv2.rectangle(self.image, (right_center-5, 355), (right_center+5, 365), (0, 255, 80), 2) #right line center rectangle
                cv2.rectangle(self.image, (line_center-5, 355), (line_center+5, 365), (0, 255, 150), 2)   #car center rectangle

                self.angled = self.estimated_center*0.05
                """
                _angle = line_center - 320
                if abs(_angle) > 20:
                    self.angled = _angle * 0.2
                    self.speed = 0.05
                else:
                    self.angled = 0
                    self.speed = 0.1
                print("line_center: {}, _angle: {}, self.angled: {}".format(line_center, _angle, self.angled))
                if self.angled > 50:
                    self.angled = 30
                elif self.angled < -50:
                    self.angled = -30
                """
            '''
            #직진일 때, 240과의 차이는 약 1~10정도.
            # 오른쪽 곡선이 나오는 순간, value가 커진다. 음... 작아져야 정상같은데...
            # 240 보다 큰 값으로 센터가 계산된다.
            # angle 이 양수이면 좌회전하고, 음수이면 우회전한다.
            
            # added for testing. should delete after!
            #self.angled = 0
            #self.speed = 0

            '''
            # except:
            #     print("Error whiteline")
            ################################################################

            ## Stop line ROI Area ##########################################
            #640x480
            stop_x_min = 240
            stop_x_max = 400
            stop_y_min = 370
            stop_y_max = 410
            stop_roi = L.copy()
            stop_roi = stop_roi[stop_y_min:stop_y_max, stop_x_min:stop_x_max] # y,x
            # try:
            _, contours, _ = cv2.findContours(stop_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            # print("contours : ", contours)
            for cont in contours:
                length = cv2.arcLength(cont, True)
                area = cv2.contourArea(cont)
                # print("Stop Flag : ", self.stop, "Area :", area, "Length : ", length)

                if not ((2000 > area > 1200) and (length > 300)):
                    continue

                # print("Len : ", len(cv2.approxPolyDP(cont, length*0.02, True)))
                if len(cv2.approxPolyDP(cont, length*0.02, True)) < 2:
                    continue
                
                (x, y, w, h) = cv2.boundingRect(cont)
                center = (x + int(w/2), y + int(h/2))

                if (70 <= center[0] <= (stop_x_max - stop_x_min)) and (self.stop == 0):
                    cv2.rectangle(self.image, (x+stop_x_min, y+stop_y_min), (x + w + stop_x_min, y + h + stop_y_min), (0, 255, 0), 2)
                    self.stop += 1
                    self.past_time = self.current_time
                    print "stopline"
                
                if self.stop == 1:
                    print("Flag stop == 1")
                    self.past_time = self.current_time
                    while self.current_time - self.past_time <= 5:
                        self.current_time = rospy.get_time()
                        self.angled = 0
                        self.speed = 0
                        self.drive(self.angled, self.speed)                    
                        print ("Stop ", str(self.current_time - self.past_time), "sec")

                    self.past_time = self.current_time
                    
                    self.past_time = self.current_time
                    while self.current_time - self.past_time <= 2:
                        self.current_time = rospy.get_time()
                        self.angled = 0
                        self.speed = 5
                        self.drive(self.angled, self.speed)                    
                        print ("After stop, Go ", str(self.current_time - self.past_time), "sec")
                    
                    self.stop = 0
                    self.speed = 1

            # except:
            #     print("Error yellowline")
            
            # Draw Stopline Area x,y
            cv2.rectangle(self.image, (stop_x_min, stop_y_min), (stop_x_max, stop_y_max), (0, 0, 255), 2)
            ###############################################################
            '''

            #self.angled = 0
            #self.speed = 0.0
            
            if(self.angled > 30):
                self.angled = 30
            if(self.angled < -30):
                self.angled = -30
                
            # Publish xycar motor & unity motor
            self.unitydrive(self.angled, self.speed)
            
            
            # Check Image
            original_img = self.image.copy()
            cv2.putText(original_img, 'Time : ', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (125, 125, 125), 1, cv2.LINE_AA)
            cv2.putText(original_img, str(self.current_time), (80, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (125, 125, 125), 1, cv2.LINE_AA)
            cv2.putText(original_img, 'Speed : ', (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (125, 125, 125), 1, cv2.LINE_AA)
            cv2.putText(original_img, str(self.speed), (80, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (125, 125, 125), 1, cv2.LINE_AA)
            
            cv2.putText(original_img, 'estimated center : ', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (125, 125, 125), 1, cv2.LINE_AA)
            cv2.putText(original_img, str(self.estimated_center), (160, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (125, 125, 125), 1, cv2.LINE_AA)

            cv2.putText(original_img, 'Angled : ', (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (125, 125, 125), 1, cv2.LINE_AA)
            cv2.putText(original_img, str(self.angled), (80, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (125, 125, 125), 1, cv2.LINE_AA)

            #robotvision_horizontal = np.hstack((original_img, img))
            cv2.imshow("RobotVision", original_img)
            #cv2.imshow("line_img", line_img)
            #cv2.imshow("edge_img", edge_img)
            cv2.waitKey(1)

if __name__ == '__main__':
    
    try:
        RVS = Robotvisionsystem()
    except:
        print("################ Encountered unexpected ERROR !! ################")
        print("################   Please restart launch file ################")
        
    """_timestamp = [i for i in range(len(RVS.center_value_data))]

    plt.plot(_timestamp, RVS.center_value_data, 'r')
    plt.plot(_timestamp, RVS.filtered_value_data, 'b')
    plt.show()"""

