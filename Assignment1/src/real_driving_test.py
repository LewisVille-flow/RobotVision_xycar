#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
    File Description:
        Sim to Real, using rosbag file, testing 진행 py    
        
    작동방법:
        $ roslaunch assignment1 driving.launch
        에서 launch 파일 수정, driving.py 가 아닌 real_driving_test.py를 실행할 수 있도록 해야 함.

"""

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
    print("\n\n::::: SIGINT RECIEVED :::::")
    import time
    time.sleep(1)
    os.system('killall -9 python rosout')
    return

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
        #self.image = cv2.imread('14.png', cv2.IMREAD_COLOR)
        self.realimage = np.empty(shape=[0]) 
        self.bridge = CvBridge() 
        self.motor = None 
        self.angled = 0
        self.speed = 0.1
        self.stop = 0
        self.estimated_center = 0
        self.estimated_center_list = []
        self.estimated_center_left = 0.0     # window 0,3 x좌표 차이, window 2 x좌표 차이, window 0,3 current x좌표 차이, window 1,2 current x좌표 차이
        self.estimated_center_right = 0.0
        
        self.CAM_FPS = 30
        self.WIDTH, self.HEIGHT = 640, 480
        
        #self.crosswalk_flag = False
        self.crosswalk_cross_flag = False
        self.stage_num = 1
        
        self.lpf = LowPassFilter(cutoff_freq=0.2, ts = 0.06)
        self.lpf1 = LowPassFilter(cutoff_freq=0.2, ts = 0.06)
        self.lpf2 = LowPassFilter(cutoff_freq=0.2, ts = 0.06)
        
        self.center_left_data = []
        self.center_right_data = []
        
        self.filtered_value_left_data = []
        self.filtered_value_right_data = []
        
        self.value_flag = False
        self.min_value = 0
        
        self.sum = []
        
        rospy.init_node('driving')
        
        #### For Real World driving!( real xycar )
        self.motor = rospy.Publisher('/xycar_motor', xycar_motor, queue_size=1)
        #self.real_image = rospy.Subscriber('/usb_cam/image_raw/compressed',CompressedImage, self.realimg_callback)
        self.real_image = rospy.Subscriber('/usb_cam/image_raw/compressed',CompressedImage, self.img_callback)
        self.unitymotor = self.motor

        #### For Simulator driving! (simul xycar)
        #self.unitymotor = rospy.Publisher('/unitymotor', PoseStamped, queue_size=1)
        #self.unity_img = rospy.Subscriber('/unitycamera', CompressedImage , self.img_callback)

        clear = lambda : os.system('clear')
        clear()
        print("REAL DRIVING TEST PY EXCUTED")
        
        self.current_time = rospy.get_time()
        self.past_time = self.current_time
        
        while self.current_time - self.past_time <= 3:
            self.current_time = rospy.get_time()
            #self.angled = 0
            self.speed = 0
            self.unitydrive(self.angled, self.speed)  
                              
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
            
            
            ### for test real
            self.ROW_ROI_real_under_ignore_percent = 0.1   # 하단 5%는 안 보련다
            self.ROW_ROI_real_percent = 0.25         # 하단 
            
            self.Height_with_ignored = self.Height*(1-self.ROW_ROI_real_under_ignore_percent)
            
            self.Row_ROI_from_real = int(self.Height_with_ignored - int(self.Height_with_ignored*self.ROW_ROI_real_percent))
            self.Row_ROI_to_real = int(self.Height_with_ignored)
            
            self.ROI_col_percent_real = 0.90 # 중앙 80% 를 보고 싶다.
            self.Col_ROI_from_real = int(self.Width * (1-self.ROI_col_percent_real)*(0.5))
            self.Col_ROI_to_real = int(self.Width-((1-self.ROI_col_percent_real)*(0.5)*self.Width))
            
        
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
        margin = 150
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

        #print("half base {}".format(base))
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
        start_end_interval = left_x[1] - left_x[0]
        
        #self.estimated_center = self.lpf.filter(start_end_interval)
        self.estimated_center = start_end_interval
        
        self.center_left_data.append(self.estimated_center)
        self.filtered_value_left_data.append(self.lpf.filter(self.estimated_center))
        
  
        left_lane = np.concatenate(left_lane)  # np.concatenate() -> array를 1차원으로 합침
        right_lane = np.concatenate(right_lane)

        out_img[nonzero_y[left_lane], nonzero_x[left_lane]] = [255, 0, 0]
        out_img[nonzero_y[right_lane], nonzero_x[right_lane]] = [0, 0, 255]
        
        cv2.imshow("out_img", out_img)

        return
    
    
    ####################################################################################    
    
    def move_with_only_with_one_line(self, binary_warped, current, name='none'):
        out_img = np.dstack((binary_warped, binary_warped, binary_warped))

        nwindows = 4
        window_height = np.int(binary_warped.shape[0] / nwindows)
        nonzero = binary_warped.nonzero()  # 선이 있는 부분의 인덱스만 저장 
        nonzero_y = np.array(nonzero[0])  # 선이 있는 부분 y의 인덱스 값
        nonzero_x = np.array(nonzero[1])  # 선이 있는 부분 x의 인덱스 값 
        margin = 100
        minpix = 50
        oneline_lane = []
        color = [0, 255, 0]
        thickness = 2

        return_value = 0
        
        win_0, win_1, win_2, win_3 = 0, 0, 0, 0
        
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
            
            # left x y 그려보기
            onelinex_test = nonzero_x[np.concatenate(oneline_lane)]
            oneliney_test = nonzero_y[np.concatenate(oneline_lane)]
            
            if(len(onelinex_test) < 2 or len(oneliney_test) < 2):
                print("no line detected in window number: " + str(w) + " in" + name + " line")
                self.estimated_center = self.estimated_center
                continue
            
            ### 4개를 평균내서 기울어진 각도를 계산할 예정
            # window 3 x좌표 차이, window 2 x좌표 차이, window 0,3 current x좌표 차이, window 1,2 current x좌표 차이
           
            # 1. window 2. 3 좌표 차이 계산
            ox_1, ox_2, oy_1, oy_2 = 0, 0, 0, 0

            if(w == 2 or w == 3):
                for i in range(3):
                    ox_1 += onelinex_test[i]
                    ox_2 += onelinex_test[len(onelinex_test)-1-i]
                    oy_1 += oneliney_test[i]
                    oy_2 += oneliney_test[len(oneliney_test)-1-i]
                ox_1 = int(ox_1/3)
                ox_2 = int(ox_2/3)
                oy_1 = int(oy_1/3)
                oy_2 = int(oy_2/3)
                diff = ox_2 - ox_1
                
                # window 2, 3 직선 그리기
                cv2.line(out_img, (ox_1, oy_1), (ox_2, oy_2), (255, 255, 0), 3)
                
                # 시작 - 끝 지점 차이 spotting - 큰 의미 X
                """if(w == 2):
                    cv2.circle(out_img, (ox_1, int(oy_1+margin+5)), 5, (255, 255, 0), -1)   # 시작 점
                    cv2.circle(out_img, (ox_2, int(oy_2+margin+5)), 5, (255, 255, 0), -1)   # 끝 점"""

                
                self.estimated_center_list.append(diff)

            # window 0, 1, 2, 3 current x좌표 차이
            if(w == 0):
                win_0 = current
            elif(w == 1):
                win_1 = current
            elif(w == 2):
                win_2 = current
            elif(w == 3):
                win_3 = current
             
            # current left를 이용한 line 그어보기
            #cv2.line(out_img, (current, win_y_low), (current, win_y_high), (255, 255, 0), 2)

            
        # 기타 시각화작업 - window 별 예측 기울기 라인 그리기
        #cv2.line(out_img, (win_0, binary_warped.shape[0]), (win_3, binary_warped.shape[0]-4*window_height), (255, 255, 0), 2)
        cv2.line(out_img, (win_1, binary_warped.shape[0]-window_height), (win_2, binary_warped.shape[0]-3*window_height), (255, 255, 0), 2)
        
        # window 0, 1, 2, 3 current x좌표 차이
        self.estimated_center_list.append(win_3 - win_0)
        self.estimated_center_list.append(win_2 - win_1)
        
        # 최종 self.estimated_center 는 이 self.estimated_center_list의 항목들을 평균 낸 것으로 하겠다.
        _temp_avg = 0
        for i, _ in enumerate(self.estimated_center_list):
            _temp_avg += self.estimated_center_list[i]
        _temp_avg /= len(self.estimated_center_list)
        
        
        
        if name == "left":
            self.center_left_data.append(_temp_avg)
            return_value = self.lpf1.filter(_temp_avg)
            self.filtered_value_left_data.append(return_value)
            
        elif name == "right":
            self.center_right_data.append(_temp_avg)
            return_value = self.lpf2.filter(_temp_avg)
            self.filtered_value_right_data.append(return_value)
        
        if name == 'none': 
            cv2.imshow("out_img", out_img)
        else:
            cv2.imshow(name, out_img)
            
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
    def follow_the_line(self, left_img, left_base, right_img, right_base):
        try:
            self.estimated_center_left = self.move_with_only_with_one_line(left_img, left_base, name='left')
        except:
            print("left line not detected")
                
        try:
            self.estimated_center_right = self.move_with_only_with_one_line(right_img, right_base, name='right')
        except:
            print("right line not detected")          

        sum = self.estimated_center_left + self.estimated_center_right
        self.sum.append(sum)
        print("left, right, sum: {}, {}, {}".format(self.estimated_center_left, self.estimated_center_right, sum))
        
        return sum
    
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

    def unitydrive(self, angle, speed):
        
        motor_msg = xycar_motor()
        motor_msg.angle = angle
        motor_msg.speed = speed

        self.motor.publish(motor_msg)

    """def unitydrive(self, angle, speed):

        unitymotor_msg = PoseStamped()
        unitymotor_msg.pose.position.x = speed
        unitymotor_msg.pose.orientation.x = angle

        self.unitymotor.publish(unitymotor_msg)""" 

    def start(self):
        while not self.image.size == (self.WIDTH * self.HEIGHT * 3):
            # print("==> Complete Load Image ")
            continue
        
        #self.trackbar()
        
        stage_cnt = 0
        line_cnt = 0
        
        self.unitydrive(0, 0)
        frame_cnt = 0
        while not rospy.is_shutdown(): # Main Loop
            frame_cnt += 1
            if(frame_cnt % 30 != 0):
                continue
            # HLS Ckeck
            # self.trackbar()
            
            # Task1 : White, Yellow line detection
            # Task2 : Traffic light -> Stop or Turn Left
            # Task3 : 90 degree line
            # Task4 : Finish line

            
            self.current_time = rospy.get_time()
            
            
            
            img = self.image.copy()
            """
            blur = cv2.GaussianBlur(img, (5, 5), 0)
            H, L, S = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HLS))
            _, _L = cv2.threshold(L, 75, 110, cv2.THRESH_BINARY)
            _, L_white_line = cv2.threshold(L, 140, 225, cv2.THRESH_BINARY)
            
            edge_img = cv2.Canny(np.uint8(L), 60, 70)
            
            #line_roi = edge_img[self.Row_ROI_from:self.Row_ROI_to, self.Col_ROI_from:self.Col_ROI_to]
            line_roi = L_white_line[self.Row_ROI_from_real:self.Row_ROI_to_real , self.Col_ROI_from_real:self.Col_ROI_to_real]
            hough_roi = img[self.Row_ROI_from_real:self.Row_ROI_to_real , self.Col_ROI_from_real:self.Col_ROI_to_real]
            
            
            line_img = line_roi.copy()
            _line_roi_h, _line_roi_w = line_roi.shape
            all_lines = cv2.HoughLinesP(line_roi, 1, math.pi/180,30,10,10)
            
            
            _, thresh = cv2.threshold(line_roi, 75, 110, cv2.THRESH_BINARY)

            left_base, right_base, crosswalk_ratio = self.plothistogram(thresh)
            #print("left right base and diff: {}, {}, {}".format(left_base, right_base, abs(left_base-right_base)))
            #return_value = self.slide_window_search(thresh, left_base, right_base)
            
            try:
                for line in all_lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.circle(hough_roi, (x1, y1), 2, (0, 0, 122), -1)
                    cv2.circle(hough_roi, (x2, y2), 2, (0, 0, 122), -1)
                    
            except:
                print("no lines here")
            cv2.imshow("hough_roi", hough_roi)"""
            
            
            ########################################################################################
            ##################### 1. BASIC - line detecting
            ########### 이미지 변환, ROI 내부 대표 좌표 검출
            
            ### half base - left part: detect yellow line
            yL, yA, yB = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2LAB))
            _, yB = cv2.threshold(yB, 160, 255, cv2.THRESH_BINARY)
            
            left_yB_roi = yB[self.Row_ROI_from_real:self.Row_ROI_to_real, self.Col_ROI_from_real:self.Col_ROI_to_real]
        
            left_yB_roi_lefthalf = left_yB_roi[:, :int(left_yB_roi.shape[1]*0.70)]  # row, col
            _lbase = self.plothalfhistogram(left_yB_roi_lefthalf)           # left yb roi 영역에서 검출되는 대표 x좌표
            
                    
            ### half base - right part: detect white line
            wH, wL, wS = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HLS))
            _, _wL = cv2.threshold(wL, 120, 225, cv2.THRESH_BINARY)
            right_wL_roi = _wL[self.Row_ROI_from_real:self.Row_ROI_to_real, self.Col_ROI_from_real:self.Col_ROI_to_real]
       
            right_wL_roi_righthalf = right_wL_roi[:, int(right_wL_roi.shape[1]*0.45):]  # row, col
            _rbase = self.plothalfhistogram(right_wL_roi_righthalf)            #right wL roi 영역에서 검출되는 대표 x좌표
            
            ### just ROI
            _, _L = cv2.threshold(wL, 140, 225, cv2.THRESH_BINARY)
            #cv2.imshow("wll", _L)
            #_L = cv2.Canny(np.uint8(_L), 60, 70)
            #_, _L = cv2.threshold(wL, 75, 110, cv2.THRESH_BINARY)
            
            line_roi = _L[self.Row_ROI_from_real:self.Row_ROI_to_real , self.Col_ROI_from_real:self.Col_ROI_to_real]
            
            
            ### 시각화
            #cv2.imshow("_line_roi_lefthalf", left_yB_roi_lefthalf)
            #print("half l test, base:{}".format(_lbase))
            
            #cv2.imshow("_line_roi_righthalf", right_wL_roi_righthalf)
            #print("half r test, base:{}".format(_rbase))
            
            
            ########### 추출한 대표 좌표를 가지고 기울기 계산
            """try:
                self.estimated_center_left = self.move_with_only_with_one_line(left_yB_roi_lefthalf, _lbase, name='left')
            except:
                print("left line not detected")
            
            try:
                self.estimated_center_right = self.move_with_only_with_one_line(right_wL_roi_righthalf, _rbase, name='right')
            except:
                print("right line not detected")          

            sum = self.estimated_center_left + self.estimated_center_right
            self.sum.append(sum)
            print("left, right, sum: {}, {}, {}".format(self.estimated_center_left, self.estimated_center_right, sum))"""
            
            ##################### 2. Move
            

            ########### stage 1. move following the line
            if(self.stage_num == 1):
                value = self.follow_the_line(left_yB_roi_lefthalf, _lbase, right_wL_roi_righthalf, _rbase)
                
                self.speed = 3.1
                """if(value < -3):
                    self.angled = -30
                elif(value > 6):
                    self.angled = 30
                elif(value > -3 and value < 6):
                    self.angled = value*0.3"""
                
                if(abs(value) < 15.0):
                    self.angled = value*0.6
                elif(abs(value) < 25):
                    self.angled = value*0.92
                elif(abs(value) < 40):
                    self.angled = value*0.8
                else:
                    self.angled = value
                
                if(self.angled < -40):
                    self.angled = -40
                elif(self.angled > 40):
                    self.angled = 40
                
                ### check stop sign
                crosswalk_img = self.crosswalk_image_create(line_roi, int(line_roi.shape[1]*0.5), window=2)
                cv2.imshow("crosswalk checker", crosswalk_img)
                
                if(self.crosswalk_histogram(crosswalk_img, 0.4, verbose=True)):
                    stage_cnt += 1
                    if(stage_cnt > 2):
                        print("\n\n :::::: crosswalk or stop sign ::::::")
                        self.stage_num = 2
                        
                        self.past_time = self.current_time
                        print ("Stop")
                        
                        while self.current_time - self.past_time <= 3:
                            self.current_time = rospy.get_time()
                            #self.angled = 0
                            self.speed = 0
                            self.unitydrive(self.angled, self.speed)                    
                            
                        stage_cnt = 0
                        print("################## STAGE 2 ##################")

                else:
                    stage_cnt = 0   
                    
            ########### stage 2. detect traffic lights, and turn left    
            elif(self.stage_num == 2):  
                value = 0  
                
                _left_base, _right_base, crosswalk_ratio = self.plothistogram(line_roi)
                
                return_value = self.slide_window_search(line_roi, _left_base, _right_base)
                
                print("     2, left base, right base: {}, {}".format(_left_base, _right_base))
                
                #if(return_value != -1 and _left_base > 260 and _left_base < int(line_roi.shape[1]*0.6) and _right_base > int(line_roi.shape[1]*0.4) and abs(_right_base -_left_base) > 170):
                if(return_value != -1 and _left_base > 100 and _right_base > 200):
                    stage_cnt += 1
                
                    
                    if(stage_cnt > 8):
                        print("turn right done")
                        self.stage_num = 3
                        stage_cnt = 0
                        print("################## STAGE 3 ##################")
                        
                        self.past_time = self.current_time
                        print ("Stop")
                        
                        while self.current_time - self.past_time <= 3:
                            self.current_time = rospy.get_time()
                            self.angled = 0
                            self.speed = 0
                            self.unitydrive(self.angled, self.speed)   
                    
                else:
                    stage_cnt = 0
                    self.speed = 3.1
                    self.angled = -27.5
                
            
            ########### stage 3. move following the line  
            elif(self.stage_num == 3):
                self.lpf1 = LowPassFilter(cutoff_freq=0.05, ts = 0.06)
                self.lpf2 = LowPassFilter(cutoff_freq=0.05, ts = 0.06)
                
                #value = self.follow_the_line(left_yB_roi_lefthalf, _lbase, right_wL_roi_righthalf, _rbase)
                value = self.move_with_only_with_one_line(right_wL_roi_righthalf, _rbase, name="right")
                print("right line value, rbase: {}, {}".format(value, _rbase))
                self.speed = 3.1
                """if(value < -3):
                    self.angled = -30
                elif(value > 6):
                    self.angled = 30
                elif(value > -3 and value < 6):
                    self.angled = value*0.6"""
                    
                if(abs(value) < 3.0):
                    self.angled = value*0.6
                elif(abs(value) < 25):
                    self.angled = value*0.92
                elif(abs(value) < 40):
                    self.angled = value*0.8
                else:
                    self.angled = value
                
                if(self.angled < -40):
                    self.angled = -40
                elif(self.angled > 40):
                    self.angled = 40
                
                ### check stop sign
                crosswalk_img = self.crosswalk_image_create(line_roi, int(line_roi.shape[1]*0.5), window=5)
                cv2.imshow("crosswalk checker", crosswalk_img)
                
                if(self.crosswalk_histogram(crosswalk_img, 0.7, verbose=True)):
                    stage_cnt += 1
                    if(stage_cnt > 3):
                        print("\n\n :::::: crosswalk or stop sign ::::::")
                        self.stage_num = 4
                        print("################## STAGE 4 ##################")
                        
                        self.past_time = self.current_time
                        print ("Stop")
                        
                        while self.current_time - self.past_time <= 3:
                            self.current_time = rospy.get_time()
                            #self.angled = 0
                            self.speed = 0
                            self.unitydrive(self.angled, self.speed)  
                else:
                    stage_cnt = 0
                    
                    
            ########### stage 4. detect traffic lights, and go straight     
            elif(self.stage_num == 4):    
                print("stage 4.... need to make ")    
                _left_base, _right_base, crosswalk_ratio = self.plothistogram(line_roi)
                
                return_value = self.slide_window_search(line_roi, _left_base, _right_base)
                
                print("     2, left base, right base: {}, {}".format(_left_base, _right_base))
                
                if(return_value != -1 and _left_base > 200 and _left_base < int(line_roi.shape[1]/2) and _right_base > int(line_roi.shape[1]*0.55) and abs(_right_base -_left_base) > 150):
                    stage_cnt += 1
                    
                    if(stage_cnt > 10):
                        print("turn right done")
                        self.stage_num = 5
                        stage_cnt = 0
                        print("################## STAGE 5 ##################")
                        
                        self.past_time = self.current_time
                        print ("Stop")
                        
                        while self.current_time - self.past_time <= 3:
                            self.current_time = rospy.get_time()
                            #self.angled = 0
                            self.speed = 0
                            self.unitydrive(self.angled, self.speed)   
                    
                else:
                    stage_cnt = 0
                    self.speed = 3.1
                    self.angled = 0
                    
            
            ########### stage 5. move following the line
            elif(self.stage_num == 5):
                value = self.follow_the_line(left_yB_roi_lefthalf, _lbase, right_wL_roi_righthalf, _rbase)
                
                self.speed = 3.1
                """if(value < -3):
                    self.angled = -30
                elif(value > 6):
                    self.angled = 30
                elif(value > -3 and value < 6):
                    self.angled = value*0.3"""
                
                if(abs(value/2) < 10):
                    self.angled = value*0.55
                else:
                    self.angled = value * 1.2    
                    
                
                ### check stop sign
                crosswalk_img = self.crosswalk_image_create(line_roi, int(line_roi.shape[1]*0.65), window=3)
                cv2.imshow("crosswalk checker", crosswalk_img)
                
                if(self.crosswalk_histogram(crosswalk_img, 0.4, verbose=True)):
                    stage_cnt += 1
                    if(stage_cnt > 3):
                        print("\n\n :::::: crosswalk or stop sign ::::::")
                        self.stage_num = 6
                        print("################## STAGE 6 ##################")
                        
                        self.past_time = self.current_time
                        print ("Stop")
                        
                        while self.current_time - self.past_time <= 3:
                            self.current_time = rospy.get_time()
                            #self.angled = 0
                            self.speed = 0
                            self.unitydrive(self.angled, self.speed)   
                else:
                    stage_cnt = 0
            
            ########### stage 6. no line area.... turn left     
            elif(self.stage_num == 6):    
                print("stage 6.... need to make ") 
                


            ########### stage 7. move following the line  
            elif(self.stage_num == 7):
                if(self.value_flag == False):    
                    value = self.follow_the_line(left_yB_roi_lefthalf, _lbase, right_wL_roi_righthalf, _rbase)
                    self.min_value = min((self.min_value, value))
                    print("                     self min value: ", self.min_value)
                    
                    if(value > self.min_value):
                        line_cnt += 1
                        if(line_cnt > 15):
                            print(" >>>>>>>>>>>>>>> flag turned on,,, two line detecter on <<<<<<<<<<<<<<<<<")
                            line_cnt = 0
                            self.value_flag = True
                    else:
                        line_cnt = 0
                        
                else:
                    _left_base, _right_base, crosswalk_ratio = self.plothistogram(line_roi)
                    return_value = self.slide_window_search(line_roi, _left_base, _right_base)
                    
                    if(return_value == -1):
                        value = value
                    else:
                        value = 0    
                        print("after max, est center: ", self.estimated_center)
                        # self.estimated_center가 양수이면 우회전해야하는 것이다."""

                self.speed = 3.1
                """if(value < -3):
                    self.angled = -30
                elif(value > 6):
                    self.angled = 30
                elif(value > -3 and value < 6):
                    self.angled = value*0.3"""
                    
                if(abs(value/2) < 10):
                    self.angled = value*0.65
                else:
                    self.angled = value * 1.1        
                
                ### check stop sign
                crosswalk_img = self.crosswalk_image_create(line_roi, int(line_roi.shape[1]*0.5), window=5)
                cv2.imshow("crosswalk checker", crosswalk_img)
                
                if(self.crosswalk_histogram(crosswalk_img, 0.65, verbose=True)):
                    stage_cnt += 1
                    if(stage_cnt > 2):
                        print("\n\n :::::: crosswalk or stop sign ::::::")
                        value_flag = False
                        max_value = 0
                        
                        break
                        
                else:
                    stage_cnt = 0
            
                    
            """ stop sign
           
                
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
            
            """
            
            
            #print("center left, right: {}, {}".format(self.estimated_center_left, self.estimated_center_right))
            #self.angled = (self.estimated_center_left + self.estimated_center_right) * 1.2
            
            
            #print("angle: ", self.angled)
            # angle이 음수이면 좌회전, 양수이면 우회전이다.
            # left, right 절댓값이 10넘어가면 angle = 30
            
            
            
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
            

            #self.angled = 0
            #self.speed = 0.0
            if(self.angled > 50):
                self.angled = 50
            if(self.angled < -50):
                self.angled = -50
            self.estimated_center = value    
            # Publish xycar motor & unity motor
            #self.angled = 30
            # 양수가 우회전

            self.unitydrive(self.angled, self.speed)
            
            
            # Check Image
            original_img = self.image   .copy()
            cv2.putText(original_img, 'Time : ', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (225, 225, 225), 1, cv2.LINE_AA)
            cv2.putText(original_img, str(self.current_time), (80, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (225, 225, 225), 1, cv2.LINE_AA)
            cv2.putText(original_img, 'Speed : ', (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (225, 225, 225), 1, cv2.LINE_AA)
            cv2.putText(original_img, str(self.speed), (80, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (225, 225, 225), 1, cv2.LINE_AA)
            
            cv2.putText(original_img, 'estimated center : ', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (225, 225, 225), 1, cv2.LINE_AA)
            cv2.putText(original_img, str(self.estimated_center), (160, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (225, 225, 225), 1, cv2.LINE_AA)

            cv2.putText(original_img, 'Angled : ', (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (225, 225, 225), 1, cv2.LINE_AA)
            cv2.putText(original_img, str(self.angled), (80, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (225, 225, 225), 1, cv2.LINE_AA)

            #robotvision_horizontal = np.hstack((original_img, img))
            cv2.imshow("RobotVision", original_img)
            #cv2.imshow("line_img", line_img)
            #cv2.imshow("edge_img", edge_img)
            cv2.waitKey(1)


if __name__ == '__main__':
    
    
    RVS = Robotvisionsystem()
    
    _timestamp_left = [i for i in range(len(RVS.center_left_data))]
    _timestamp_right = [i for i in range(len(RVS.center_right_data))]
    _timestamp_sum = [i for i in range(len(RVS.sum))]

    plt.subplot(311)
    plt.plot(_timestamp_left, RVS.center_left_data, 'r')
    plt.plot(_timestamp_left, RVS.filtered_value_left_data, 'b')
    plt.title("left")
    
    plt.subplot(312)
    plt.plot(_timestamp_right, RVS.center_right_data, 'r')
    plt.plot(_timestamp_right, RVS.filtered_value_right_data, 'b')
    plt.title("right")
    
    plt.subplot(313)
    plt.plot(_timestamp_sum, RVS.sum, 'r')
    plt.title("left + right")
    
    plt.show()

