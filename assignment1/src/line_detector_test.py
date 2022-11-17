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
        self.image = cv2.imread('3.png', cv2.IMREAD_COLOR)
        self.Height, self.Width, self.Channel = self.image.shape

        self.ROI_row_percent = 0.30 # 하단 30%
        self.ROI_col_percent = 0.80 # 중앙 80% 를 보고 싶다.

        self.Row_ROI_from = self.Height - int(self.Height*self.ROI_row_percent)
        self.Row_ROI_to = self.Height
        self.Col_ROI_from = int(self.Width * (1-self.ROI_col_percent)*(0.5))
        self.Col_ROI_to = int(self.Width-((1-self.ROI_col_percent)*(0.5)*self.Width))
           
        self.Center_pos = int(self.Width * 0.5)
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

    def plothistogram(self, image):
        histogram = np.sum(image[image.shape[0]//2:, :], axis=0)
        midpoint = np.int(histogram.shape[0]/2)
        leftbase = np.argmax(histogram[:midpoint])
        rightbase = np.argmax(histogram[midpoint:]) + midpoint
        
        return leftbase, rightbase

    def slide_window_search(self, binary_warped, left_current, right_current):
        out_img = np.dstack((binary_warped, binary_warped, binary_warped))

        nwindows = 4
        window_height = np.int(binary_warped.shape[0] / nwindows)
        nonzero = binary_warped.nonzero()  # 선이 있는 부분의 인덱스만 저장 
        print("nonzero: {}".format(nonzero))
        nonzero_y = np.array(nonzero[0])  # 선이 있는 부분 y의 인덱스 값
        nonzero_x = np.array(nonzero[1])  # 선이 있는 부분 x의 인덱스 값 
        margin = 100
        minpix = 50
        left_lane = []
        right_lane = []
        color = [0, 255, 0]
        thickness = 2

        for w in range(nwindows):
            win_y_low = binary_warped.shape[0] - (w + 1) * window_height  # window 윗부분
            win_y_high = binary_warped.shape[0] - w * window_height  # window 아랫 부분
            win_xleft_low = left_current - margin  # 왼쪽 window 왼쪽 위
            win_xleft_high = left_current + margin  # 왼쪽 window 오른쪽 아래
            win_xright_low = right_current - margin  # 오른쪽 window 왼쪽 위 
            win_xright_high = right_current + margin  # 오른쪽 window 오른쪽 아래

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
            print("good_left good_left: {}, {}".format(good_left, good_right))
        
        print("left_lane left_lane: {}, {}".format(left_lane, right_lane))
        
        left_lane = np.concatenate(left_lane)  # np.concatenate() -> array를 1차원으로 합침
        right_lane = np.concatenate(right_lane)

        leftx = nonzero_x[left_lane]
        lefty = nonzero_y[left_lane]
        rightx = nonzero_x[right_lane]
        righty = nonzero_y[right_lane]
        print("nonzero_x nonzero_y: {}, {}".format(nonzero_x, nonzero_y))
        print("left_lane right_lane: {}, {}".format(left_lane, right_lane))
        
        # 얘네가 빈 리스트이다.
        
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
        left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

        ltx = np.trunc(left_fitx)  # np.trunc() -> 소수점 부분을 버림
        rtx = np.trunc(right_fitx)
        

        out_img[nonzero_y[left_lane], nonzero_x[left_lane]] = [255, 0, 0]
        out_img[nonzero_y[right_lane], nonzero_x[right_lane]] = [0, 0, 255]
        

        cv2.imshow("out_img", out_img)
        '''plt.imshow(out_img)
        plt.plot(left_fitx, ploty, color = 'yellow')
        plt.plot(right_fitx, ploty, color = 'yellow')
        plt.xlim(0, 1280)
        plt.ylim(720, 0)
        plt.show()'''

        ret = {'left_fitx' : ltx, 'right_fitx': rtx, 'ploty': ploty}

        return ret

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
    
    def start(self):
        
        
        img = self.image.copy()
        blur = cv2.GaussianBlur(img, (5, 5), 0)
        H, L, S = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HLS))
        _, L = cv2.threshold(L, 75, 110, cv2.THRESH_BINARY)
        edge_img = cv2.Canny(np.uint8(L), 60, 70)
        
        
        line_roi = edge_img[self.Row_ROI_from:self.Row_ROI_to, self.Col_ROI_from:self.Col_ROI_to]
        _line_roi_h, _line_roi_w = line_roi.shape
        all_lines = cv2.HoughLinesP(line_roi, 1, math.pi/180,30,10,10)
        
        # warpping
        #_gray = cv2.cvtColor(line_roi, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(line_roi, 75, 110, cv2.THRESH_BINARY)
        
        wrapped_img, _back = self.wrapping(line_roi)
        cv2.imshow("wrapped_img", wrapped_img)
        
        # histogram of white line
        left_base, right_base = self.plothistogram(thresh)
        
        draw_info = self.slide_window_search(thresh, left_base, right_base)
        #m, r = self.draw_lane_lines(line_roi, wrapped_img, _back, draw_info)
        
        
        """
        긁어온 함수 테스트 - cv2.poly 안되서 일단 보류중
        #averaged_lines = self.average_slope_intercept(img, all_lines)
        #lines_image = self.show_lines(img, averaged_lines)
        
        #combine_image = cv2.addWeighted(img, 0.8, lines_image, 1, 1)
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
            cv2.line(self.image, (self.Center_pos, 0), (self.Center_pos, self.Width), (0, 255, 255), 1) # 세로선_1
            cv2.line(self.image, (320, 0), (320, 480), (0, 255, 255), 1) # 세로선_2
            
            cv2.imshow("line_img", line_img)
            cv2.imshow("self_image", self.image)
            cv2.imshow("line_roi", line_roi)
            #_x, _y, _w, _h = cv2.selectROI("location", line_roi, False)
            #print("x, y, w, h : {}, {}, {}, {}".format(_x, _y, _w, _h) )
            
            
        if cv2.waitKey() == ord('q'):
            cv2.destroyAllWindows()
        """
        안쪽    x, y, w, h : 305, 3, 172, 141
        바깥쪽  x, y, w, h : 201, 3, 392, 141
        
        """
        
if __name__ == '__main__':
    TEST = Line_Detect()