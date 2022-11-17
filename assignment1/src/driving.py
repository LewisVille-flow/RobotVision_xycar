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


def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class Robotvisionsystem:
    def __init__(self):
        self.image = np.empty(shape=[0])
        self.realimage = np.empty(shape=[0]) 
        self.bridge = CvBridge() 
        self.motor = None 
        self.angled = 0
        self.speed = 0.1
        self.stop = 0

        self.CAM_FPS = 30
        self.WIDTH, self.HEIGHT = 640, 480

        rospy.init_node('driving')
        
        # self.motor = rospy.Publisher('/xycar_motor', xycar_motor, queue_size=1)
        # self.real_image = rospy.Subscriber('/usb_cam/image_raw/compressed',CompressedImage, self.realimg_callback)

        self.unitymotor = rospy.Publisher('/unitymotor', PoseStamped, queue_size=1)
        self.unity_img = rospy.Subscriber('/unitycamera', CompressedImage , self.img_callback)

        print ("----- Xycar self driving -----")
        self.start()    

    def img_callback(self, data):
        # print data
        try:
            self.image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8") # mono8, mono16, bgr8, rgb8, bgra8, rgba8, passthrough
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
    def nothing(self):
    	pass
 
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
            
            img = self.image.copy()
            
            blur = cv2.GaussianBlur(img, (5, 5), 0)
            H, L, S = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HLS))
            _, L = cv2.threshold(L, 75, 110, cv2.THRESH_BINARY)
            edge_img = cv2.Canny(np.uint8(L), 60, 70)
            
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
            #self.speed = 0
            
            # Publish xycar motor & unity motor
            self.unitydrive(self.angled, self.speed)
            
            # Check Image
            original_img = self.image.copy()
            cv2.putText(original_img, 'Time : ', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (125, 125, 125), 1, cv2.LINE_AA)
            cv2.putText(original_img, str(self.current_time), (80, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (125, 125, 125), 1, cv2.LINE_AA)
            cv2.putText(original_img, 'Speed : ', (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (125, 125, 125), 1, cv2.LINE_AA)
            cv2.putText(original_img, str(self.speed), (80, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (125, 125, 125), 1, cv2.LINE_AA)
            cv2.putText(original_img, 'Angled : ', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (125, 125, 125), 1, cv2.LINE_AA)
            cv2.putText(original_img, str(self.angled), (80, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (125, 125, 125), 1, cv2.LINE_AA)

            robotvision_horizontal = np.hstack((original_img, img))
            cv2.imshow("RobotVision", robotvision_horizontal)
            cv2.imshow("line_img", line_img)
            cv2.imshow("edge_img", edge_img)
            cv2.waitKey(1)
            
            
            
if __name__ == '__main__':
    RVS = Robotvisionsystem()

