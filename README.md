# RobotVision_xycar

### Purpose of project
자율주행에 쓰이는 로봇비전시스템에 대한 이해 및 실습을 위한 OpenCV, Xycar X model을 이용한 Term Project(2022-2 ROB4007).

### 완료 영상
<p align="center">
	<img src="./Assignment1/src/image/submission/1.gif" width=80% height=80%>
	<figcaption align="center">[사진 1] Unity 환경에서 opencv를 이용하여 라인 검출 및 주행. 2.5배속.</figcaption>
</p>

<p align="center">
	<img src="./Assignment2/1.gif" width=75% height=80%>
	<figcaption align="center">[사진 2] 실험실에서의 OpenCV 라인검출 동작 테스트, part2. 2배속.</figcaption>
</p>   

*** 


**Assignment1**: OpenCV로 차선 인식하기(in Unity)
	- 시작지점에서부터 종료지점까지, 유니티 환경에서 OpenCV 함수들을 이용하여 차선을 인식한 주행을 완료하는 것.
<p align="center">
	<img src="./image/5.png" width="250" height="250">
	<img src="./image/1.png" width="250" height="250">
</p>  

**Assignment2**: Simulation to Real
	- 실험실 환경에서 과제 1이 잘 작동하는지 확인하고, Real enviornment에서 잘 동작할 수 있도록 완료하는 것.
<p align="center"><img src="./image/8.png" width=70% height=70%></p>  

# How to start
### Setup Guide for different OS
Nvidia AGX Xavier installed in Xycar is called Single-board computer (SBC). Your Computer is called PC. Data from many sensor installed in Xycar sent to the SBC. If you want see sensor data, you do connect your PC to SBC. Nvidia AGX Xavier OS is Jetpack 4.2.3 & ROS1 Melodic. Recommend PC OS is Ubuntu 18.04 & Windows10 or 11.

[1. Windows 10 + WSL2](./GettingStartedGuide/windows.md)  
[2. Ubuntu 18.04](./GettingStartedGuide/ubuntu.md)

### Start Guide for Ubuntu 18.04
Window, Mac OS에서 동작하는지는 확인해보지 않았습니다. Setup 가이드를 참고하세요.

**Assignment 1**
```bash
# (1) Terminal 1 
$ roslaunch rosbridge_server rosbridge_websocket.launch
```  
```bash  
# (2) Terminal 2
# Unity is on, Input Ubuntu IP(localhost). => "ws://localhost:9090" and Press "Enter".
$ ~/xycar_ws/src/RobotVision_xycar/Build/Linux/Start/RVS_start.x86_64

# If you input right host ip, ros can connect RVS. 
$ ~/xycar_ws/src/RobotVision_xycar/Build/Linux/Xycar/RVS.x86_64
```
```bash
# (3) Terminal 3
$ roslaunch assignment1 driving.launch   
```

**Assignment 2**
Assignment 2는 laptop과 Xycar가 같은 네트워크에 접속해있어야 하며, network ip 주소가 필요하다.
```bash
# (1) laptop setting
# bashrc에서 ROS_MASTER_URI 와 ROS_HOSTNAME을 laptop ip 로 변경해야 한다.
# laptop ip를 192.168.x.x라고 했을 때
$ export ROS_MASTER_URI=http://192.168.x.x:11311 
$ export ROS_HOSTNAME=192.168.x.x
# 이후 bashrc에서 이전에 ROS_MASTER_URI, ROS_HOSTNAME 을 localhost라고 해둔 것을 # 처리할 것.
$ gedit ~/.bashrc
```  
  
Xycar에 이미 launch file이 만들어져있다고 가정하겠다.
```bash
# (2) Xycar start
# launch file이 unity가 아닌 real로 세팅되어있는지 확인 후 실행.
# Xycar와 ssh 연결 후, Xycar에서
$ roslaunch assignment1 real_driving.launch
# laptop에서도
$ roslaunch assignment1 driving.launch
```

### Update
2022.12 Project 완료  
2023.02 Project Description update
