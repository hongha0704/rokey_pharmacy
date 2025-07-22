💊 Rokey_Pharmacy
===
ROKEY B-1조 협동-2 Project (AI기반 협동 로봇 작업 어시스턴트 구현 프로젝트)
---

### 🔗 출처 및 라이선스

이 프로젝트는 **두산로보틱스(Doosan Robotics Inc.)**에서 배포한 ROS 2 패키지를 기반으로 합니다.  
해당 소스코드는 [BSD 3-Clause License](https://opensource.org/licenses/BSD-3-Clause)에 따라 공개되어 있으며,  
본 저장소 또한 동일한 라이선스를 따릅니다. 자세한 내용은 `LICENSE` 파일을 참고하시기 바랍니다.

> ⚠️ 본 저장소는 두산로보틱스의 공식 저장소가 아니며, 비공식적으로 일부 수정 및 구성을 포함하고 있습니다.  
> 공식 자료는 [두산로보틱스 공식 홈페이지](http://www.doosanrobotics.com/kr/)를 참고해 주세요.   
> github (https://github.com/DoosanRobotics/doosan-robot2)
---

### 🔨 개발환경
본 프로젝트는 Ubuntu 22.04 (ROS2 humble) 환경에서 개발되었습니다.   
&nbsp;

### 🦾 작업공간
<img src="rokey_project/image/workspace/IMG_3175.jpg" width="50%" height="50%" title="px(픽셀) 크기 설정" alt="project_management"></img>   
&nbsp;

### 💻 코드 실행

#### **robot control node**
code: [main_robot_control](rokey_project/rokey_project/main_robot_control.py)
```bash
ros2 run rokey_project main_robot_control
```

#### **vision**
code: [main_vision_realsense](rokey_project/rokey_project/main_vision_realsense.py)
```bash
ros2 run rokey_project main_vision_realsense
```
&nbsp;

### 📷 시연 영상
https://youtu.be/FMOeqKwD2Ls

---

&nbsp;

## 목차

#### [1. 📘 프로젝트 개요](#1--프로젝트-개요-1)   
#### [2. 👥 프로젝트 팀 구성 및 역할분담](#2--프로젝트-팀-구성-및-역할분담-1)   
#### [3. 🗓 프로젝트 구현 일정](#3--프로젝트-구현-일정-1)   
#### [4. 📌 SKILLS](#4--skills-1)   
#### [5. 🤖 Hardware](#5--hardware-1)   
#### [6. 🎬 System Flow](#6--system-flow-1)   
#### [7. 🛠️ Node Architecture](#7-%EF%B8%8F-node-architecture-1)   
#### [8. ✨ 주요 기능](#8--주요-기능)   
#### [9. 🔍 프로젝트 기대효과](#9--프로젝트-기대효과-1)   



---

&nbsp;

## 1. 📘 프로젝트 개요
의료 현장에서 의사·간호사·약사 등의 인력 부족, 처방·조제·투약 등 여러 단계에서 발생하는 문제로 투약 사고가 발생하고 있습니다. 이는 환자의 안전에 위해가 되는 만큼, 업무 효율화를 통한 안전한 투약 환경을 만들 필요가 있습니다.   
따라서 본 프로젝트에서는, 로봇 매니퓰레이터와 AI 비전 기술을 활용하여 약 조제를 자동화하는 시스템을 만들어 의료진의 업무 부담을 덜고, 의료 사고 발생율을 줄이고자 합니다.   

### **프로젝트 목표**
1. **약 조제 오류로 인한 사고 방지**   
AI Vision 기술을 활용한 약 분류 및 조제로 안정성 향상   
2. **의료 인력 부족 해결**   
약 조제의 자동화로 의료진 업무 부담 감소 및 인력 부족 문제 해결   
3. **고령화 대응**   
스스로 약을 복용하기 어려운 고령층을 위해 AI Voice를 활용한 음성 안내 서비스 제공

&nbsp;

## 2. 👥 프로젝트 팀 구성 및 역할분담
|이름|담당 업무|
|--|--|
|백홍하(팀장)|Vision(약 탐지, 좌표 추정), Robot 제어, ROS2 통신|
|서형원|Robot 제어(일반 의약품 전달)|
|정민섭|Voice(약 추천 및 설명), ROS2 통신, 초음파 센서 처리|
|정서윤|Vision(QR코드 인식, 약 탐지, 서랍 text 분류), 통합|

&nbsp;

## 3. 🗓 프로젝트 구현 일정
**진행 일자: 25.5.26(월) ~ 25.6.5(목) (11일)**
<img src="rokey_project/image/notion/250717_project_management.png" width="100%" height="100%" title="px(픽셀) 크기 설정" alt="project_management"></img>

&nbsp;

## 4. 📌 SKILLS
### **Development Environment**
<div align=left>
  
  ![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)
  ![Visual Studio Code](https://img.shields.io/badge/Visual%20Studio%20Code-0078d7.svg?style=for-the-badge&logo=visual-studio-code&logoColor=white)
</div>

[![My Skills](https://skillicons.dev/icons?i=ubuntu,vscode&theme=light)](https://skillicons.dev)

### **Robotics**
![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)   
[![My Skills](https://skillicons.dev/icons?i=ros&theme=light)](https://skillicons.dev)

### **Programming Languages**
![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)   
[![My Skills](https://skillicons.dev/icons?i=python&theme=light)](https://skillicons.dev)

### **AI & Computer Vision**
<div align=left>
  
  ![PyTorch](https://img.shields.io/badge/PyTorch-%23EE4C2C.svg?style=for-the-badge&logo=PyTorch&logoColor=white)
  ![OpenCV](https://img.shields.io/badge/opencv-%23white.svg?style=for-the-badge&logo=opencv&logoColor=white)
  ![YOLO](https://img.shields.io/badge/YOLO-111F68?style=for-the-badge&logo=YOLO&logoColor=white)
</div>

[![My Skills](https://skillicons.dev/icons?i=pytorch,opencv&theme=light)](https://skillicons.dev) 

&nbsp;

## 5. 🤖 Hardware
### **Robot**
- Doosan Robotics m0609, OnRobot RG2 Gripper
### **Vision Camera**
- Intel RealSense D435i
### **SBC**
- Raspberrypi4 4gb
### **Mic**
- Logitech HD Webcam C270
### **Speaker**
- Blutooth speaker
### **Sensor**
- HC-SRO4 Ultrasonic Sensor

&nbsp;

## 6. 🎬 System Flow
<img src="rokey_project/image/system_flow/ROKEY_Pharmacy_detail.drawio.png" width="75%" height="75%" title="px(픽셀) 크기 설정" alt="system_flow"></img>

&nbsp;

## 7. 🛠️ Node Architecture
<img src="rokey_project/image/node_architecture/250717_node_architecture.png" width="75%" height="75%" title="px(픽셀) 크기 설정" alt="system_flow"></img>

&nbsp;

## 8. ✨ 주요 기능
### 1. 초음파 센서 사람 감지
- 5~37 cm 거리에 있는 사용자 3초 이상 감지   
- moving average 필터   
<img src="https://github.com/user-attachments/assets/d390d406-e97c-4f00-8c67-663e4a36e04a" width="50%" height="50%" title="px(픽셀) 크기 설정" alt="image"></img>

&nbsp;
### 2. 처방전 QR 인식 자세 & voice 약국 안내 음성
- 로봇의 동작은 모두 movesj 로 자연스럽게 연결   
- voice: “안녕하세요 rokey약국입니다. qr을 스캔하거나 hello rokey를 말해주세요”  
<img src="https://github.com/user-attachments/assets/81733333-48ba-4921-8eac-3c5cf5c7d0fd" width="50%" height="50%" title="px(픽셀) 크기 설정" alt="image"></img>

&nbsp;
### 3. QR코드 인식
<img src="https://github.com/user-attachments/assets/af6e1221-5292-4f17-baa2-be276dffb9e0" width="50%" height="50%" title="px(픽셀) 크기 설정" alt="image"></img>
<img src="https://github.com/user-attachments/assets/44a135ab-882e-4c1f-ba33-fa2a725d61aa" width="30%" height="30%" title="px(픽셀) 크기 설정" alt="image"></img>

&nbsp;
### 4. 서랍 바라보는 모션 / 서랍 text 인식
<img src="https://github.com/user-attachments/assets/8a1c71d9-1aed-4b71-9499-7248f7c92c85" width="50%" height="50%" title="px(픽셀) 크기 설정" alt="image"></img>
<img src="https://github.com/user-attachments/assets/df43f7af-182a-4cc6-8a96-5676fc8755ae" width="30%" height="30%" title="px(픽셀) 크기 설정" alt="image"></img>

&nbsp;
### 5. 인식한 text 서랍 열기
<img src="https://github.com/user-attachments/assets/1cd024d4-4a71-4e48-9322-bc1b2d39cb87" width="50%" height="50%" title="px(픽셀) 크기 설정" alt="image"></img>

&nbsp;
### 6. 서랍 안 바라보는 자세 이동
<img src="https://github.com/user-attachments/assets/73f02dd5-6f5a-4c0c-83bf-4d5501e130c1" width="50%" height="50%" title="px(픽셀) 크기 설정" alt="image"></img>

&nbsp;
### 7. 약 탐지
- YOLO 활용 약 구분   
- 타원형 알약은 instance segmentation을 활용하여 타원 모양 추정 -> 회전각 theta 계산   
- 알약의 중심점 좌표와 회전각 정보(x, y, theta)를 robot_control_node에 퍼블리시   
<img src="https://github.com/user-attachments/assets/b68e3548-eaa7-49d4-8773-d03e36454e04" width="40%" height="40%" title="px(픽셀) 크기 설정" alt="image"></img>
<img src="https://github.com/user-attachments/assets/82cb9e45-f7db-4e4d-a0e6-d1cebfa76185" width="40%" height="40%" title="px(픽셀) 크기 설정" alt="image"></img>

&nbsp;
### 8. 약 pick and place
- gripper 너비 알약 크기에 맞춰 조정   
- 카메라캘리브레이션을 활용하여 알약의 (x, y) 정보를 월드좌표로 변환한 후 로봇 해당 위치로 이동   
- 알약의 theta 값 만큼 로봇 6축 회전 후 pick
- 알약을 약봉투에 넣을 때 위아래로 2번 흔드는 모션 추가하여 잘 떨어지지 않을 때를 방지   
<img src="https://github.com/user-attachments/assets/9a691ccf-ccf2-4e91-a2be-dbb930160938" width="50%" height="50%" title="px(픽셀) 크기 설정" alt="image"></img>

&nbsp;
### 9. 서랍 넣기
- 알약을 모두 처방한 후 서랍 넣기   
- 서랍을 살짝 들고 넣기 (서랍의 바닥면 마찰 최소화)   
- 마무리로 밀어 넣기   
<img src="https://github.com/user-attachments/assets/9e0def9a-e57a-4792-8128-c4c5f8eef28d" width="50%" height="50%" title="px(픽셀) 크기 설정" alt="image"></img>

&nbsp;
### 10. 약 포장 대기 상태 이동  
- 약사가 약을 포장하고 신호를 줄 때까지 대기   
<img src="https://github.com/user-attachments/assets/8b859276-1b33-4857-8473-1b3b172934a5" width="50%" height="50%" title="px(픽셀) 크기 설정" alt="image"></img>

&nbsp;
### 11. 약사 약 포장 후 신호
- 약사가 약을 검사 후 포장
- 그리퍼 사이에 약봉지를 끼워 넣고 x축 방향으로 외력을 주어 신호 전달   
<img src="https://github.com/user-attachments/assets/b2ac0cbc-f5f4-41fa-9217-4e2ca8568bb1" width="50%" height="50%" title="px(픽셀) 크기 설정" alt="image"></img>

&nbsp;
### 12. 약 봉투에 넣기 / 해당 약에 대한 설명
- 외력 신호를 받은 로봇은 약을 약 봉투에 넣음   
- voice로 처방한 약에 대한 설명 (예: “해당 약은 위염치료제이며 다른 약 복용시 위 손상을 막아줍니다. 감사합니다 안녕히가세요.”)   
<img src="https://github.com/user-attachments/assets/e8281509-4721-49a5-9d01-cc0180c0407e" width="50%" height="50%" title="px(픽셀) 크기 설정" alt="image"></img>

&nbsp;

## 9. 🔍 프로젝트 기대효과
### **활용 방안**
- 약국 내 조제 공정
- 의료 어시스턴트
- 창고, 서랍 정리

### **기대 효과**
- 약물 사고 예방 → 의료 사고, 부작용 최소화
- 약사의 단순 반복업무 감소 → 핵심 업무 집중 가능
- 팬데믹 등 상황에서 비대면 복약 시스템 활용 가능

&nbsp;
