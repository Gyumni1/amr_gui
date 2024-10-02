from threading import Thread

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import QtGui, uic
from PyQt5.QtCore import *

import rclpy as rp
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile, qos_profile_sensor_data

#from interfaces_pkg.msg import *
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import PoseWithCovarianceStamped 

from cv_bridge import CvBridge

from ament_index_python.packages import get_package_share_directory


import numpy as np
import signal
import cv2
import sys
import os
import yaml
import time

# 현재 파일의 디렉토리 경로를 가져옵니다.
# current_dir = os.path.dirname(os.path.abspath(__file__))

# ui_file = os.path.join(get_package_share_directory('ui_pkg'), 'ui', 'monitor.ui')
ui_file = os.path.join(get_package_share_directory('ui_pkg'), 'ui', 'odm.ui')
# map_yaml_file = os.path.join(get_package_share_directory('main_pkg'), 'map', 'home.yaml')# 
yaml_file = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml')

print(yaml_file)
# yaml_file = os.path.join(current_dir, 'map.yaml')
# map_yaml_file = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml')# 
from_class = uic.loadUiType(ui_file)[0]

#image file is nav2 yaml file.
with open(yaml_file, 'r') as file:
    map_yaml_data = yaml.full_load(file)
    image_file = map_yaml_data['image']

current_dir = os.path.dirname(yaml_file)

image_path = os.path.join(current_dir, image_file)

if not os.path.exists(image_path):
    print(f"Image file does not exist: {image_path}")
else:
    print(f"Image file loaded from: {image_path}")


global amcl1
amcl1 = PoseWithCovarianceStamped()

# global path_1, path_before_1
# path_1 = AstarMsg() #####
# path_before_1 = AstarMsg()

global start_point_1
# robot_positions= [-1.75, -0.6]
robot_positions= [50, 50]

class AmclSubscriber(Node):

    def __init__(self):
        super().__init__('amcl_subscriber')
        amcl_pose_qos = QoSProfile(
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1)
        
        self.pose1 = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/amcl_pose', 
            self.amcl_callback, 
            amcl_pose_qos)
        
    def amcl_callback(self, amcl):
        global amcl1
        amcl1 = amcl
        robot_positions[0] = amcl.pose.pose.position.x
        robot_positions[1] = amcl.pose.pose.position.y



class PathSubscriber(Node):
    
    def __init__(self):
        super().__init__('path_subscriber')
        
        # self.sub1 = self.create_subscription(
        #     AstarMsg,      #####
        #     '/astar_paths',
        #     self.path_callback,
        #     10
        # )
    
        
    def path_callback(self, path):
        global path_1, amcl1, start_point_1
        path_1 = path
        start_point_1 = amcl1
        
        

        

class CamSubscriber(Node):
    def __init__(self, ui):
        super().__init__('cam_subscriber')
        self.ui = ui

        # RealSense 카메라에서 이미지 수신
        # self.sub1 = self.create_subscription(
        #     CompressedImage,  # 또는 Image 타입으로 바꿀 수 있습니다.
        #     '/camera/image_raw',  # 사용 중인 주제로 변경
        #     self.listener_callback,
        #     qos_profile_sensor_data)
        self.sub1 = self.create_subscription(
            Image,  # 또는 Image 타입으로 바꿀 수 있습니다.
            '/camera/image_raw',  # 사용 중인 주제로 변경
            self.listener_callback,
            qos_profile_sensor_data)
        
        self.bridge = CvBridge()
    
    # def listener_callback(self, data):
    #     # 수신한 압축된 이미지를 NumPy 배열로 변환
    #     np_arr = np.frombuffer(data.data, np.uint8)
        

        # image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # 이미지 디코딩
        # if image_np is None or image_np.size == 0:
        #     print("Error: Received empty image")
        #     return
        # print(f"Received image with shape: {image_np.shape}")
        # image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)  # 색상 변환
        # height, width, channel = image_np.shape
        # bytes_per_line = 3 * width
        # q_image = QImage(image_np.data, width, height, bytes_per_line, QImage.Format_RGB888)  # QImage 생성
        # self.pixmap = QPixmap.fromImage(q_image)  # QPixmap 생성
        # self.pixmap = self.pixmap.scaled(self.ui.cam_label.width(), self.ui.cam_label.width())  # 크기 조정
        # self.ui.cam_label.setPixmap(self.pixmap)  # GUI 업데이트
    def listener_callback(self, data):
        width = data.width
        height = data.height
        bytes_per_line = 3 * width
        q_image = QImage(data.data, width, height, bytes_per_line, QImage.Format_RGB888)
        self.pixmap = QPixmap.fromImage(q_image)  # QPixmap 생성
        self.pixmap = self.pixmap.scaled(self.ui.cam_label.width(), self.ui.cam_label.width())  # 크기 조정
        self.ui.cam_label.setPixmap(self.pixmap)  # GUI 업데이트

        
# class WindowClass(QMainWindow, from_class):
# class WindowClass(QDialog, from_class):
class WindowClass(QDialog, from_class):        

    def __init__(self):
        super().__init__()

        try:
            self.setupUi(self)
        except Exception as e:
            print(f"Error during setupUi: {e}")


        # self.setupUi(self)
        self.setWindowTitle('Dialog') #????

        self.load_map_image()

        self.map_timer = QTimer(self)
        self.map_timer.timeout.connect(self.updateMap)
        self.map_timer.start(200)

        # timer = QTimer(self)
        # timer.timeout.connect(self.time)
        # timer.timeout.connect(self.updateMap)
        # timer.start(200)

        # self.time = 0        
        # self.follow_label.setText('Wait 🔴')
        # self.follow_node = rp.create_node('following_mode')
        # self.follow_publisher = self.follow_node.create_publisher(String, '/follow', 10)


        # # 사람 선택
        # self.isCaptureOn = False
        # self.capture_btn.setText('Capture Person')
        # self.capture_label.hide()
        # self.capture_btn.clicked.connect(self.clickCapture)

        # self.capture_node = rp.create_node('capture_mode')
        # self.capture_publisher = self.capture_node.create_publisher(String, '/capturing', 10)
#-------------------------------
        # with open(map_yaml_file) as f:
        #     map_yaml_data = yaml.full_load(f)

        # self.pixmap = QPixmap(os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'map', map_yaml_data['image']))
        # # self.pixmap = QPixmap(os.path.join(get_package_share_directory('main_pkg'), 'map', map_yaml_data['image']))
        # self.height = self.pixmap.size().height()
        # self.width = self.pixmap.size().width()
        # self.image_scale = 6
        # self.pixmap = self.pixmap.transformed(QTransform().scale(-1, -1))
        # self.map.setPixmap(self.pixmap.scaled(self.width * self.image_scale, self.height * self.image_scale, Qt.KeepAspectRatio))
    
        # self.now_x = 0
        # self.now_y = 0

        # self.map_resolution = map_yaml_data['resolution']
        # self.map_origin = map_yaml_data['origin'][:2]

    def load_map_image(self):
        # map QLabel 객체 가져오기     find map_label
        self.map_label = self.findChild(QLabel, 'map')

        if self.map_label is None:
            print("Failed to find QLabel named 'map'")
            return

        # 이미지 불러오기     
        self.pixmap = QPixmap(image_path)

        if self.pixmap.isNull():
            print(f"Failed to load image from: {image_path}")
            return

        # 이미지 크기 가져오기
        self.height = self.pixmap.size().height()
        self.width = self.pixmap.size().width()

        print(f"Map size: {self.width} x {self.height} pixels")

        # 이미지 스케일 설정
        self.image_scale = 2

        # 이미지 변환 (-90도 회전)
        # transform = QTransform().rotate(-90)
        # rotated_pixmap = self.pixmap.transformed(transform)
        rotated_pixmap = self.pixmap

        # 이미지 이동 설정 (왼쪽으로 20픽셀 이동, 위로 20픽셀 이동)
        translated_pixmap = QPixmap(rotated_pixmap.size())

        painter = QPainter(translated_pixmap)

        if not painter.isActive():
            print("Painter is not active")
            return
        move_x = 70  # 왼쪽으로 70픽셀 이동
        move_y = 0  # 위로 0픽셀 이동
        painter.drawPixmap(-move_x, -move_y, rotated_pixmap)  # x 좌표를 -로 설정하여 왼쪽으로 이동
        painter.end()

        # QLabel 크기에 맞게 이미지 조정 및 설정
        scaled_pixmap = translated_pixmap.scaled(self.width * self.image_scale, self.height * self.image_scale, Qt.KeepAspectRatio)
        self.map_label.setPixmap(scaled_pixmap)

        # map resolution 및 origin 설정
        self.map_resolution = map_yaml_data['resolution']
        self.map_origin = map_yaml_data['origin'][:2]
        # print(f"Map resolution, map_origin : {self.map_resolution} x {self.map_origin} ")

        self.previous_positions = [] 

    def updateMap(self):
        # self.map.setPixmap(self.pixmap.scaled(self.width * self.image_scale, self.height * self.image_scale, Qt.KeepAspectRatio))
        # painter = QPainter(self.map.pixmap())
        #####change part#######
        updated_pixmap = QPixmap(self.map_label.pixmap())
        painter = QPainter(updated_pixmap)

        global robot_positions

        # target_robot_name = 'robot1'
        # position = robot_positions.get(target_robot_name) 
        position = robot_positions

        print(f"Raw grid position: ({position[0]}, {position[1]})")

        # if position: 
        grid_x, grid_y = self.calc_grid_position(position[0], position[1])
        grid_x = int(grid_x)
        grid_y = int(grid_y)
        grid_y = self.map_label.height() - grid_y
        # grid_y = 400 - int(grid_x) 
        # grid_x = 100 - int(grid_y) 
            # if target_robot_name == 'robot1':
        
        #QPen(Qt.red, 20, Qt.SolidLine)

            # elif target_robot_name == 'robot2':
            #     pen = QPen(Qt.green, 10)
        # print(f"Drawpoint position: ({grid_x-140}, {grid_y+380})")
        

        #지나간 점 지우기
        if self.previous_positions:
            for prev_x, prev_y in self.previous_positions:
                # 배경색으로 이전 점 지우기
                painter.setPen(QPen(Qt.white, 10))  # 배경색으로 설정
                painter.drawPoint(prev_x, prev_y)

        pen = QPen(Qt.blue, 10)
        painter.setPen(pen)

        current_point_x = grid_x - 140
        current_point_y = grid_y + 380

        painter.drawPoint(current_point_x, current_point_y)
        # painter.drawText(current_point_x - 10, current_point_y + 15, "1")


        # painter.drawPoint(grid_x -140, grid_y + 380)
        # painter.drawText(grid_x -150, grid_y + 380, "1")

            # 현재 점을 리스트에 추가
        self.previous_positions.append((current_point_x, current_point_y))



        painter.end()
        self.map_label.setPixmap(updated_pixmap)
        
        # # 로봇 번호 표시
        # self.font = QFont()
        # self.font.setBold(True)
        # self.font.setPointSize(15)
        # painter.setFont(self.font)
        
        # 1번 로봇 좌표
        # x, y = self.calc_grid_position(amcl1.pose.pose.position.x, amcl1.pose.pose.position.y)

        # painter.setPen(QPen(Qt.red, 20, Qt.SolidLine))
        # painter.drawPoint(int((self.width - x) * self.image_scale), int(y * self.image_scale))
        # painter.drawText(int((self.width - x) * self.image_scale + 13), int(y * self.image_scale + 5), '1')


        # ######################

        # if start_point_1 is not None:
        #     x_start, y_start = self.calc_grid_position(start_point_1.pose.pose.position.x, start_point_1.pose.pose.position.y)
        #     x_start, y_start = int((self.width - x_start) * self.image_scale), int(y_start * self.image_scale)
        
        # x_before_recorded = False
        
        # 완료된 waypoint 지우기    path creating 
        # if path_1.length == 0 and path_before_1.length != 0:    
        #     for i in range(path_before_1.length):
        #         x, y = self.calc_grid_position(path_before_1.poses[i].position.x, path_before_1.poses[i].position.y)
        #         x, y = int((self.width - x) * self.image_scale), int(y * self.image_scale)
        #         painter.setPen(QPen(QColor(0, 0, 0, 0)))
                
        #         if x_before_recorded:
        #             painter.drawLine(x_before, y_before, x, y)
        #         else:
        #             painter.drawLine(x_start, y_start, x, y)
                
        #         painter.drawPoint(x, y)
        #         x_before, y_before = x, y
        #         x_before_recorded = True
        
        # # 신규 waypoint 그리기
        # for i in range(len(path_1.poses)):
        #     x, y = self.calc_grid_position(path_1.poses[i].position.x, path_1.poses[i].position.y)
        #     x, y = int((self.width - x) * self.image_scale), int(y * self.image_scale)
        #     painter.setPen(QPen(Qt.darkRed, 5))
            
        #     if x_before_recorded:
        #         painter.drawLine(x_before, y_before, x, y)
        #     else:
        #         painter.drawLine(x_start, y_start, x, y)
            
        #     painter.setPen(QPen(Qt.darkRed, 10))
        #     painter.drawPoint(x, y)
        #     x_before, y_before = x, y
        #     x_before_recorded = True

        # painter.end()

    def calc_grid_position(self, x, y):
        pos_x = (x - self.map_origin[0]) / self.map_resolution * 2
        pos_y = (y - self.map_origin[1]) / self.map_resolution * 2
        return pos_x, pos_y



    def clickCapture(self):
        msg = String()
        if self.isCaptureOn == False:
            self.capture_btn.setText('Stop')
            self.isCaptureOn = True
            self.capture_label.show()
            self.capture_label.setText('Capturing 🟢')

            msg.data = 'capture_start'
            
        else:
            self.capture_btn.setText('Capture Person')
            self.isCaptureOn = False
            self.capture_label.show()
            self.capture_label.setText('Stop 🔴')

            msg.data = 'capture_stop'

        self.capture_publisher.publish(msg)


def main():
    rp.init()
    executor = MultiThreadedExecutor()

    app = QApplication(sys.argv)
    # myWindows = WindowClass()
    # myWindows.show()

    try:
        myWindows = WindowClass()
        myWindows.show()
    except Exception as e:
        print(f"Error creating WindowClass: {e}")
    
    # cam_subscriber = CamSubscriber(myWindows)
    # executor.add_node(cam_subscriber)

    amcl_subscriber = AmclSubscriber()
    executor.add_node(amcl_subscriber)
    
    # path_subscriber = PathSubscriber()
    # executor.add_node(path_subscriber)

    #pi_cam_subscriber = PiCamSubscriber(myWindows)
    #executor.add_node(pi_cam_subscriber)

    #hand_status_subscriber = HandSubscriber(myWindows)
    #executor.add_node(hand_status_subscriber)

    thread = Thread(target=executor.spin)
    thread.start()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

