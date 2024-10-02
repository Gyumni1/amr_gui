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

ui_file = os.path.join(get_package_share_directory('ui_pkg'), 'ui', 'monitor.ui')
# map_yaml_file = os.path.join(get_package_share_directory('main_pkg'), 'map', 'home.yaml')# 
map_yaml_file = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml')# 
from_class = uic.loadUiType(ui_file)[0]


global amcl1
amcl1 = PoseWithCovarianceStamped()

# global path_1, path_before_1
# path_1 = AstarMsg() #####
# path_before_1 = AstarMsg()

global start_point_1


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

        
        
class WindowClass(QMainWindow, from_class):
    # class WindowClass(QDialog, from_class):
    def __init__(self):
        super().__init__()

        try:
            self.setupUi(self)
        except Exception as e:
            print(f"Error during setupUi: {e}")


        # self.setupUi(self)
        self.setWindowTitle('Dialog') #????

        timer = QTimer(self)
        timer.timeout.connect(self.time)
        timer.timeout.connect(self.updateMap)
        timer.start(200)


        self.time = 0
        
        self.follow_label.setText('Wait 🔴')

        self.follow_node = rp.create_node('following_mode')
        self.follow_publisher = self.follow_node.create_publisher(String, '/follow', 10)


        # 사람 선택
        self.isCaptureOn = False
        self.capture_btn.setText('Capture Person')
        self.capture_label.hide()
        self.capture_btn.clicked.connect(self.clickCapture)

        self.capture_node = rp.create_node('capture_mode')
        self.capture_publisher = self.capture_node.create_publisher(String, '/capturing', 10)






        with open(map_yaml_file) as f:
            map_yaml_data = yaml.full_load(f)

        self.pixmap = QPixmap(os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'map', map_yaml_data['image']))
        # self.pixmap = QPixmap(os.path.join(get_package_share_directory('main_pkg'), 'map', map_yaml_data['image']))
        self.height = self.pixmap.size().height()
        self.width = self.pixmap.size().width()
        self.image_scale = 6
        self.pixmap = self.pixmap.transformed(QTransform().scale(-1, -1))
        self.map.setPixmap(self.pixmap.scaled(self.width * self.image_scale, self.height * self.image_scale, Qt.KeepAspectRatio))
    
        self.now_x = 0
        self.now_y = 0

        self.map_resolution = map_yaml_data['resolution']
        self.map_origin = map_yaml_data['origin'][:2]

    def updateMap(self):
        self.map.setPixmap(self.pixmap.scaled(self.width * self.image_scale, self.height * self.image_scale, Qt.KeepAspectRatio))

        painter = QPainter(self.map.pixmap())
        
        # # 로봇 번호 표시
        # self.font = QFont()
        # self.font.setBold(True)
        # self.font.setPointSize(15)
        # painter.setFont(self.font)
        
        # 1번 로봇 좌표
        x, y = self.calc_grid_position(amcl1.pose.pose.position.x, amcl1.pose.pose.position.y)

        painter.setPen(QPen(Qt.red, 20, Qt.SolidLine))
        painter.drawPoint(int((self.width - x) * self.image_scale), int(y * self.image_scale))
        painter.drawText(int((self.width - x) * self.image_scale + 13), int(y * self.image_scale + 5), '1')


        ######################

        if start_point_1 is not None:
            x_start, y_start = self.calc_grid_position(start_point_1.pose.pose.position.x, start_point_1.pose.pose.position.y)
            x_start, y_start = int((self.width - x_start) * self.image_scale), int(y_start * self.image_scale)
        
        x_before_recorded = False
        
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
        pos_x = (x - self.map_origin[0]) / self.map_resolution
        pos_y = (y - self.map_origin[1]) / self.map_resolution
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
    
    cam_subscriber = CamSubscriber(myWindows)
    executor.add_node(cam_subscriber)

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
