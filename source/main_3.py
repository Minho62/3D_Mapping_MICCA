#!/usr/bin/env python
import subprocess
import sys
import re
import shutil
import signal
import os
import logging
import serial
import time
import socket
import threading
import serial.tools.list_ports as sp
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

from urllib import request
from glob import glob
from subprocess import check_output
from datetime import datetime
from pathlib import Path

import psutil
import pyudev
context = pyudev.Context()
def get_usb_stat(device=None, manufacturer=None):
    usb_list = [[i.device, i.manufacturer] for i in sp.comports()]
    try:
        if(device==None and manufacturer==None):
            return usb_list
        elif(device!=None):
            return usb_list[[i[0] for i in usb_list].index(device)]
        else:
            return usb_list[[i[1] for i in usb_list].index(manufacturer)]
    except:
        return [[]]
    


def is_usb_device(device):
    """Check if the given device is a USB device."""
    if 'usb' in device.device_path:
        return True
    for ancestor in device.ancestors:
        if ancestor.subsystem == 'usb':
            return True
    return False

def new_get_mount_points():
    """Get mount information for USB devices."""
    partitions = psutil.disk_partitions()
    new_result = []
    
    
    for partition in partitions:
        # Check if the partition is a USB device
        for device in context.list_devices(subsystem='block', DEVTYPE='partition'):
            if is_usb_device(device) and partition.device == device.device_node:
                new_result.append([partition.device, partition.fstype])

    return new_result


def socket_isOpen(host=None):
    try:
        request.urlopen(host, timeout=.1)
        return True
    except request.URLError as err:
        return False

def read_6D_PCDFile(fname):
    x = []
    y = []
    z = []
    intensity = []
    roll = []
    pitch = []
    yaw = []
    _time = []
    try:
        with open(fname, 'r') as file:    # hello.txt 파일을 읽기 모드(r)로 열기
            lines = file.readlines()
        for i in range(11, 11+int(lines[9].split()[-1])):
            x.append(float(lines[i].split()[0]))
            y.append(float(lines[i].split()[1]))
            z.append(float(lines[i].split()[2]))
            intensity.append(float(lines[i].split()[3]))
            roll.append(float(lines[i].split()[4]))
            pitch.append(float(lines[i].split()[5]))
            yaw.append(float(lines[i].split()[6]))
            _time.append(float(lines[i].split()[7]))
    except Exception as e:
        print(e)
    return x,y,z, intensity, roll, pitch, yaw, _time

def slice_points2img(slice_points, normal):
    dist = np.sum(slice_points*normal, axis=1)

    dis_2d, dis_3d = (normal[0]**2+normal[1]**2)**(1/2), (normal[0]**2+normal[1]**2+normal[2]**2)**(1/2)
    cos_a, sin_a = 1, 0
    cos_b, sin_b = 1, 0
    cos_r, sin_r = normal[2]/dis_3d, dis_2d/dis_3d
    R_matrix = np.array([[normal[1]/dis_2d,             -normal[0]/dis_2d,              0           ],
                         [(normal[0]*normal[2])/dis_2d, (normal[1]*normal[2])/dis_2d,   -dis_2d     ],
                         [normal[0],                    normal[1],                      normal[2]   ]])

    projected_points = slice_points - np.dot(dist.reshape(len(dist), 1), normal.reshape(1, 3))
    projected_xy_points = np.array([np.dot(R_matrix, projected_points[i])[:2] for i in range(len(projected_points))])
    projected_xy_points[:, 1] = -projected_xy_points[:, 1]
    for i in range(len(projected_xy_points)):
        projected_xy_points[i][1] = -projected_xy_points[i][1]
    return projected_xy_points

class SERIAL:
    def __init__(self, port, baudrate=1000000, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE):
        self.ser = serial.Serial()

        self.ser.port = port
        self.ser.baudrate = baudrate
        self.ser.bytesize = bytesize
        self.ser.parity = parity
        self.ser.stopbits = stopbits
        self.ser.rtscts = False
        self.ser.dsrdtr = False
        self.ser.open()

    def write(self, data):
        self.ser.write(data)
        self.ser.flush()

    def reads(self, size=6):
        return self.ser.read(size)

class LIO_SAM:
    def __init__(self, serial):
        self.proc_lio_sam = None
        self.proc_ros_bag = None
        self.proc_roscore = None
        self.serial = serial
        self.start_time_str = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        self.save_dir = os.path.join(os.getcwd(), 'USB', self.start_time_str)
        self.robot_ws_folder_name = 'catkin_ws' #'ros2_ws'
        self.lio_sam_folder_name = 'LIO-SAM-master' #'LIO-SAM-ros2'
        self.start_time = time.time()
        self.ros_bag_rate = 0.5

        DEVNULL = open(os.devnull, 'wb')
        self.proc_roscore = subprocess.Popen(["roscore"], stdout=DEVNULL, stderr=DEVNULL, shell=True)
        ## LIO-SAM PCD파일 USB안에 저장할 수 있게 파라미터 변경
        self.change_lio_param()
        #subprocess.Popen(['cd ~/{}/ && source install/setup.bash'.format(self.robot_ws_folder_name)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
        subprocess.Popen(['cd ~/{}/ && source devel/setup.bash'.format(self.robot_ws_folder_name)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, preexec_fn=os.setsid)

        
    def change_lio_param(self):
        param_lines = []
        save_pcd_idx = -1
        #with open(os.path.join(Path.home(), self.robot_ws_folder_name, 'install', 'lio_sam', 'share', 'lio_sam', 'config', 'params.yaml'), 'r') as f:
        with open(os.path.join(Path.home(), self.robot_ws_folder_name, 'src', 'LIO-SAM-master', 'config', 'params.yaml'), 'r') as f:
            for i, line in enumerate(f):
                param_lines.append(line)
                if('savePCDDirectory' in line):
                    save_pcd_idx = i

        start_idx = param_lines[save_pcd_idx].find('"')
        end_idx = start_idx+param_lines[save_pcd_idx][start_idx+1:].find('"')
        slash_idxes = [t.start() for t in re.finditer('/', self.save_dir)]
        param_lines[save_pcd_idx] = "".join([param_lines[save_pcd_idx][:start_idx+1], os.path.join(self.save_dir[slash_idxes[2]:], 'LOAM_PC'),  param_lines[save_pcd_idx][end_idx:]])
        print(param_lines[save_pcd_idx])

        #with open(os.path.join(Path.home(), self.robot_ws_folder_name, 'install', 'lio_sam', 'share', 'lio_sam', 'config', 'params.yaml'), 'w+') as f:
        with open(os.path.join(Path.home(), self.robot_ws_folder_name, 'src', 'LIO-SAM-master', 'config', 'params.yaml'), 'w+') as f:
            for line in param_lines:
                f.write(line)

    def stop(self):
        print('LIO-SAM STOP')
        if(self.proc_lio_sam != None):
            try:
                os.killpg(os.getpgid(self.proc_lio_sam.pid), signal.SIGINT)
            except:
                pass
        self.proc_lio_sam = None
        print('ROS BAG STOP')
        if(self.proc_ros_bag != None):
            os.killpg(self.proc_ros_bag.pid, signal.SIGINT)
        self.proc_ros_bag = None
    
    # def stop_ros_bag(self):
    #     print('ROS BAG STOP')
    #     if(self.proc_ros_bag != None):
    #         os.killpg(self.proc_ros_bag.pid, signal.SIGINT)
    #     self.proc_ros_bag = None

    def send_process_data(self, i):
        checksum = (1 << 8) - 1 - sum([0x02, i, 0x01])
        send_bytes = bytes([0xff, 0xff, 0x02, i, 0x01, checksum])
        self.serial.write(send_bytes)

    def cave_mode(self, data):
        ## LIO-SAM START
        DEVNULL = open(os.devnull, 'wb')
        #self.proc_lio_sam = subprocess.Popen(["export ROS_DOMAIN_ID=43 && /opt/ros/humble/bin/ros2 launch lio_sam run.launch.py"], stdout=DEVNULL, stderr=DEVNULL, shell=True)
        self.proc_lio_sam = subprocess.Popen(["/opt/ros/noetic/bin/roslaunch lio_sam run.launch"], stdout=DEVNULL, stderr=DEVNULL, shell=True)

        i = 0
        while(i<15):
            print("lio_sam open waiting...: ", i)
            self.send_process_data(int(5*i/14))
            i += 1
            time.sleep(0.6)
            
        ## ROS BAG START
        os.makedirs(format(os.path.join(self.save_dir, 'ROS_BAG_LOAM')))
        self.proc_ros_bag = subprocess.Popen(['rosbag record -O {} /imu/data /velodyne_points'.format(os.path.join(self.save_dir, 'ROS_BAG_LOAM/rosbag.bag'))], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid, shell=True)

        # #정지할때까지
        # while(1):
        #     serial_data_proc = SERIAL_DATA_PROC()
        #     if(serial_data_proc.lio_sam_stat == 'STOP'):
        #         serial_data_proc.lio_sam_stat = False
        #         lio_sam = lio_sam.stop()
        #         break
            
            
        
        # self.stop()
        # self.stop_ros_bag()

    def processing(self, *data):
        # (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        #self.stop()
        total_time = (time.time() - self.start_time + 2) / self.ros_bag_rate
        
        ## LIO-SAM START
        DEVNULL = open(os.devnull, 'wb')
        #self.proc_lio_sam = subprocess.Popen(["export ROS_DOMAIN_ID=43 && /opt/ros/humble/bin/ros2 launch lio_sam run.launch.py"], stdout=DEVNULL, stderr=DEVNULL, shell=True)
        self.proc_lio_sam = subprocess.Popen(["/opt/ros/noetic/bin/roslaunch lio_sam run.launch"], stdout=DEVNULL, stderr=DEVNULL, shell=True)

        i = 0
        while(i<15):
            print("lio_sam open waiting...: ", i)
            self.send_process_data(int(5*i/14))
            i += 1
            time.sleep(0.5)
            
        ## ROS BAG START
        #self.proc_ros_bag = subprocess.Popen(['ros2 bag record -a -o {}'.format(os.path.join(self.save_dir, 'ROS_BAG_LOAM'))], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid, shell=True)
        os.makedirs(format(os.path.join(self.save_dir, 'ROS_BAG_LOAM')))
        self.proc_ros_bag = subprocess.Popen(['rosbag record -O {} /imu/data /velodyne_points'.format(os.path.join(self.save_dir, 'ROS_BAG_LOAM/rosbag.bag'))], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid, shell=True)
        #print(['rosbag record -O {}'.format(os.path.join(self.save_dir, 'ROS_BAG_LOAM/rosbag.bag'))])


        # ## ROS BAG START
        # #self.proc_ros_bag = subprocess.Popen(['export ROS_DOMAIN_ID=43 && ros2 bag play -r {} {}'.format(self.ros_bag_rate, os.path.join(self.save_dir, 'ROS_BAG_LOAM'))], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid, shell=True)
        # self.proc_ros_bag = subprocess.Popen(['rosbag play -r {} {}'.format(self.ros_bag_rate, os.path.join(self.save_dir, 'ROS_BAG_LOAM'))], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid, shell=True)
        # print(['rosbag play -r {} {}'.format(self.ros_bag_rate, os.path.join(self.save_dir, 'ROS_BAG_LOAM'))])
        i = 0
        while(i<total_time):
            print("slam drawing now...: ", i)
            self.send_process_data(int(5*i/total_time + 5))
            i += 1
            time.sleep(0.3)


        
        self.stop()
        #self.stop_ros_bag()
        ## 후처리
        # i: 10 ~ 15
        for i in range(10, 15, 1):
            self.send_process_data(i)
            time.sleep(1)


        #clould_point_pcd_files = [os.path.join(self.save_dir, 'LOAM_PCD', 'cloudCorner.pcd'), os.path.join(self.save_dir, 'LOAM_PCD', 'cloudGlobal.pcd'), os.path.join(self.save_dir, 'LOAM_PCD', 'cloudSurf.pcd')]
        clould_point_pcd_files = [os.path.join(self.save_dir, 'LOAM_PCD', 'CornerMap.pcd'), os.path.join(self.save_dir, 'LOAM_PCD', 'GlobalMap.pcd'), os.path.join(self.save_dir, 'LOAM_PCD', 'SurfMap.pcd')]
        if(not all([os.path.isfile(pcd_file) for pcd_file in clould_point_pcd_files])):
            print('PCD FILES NOT GENERATE')
            self.send_process_data(20)
            return
        print('PCD FILES GENERATE')

        
        pointcloud_as_array = np.array([[0, 0, 0]])
        for pcd_file in clould_point_pcd_files:
            pointcloud_as_array = np.append(pointcloud_as_array, np.asarray(o3d.io.read_point_cloud(pcd_file).points), axis=0)

        x_list, y_list, z_list, _, roll_list, pitch_list, yaw_list, _ = read_6D_PCDFile(os.path.join(self.save_dir, 'LOAM_PCD', 'transformations.pcd'))

        roll, pitch, yaw = 0.0, 0.0, 0.0
        for i in range(len(roll_list)):
            roll += roll_list[i]
            pitch += pitch_list[i]
            yaw += yaw_list[i]
        roll /= len(roll_list)
        pitch /= len(pitch_list)
        yaw /= len(yaw_list)

        # x, y, z = np.cos(yaw)*np.cos(pitch), np.sin(yaw)*np.cos(pitch), np.sin(pitch)
        # normal = np.array([x, y, z])
        x = -np.cos(yaw)*np.sin(pitch)*np.sin(roll)-np.sin(yaw)*np.cos(roll)
        y = -np.sin(yaw)*np.sin(pitch)*np.sin(roll)+np.cos(yaw)*np.cos(roll)
        z =  np.cos(pitch)*np.sin(roll)
        normal = np.array([y, x, z])

        iterval = 0.25
        front_z = ((iterval**2)/(((normal[0]/normal[2])**2)+((normal[1]/normal[2])**2)+1))**(1/2)
        front_x = (normal[0]/normal[2])*front_z
        front_y = (normal[1]/normal[2])*front_z
        front_point = np.array([front_x, front_y, front_z])

        back_z = (-1)*(((iterval**2)/(((normal[0]/normal[2])**2)+((normal[1]/normal[2])**2)+1))**(1/2))
        back_x = (normal[0]/normal[2])*back_z
        back_y = (normal[1]/normal[2])*back_z
        back_point = np.array([back_x, back_y, back_z])

        points = []

        for point_index in range(len(pointcloud_as_array)):
            if((np.sum(normal*pointcloud_as_array[point_index])<=np.sum(normal*front_point)) and (np.sum(normal*pointcloud_as_array[point_index])>=np.sum(normal*back_point))):
                points.append(pointcloud_as_array[point_index])
        points = np.array(points)

        projected_xy_points = slice_points2img(pointcloud_as_array, normal)

        plt.scatter(projected_xy_points[:, 0], projected_xy_points[:, 1], s = 5)
        plt.axis('off')
        plt.savefig(os.path.join(self.save_dir, 'LOAM_PCD', 'output.png'), bbox_inches='tight', dpi=300)
        
        y = projected_xy_points[:, 0]  # x 좌표
        z = projected_xy_points[:, 1]  # y 좌표
        
        points = np.column_stack((y, z))
        # import pandas as df
        # from scipy.spatial import ConvexHull, convex_hull_plot_2d
        # from shapely.geometry import Point, LineString, Polygon
        # import ezdxf
        # np.bool = bool

        # # Convex Hull 계산
        # hull = ConvexHull(points)
        # hull_vertices = points[hull.vertices]
        
        # # 4. Shapely를 이용한 면적 계산
        # polygon = Polygon(hull_vertices)
        # area = polygon.area
        # print(f"투영된 2D 다각형의 면적: {area} m^3")
        
        # plt.figure()
        # plt.plot(points[:, 0], points[:, 1], 'o', label='Points',  markersize=3)
        
        # # Convex Hull의 외곽선 그리기
        # for simplex in hull.simplices:
        #     plt.plot(points[simplex, 0], points[simplex, 1], 'r-')
        
        
        # # DXF 파일로 저장하기
        # doc = ezdxf.new(dxfversion='R2010')
        # msp = doc.modelspace()
        
        # # 점들 추가 (원하는 경우)
        # for point in points:
        #     msp.add_point(point)
        
        # # Convex Hull 외곽선 추가
        # for simplex in hull.simplices:
        #     start_point = points[simplex[0]]
        #     end_point = points[simplex[1]]
        #     msp.add_line(start=start_point, end=end_point)
        
        # # DXF 파일로 저장
        # doc.saveas("convex_hull.dxf")
        # print("DXF 파일이 'convex_hull.dxf'로 저장되었습니다.")

        
        # i: 15 ~ 19
        for i in range(10, 20, 1):
            checksum = (1 << 8) - 1 - sum([0x02, i, 0x01])
            send_bytes = bytes([0xff, 0xff, 0x02, i, 0x01, checksum])
            self.serial.write(send_bytes)
            time.sleep(1)

        
        ## 후처리 종료
#        try:
#            subprocess.Popen(['sudo -S umount -f {}'.format(os.path.join(os.getcwd(), 'USB'))], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True).communicate(input=b'14211421\n')
#        except:
#            pass
class SERIAL_DATA_PROC:
    def __init__(self):
        self.lio_sam_stat = False
        self.emergency_stop_stat = False
        self.USB_connect = True
        self.timeimg = 0

    def serial_data(self, read_bytes):
        checksum = (1 << 8) - 1 - sum([read_bytes[i] for i in range(2, 5)])
        if((read_bytes[0] == 0xff) and (read_bytes[1] == 0xff) and checksum == read_bytes[-1]):
            if((read_bytes[2] == 0x02) and (read_bytes[3] == 0x00) and (read_bytes[4] == 0x02)):
                ## TUNNEL 측정 시작
                print("TUNNEL 측정 시작 명령어 입력")
                self.lio_sam_stat = 'START'
            elif((read_bytes[2] == 0x02) and (read_bytes[3] == 0x00) and (read_bytes[4] == 0x03)):
                ## TUNNEL 측정 정지 후 프로세스 후처리
                print("TUNNEL 측정 기록 시작, 프로세스 처리 명령어 입력")
                self.lio_sam_stat = 'PROCESS_TUNNEL'
            elif((read_bytes[2] == 0x02) and (read_bytes[3] == 0x00) and (read_bytes[4] == 0x01)):
                ## TUNNEL 측정 정지
                print("TUNNEL 측정 정지 명령어 입력")
                self.lio_sam_stat = 'STOP'


            elif((read_bytes[2] == 0x02) and (read_bytes[3] == 0x00) and (read_bytes[4] == 0x05)):
                ## CAVE 측정 대기
                print("CAVE 측정 대기")
                self.lio_sam_stat = 'START'
            elif((read_bytes[2] == 0x02) and (read_bytes[3] == 0x00) and (read_bytes[4] == 0x07)):
                ## CAVE 측정 시작
                print("CAVE 측정 시작 명령어 입력")
                print("START CAVE BAG")
                self.lio_sam_stat = 'PROCESS_CAVE'
                #self.timeimg.append(time.time())
            elif((read_bytes[2] == 0x02) and (read_bytes[3] == 0x00) and (read_bytes[4] == 0x04)):
                ## CAVE 측정 정지
                print("CAVE 측정 정지 명령어 입력")
                self.lio_sam_stat = 'STOP'
                print("SAVE CAVE BAG")
                #self.timeimg.append(time.time())
            elif((read_bytes[2] == 0x02) and (read_bytes[3] == 0x00) and (read_bytes[4] == 0x06)):
                ## CAVE 측정 정지 후 프로세스 후처리
                print("CAVE 측정 정지, 프로세스 처리 명령어 입력")
                self.lio_sam_stat = 'PROCESS_CAVE'
            

                
            elif((read_bytes[2] == 0x03) and (read_bytes[3] == 0x00) and (read_bytes[4] == 0x02)):
                ## USB DISCONNECT
                self.USB_connect = False
            elif((read_bytes[2] == 0x98) and (read_bytes[3] == 0x43) and (read_bytes[4] == 0x17)):
                ## EMERGENCY STOP
                self.emergency_stop_stat = True

class SBC_CONTROL:
    def __init__(self, logger, search_time = 5, send_cnt = 3):
        self.logger = logger
        self.arduino_state = False
        self.lidar_proc = None
        self.lidar_state = False
        self.lidar_time = time.time()
        self.imu_proc = None
        self.imu_state = False
        self.imu_time = time.time()
        self.usb_state = False
        self.usb_time = time.time()
        self.search_time = search_time
        self.send_cnt = send_cnt
        self.ser = None
    
    def arduino(self):
        ## 아두이노 연결 확인
        while(get_usb_stat(manufacturer="Arduino (www.arduino.cc)") == [[]]):
            print("ARDUINO NOT FOUND")
            self.logger.info("ARDUINO NOT FOUND")
            self.arduino_state = False
            time.sleep(self.search_time)
        self.ser = SERIAL(port=get_usb_stat(manufacturer="Arduino (www.arduino.cc)")[0])
        time.sleep(self.search_time/2)

        # 아두이노에게 boot success 보내기
        for _ in range(self.send_cnt):
            self.ser.write(bytes([0xFF, 0xFF, 0x01, 0x01, 0x02, 0xFB]))
            time.sleep(5)

        self.arduino_state = True
        self.logger.info("ARDUINO CONNECTED")
        print("ARDUINO CONNECTED")
        return self.ser

    def lidar(self):
        if(self.lidar_time + self.search_time < time.time()):
            if(socket_isOpen(host='http://192.168.1.201') and (self.lidar_state == False) and (self.lidar_proc == None)):
                ## VELODYNE LIDAR 최초 ROS 연결
                print("VELODYNE LIDAR 최초 ROS 연결")
                time.sleep(2)
                DEVNULL = open(os.devnull, 'wb')
                #self.lidar_proc = subprocess.Popen(['/opt/ros/humble/bin/ros2', "launch", "velodyne", "velodyne-all-nodes-VLP16-launch.py"],  stdout=DEVNULL, stderr=DEVNULL)
                self.lidar_proc = subprocess.Popen(["/opt/ros/noetic/bin/roslaunch velodyne_pointcloud VLP16_points.launch"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)

                time.sleep(1)
#                self.lidar_proc = subprocess.Popen(['/opt/ros/humble/bin/ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
#                self.lidar_proc = subprocess.Popen([sys.executable, "/opt/ros/humble/bin/ros2", "launch", "velodyne", "velodyne-all-nodes-VLP16-launch.py"], stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
#                time.sleep(1)  # maybe needed to wait the process to do something useful
#                try:
#                    self.lidar_proc.communicate(timeout=5)
#                except:
#                    pass
                self.lidar_state = True
                self.logger.info("Velodyne LIDAR CONNECT")
                ## 아두이노에게 VELODYNE LIDAR success 보내기
                for _ in range(self.send_cnt):
                    self.ser.write(bytes([0xFF, 0xFF, 0x01, 0x02, 0x02, 0xFA]))

            elif(socket_isOpen(host='http://192.168.1.201') and (self.lidar_state == False)):
                ## 사용자에 의해 끊김, VELODYNE ROS 종료
                print("사용자에 의해 끊김, VELODYNE ROS 종료")
                if(self.lidar_proc != None):
                    try:
                        os.killpg(os.getpgid(self.lidar_proc.pid), signal.SIGINT)
                    except:
                        pass
                self.lidar_proc = None
                self.logger.info("Velodyne LIDAR DISCONNECT")
                ## 아두이노에게 VELODYNE LIDAR fail 보내기
                for _ in range(self.send_cnt):
                    self.ser.write(bytes([0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB]))

            elif(not socket_isOpen(host='http://192.168.1.201') and (self.lidar_state == True)):
                ## 도중 연결 끊김, VELODYNE ROS 종료
                print("도중 연결 끊김, VELODYNE ROS 종료")
#                os.killpg(self.lidar_proc.pid, signal.SIGINT)
                self.lidar_state = False
                self.lidar_proc = None
                self.logger.info("Velodyne LIDAR DISCONNECT")
                ## 아두이노에게 VELODYNE LIDAR fail 보내기
                for _ in range(self.send_cnt):
                    self.ser.write(bytes([0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB]))
            self.lidar_time = time.time()

    def imu(self):
        if(self.imu_time + self.search_time < time.time()):
            if((get_usb_stat(manufacturer="Xsens") != [[]]) and (self.imu_state == False) and (self.imu_proc == None)):
                ## XSENS IMU ROS 최초 연결
                time.sleep(2)
#                my_env = os.environ.copy()
#                my_env["PATH"] = f"/usr/sbin:/sbin:{my_env['PATH']}"
                #DEVNULL = open(os.devnull, 'wb')
                #self.imu_proc = subprocess.Popen(['/opt/ros/humble/bin/ros2', 'launch', 'bluespace_ai_xsens_mti_driver', 'xsens_mti_node.launch.py'],  stdout=DEVNULL, stderr=DEVNULL)
                self.imu_proc = subprocess.Popen(["/opt/ros/noetic/bin/roslaunch xsens_mti_driver xsens_mti_node.launch"],  stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)

#                self.lidar_proc = subprocess.Popen(['/opt/ros/humble/bin/ros2', 'launch', 'bluespace_ai_xsens_mti_driver', 'xsens_mti_node.launch.py'],  stdout=DEVNULL, stderr=DEVNULL)
#                self.imu_proc = subprocess.Popen(['ros2 launch bluespace_ai_xsens_mti_driver xsens_mti_node.launch.py'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, preexec_fn=os.setsid, env=my_env)
                time.sleep(1)  # maybe needed to wait the process to do something useful
#                self.imu_proc.terminate()

                self.imu_state = True
                self.logger.info("Xsens IMU CONNECT")
                print("Xsens IMU CONNECT")
                ## 아두이노에게 XSENS IMU success 보내기
                for _ in range(self.send_cnt):
                    self.ser.write(bytes([0xFF, 0xFF, 0x01, 0x03, 0x02, 0xF9]))

            elif((get_usb_stat(manufacturer="Xsens") != [[]]) and (self.imu_state == False)):
                ## 사용자에 의해 끊김, XSENS ROS 종료
                if(self.imu_proc != None):
                    try:
                        os.killpg(os.getpgid(self.imu_proc.pid), signal.SIGINT)
                    except:
                        pass
                self.imu_proc = None
                self.logger.info("Xsens IMU DISCONNECT")
                print("사용자에 의해 끊김, XSENS ROS 종료")
                ## 아두이노에게 XSENS IMU fail 보내기
                for _ in range(self.send_cnt):
                    self.ser.write(bytes([0xFF, 0xFF, 0x01, 0x03, 0x01, 0xFA]))

            elif((get_usb_stat(manufacturer="Xsens") == [[]]) and (self.imu_state == True)):
                ## 도중 연결 끊김, XSENS ROS 종료
#                os.killpg(self.imu_proc.pid, signal.SIGINT)
                self.imu_state = False
                self.imu_proc = None
                self.logger.info("Xsens IMU DISCONNECT")
                print("도중 연결 끊김, XSENS ROS 종료")
                ## 아두이노에게 XSENS IMU fail 보내기
                for _ in range(self.send_cnt):
                    self.ser.write(bytes([0xFF, 0xFF, 0x01, 0x03, 0x01, 0xFA]))
            self.imu_time = time.time()




    def search_usb(self):
        if((self.usb_time + self.search_time < time.time()) and (len(new_get_mount_points()) == 0)):
            self.logger.info("USB NOT FOUND")
            print("USB NOT FOUND")
            self.usb_state = False
            ## 아두이노에게 USB connect fail 보내기
            for _ in range(self.send_cnt):
                self.ser.write(bytes([0xFF, 0xFF, 0x01, 0x04, 0x01, 0xF9]))
            self.usb_time = time.time()
        elif((self.usb_time + self.search_time < time.time()) and (len(new_get_mount_points()) != 0)):
            
            if(self.usb_state == False):
                self.logger.info("USB FOUND")
                print("USB FOUND")
                self.usb_state = True
            ## 아두이노에게 USB connect success 보내기
            for _ in range(self.send_cnt):
                self.ser.write(bytes([0xFF, 0xFF, 0x01, 0x04, 0x02, 0xF8]))
            self.usb_time = time.time()

    def mount_usb(self):
#        try:
#            subprocess.Popen(['sudo -S umount -f {}'.format(os.path.join(os.getcwd(), 'USB'))], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True).communicate(input=b'14211421\n')
#        except:
#            pass
#        if(not os.path.isdir(os.path.join(os.getcwd(), 'USB'))):
#            os.mkdir(os.path.join(os.getcwd(), 'USB'))
        subprocess.Popen(['sudo -S mount -t {} {} {}'.format(new_get_mount_points()[-1][1], new_get_mount_points()[-1][0], os.path.join(os.getcwd(), 'USB'))]
                        , stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True).communicate(input=b'14211421\n')
        self.logger.info("USB MOUNT")

    def umount_usb(self):
        self.logger.info("USB UMOUNT START")
        try:
            subprocess.Popen(['sudo -S umount -f {}'.format(os.path.join(os.getcwd(), 'USB'))], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True).communicate(input=b'14211421\n')
#            subprocess.Popen(['sudo -S fuser -ck {}'.format(os.path.join(os.getcwd(), 'USB'))], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True).communicate(input=b'jetson\n')
            subprocess.Popen(['sudo -S eject {}'.format(new_get_mount_points()[-1][0])], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True).communicate(input=b'14211421\n')
        except:
            pass
        self.logger.info("USB UMOUNT END")

    def emergency_stop(self):
        self.umount_usb()
        subprocess.Popen(['sudo init 0'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True).communicate(input=b'14211421\n')

    def state(self):
        return self.imu_state and self.lidar_state and self.usb_state

def main():
    logging.basicConfig(filename=os.path.join(os.getcwd(), 'log', datetime.now().strftime('%Y-%m-%d-%H-%M-%S')), level=logging.DEBUG, 
                        format="[ %(asctime)s | %(levelname)s ] %(message)s",
                        datefmt="%Y-%m-%d %H:%M:%S")
    logger = logging.getLogger()

    sbc_control = SBC_CONTROL(logger=logger, search_time=2, send_cnt=2)
    ser = sbc_control.arduino()
    serial_data_proc = SERIAL_DATA_PROC()
    
    while(True):
        sbc_control.search_usb()
        sbc_control.lidar()
        sbc_control.imu()

        try:
            if(ser.ser.in_waiting > 0):
                print("serial input")
                serial_data_proc.serial_data(ser.reads(size=6))
        except:
            pass
        
        if(serial_data_proc.emergency_stop_stat):
            print("EMERGENCY STOP")
            logger.info("EMERGENCY STOP")
            if('lio_sam' in locals() and lio_sam != None):
                lio_sam = lio_sam.stop()
            sbc_control.emergency_stop()

        if(sbc_control.state() and (serial_data_proc.lio_sam_stat == 'START')):
            print("LIO-SAM START")
            logger.info("LIO-SAM START")
            sbc_control.mount_usb()
            lio_sam = LIO_SAM(serial=ser)
            serial_data_proc.lio_sam_stat = "MEASUREMENT"
        elif(sbc_control.state() and (serial_data_proc.lio_sam_stat in ['PROCESS_TUNNEL'])):
            print("ROS BAG START and PROCESSING")
            logger.info("ROS BAG START and PROCESSING")

            #serial_data_proc.timeimg.append(serial_data_proc.lio_sam_stat)
            thread = threading.Thread(target=lio_sam.processing, args = (serial_data_proc.timeimg,))
            thread.start()
            serial_data_proc.lio_sam_stat = False
            

        elif(sbc_control.state() and (serial_data_proc.lio_sam_stat in ['PROCESS_CAVE'])):
            print("ROS BAG START and PROCESSING")
            logger.info("ROS BAG START and PROCESSING")

            #serial_data_proc.timeimg.append(serial_data_proc.lio_sam_stat)
            thread = threading.Thread(target=lio_sam.cave_mode, args = (serial_data_proc.timeimg,))
            thread.start()
            serial_data_proc.lio_sam_stat = False
            

        elif(serial_data_proc.lio_sam_stat == 'STOP'):
            print("LIO-SAM STOP AND ROS BAG STOP")
            logger.info("LIO-SAM START, ROS BAG STOP")
            serial_data_proc.lio_sam_stat = False
            lio_sam = lio_sam.stop()
            #lio_sam = lio_sam.stop_ros_bag()
            sbc_control.lidar_state = False
            sbc_control.imu_state = False

        if(not serial_data_proc.USB_connect and sbc_control.usb_state):
            sbc_control.umount_usb()
if __name__ == "__main__":
    main()
