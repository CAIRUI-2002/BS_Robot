#!/usr/bin/env python3
# 文件名: SLAM_raw.py
# 部署在: 通用主机 (与Java后端同一台机器)
# 功能: 订阅 /SLAM_ALIGNED_POINTS, /SLAM_ODOM 和 /FIBOCOM/net_rtk/gngga 
#       -> WebSocket 转发 SLAM 数据给网页
#       -> HTTP POST 转发 GPS 数据给 Java Spring Boot

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

# 引入 GPS 话题的消息类型
from fibocom_msgs.msg import FibocomRtkgngga

import asyncio
import websockets
import threading
import json
import struct
import math
import requests  # 用于发送 HTTP POST 请求

# 配置 WebSocket 端口
WS_PORT = 9090

class SlamBridge(Node):
    def __init__(self):
        super().__init__('slam_bridge_node')

        # --- 1. 配置 QoS ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # --- 2. 订阅话题 ---

        # A. 点云 (SLAM 建图后的点云)
        self.create_subscription(
            PointCloud2,
            '/SLAM_ALIGNED_POINTS',  
            self.lidar_callback,
            sensor_qos
        )

        # B. 机器狗位姿 (SLAM 输出的里程计)
        self.create_subscription(
            Odometry,
            '/SLAM_ODOM',            
            self.odom_callback,
            10
        )

        # C. 机器狗 RTK GPS 模块
        self.create_subscription(
            FibocomRtkgngga,
            '/FIBOCOM/net_rtk/gngga',
            self.gngga_callback,
            10
        )
        
        # Java Spring Boot 后端接收 GPS 数据的接口地址
        self.backend_url = "http://10.21.31.104:8080/api/gps/update"

        self.get_logger().info(f'🚀 SLAM & GPS Bridge 已启动，WebSocket 端口 {WS_PORT}...')

        # 全局缓存数据
        self.latest_scan = []
        self.robot_pose = {'x': 0, 'y': 0, 'yaw': 0, 'pitch': 0}
        self.latest_path = []

    # ====================================================
    # GPS 处理逻辑 (已移除质量和卫星数限制)
    # ====================================================
    def gngga_callback(self, msg):
        """ 解析 /FIBOCOM/net_rtk/gngga 的 GPS 数据 """
        qual = msg.qual
        sats = msg.sats
        
        # 不管质量和卫星数如何，直接将 NMEA 的“度分”格式转换为“十进制度数”格式
        lat_deg = self.nmea_to_decimal(msg.lat)
        lon_deg = self.nmea_to_decimal(msg.lon)
        
        # 根据半球方向加负号
        if msg.lat_dir == 'S': lat_deg = -lat_deg
        if msg.lon_dir == 'W': lon_deg = -lon_deg

        payload = {
            "lat": round(lat_deg, 6),
            "lon": round(lon_deg, 6),
            "qual": qual,
            "sats": sats
        }
        
        # 实时发送给 Java Spring Boot 后端
        self.send_to_backend(payload)

    def nmea_to_decimal(self, nmea_val):
        """ 将 ddmm.mmmm (度分) 转换为 dd.dddd (十进制度数) """
        if nmea_val == 0.0:
            return 0.0
        degrees = int(nmea_val / 100)
        minutes = nmea_val - (degrees * 100)
        return degrees + (minutes / 60.0)

    def send_to_backend(self, payload):
        """ 通过 HTTP POST 发送 GPS 数据给 Spring Boot """
        try:
            # 关键：设置 timeout=0.5 秒防止阻塞 ROS 2 核心线程，影响点云刷新
            requests.post(self.backend_url, json=payload, timeout=0.5)
        except requests.exceptions.Timeout:
            self.get_logger().warning("⚠️ GPS 转发超时：后端服务器响应过慢")
        except requests.exceptions.ConnectionError:
            self.get_logger().warning("⚠️ GPS 转发失败：无法连接到后端服务器")
        except Exception as e:
            pass

    # ====================================================
    # 以下为原有的 SLAM 点云与里程计处理逻辑 (未作修改)
    # ====================================================
    def lidar_callback(self, msg):
        """ 解析 /SLAM_ALIGNED_POINTS 的 PointCloud2 数据 """
        points = []
        offset_x = -1; offset_y = -1; offset_z = -1; offset_i = -1

        for field in msg.fields:
            if field.name == 'x': offset_x = field.offset
            if field.name == 'y': offset_y = field.offset
            if field.name == 'z': offset_z = field.offset
            if field.name == 'intensity': offset_i = field.offset

        if offset_x == -1 or offset_y == -1 or offset_z == -1:
            return

        point_step = msg.point_step  
        data = msg.data
        num_points = msg.width * msg.height
        skip = 1

        for i in range(0, num_points, skip):
            base = i * point_step
            try:
                x = struct.unpack_from('<f', data, base + offset_x)[0]
                y = struct.unpack_from('<f', data, base + offset_y)[0]
                z = struct.unpack_from('<f', data, base + offset_z)[0]
                
                intensity = 0.0
                if offset_i != -1:
                    intensity = struct.unpack_from('<f', data, base + offset_i)[0]

                if math.sqrt(x*x + y*y + z*z) < 50.0:
                    points.append(round(x, 3)) 
                    points.append(round(y, 3))
                    points.append(round(z, 3))
                    points.append(round(intensity, 2))  
            except Exception as e:
                pass

        self.latest_scan = points

    def odom_callback(self, msg):
        """ 解析 /SLAM_ODOM 位姿 """
        tx = msg.pose.pose.position.x
        ty = msg.pose.pose.position.y
        tz = msg.pose.pose.position.z  

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp) 
        else:
            pitch = math.asin(sinp)

        self.robot_pose = {
            'x': round(tx, 3),
            'y': round(ty, 3),
            'z': round(tz, 3),
            'yaw': round(yaw, 3),
            'pitch': round(pitch, 3) 
        }

# --- WebSocket 服务 ---
async def ws_handler(websocket, path):
    global rclpy_node 
    print("🌐 网页已连接 (SLAM Mode)")
    try:
        while True:
            packet = {
                'type': 'slam_data',
                'robot': rclpy_node.robot_pose,
                'scan': rclpy_node.latest_scan, 
                'path': rclpy_node.latest_path 
            }
            await websocket.send(json.dumps(packet))
            await asyncio.sleep(0.1)
    except Exception as e:
        pass

def ros_thread(node):
    rclpy.spin(node)

if __name__ == '__main__':
    rclpy.init()
    rclpy_node = SlamBridge()

    t = threading.Thread(target=ros_thread, args=(rclpy_node,))
    t.start()

    print(f"🚀 SLAM WebSocket 服务启动在端口 {WS_PORT}")
    start_server = websockets.serve(ws_handler, "0.0.0.0", WS_PORT)
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()