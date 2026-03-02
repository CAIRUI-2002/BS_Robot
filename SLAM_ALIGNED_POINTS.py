#!/usr/bin/env python3
# 文件名: SLAM_raw.py
# 部署在: 通用主机 (与Java后端同一台机器)
# 功能: 订阅 /SLAM_ALIGNED_POINTS 和 /SLAM_ODOM -> WebSocket 转发给网页

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

import asyncio
import websockets
import threading
import json
import struct
import math

# 配置 WebSocket 端口
WS_PORT = 9090

class SlamBridge(Node):
    def __init__(self):
        super().__init__('slam_bridge_node')

        # --- 1. 配置 QoS ---
        # SLAM 输出的话题通常是 Reliable 的，但为了兼容性保持 Best Effort 也可以
        # 如果发现收不到数据，可以将 reliability 改为 ReliabilityPolicy.RELIABLE
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # --- 2. 订阅话题 (已修改为 SLAM 话题) ---

        # A. 点云 (SLAM 建图后的点云)
        self.create_subscription(
            PointCloud2,
            '/SLAM_ALIGNED_POINTS',  # ✅ 修改点 1
            self.lidar_callback,
            sensor_qos
        )

        # B. 机器狗位姿 (SLAM 输出的里程计)
        self.create_subscription(
            Odometry,
            '/SLAM_ODOM',            # ✅ 修改点 2
            self.odom_callback,
            10
        )

        self.get_logger().info(f'🚀 SLAM Bridge (/SLAM_ALIGNED_POINTS + /SLAM_ODOM) 已启动，端口 {WS_PORT}...')

        # 全局缓存数据
        self.latest_scan = []
        self.robot_pose = {'x': 0, 'y': 0, 'yaw': 0}
        self.latest_path = []

    def lidar_callback(self, msg):
        """ 解析 /SLAM_ALIGNED_POINTS 的 PointCloud2 数据 """
        points = []

        # --- Step 1: 获取字段偏移量 ---
        # 根据您提供的 echo 数据：x=0, y=4, z=8, intensity=32
        offset_x = -1
        offset_y = -1
        offset_z = -1
        offset_i = -1

        for field in msg.fields:
            if field.name == 'x': offset_x = field.offset
            if field.name == 'y': offset_y = field.offset
            if field.name == 'z': offset_z = field.offset
            if field.name == 'intensity': offset_i = field.offset

        # 如果没有 XYZ 坐标，无法可视化，直接返回
        if offset_x == -1 or offset_y == -1 or offset_z == -1:
            return

        point_step = msg.point_step  # 根据 echo 应为 48
        data = msg.data
        num_points = msg.width * msg.height

        # 降采样 (每 2 个点取 1 个，如果点云太密集导致网页卡顿，可改为 3 或 4)
        skip = 1

        for i in range(0, num_points, skip):
            base = i * point_step
            try:
                # --- Step 2: 解析数据 ---
                # x, y, z, intensity 都是 float32 (datatype=7), 占用 4 字节
                # struct 格式 '<f' 代表小端序 float
                x = struct.unpack_from('<f', data, base + offset_x)[0]
                y = struct.unpack_from('<f', data, base + offset_y)[0]
                z = struct.unpack_from('<f', data, base + offset_z)[0]
                
                # 获取强度
                intensity = 0.0
                if offset_i != -1:
                    intensity = struct.unpack_from('<f', data, base + offset_i)[0]

                # --- Step 3: 打包数据 ---
                # 距离过滤：保留 50米内的点 (SLAM 点云通常已经是有效的，可以放宽范围)
                # 您的数据单位看起来是米 (x: 0.003...)
                if math.sqrt(x*x + y*y + z*z) < 50.0:
                    points.append(round(x, 3)) # 保留3位小数，提高精度
                    points.append(round(y, 3))
                    points.append(round(z, 3))
                    points.append(round(intensity, 2))  # ✅ 保留小数 
            except Exception as e:
                # self.get_logger().warn(f"解析错误: {e}")
                pass

        self.latest_scan = points

    def odom_callback(self, msg):
        """ 解析 /SLAM_ODOM 位姿 """
        tx = msg.pose.pose.position.x
        ty = msg.pose.pose.position.y
        tz = msg.pose.pose.position.z  # ✅ 提取真实的 Z 轴高度

        # 提取四元数
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # 1. 计算偏航角 (Yaw) - 左右转向
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # 2. ✅ 新增：计算俯仰角 (Pitch) - 上下抬头/低头
        sinp = 2 * (qw * qy - qz * qx)
        # 防止极端情况下浮点数溢出超过 [-1, 1] 导致 math.asin 报错
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp) 
        else:
            pitch = math.asin(sinp)

        # 3. 将数据打包发给网页
        self.robot_pose = {
            'x': round(tx, 3),
            'y': round(ty, 3),
            'z': round(tz, 3),
            'yaw': round(yaw, 3),
            'pitch': round(pitch, 3) # ✅ 新增：将 pitch 添加到数据包中
        }

# --- WebSocket 服务 ---
async def ws_handler(websocket, path):
    global rclpy_node 
    print("🌐 网页已连接 (SLAM Mode)")
    try:
        while True:
            # 组合数据包
            packet = {
                'type': 'slam_data',
                'robot': rclpy_node.robot_pose,
                # 格式: [x1, y1, z1, i1, x2, y2, z2, i2, ...]
                'scan': rclpy_node.latest_scan, 
                'path': rclpy_node.latest_path # 这里暂时保留空数组，除非您有 /plan 话题
            }

            # 发送
            await websocket.send(json.dumps(packet))

            # 控制帧率 (10Hz)
            await asyncio.sleep(0.1)
    except Exception as e:
        pass

def ros_thread(node):
    rclpy.spin(node)

if __name__ == '__main__':
    rclpy.init()
    rclpy_node = SlamBridge()

    # 1. 启动 ROS 线程
    t = threading.Thread(target=ros_thread, args=(rclpy_node,))
    t.start()

    # 2. 启动 WebSocket 主循环
    print(f"🚀 SLAM WebSocket 服务启动在端口 {WS_PORT}")
    start_server = websockets.serve(ws_handler, "0.0.0.0", WS_PORT)
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()
