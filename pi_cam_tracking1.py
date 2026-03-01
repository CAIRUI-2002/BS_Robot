import cv2
import numpy as np
import math
import requests
import time
import sys

# ==========================================
# ★ 1. 网络配置 (请确认 IP 正确)
# ==========================================
SERVER_IP = "10.21.31.104"   # 你的电脑 IP
SERVER_PORT = "8080"
API_URL = f"http://{SERVER_IP}:{SERVER_PORT}/api/pipeline/update"

# ==========================================
# ★ 2. 定义全局变量
# ==========================================
GLOBAL_ANGLE = None      
GLOBAL_ARROW = 0         # 箭头状态 (0=无, 1=左, 2=右, 3=双)
LAST_SEND_TIME = 0       # 发送频率限制

# ==========================================
# ★ 3. 网络发送与控制逻辑
# ==========================================
def send_signal(signal_code):
    """ 发送控制信号到服务器 """
    try:
        params = {"signal": signal_code}
        # 0.1秒超时，网络卡顿不影响视觉
        requests.get(API_URL, params=params, timeout=0.1)
    except Exception:
        pass # 忽略错误，防止刷屏

def control_robot_based_on_vision():
    """ 
    ★★★ 决策核心：根据视觉结果发送指令 ★★★ 
    """
    global GLOBAL_ARROW, LAST_SEND_TIME

    # 频率限制：每 0.15 秒发送一次 (约 6-7Hz)
    current_time = time.time()
    if current_time - LAST_SEND_TIME < 0.15:
        return

    command = 0
    debug_str = ""

    # ⚠️ 保持你要求的逻辑：
    # 视觉看到左箭头(1) -> 发送 1
    # 视觉看到右箭头(2) -> 发送 2
    if GLOBAL_ARROW == 1:
        command = 1
        debug_str = "⬅️ 左箭头 -> 发送 1"
    elif GLOBAL_ARROW == 2:
        command = 2
        debug_str = "➡️ 右箭头 -> 发送 2"
    elif GLOBAL_ARROW == 3:
        command = 0
        debug_str = "⏸️ 双闪 -> 发送 0"
    else:
        command = 0
        debug_str = "⬆️ 无信号 -> 发送 0"

    # 在终端同一行刷新打印，不刷屏
    print(f"👁️ 视觉: {GLOBAL_ARROW} | {debug_str}            ")
    
    send_signal(command)
    LAST_SEND_TIME = current_time

# ==========================================
# ★ 4. 视觉算法函数 (保持不变)
# ==========================================
def check_circle_outer_white(image, cx, cy, r):
    h, w = image.shape[:2]
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.circle(mask, (cx, cy), int(r + 15), 255, -1)
    cv2.circle(mask, (cx, cy), int(r + 5), 0, -1)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ring_pixels = gray[mask > 0]
    if len(ring_pixels) == 0: return False
    white_pixels = np.sum(ring_pixels > 180)
    return (white_pixels / len(ring_pixels)) > 0.6

def detect_circle(image):
    h, w = image.shape[:2]
    # 仅在屏幕中下方搜索，排除无关干扰
    roi_y_start = int(h * 0.2) 
    roi_y_end = int(h * 0.9)
    roi = image[roi_y_start:roi_y_end, :]
    
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    
    # 霍夫圆检测
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 1, 80, 
                             param1=50, param2=25, minRadius=30, maxRadius=150)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        # 找最匹配的一个圆
        for circle in circles[0]:
            cx, cy, r = int(circle[0]), int(circle[1]), int(circle[2])
            # 过滤边缘和尺寸不对的圆
            if r < 30 or r > 150 or cx < w * 0.1 or cx > w * 0.9: continue
            # 校验外圈是否有白环(管线仪特征)
            if check_circle_outer_white(image, cx, cy + roi_y_start, r):
                return (cx, cy + roi_y_start, r)
    return None

def detect_green_arrow(image, circle_info):
    if circle_info is None: return 0
    x, y, r = circle_info
    h, w = image.shape[:2]
    
    # 转换 HSV 检测绿色
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([35, 60, 60]), np.array([85, 255, 255]))
    
    # 定义左右搜索区域
    y_min, y_max = max(0, y - int(r*1.5)), min(h, y + int(r*1.5))
    
    # 左侧区域像素计数
    left_roi = mask[y_min:y_max, max(0, x - int(r*3.5)):max(0, x - int(r*0.8))]
    left_cnt = cv2.countNonZero(left_roi)
    
    # 右侧区域像素计数
    right_roi = mask[y_min:y_max, min(w, x + int(r*0.8)):min(w, x + int(r*3.5))]
    right_cnt = cv2.countNonZero(right_roi)
    
    # 判断逻辑
    threshold = 100 # 像素阈值，根据分辨率调整
    if left_cnt > threshold and right_cnt > threshold: return 3 # 双闪
    if left_cnt > threshold: return 1  # 左箭头
    if right_cnt > threshold: return 2 # 右箭头
    return 0

# ==========================================
# ★ 5. 主程序 (实时摄像头循环)
# ==========================================
def main():
    print("🚀 实时自动巡线系统已启动")
    print(f"📡 连接服务器: {API_URL}")
    print("📷 正在打开摄像头...")

    # 打开摄像头 (索引 0 通常是默认 USB 摄像头)
    cap = cv2.VideoCapture(0)
    
    # ⚡️ 关键设置：降低分辨率以提高帧率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print("❌ 错误：无法打开摄像头！请检查 USB 连接。")
        return

    global GLOBAL_ARROW

    try:
        while True:
            # 1. 读取一帧
            ret, frame = cap.read()
            if not ret:
                print("❌ 读取视频帧失败")
                break

            # 2. 视觉识别
            circle_info = detect_circle(frame)
            
            if circle_info:
                # 找到管线仪屏幕 -> 识别箭头
                arrow = detect_green_arrow(frame, circle_info)
                GLOBAL_ARROW = arrow
                
                # (可选) 在画面上画圈和文字，方便调试
                cx, cy, r = circle_info
                cv2.circle(frame, (cx, cy), r, (0, 255, 0), 2)
                cv2.putText(frame, f"Signal: {arrow}", (cx-20, cy-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            else:
                # 没找到屏幕 -> 归零
                GLOBAL_ARROW = 0

            # 3. 发送控制指令
            control_robot_based_on_vision()

            # 4. 显示实时画面 (如果是无头模式运行，可以注释掉这两行)
            #cv2.imshow("Robot Vision Eye", frame)
            
            # 按 'q' 键退出
            #if cv2.waitKey(1) & 0xFF == ord('q'):
                #break

    except KeyboardInterrupt:
        print("\n🛑 用户强制停止")
    finally:
        cap.release()
        #cv2.destroyAllWindows()
        print("✅ 系统已安全退出")

if __name__ == "__main__":
    main()
