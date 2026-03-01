# 文件名: arm_server.py
# 部署位置: 机械臂内部树莓派
# 功能: 提供 HTTP 接口控制机械臂 (带线程锁 + CORS 跨域支持)

from flask import Flask, request, jsonify
from flask_cors import CORS  # ✅ 关键 1: 引入 CORS 库
from pymycobot.mycobot import MyCobot
import time
from threading import Lock   # ✅ 关键 2: 引入线程锁

app = Flask(__name__)
CORS(app)  # ✅ 关键 3: 允许所有来源跨域访问 (解决浏览器报错)

# ================= 配置区域 =================
PORT = '/dev/ttyAMA0'
BAUD = 115200
# ===========================================

# ✅ 全局锁：防止“发送指令”和“读取角度”同时抢占串口
arm_lock = Lock()

last_stop_time = 0

# 初始化连接
try:
    print(f"🔌 正在连接机械臂: {PORT} @ {BAUD}...")
    mc = MyCobot(PORT, BAUD)
    time.sleep(0.05)
    with arm_lock:
        mc.power_on()
    print(f"✅ 机械臂服务启动成功！(端口 5000)")
except Exception as e:
    print(f"❌ 连接失败: {e}")

# --- 1. 绝对移动接口 ---
@app.route('/move', methods=['GET'])
def move_arm():
    try:
        j1 = float(request.args.get('j1', 0))
        j2 = float(request.args.get('j2', 0))
        j3 = float(request.args.get('j3', 0))
        j4 = float(request.args.get('j4', 0))
        j5 = float(request.args.get('j5', 0))
        j6 = float(request.args.get('j6', 0))
        speed = int(request.args.get('speed', 10))
        angles = [j1, j2, j3, j4, j5, j6]
        
        # ✅ 加锁操作
        with arm_lock:
            mc.send_angles(angles, speed)
            
        return "OK"
    except Exception as e:
        print(f"Move Error: {e}")
        return f"Error: {e}", 500

# --- 2. 点动控制接口 ---
@app.route('/jog', methods=['GET'])
def jog_arm():
    global last_stop_time
    try:
        if time.time() - last_stop_time < 0.5:
            return "Ignored"
        index = int(request.args.get('index', 1))
        direction = int(request.args.get('direction', 1))
        speed = int(request.args.get('speed', 10))

        # ✅ 加锁操作
        with arm_lock:
            mc.jog_angle(index, direction, speed)

        return "OK"
    except Exception as e:
        print(f"Jog Error: {e}")
        return "Error", 500

# --- 3. 停止接口 ---
@app.route('/stop', methods=['GET'])
def stop_arm():
    global last_stop_time
    try:
        last_stop_time = time.time()
        # ✅ 加锁操作
        with arm_lock:
            mc.stop()
        return "OK"
    except Exception as e:
        return "Error", 500

# --- 4. ✅ 新增：获取实时角度接口 ---
@app.route('/get_angles', methods=['GET'])
def get_arm_angles():
    try:
        # ✅ 加锁读取 (读取是最容易崩溃的，必须加锁)
        with arm_lock:
            angles = mc.get_angles()
            
        # mycobot 读取失败时可能会返回空或者 None
        if angles and len(angles) == 6:
            return jsonify(angles)
        else:
            return jsonify([]) # 返回空数组表示暂无数据
    except Exception as e:
        print(f"Read Error: {e}")
        return jsonify([]), 500

if __name__ == '__main__':
    # threaded=True 允许并发请求处理
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
