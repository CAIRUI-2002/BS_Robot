import os
# ✅ 1. 核心配置：强制使用 TCP 传输，解决 "Could not find ref" 报错
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"

import cv2
import threading
import time
import base64
from flask import Flask, Response, jsonify, request  # ✅ 确保引入了 request

app = Flask(__name__)

# ✅ 强化版跨域处理：全局放行
@app.after_request
def add_cors_headers(response):
    response.headers['Access-Control-Allow-Origin'] = '*'
    response.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS, PUT, DELETE'
    response.headers['Access-Control-Allow-Headers'] = '*'
    return response

# ✅ 新增：专门处理浏览器的 OPTIONS 预检请求试探
@app.route('/', defaults={'path': ''}, methods=['OPTIONS'])
@app.route('/<path:path>', methods=['OPTIONS'])
def options_handler(path):
    return '', 204


# ================= 配置区域 =================
# 机器狗 RTSP 地址
RTSP_URL_FRONT = "rtsp://10.21.31.103:8554/video1"
RTSP_URL_REAR  = "rtsp://10.21.31.103:8554/video2"

# ⚡️ 性能优化参数 (如果还卡，就把这里改得更小，例如 320x180)
RESIZE_WIDTH  = 640
RESIZE_HEIGHT = 360
JPEG_QUALITY  = 50  # 画质 0-100，越低越流畅
# ===========================================

class VideoStream:
    """
    高性能视频读取类：后台线程读取 + 自动重连 + 实时缩放
    """
    def __init__(self, src, name="Camera"):
        self.src = src
        self.name = name
        self.stream = None
        self.grabbed = False
        self.frame = None
        self.stopped = False
        self.lock = threading.Lock()
        
        # 初始化连接
        self.connect()
        
        # 启动后台更新线程
        self.t = threading.Thread(target=self.update, args=())
        self.t.daemon = True
        self.t.start()

    def connect(self):
        """连接或重连摄像头"""
        if self.stream:
            self.stream.release()
        
        # 使用 FFMPEG 后端
        self.stream = cv2.VideoCapture(self.src, cv2.CAP_FFMPEG)
        # 设置缓冲区为 1，减少延迟
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        self.grabbed, self.frame = self.stream.read()
        if self.grabbed:
            print(f"✅ {self.name} 连接成功!")
        else:
            print(f"❌ {self.name} 连接失败")

    def update(self):
        """后台线程：循环读取最新帧"""
        while True:
            if self.stopped:
                if self.stream: self.stream.release()
                return

            # 读取一帧
            grabbed, frame = self.stream.read()
            
            # 如果读取失败（信号丢失/解码失败）
            if not grabbed:
                print(f"⚠️ {self.name} 信号中断，尝试重连...")
                time.sleep(1.0) # 等待 1 秒再重连
                self.connect()
                continue
            
            # 读取成功，立即进行缩放 (在子线程做，减轻主线程负担)
            try:
                # 缩放处理
                frame_small = cv2.resize(frame, (RESIZE_WIDTH, RESIZE_HEIGHT))
                
                with self.lock:
                    self.grabbed = True
                    self.frame = frame_small
            except Exception as e:
                # 极少数情况下 resize 可能会报错，忽略这一帧
                continue
            
            #稍微休眠，避免死循环占用 100% CPU
            time.sleep(0.01)

    def read(self):
        """返回当前最新的一帧"""
        with self.lock:
            return self.frame if self.grabbed else None

    def stop(self):
        self.stopped = True

# ================= 启动双路视频流 =================

print(f"🚀 正在启动双路视频服务...")
print(f"   分辨率限制: {RESIZE_WIDTH}x{RESIZE_HEIGHT}")

# 启动前摄线程
front_cam = VideoStream(RTSP_URL_FRONT, "前视广角")

# 启动后摄线程
rear_cam = VideoStream(RTSP_URL_REAR, "后视广角")

def generate_frames(camera_stream):
    """视频流生成器"""
    while True:
        frame = camera_stream.read()
        
        # 如果还没读到帧（启动初期或掉线），发个空包或者等待
        if frame is None:
            time.sleep(0.1)
            continue

        try:
            # 编码为 JPEG
            ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            frame_bytes = buffer.tobytes()
            
            # 生成 MJPEG 流
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            
            # 控制发送帧率 (约 20-25fps)，太快了浏览器也渲染不过来
            time.sleep(0.04)
            
        except Exception as e:
            # 编码错误忽略
            continue

@app.route('/front_feed')
def front_feed():
    return Response(generate_frames(front_cam),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/rear_feed')
def rear_feed():
    return Response(generate_frames(rear_cam),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# ✅ 新增：高精度原帧抓拍 API
# ✅ 修改：在 methods 中加入 'OPTIONS'
@app.route('/api/capture', methods=['GET', 'OPTIONS'])
def capture_images():
    # 如果是浏览器的预检请求，直接返回空内容放行
    if request.method == 'OPTIONS':
        return '', 204

    # --- 以下保持你原有的逻辑不变 ---
    frame_front = front_cam.read()
    frame_rear = rear_cam.read()

    def frame_to_base64(frame):
        if frame is None:
            return None
        ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if not ret:
            return None
        b64_bytes = base64.b64encode(buffer)
        return "data:image/jpeg;base64," + b64_bytes.decode('utf-8')

    front_b64 = frame_to_base64(frame_front)
    rear_b64 = frame_to_base64(frame_rear)

    if not front_b64 and not rear_b64:
        return jsonify({"status": "error", "message": "无法获取摄像头画面"}), 500

    return jsonify({
        "status": "success",
        "front": front_b64,
        "rear": rear_b64
    })
if __name__ == '__main__':
    print(f"🌐 Web服务已启动: http://localhost:5000")
    # threaded=True 允许多个浏览器同时访问
    app.run(host='0.0.0.0', port=5000, threaded=True)