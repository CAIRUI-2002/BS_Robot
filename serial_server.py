# serial_server.py (最终修复版)
import serial
import struct
import time
import threading
import json
import asyncio
from contextlib import asynccontextmanager
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

# --- 用户配置参数 ---
PORT = '/dev/ttyACM0'
BAUDRATE = 921600 
FFT_LENGTH = 1024
CH_SIZE = FFT_LENGTH * 2 + 2 
PACKET_SIZE = CH_SIZE * 2    

# 全局变量
loop = None

# WebSocket 连接管理器
class ConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        print(f"✅ 客户端连接，当前数量: {len(self.active_connections)}")

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, ch1: list, ch2: list):
        message = json.dumps({
            "type": "signal", 
            "ch1": ch1, 
            "ch2": ch2
        })
        # 复制列表进行遍历，防止多线程操作时列表变动
        for connection in list(self.active_connections):
            try:
                await connection.send_text(message)
            except Exception as e:
                print(f"发送异常: {e}")
                self.disconnect(connection)

manager = ConnectionManager()

# --- 串口解析线程 ---
def serial_thread_task():
    global loop
    print(f"🚀 正在尝试打开 {PORT}...")
    try:
        ser = serial.Serial(port=PORT, baudrate=BAUDRATE, timeout=0.1)
        print("✅ 串口已打开，开始双通道数据采集...")
    except Exception as e:
        print(f"❌ 串口错误: {e}")
        return

    raw_buffer = bytearray()

    while True:
        try:
            if ser.in_waiting > 0:
                raw_buffer.extend(ser.read(ser.in_waiting))
            
            while len(raw_buffer) >= PACKET_SIZE:
                end_idx = raw_buffer.find(b'\xff\xff')
                
                if end_idx != -1:
                    sync_idx = end_idx - CH_SIZE
                    if sync_idx >= 0 and raw_buffer[sync_idx:sync_idx+2] == b'\xfe\xfe':
                        start_idx = sync_idx - (CH_SIZE - 2)
                        if start_idx >= 0:
                            ch1_raw = raw_buffer[start_idx : start_idx + 2048]
                            ch2_raw = raw_buffer[sync_idx + 2 : sync_idx + 2 + 2048]
                            
                            ch1_data = list(struct.unpack('<' + 'H' * FFT_LENGTH, ch1_raw))
                            ch2_data = list(struct.unpack('<' + 'H' * FFT_LENGTH, ch2_raw))

                            if loop and manager.active_connections:
                                asyncio.run_coroutine_threadsafe(
                                    manager.broadcast(ch1_data, ch2_data), loop
                                )

                            raw_buffer = raw_buffer[end_idx + 2:]
                            continue
                        else:
                            raw_buffer = raw_buffer[end_idx + 2:]
                    else:
                        raw_buffer = raw_buffer[end_idx + 2:]
                else:
                    if len(raw_buffer) > PACKET_SIZE * 5:
                        raw_buffer = raw_buffer[-PACKET_SIZE*2:]
                    break 
            
            time.sleep(0.001)

        except Exception as e:
            print(f"解析循环错误: {e}")
            time.sleep(1)

# --- ✅ 新增：Lifespan 生命周期管理器 (替换旧的 @app.on_event) ---
@asynccontextmanager
async def lifespan(app: FastAPI):
    # 启动时执行
    global loop
    loop = asyncio.get_running_loop()
    # 启动后台串口线程
    t = threading.Thread(target=serial_thread_task, daemon=True)
    t.start()
    yield
    # 关闭时执行 (如有清理工作写在这里)
    print("🛑 服务正在关闭...")

# 初始化 App 时传入 lifespan
app = FastAPI(lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.websocket("/ws/signal")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8001)
