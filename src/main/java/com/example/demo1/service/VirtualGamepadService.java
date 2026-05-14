package com.example.demo1.service;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import org.springframework.beans.factory.DisposableBean;
import org.springframework.beans.factory.InitializingBean;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.messaging.simp.SimpMessagingTemplate;
import org.springframework.stereotype.Service;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

@Service
public class VirtualGamepadService implements InitializingBean, DisposableBean {

    // ✅ M20 Pro 运动主机 IP (请确保 IP 正确)
    private static final String ROBOT_IP = "10.21.31.103";
    private static final int ROBOT_PORT = 30000;

    @Autowired
    private SimpMessagingTemplate template;

    private DatagramSocket socket;
    private ScheduledExecutorService scheduler;
    // 新增：用于接收 UDP 回传的线程与 JSON 解析器
    private Thread udpListenerThread;
    private final ObjectMapper objectMapper = new ObjectMapper();

    // 运动参数 (手动模式使用 - 由前端 /app/move 实时更新)
    private float vx = 0;   // 前后
    private float vy = 0;   // 左右
    private float vz = 0;   // ✅ 新增：高度方向速度
    private float vyaw = 0; // 转向
    private int currentMode = 0;

    // ✅ 新增：自动巡线相关参数
    private boolean isAutoTracking = false; // 是否开启自动巡线
    private int pipelineSignal = 0;         // 管线仪信号: 0=中, 1=右, 2=左
    private long lastSignalTime = 0;        // 上次收到信号的时间 (用于超时安全保护)

    // ✅ [修改点 1] 新增：自动巡线速度设置 (默认值)
    // 这些值现在可以由前端滑条通过 setAutoSpeeds 动态修改
    private float autoForwardSpeed = 0.2f;
    private float autoStrafeSpeed = 0.2f;

    // ✅ [新增] 终极 S 型巡线：姿态状态机与弯道补偿控制变量
    private int currentPosture = 0;       // 0=初始平行/未知, 1=车头已偏右, 2=车头已偏左
    private long turnEndTime = 0;         // 记录本次转向结束的时间戳
    private int currentTurnDirection = 0; // 当前正在执行的转向：1(右), 2(左), 0(无/直行)
    private long lastCrossTime = 0;       // 记录上一次成功跨越(收到0)或调整姿态的时间
    
    // ⚡ 核心参数调优
    private static final long BASE_TURN_MS = 1000;   // 基础 1x 转向时间 (约代表 30°)
    private static final long CURVE_TIMEOUT_MS = 4000; // 弯道超时时间设为 4 秒 (4000ms)

    @Override
    public void afterPropertiesSet() throws Exception {
        socket = new DatagramSocket();
        scheduler = Executors.newSingleThreadScheduledExecutor();

        // 👇 新增：启动 UDP 监听线程接收机器狗主动上报的状态数据
        startUdpListener();

        // 1. 心跳 (10Hz) - 保持连接必须
        scheduler.scheduleAtFixedRate(this::sendHeartbeat, 0, 100, TimeUnit.MILLISECONDS);

        // 2. 运动指令 (20Hz) - 负责发送移动控制
        scheduler.scheduleAtFixedRate(this::sendMotionLoop, 0, 50, TimeUnit.MILLISECONDS);

        System.out.println("✅ M20 Pro 服务启动 - 已集成动态速度调节与设备状态监听功能");
    }


    /**
     * ✅ [修改点 2] 新增：设置自动巡线速度 (供 Controller 调用)
     * @param vx 前进速度
     * @param vy 横移速度
     */
    public void setAutoSpeeds(float vx, float vy) {
        this.autoForwardSpeed = vx;
        this.autoStrafeSpeed = vy;
        System.out.println("⚙️ [后端] 自动巡线速度更新: Forward=" + vx + ", Strafe=" + vy);
    }

    /**
     * 接收管线仪数据的接口 (供 Controller 调用)
     * @param signal 0=正中, 1=电缆在左(需左移), 2=电缆在右(需右移)
     */
    public void updatePipelineSignal(int signal, Float depth) { // 修改方法签名
        this.pipelineSignal = signal;
        this.lastSignalTime = System.currentTimeMillis(); // 更新心跳时间

        // 推送到 /topic/pipeline，将单值改为对象
        if (template != null) {
            Map<String, Object> payload = new HashMap<>();
            payload.put("signal", signal);
            payload.put("depth", depth); // 如果没有深度，这里会是 null
            template.convertAndSend("/topic/pipeline", (Object) payload);
        }
    }

    /**
     * 开启/关闭自动巡线模式
     */
    public void setAutoTracking(boolean enable) {
        this.isAutoTracking = enable;
        if (enable) {
            // ✅ [修改] 每次启动时重置姿态机和计时器
            this.currentPosture = 0; 
            this.currentTurnDirection = 0;
            this.turnEndTime = 0;
            this.lastCrossTime = System.currentTimeMillis(); // 启动时初始化跨越时间

            updateCommand(0, 0, 0, 0, 3); 
            System.out.println("🚀 终极 S 型自动巡线已启动 (带弯道补偿)");
        } else {
            // 关闭时，速度归零
            this.vx = 0;
            this.vy = 0;
            this.vz = 0; // ✅ 可选：安全起见，也将 vz 归零
            this.vyaw = 0;
            System.out.println("🛑 自动巡线已停止");
        }
    }

    /**
     * 更新控制指令 (手动/模式切换)
     */
    public void updateCommand(float v, float y, float w, float z, int mode) {
    this.vx = v;
    this.vy = y;
    this.vyaw = w;
    this.vz = z; // ✅ 更新高度速度

        // 防止重复发送相同的模式切换指令
        if (mode == currentMode && mode != 0) return;

        // === 模式切换逻辑 ===
        if (mode == 1) {
            sendJsonCommand(2, 22, "\"MotionParam\": 1"); // 站立
            System.out.println("🤖 切换: 站立 (Mode 1)");
            currentMode = 1;
            isAutoTracking = false;
        } else if (mode == 4) {
            sendJsonCommand(2, 22, "\"MotionParam\": 4"); // 趴下
            System.out.println("🤖 切换: 趴下 (Mode 4)");
            this.vx = 0; this.vy = 0; this.vyaw = 0;
            currentMode = 4;
            isAutoTracking = false;
        } else if (mode == 3) {
            sendJsonCommand(2, 22, "\"MotionParam\": 3"); // 运动
            System.out.println("🤖 切换: 常规足式运动 (Mode 3)");
            currentMode = 3;
        } else if (mode == 6) {
            sendJsonCommand(2, 22, "\"MotionParam\": 6"); // 轮式
            System.out.println("🤖 切换: 轮式模式 (Mode 6)");
            currentMode = 6;
            isAutoTracking = false;
        }
    }

    private void sendHeartbeat() {
        sendJsonCommand(100, 100, null);
    }

    /**
     * 运动控制循环 (20Hz)
     * 负责计算并发送当前的速度参数
     */
    private void sendMotionLoop() {
        // ✅ 修正：解除对 currentMode == 1 的拦截，允许在站立状态下调整高度和姿态
        if (currentMode == 0 || currentMode == 2 || currentMode == 4) {
            return;
        }

        float finalVx = 0;
        float finalVy = 0;
        float finalYaw = 0;

        if (isAutoTracking) {
            // --- A. 自动巡线逻辑 (姿态记忆 + 1x/2x折返 + 3秒弯道补偿) ---

            // 1. 安全保护：如果超过 2秒 没收到信号，自动停车
            if (System.currentTimeMillis() - lastSignalTime > 2000) {
                finalVx = 0;
                finalVy = 0;
                finalYaw = 0;
                currentTurnDirection = 0; 
            } else {
                long currentTime = System.currentTimeMillis();

                // 2. 状态机判断 (仅在非转向期间接收新指令)
                if (currentTime >= turnEndTime) {
                    
                    if (pipelineSignal == 0) {
                        // 在正上方：刷新跨越时间，不改变姿态，利用惯性冲到另一边
                        lastCrossTime = currentTime;
                        
                    } else if (pipelineSignal == 1) { // 需向右走
                        if (currentPosture == 0) {
                            // [起步切入] 平行状态 -> 向右转 1x 时间 (约 30°)
                            currentTurnDirection = 1;
                            turnEndTime = currentTime + BASE_TURN_MS;
                            currentPosture = 1;
                            lastCrossTime = currentTime;
                            System.out.println("🔄 [起步] 向右转 1x 角度");
                            
                        } else if (currentPosture == 2) {
                            // [S型折返] 当前偏左 -> 需要转 2x 时间才能变成偏右 (约 60°)
                            currentTurnDirection = 1;
                            turnEndTime = currentTime + (BASE_TURN_MS * 2); 
                            currentPosture = 1;
                            lastCrossTime = currentTime;
                            System.out.println("🔄 [折返] 向右转 2x 角度 (抵消左偏+建立右偏)");
                            
                        } else if (currentPosture == 1) {
                            // 已经偏右，但如果超过 3 秒未跨越中心线，说明遇到急弯
                            if (currentTime - lastCrossTime > CURVE_TIMEOUT_MS) {
                                currentTurnDirection = 1;
                                turnEndTime = currentTime + BASE_TURN_MS; // 追加 1x 角度
                                lastCrossTime = currentTime; // 刷新时间，防止连续追加
                                System.out.println("⚠️ [弯道补偿] 3秒未跨越，追加向右转 1x 角度");
                            }
                        }
                        
                    } else if (pipelineSignal == 2) { // 需向左走
                        if (currentPosture == 0) {
                            // [起步切入] 平行状态 -> 向左转 1x 时间 (约 30°)
                            currentTurnDirection = 2;
                            turnEndTime = currentTime + BASE_TURN_MS;
                            currentPosture = 2;
                            lastCrossTime = currentTime;
                            System.out.println("🔄 [起步] 向左转 1x 角度");
                            
                        } else if (currentPosture == 1) {
                            // [S型折返] 当前偏右 -> 需要转 2x 时间才能变成偏左 (约 60°)
                            currentTurnDirection = 2;
                            turnEndTime = currentTime + (BASE_TURN_MS * 2); 
                            currentPosture = 2;
                            lastCrossTime = currentTime;
                            System.out.println("🔄 [折返] 向左转 2x 角度 (抵消右偏+建立左偏)");
                            
                        } else if (currentPosture == 2) {
                            // 已经偏左，检查 3 秒急弯补偿
                            if (currentTime - lastCrossTime > CURVE_TIMEOUT_MS) {
                                currentTurnDirection = 2;
                                turnEndTime = currentTime + BASE_TURN_MS; // 追加 1x 角度
                                lastCrossTime = currentTime; // 刷新时间
                                System.out.println("⚠️ [弯道补偿] 3秒未跨越，追加向左转 1x 角度");
                            }
                        }
                    }
                }

                // 3. 执行运动控制
                finalVx = this.autoForwardSpeed; // 基础状态始终保持前进
                finalVy = 0;                     // 彻底关闭横移

                // 检查定时器，执行具体的转向动作
                if (currentTime < turnEndTime) {
                    finalVx = this.autoForwardSpeed * 0.8f; // 转向时稍微减速增加抓地力
                    if (currentTurnDirection == 1) {
                        finalYaw = -this.autoStrafeSpeed; // 向右角速度
                    } else if (currentTurnDirection == 2) {
                        finalYaw = this.autoStrafeSpeed;  // 向左角速度
                    }
                } else {
                    // 时间到了：角速度归零，机器狗带着已建立的偏角保持纯直行
                    finalYaw = 0;
                    currentTurnDirection = 0; 
                }
            }
        } else {
            // --- B. 手动遥控逻辑 ---
            // 直接使用前端传来的值 (前端滑条已直接改变了 this.vx / this.vy)
            finalVx = this.vx;
            finalVy = this.vy;
            finalYaw = this.vyaw;
        }

        // ==========================================
        // 发送指令 (应用全局缩放系数)
        // ==========================================

        float scaleX = 1.5f;
        float scaleY = 2.0f;
        float scaleYaw = 0.8f;
        float finalVz = this.vz; // ✅ 获取当前高度速度

        // 构建 Items JSON
        String items = String.format(
        "\"X\": %.4f, \"Y\": %.4f, \"Z\": %.4f, \"Roll\": 0.0, \"Pitch\": 0.0, \"Yaw\": %.4f",
        finalVx * scaleX,
        finalVy * scaleY,
        finalVz, // ✅ 将 0.0 替换为实际速度值
        finalYaw * scaleYaw
        );

        sendJsonCommand(2, 21, items);
    }

    /**
     * 持续阻塞接收机器狗发来的 UDP 数据包
     */
    private void startUdpListener() {
        udpListenerThread = new Thread(() -> {
            byte[] buffer = new byte[65535];
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
            
            while (!Thread.currentThread().isInterrupted() && !socket.isClosed()) {
                try {
                    socket.receive(packet);
                    
                    // 将有效负载转为字符串。由于数据包前带有 16 字节的二进制协议头，
                    // 我们直接使用 indexOf("{") 安全截取真正的 JSON 字符串起始位置。
                    String rawMsg = new String(packet.getData(), packet.getOffset(), packet.getLength(), "UTF-8");
                    int jsonStartIndex = rawMsg.indexOf("{");
                    
                    if (jsonStartIndex != -1) {
                        String jsonStr = rawMsg.substring(jsonStartIndex);
                        parseAndPushRobotStatus(jsonStr);
                    }
                } catch (Exception e) {
                    // Socket 关闭导致的正常中断不做处理
                }
            }
        });
        udpListenerThread.setDaemon(true);
        udpListenerThread.setName("M20Pro-UdpStatusListener");
        udpListenerThread.start();
    }

    /**
     * 解析 JSON 结构并通过 WebSocket 推送给前端
     */
    private void parseAndPushRobotStatus(String jsonStr) {
        try {
            JsonNode rootNode = objectMapper.readTree(jsonStr);
            JsonNode deviceNode = rootNode.path("PatrolDevice");
            
            if (!deviceNode.isMissingNode()) {
                int type = deviceNode.path("Type").asInt();
                int command = deviceNode.path("Command").asInt();
                
                // ==================================================
                // 1. 拦截 Type=1002, Command=5 (设备常规状态集：电量与灯光)
                // ==================================================
                if (type == 1002 && command == 5) {
                    JsonNode itemsNode = deviceNode.path("Items");
                    
                    // 提取左右电池剩余百分比
                    JsonNode batteryNode = itemsNode.path("BatteryStatus");
                    double batteryLeft = batteryNode.path("BatteryLevelLeft").asDouble();
                    double batteryRight = batteryNode.path("BatteryLevelRight").asDouble();
                    
                    // 提取前后照明灯状态 (0=关闭, 1=开启)
                    JsonNode fillNode = itemsNode.path("Led").path("Fill");
                    int lightFront = fillNode.path("Front").asInt();
                    int lightBack = fillNode.path("Back").asInt();
                    
                    // 封装轻量级 Map 载荷
                    Map<String, Object> statusPayload = new HashMap<>();
                    statusPayload.put("batteryLeft", batteryLeft);
                    statusPayload.put("batteryRight", batteryRight);
                    statusPayload.put("lightFront", lightFront);
                    statusPayload.put("lightBack", lightBack);
                    
                    // 推送到前端专门的主题通道
                    if (template != null) {
                        template.convertAndSend("/topic/robot_status", (Object) statusPayload);
                    }
                }
                
                // ==================================================
                // 👇 新增：2. 拦截 Type=1002, Command=4 (运动状态集：剩余续航里程) 👇
                // ==================================================
                else if (type == 1002 && command == 4) {
                    JsonNode itemsNode = deviceNode.path("Items");
                    JsonNode motionNode = itemsNode.path("MotionStatus");
                    
                    if (!motionNode.isMissingNode()) {
                        // 提取预计剩余续航里程 (单位: km)
                        double remainMile = motionNode.path("RemainMile").asDouble();
                        
                        Map<String, Object> motionPayload = new HashMap<>();
                        motionPayload.put("remainMile", remainMile);
                        
                        // 为了节约前端监听开销，我们直接复用同一个状态推送通道
                        if (template != null) {
                            template.convertAndSend("/topic/robot_status", (Object) motionPayload);
                        }
                    }
                }
                // 👆 新增结束 👆
            }
        } catch (Exception e) {
            // 解析异常忽略，防止偶发畸形包刷屏
        }
    }

    private void sendJsonCommand(int type, int command, String items) {
        String json = buildJson(type, command, items);
        sendUdp(json);
    }

    private void sendUdp(String json) {
        try {
            byte[] jsonBytes = json.getBytes("UTF-8");
            int len = jsonBytes.length;
            ByteBuffer buffer = ByteBuffer.allocate(16 + len);
            buffer.order(ByteOrder.LITTLE_ENDIAN);

            buffer.put((byte) 0xEB); buffer.put((byte) 0x91);
            buffer.put((byte) 0xEB); buffer.put((byte) 0x90);
            buffer.putShort((short) len);
            buffer.put((byte) 0x01); buffer.put((byte) 0x00); buffer.put((byte) 0x01);
            for (int i = 0; i < 7; i++) buffer.put((byte) 0x00);
            buffer.put(jsonBytes);

            byte[] packetData = buffer.array();
            DatagramPacket packet = new DatagramPacket(packetData, packetData.length,
                    InetAddress.getByName(ROBOT_IP), ROBOT_PORT);
            socket.send(packet);
        } catch (Exception e) { System.err.println("UDP Error: " + e.getMessage()); }
    }

    private String buildJson(int type, int command, String itemsContent) {
        SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
        String time = sdf.format(new Date());
        String items = (itemsContent == null || itemsContent.isEmpty()) ? "{}" : "{ " + itemsContent + " }";
        return String.format(
                "{\"PatrolDevice\": {\"Type\": %d, \"Command\": %d, \"Time\": \"%s\", \"Items\": %s}}",
                type, command, time, items
        );
    }

    @Override
    public void destroy() {
        if (scheduler != null) scheduler.shutdown();
        // 关闭 socket 会自动触发 receive() 的 SocketException 从而退出循环
        if (socket != null && !socket.isClosed()) socket.close();
        if (udpListenerThread != null) udpListenerThread.interrupt();
    }
}