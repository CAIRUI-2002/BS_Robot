package com.example.demo1.service;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import org.springframework.beans.factory.DisposableBean;
import org.springframework.beans.factory.InitializingBean;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.messaging.simp.SimpMessagingTemplate;
import org.springframework.stereotype.Service;

import com.example.demo1.model.CablePoint;
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
    private volatile float vx = 0; // 前后
    private volatile float vy = 0; // 左右
    private volatile float vz = 0; // ✅ 新增：高度方向速度
    private volatile float vyaw = 0; // 转向
    private volatile int currentMode = 0;
    private volatile long lastManualCommandTime = System.currentTimeMillis();
    private volatile boolean manualCommandTimedOut = false;
    private static final long MANUAL_COMMAND_TIMEOUT_MS = 350L;

    // ✅ 新增：自动巡线相关参数
    private volatile boolean isAutoTracking = false; // 是否开启自动巡线
    // 管线仪信号：0=正中，1=向右找线，2=向左找线
    // 由管线仪数据接收线程写入，由运动控制线程读取，因此使用 volatile
    private volatile int pipelineSignal = 0;
    private volatile long lastSignalTime = 0;

    // 上一次真正参与运动控制的左右信号
    // 只记录 1 或 2，不记录中心信号 0
    private volatile int lastAppliedPipelineDirection = 0;

    // 信号从 1 跳变为 2，或者从 2 跳变为 1 后，强制反转的结束时间
    private volatile long signalReversalEndTime = 0L;

    // 自动巡线参数，可由前端通过 setAutoSpeeds 动态修改
    private volatile float autoForwardSpeed = 0.2f;

    // 普通左右跟线使用的固定偏航值，不再使用 S 型正弦摆动
    private volatile float normalTrackingYaw = 0.30f;

    // 自动巡线仅使用管线仪中心/左右信号进行固定偏航跟线。
    // 已移除 S 型正弦摆动、路径弯道识别和大转角横移模式。
    private static final float MAX_YAW_CMD = 0.75f;
    private static final long PIPELINE_TIMEOUT_MS = 1000;
    private static final float MIN_AUTO_FORWARD_SPEED = 0.08f;
    private static final float YAW_SMOOTHING = 0.22f;

    // 管线仪左右信号发生反转后，强制转向持续时间
    private static final long SIGNAL_REVERSAL_HOLD_MS = 550L;

    // 信号反转时直接使用的偏航指令
    private static final float SIGNAL_REVERSAL_YAW = 0.45f;

    // 到达电缆中心时的前进速度比例
    private static final float CENTER_FORWARD_RATIO = 0.35f;

    // 信号反转期间的前进速度比例；0 表示先停止前进并完成反向转向
    private static final float SIGNAL_REVERSAL_FORWARD_RATIO = 0.0f;

    // 自动巡线偏航平滑状态
    private float lastAutoYawCmd = 0.0f;

    // 保留路径点计数仅用于兼容现有前端/Controller 接口，不参与运动控制
    private volatile int syncedCablePathPointCount = 0;

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
     * 设置自动巡线参数（供 Controller 调用）。
     *
     * @param vx 自动巡线前进速度
     * @param vy 普通左右跟线固定偏航值，取绝对值并限制在 0～MAX_YAW_CMD
     */
    public void setAutoSpeeds(float vx, float vy) {
        this.autoForwardSpeed = vx;
        this.normalTrackingYaw = clamp(Math.abs(vy), 0.0f, MAX_YAW_CMD);
        System.out.println("⚙️ [后端] 自动巡线参数更新: Forward="
                + vx + ", TrackingYaw=" + this.normalTrackingYaw);
    }

    /**
     * 保留现有路径同步接口以兼容 Controller 和前端。
     * 路径数据不再用于弯道识别、模式切换或运动控制。
     */
    public void updateCablePathPrediction(List<CablePoint> path) {
        this.syncedCablePathPointCount = path == null ? 0 : path.size();
    }

    /**
     * 接收管线仪数据的接口 (供 Controller 调用)
     * 
     * @param signal   0=正中，1=向右找线，2=向左找线
     * @param depth    管线仪深度，可为空
     * @param angleDeg 管线仪罗盘角度，仅透传给前端显示，不参与自动巡线
     */
    public void updatePipelineSignal(int signal, Float depth, Float angleDeg) {
        this.pipelineSignal = signal;
        long now = System.currentTimeMillis();
        this.lastSignalTime = now;
        // 推送到 /topic/pipeline，将单值改为对象
        if (template != null) {
            Map<String, Object> payload = new HashMap<>();
            payload.put("signal", signal);
            payload.put("depth", depth);
            payload.put("angle", angleDeg);
            payload.put("rawAngle", angleDeg);
            // 以下字段保留为中性值，兼容仍在读取旧弯道状态的前端代码。
            payload.put("predictedYaw", null);
            payload.put("pathTurnDeg", 0.0f);
            payload.put("signalFutureBiasDeg", 0.0f);
            payload.put("cornerConfidence", 0.0f);
            payload.put("cornerArea", false);
            payload.put("cornerStrafeMode", false);
            payload.put("pathPointCount", this.syncedCablePathPointCount);
            template.convertAndSend("/topic/pipeline", (Object) payload);
        }
    }

    /**
     * 开启/关闭自动巡线模式
     */
    public void setAutoTracking(boolean enable) {
        this.isAutoTracking = enable;
        if (enable) {
            // 每次启动时重置偏航和反转状态
            this.lastAutoYawCmd = 0.0f;

            // 重置管线仪方向反转状态
            this.lastAppliedPipelineDirection = 0;
            this.signalReversalEndTime = 0L;

            updateCommand(0, 0, 0, 0, 3);
            System.out.println("🚀 自动巡线已启动：仅使用管线仪中心/左右信号进行固定偏航巡线");
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
        this.lastManualCommandTime = System.currentTimeMillis();
        this.manualCommandTimedOut = false;

        // 防止重复发送相同的模式切换指令
        if (mode == currentMode && mode != 0)
            return;

        // === 模式切换逻辑 ===
        if (mode == 1) {
            sendJsonCommand(2, 22, "\"MotionParam\": 1"); // 站立
            System.out.println("🤖 切换: 站立 (Mode 1)");
            currentMode = 1;
            isAutoTracking = false;
        } else if (mode == 4) {
            sendJsonCommand(2, 22, "\"MotionParam\": 4"); // 趴下
            System.out.println("🤖 切换: 趴下 (Mode 4)");
            this.vx = 0;
            this.vy = 0;
            this.vyaw = 0;
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

    private float clamp(float v, float min, float max) {
        return Math.max(min, Math.min(max, v));
    }

    private float smoothYaw(float targetYaw) {
        this.lastAutoYawCmd += (targetYaw - this.lastAutoYawCmd) * YAW_SMOOTHING;
        return this.lastAutoYawCmd;
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
        boolean manualCommandExpired = false;

        if (isAutoTracking) {
            // --- A. 自动巡线：使用固定偏航跟线，不再执行 S 型正弦摆动 ---
            long now = System.currentTimeMillis();

            // 管线仪信号超时：超过 1 秒没有新数据，立即停车。
            if (now - lastSignalTime > PIPELINE_TIMEOUT_MS) {
                finalVx = 0.0f;
                finalVy = 0.0f;
                finalYaw = 0.0f;
                this.lastAutoYawCmd = 0.0f;
            } else {
                // 直接使用当前管线仪信号，不继承中心信号之前的旧方向。
                int activeDirection = pipelineSignal;
                boolean validDirection = activeDirection == 1 || activeDirection == 2;

                // 识别 1 -> 2 或 2 -> 1 的跨线方向反转。
                boolean directionReversed = validDirection
                        && lastAppliedPipelineDirection != 0
                        && activeDirection != lastAppliedPipelineDirection;

                if (directionReversed) {
                    signalReversalEndTime = now + SIGNAL_REVERSAL_HOLD_MS;
                    lastAutoYawCmd = 0.0f;

                    System.out.println(
                            "🔄 管线仪方向反转："
                                    + lastAppliedPipelineDirection
                                    + " -> "
                                    + activeDirection
                                    + "，强制反向转向 "
                                    + SIGNAL_REVERSAL_HOLD_MS
                                    + "ms");
                }

                // 中心信号 0 不覆盖上一次有效方向，使 1 -> 0 -> 2 仍能识别为反转。
                if (validDirection) {
                    lastAppliedPipelineDirection = activeDirection;
                }

                boolean signalReversalActive = now < signalReversalEndTime;

                float targetYaw;

                if (signalReversalActive) {
                    // 跨线后立即建立反向偏航，不经过慢速平滑。
                    targetYaw = activeDirection == 1
                            ? -SIGNAL_REVERSAL_YAW
                            : SIGNAL_REVERSAL_YAW;
                    targetYaw = clamp(targetYaw, -MAX_YAW_CMD, MAX_YAW_CMD);
                    lastAutoYawCmd = targetYaw;
                    finalYaw = targetYaw;

                } else if (activeDirection == 1) {
                    // 要求向右找线：使用固定负偏航，左右控制完全对称。
                    if (lastAutoYawCmd > 0.0f) {
                        lastAutoYawCmd = 0.0f;
                    }
                    targetYaw = -normalTrackingYaw;
                    finalYaw = smoothYaw(clamp(targetYaw, -MAX_YAW_CMD, MAX_YAW_CMD));

                } else if (activeDirection == 2) {
                    // 要求向左找线：使用固定正偏航，左右控制完全对称。
                    if (lastAutoYawCmd < 0.0f) {
                        lastAutoYawCmd = 0.0f;
                    }
                    targetYaw = normalTrackingYaw;
                    finalYaw = smoothYaw(clamp(targetYaw, -MAX_YAW_CMD, MAX_YAW_CMD));

                } else {
                    // 信号为 0：位于电缆上方，立即停止继续向原方向偏航。
                    lastAutoYawCmd = 0.0f;
                    finalYaw = 0.0f;
                }

                // 已完全移除自动横移控制，自动巡线时 Y 方向始终为 0。
                finalVy = 0.0f;

                float baseForward = this.autoForwardSpeed <= 0.01f
                        ? 0.0f
                        : Math.max(this.autoForwardSpeed, MIN_AUTO_FORWARD_SPEED);

                if (signalReversalActive) {
                    // 方向反转期间优先完成转向。
                    finalVx = baseForward * SIGNAL_REVERSAL_FORWARD_RATIO;
                } else if (pipelineSignal == 0) {
                    // 到达电缆中心时减速，给后续反向留出响应时间。
                    finalVx = baseForward * CENTER_FORWARD_RATIO;
                } else {
                    finalVx = baseForward;
                }
            }
        } else {
            // --- B. 手动遥控逻辑 ---
            // 浏览器控制循环失联时不能无限重复最后一条转向指令。
            manualCommandExpired = System.currentTimeMillis() - this.lastManualCommandTime > MANUAL_COMMAND_TIMEOUT_MS;
            if (manualCommandExpired) {
                finalVx = 0.0f;
                finalVy = 0.0f;
                finalYaw = 0.0f;
                boolean hadActiveCommand = Math.abs(this.vx) > 0.0001f ||
                        Math.abs(this.vy) > 0.0001f ||
                        Math.abs(this.vyaw) > 0.0001f ||
                        Math.abs(this.vz) > 0.0001f;
                if (hadActiveCommand && !this.manualCommandTimedOut) {
                    this.manualCommandTimedOut = true;
                    System.out.println("⚠️ 运动指令超过 " + MANUAL_COMMAND_TIMEOUT_MS + "ms 未刷新，已由看门狗自动停车");
                }
            } else {
                // 直接使用前端传来的值 (前端滑条已直接改变了 this.vx / this.vy)
                finalVx = this.vx;
                finalVy = this.vy;
                finalYaw = this.vyaw;
            }
        }

        // ==========================================
        // 发送指令 (应用全局缩放系数)
        // ==========================================

        float scaleX = 1.5f;
        float scaleY = 2.0f;
        float scaleYaw = 0.8f;
        float finalVz = manualCommandExpired ? 0.0f : this.vz;

        // 构建 Items JSON
        String items = String.format(
                "\"X\": %.4f, \"Y\": %.4f, \"Z\": %.4f, \"Roll\": 0.0, \"Pitch\": 0.0, \"Yaw\": %.4f",
                finalVx * scaleX,
                finalVy * scaleY,
                finalVz, // ✅ 将 0.0 替换为实际速度值
                finalYaw * scaleYaw);

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

            buffer.put((byte) 0xEB);
            buffer.put((byte) 0x91);
            buffer.put((byte) 0xEB);
            buffer.put((byte) 0x90);
            buffer.putShort((short) len);
            buffer.put((byte) 0x01);
            buffer.put((byte) 0x00);
            buffer.put((byte) 0x01);
            for (int i = 0; i < 7; i++)
                buffer.put((byte) 0x00);
            buffer.put(jsonBytes);

            byte[] packetData = buffer.array();
            DatagramPacket packet = new DatagramPacket(packetData, packetData.length,
                    InetAddress.getByName(ROBOT_IP), ROBOT_PORT);
            socket.send(packet);
        } catch (Exception e) {
            System.err.println("UDP Error: " + e.getMessage());
        }
    }

    private String buildJson(int type, int command, String itemsContent) {
        SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
        String time = sdf.format(new Date());
        String items = (itemsContent == null || itemsContent.isEmpty()) ? "{}" : "{ " + itemsContent + " }";
        return String.format(
                "{\"PatrolDevice\": {\"Type\": %d, \"Command\": %d, \"Time\": \"%s\", \"Items\": %s}}",
                type, command, time, items);
    }

    @Override
    public void destroy() {
        if (scheduler != null)
            scheduler.shutdown();
        // 关闭 socket 会自动触发 receive() 的 SocketException 从而退出循环
        if (socket != null && !socket.isClosed())
            socket.close();
        if (udpListenerThread != null)
            udpListenerThread.interrupt();
    }
}
