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
    // 运动模式与控制权必须分开记录。机器狗可以仍处于步态模式，但上位机已经停止发包、让出控制权。
    private volatile boolean upperControlActive = false;
    // 自主导航/寻找破损点由前端显式续租；任务运行期间不能按手动控制的 350ms 超时释放。
    private volatile boolean autonomousControlHold = false;
    private volatile long lastAutonomousControlHoldTime = 0L;
    private volatile long lastManualCommandTime = System.currentTimeMillis();
    private volatile boolean manualCommandTimedOut = false;
    private static final long MANUAL_COMMAND_TIMEOUT_MS = 350L;
    // 手动指令超时后再发送约 3 帧零速度，确认停车；随后停止运动包，但状态心跳继续运行。
    private static final long CONTROL_RELEASE_STOP_BURST_MS = 150L;
    // 自主任务也必须有失联保护，防止浏览器崩溃后永久占用机器狗控制权。
    private static final long AUTONOMOUS_CONTROL_HOLD_TIMEOUT_MS = 1500L;

    // ✅ 新增：自动巡线相关参数
    private volatile boolean isAutoTracking = false; // 是否开启自动巡线
    // 管线仪信号：0=正中，1=向右找线，2=向左找线
    // 由管线仪数据接收线程写入，由运动控制线程读取，因此使用 volatile
    private volatile int pipelineSignal = 0;
    private volatile long lastSignalTime = 0;

    // 上一次收到的有效左右信号
    // 只记录 1 或 2，不记录中心信号 0，用于识别 1 <-> 2 连续跳变
    private volatile int lastReceivedPipelineDirection = 0;

    // 信号从 1 跳变为 2，或者从 2 跳变为 1 后，临时视为位于电缆正上方
    private volatile long signalJumpCenterEndTime = 0L;

    // 记录最近一次左右跳变及原有的强制反向转向结束时间。
    // 第一次跨线保留原来的“停止前进并转向”逻辑；短时间内再次跳变才判定为中心抖动。
    private volatile long lastDirectionJumpTime = 0L;
    private volatile long signalReversalEndTime = 0L;

    // 自动巡线参数，可由前端通过 setAutoSpeeds 动态修改
    private volatile float autoForwardSpeed = 0.3f;

    // 普通左右跟线使用的固定偏航值，不再使用 S 型正弦摆动
    private volatile float normalTrackingYaw = 0.30f;

    // 自动巡线仅使用管线仪中心/左右信号进行固定偏航跟线。
    // 已移除 S 型正弦摆动、路径弯道识别和大转角横移模式。
    private static final float MAX_YAW_CMD = 0.75f;
    private static final long PIPELINE_TIMEOUT_MS = 1000;
    private static final float MIN_AUTO_FORWARD_SPEED = 0.08f;
    private static final float YAW_SMOOTHING = 0.22f;

    // 两次左右跳变间隔不超过该时间时，判定为电缆正上方的连续抖动。
    private static final long CONTINUOUS_SIGNAL_JUMP_WINDOW_MS = 350L;

    // 连续跳变后保持中心状态的持续时间；后续跳变会不断续期。
    private static final long SIGNAL_JUMP_CENTER_HOLD_MS = 250L;

    // 保留原有的跨线控制参数：停止前进并按当前方向强制转向。
    private static final long SIGNAL_REVERSAL_HOLD_MS = 550L;
    private static final float SIGNAL_REVERSAL_YAW = 0.45f;
    private static final float SIGNAL_REVERSAL_FORWARD_RATIO = 0.0f;

    // 到达电缆中心时的前进速度比例
    private static final float CENTER_FORWARD_RATIO = 0.35f;

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

        // 1. 状态心跳 (10Hz) - 后端运行期间始终发送，用于持续接收机器狗状态上报
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
     * 设置机器狗前、后侧照明灯。
     *
     * @param front 0=关闭，1=开启
     * @param back  0=关闭，1=开启
     * @return UDP 指令是否成功交给本机网络栈发送；设备是否执行由后续状态上报确认
     */
    public boolean setLights(int front, int back) {
        if ((front != 0 && front != 1) || (back != 0 && back != 1)) {
            throw new IllegalArgumentException("照明灯状态只能是 0 或 1");
        }

        String items = String.format("\"Front\": %d, \"Back\": %d", front, back);
        return sendJsonCommand(1101, 2, items);
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
     * @param rawDepthCm 管线仪原始深度（cm），可为空
     * @param angleDeg 管线仪罗盘角度，仅透传给前端显示，不参与自动巡线
     */
    public void updatePipelineSignal(int signal, Float rawDepthCm, Float angleDeg) {
        updatePipelineSignal(signal, rawDepthCm, angleDeg, null, null);
    }

    /**
     * 接收带采集时间信息的管线仪数据。时间戳用于前端恢复深度采集时刻的位置。
     */
    public void updatePipelineSignal(int signal, Float rawDepthCm, Float angleDeg,
            Long captureTimeMs, Long backendReceiveTimeMs) {
        long now = System.currentTimeMillis();
        boolean validDirection = signal == 1 || signal == 2;

        // 左右信号发生 1 <-> 2 跳变时，将短时间内的连续跳变视为位于电缆正上方。
        // 在数据接收处识别跳变，避免两个信号在 20Hz 运动控制循环之间快速切换而被漏掉。
        if (validDirection) {
            boolean directionJumped = lastReceivedPipelineDirection != 0
                    && signal != lastReceivedPipelineDirection;
            if (directionJumped) {
                boolean continuousJump = lastDirectionJumpTime != 0L
                        && now - lastDirectionJumpTime <= CONTINUOUS_SIGNAL_JUMP_WINDOW_MS;

                // 每次真正跨线都保留原来的停车转向动作；如果属于连续抖动，中心状态具有更高优先级。
                signalReversalEndTime = now + SIGNAL_REVERSAL_HOLD_MS;
                if (continuousJump) {
                    signalJumpCenterEndTime = now + SIGNAL_JUMP_CENTER_HOLD_MS;
                    System.out.println(
                            "📍 管线仪左右连续跳变："
                                    + lastReceivedPipelineDirection
                                    + " -> "
                                    + signal
                                    + "，按电缆中心状态保持 "
                                    + SIGNAL_JUMP_CENTER_HOLD_MS
                                    + "ms");
                } else {
                    System.out.println(
                            "🔄 管线仪方向反转："
                                    + lastReceivedPipelineDirection
                                    + " -> "
                                    + signal
                                    + "，停止前进并转向 "
                                    + SIGNAL_REVERSAL_HOLD_MS
                                    + "ms");
                }
                lastDirectionJumpTime = now;
            }
            lastReceivedPipelineDirection = signal;
        }

        this.pipelineSignal = signal;
        this.lastSignalTime = now;
        // 推送到 /topic/pipeline，将单值改为对象
        if (template != null) {
            Map<String, Object> payload = new HashMap<>();
            payload.put("signal", signal);
            payload.put("depth", rawDepthCm);
            payload.put("rawDepthCm", rawDepthCm);
            payload.put("depthUnit", "cm");
            payload.put("captureTimeMs", captureTimeMs);
            payload.put("backendReceiveTimeMs", backendReceiveTimeMs);
            payload.put("serverReceiveTimeMs", now);
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
    public synchronized void setAutoTracking(boolean enable) {
        this.isAutoTracking = enable;
        if (enable) {
            // 每次启动时重置偏航和左右跳变中心状态
            this.lastAutoYawCmd = 0.0f;

            // 重置管线仪左右跳变中心状态
            this.lastReceivedPipelineDirection = 0;
            this.signalJumpCenterEndTime = 0L;
            this.lastDirectionJumpTime = 0L;
            this.signalReversalEndTime = 0L;

            updateCommand(0, 0, 0, 0, 3);
            System.out.println("🚀 自动巡线已启动：仅使用管线仪中心/左右信号进行固定偏航巡线");
        } else {
            // 关闭时，速度归零
            this.vx = 0;
            this.vy = 0;
            this.vz = 0; // ✅ 可选：安全起见，也将 vz 归零
            this.vyaw = 0;
            // 从下一帧开始进入短暂的零速度停车窗口，窗口结束后自动释放控制权。
            acquireUpperControl();
            this.lastManualCommandTime = System.currentTimeMillis() - MANUAL_COMMAND_TIMEOUT_MS;
            this.manualCommandTimedOut = false;
            System.out.println("🛑 自动巡线已停止，正在停车并释放上位机控制权");
        }
    }

    /**
     * 更新控制指令 (手动/模式切换)
     */
    public synchronized void updateCommand(float v, float y, float w, float z, int mode) {
        updateCommand(v, y, w, z, mode, false);
    }

    /**
     * 更新控制指令，并显式声明自主任务是否需要持续持有控制权。
     * 自主任务必须持续刷新 holdControl；若浏览器失联，后端仍会在租约超时后安全释放。
     */
    public synchronized void updateCommand(float v, float y, float w, float z, int mode, boolean holdControl) {
        acquireUpperControl();
        this.vx = v;
        this.vy = y;
        this.vyaw = w;
        this.vz = z; // ✅ 更新高度速度
        long now = System.currentTimeMillis();
        this.lastManualCommandTime = now;
        this.autonomousControlHold = holdControl;
        if (holdControl) {
            this.lastAutonomousControlHoldTime = now;
        }
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
        // 协议要求持续发送心跳，机器狗会向心跳来源 IP/端口上报实时状态和异常状态。
        // 心跳只负责维持状态通道；运动控制权通过是否发送 Command=21 运动包来区分。
        sendJsonCommand(100, 100, null);
    }

    private void acquireUpperControl() {
        if (!upperControlActive) {
            upperControlActive = true;
            System.out.println("🎮 上位机已取得机器狗运动控制权");
        }
    }

    private void releaseUpperControl() {
        if (!upperControlActive || isAutoTracking || autonomousControlHold) {
            return;
        }

        this.vx = 0.0f;
        this.vy = 0.0f;
        this.vz = 0.0f;
        this.vyaw = 0.0f;
        this.upperControlActive = false;
        System.out.println("🕹️ 上位机已停止发送运动包，状态心跳保持运行，运动控制权已让给原厂手柄");
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
    private synchronized void sendMotionLoop() {
        if (!upperControlActive) {
            return;
        }

        long now = System.currentTimeMillis();
        long manualIdleTime = now - this.lastManualCommandTime;
        boolean autonomousHoldActive = autonomousControlHold
                && now - lastAutonomousControlHoldTime < AUTONOMOUS_CONTROL_HOLD_TIMEOUT_MS;

        if (autonomousControlHold && !autonomousHoldActive) {
            autonomousControlHold = false;
            System.out.println("⚠️ 自主运动控制租约超过 " + AUTONOMOUS_CONTROL_HOLD_TIMEOUT_MS
                    + "ms 未刷新，已停止自主运动并准备释放控制权");
        }

        // 自动巡线关闭或手动控制停止后，先保留一个很短的零速度发送窗口，
        // 超过窗口后必须彻底停止发包；持续发送零速度同样会覆盖原厂手柄。
        if (!isAutoTracking
                && !autonomousHoldActive
                && manualIdleTime >= MANUAL_COMMAND_TIMEOUT_MS + CONTROL_RELEASE_STOP_BURST_MS) {
            releaseUpperControl();
            return;
        }

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
                boolean signalJumpCenterActive = now < signalJumpCenterEndTime;
                boolean signalReversalActive = validDirection && now < signalReversalEndTime;

                float targetYaw;

                if (signalJumpCenterActive) {
                    // 左右连续跳变说明机器狗位于电缆中心附近：停止偏航，保持当前朝向继续前进。
                    lastAutoYawCmd = 0.0f;
                    finalYaw = 0.0f;

                } else if (signalReversalActive) {
                    // 连续跳变停止后，恢复原有的跨线动作：先停止前进并完成反向转向。
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

                if (signalJumpCenterActive) {
                    // 跳变中心状态复用现有中心速度比例，保持朝向低速通过中心区域。
                    finalVx = baseForward * CENTER_FORWARD_RATIO;
                } else if (signalReversalActive) {
                    // 保留原有逻辑：跨线后先停止前进并完成转向，再恢复向前运动。
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
            manualCommandExpired = manualIdleTime >= MANUAL_COMMAND_TIMEOUT_MS;
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
                    System.out.println("⚠️ 运动指令超过 " + MANUAL_COMMAND_TIMEOUT_MS
                            + "ms 未刷新，正在发送最后的零速度停车帧并释放控制权");
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

    private boolean sendJsonCommand(int type, int command, String items) {
        String json = buildJson(type, command, items);
        return sendUdp(json);
    }

    private boolean sendUdp(String json) {
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
            return true;
        } catch (Exception e) {
            System.err.println("UDP Error: " + e.getMessage());
            return false;
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
