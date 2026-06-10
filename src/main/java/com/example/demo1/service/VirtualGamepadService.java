package com.example.demo1.service;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
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
    private float vx = 0;   // 前后
    private float vy = 0;   // 左右
    private float vz = 0;   // ✅ 新增：高度方向速度
    private float vyaw = 0; // 转向
    private int currentMode = 0;

    // ✅ 新增：自动巡线相关参数
    private boolean isAutoTracking = false; // 是否开启自动巡线
    private int pipelineSignal = 0;         // 管线仪信号: 0=中, 1=向右找线, 2=向左找线
    private long lastSignalTime = 0;        // 上次收到信号的时间 (用于超时安全保护)

    // ✅ [修改点 1] 新增：自动巡线速度设置 (默认值)
    // 这些值现在可以由前端滑条通过 setAutoSpeeds 动态修改
    private float autoForwardSpeed = 0.2f;
    private float autoStrafeSpeed = 0.2f;  // S 型巡线摆动强度；前端 autoSpeedY 滑条会更新这个值

    // ✅ 新增：管线仪罗盘角度闭环参数
    // 角度约定：90°=机器狗朝向与电缆朝向一致；>90°=车头偏电缆左侧；<90°=车头偏电缆右侧
    private volatile Float cableAngleDeg = null;
    private long lastAngleTime = 0L;
    private float lastAngleErrorDeg = 0.0f;
    private long lastAngleControlTime = 0L;

    private static final float TARGET_CABLE_ANGLE_DEG = 90.0f;
    private static final float ANGLE_DEADBAND_DEG = 3.0f;   // 小角度死区，防止抖动
    private static final float ANGLE_KP = 0.55f;            // S 型巡线角度修正增益，现场可调
    private static final float ANGLE_KD = 0.02f;            // S 型巡线角度阻尼，现场可调
    private static final float MAX_YAW_CMD = 0.75f;         // 自动巡线最大偏航角速度
    private static final float MAX_AUTO_STRAFE = 0.35f;     // 前端摆动强度最大值
    private static final long PIPELINE_TIMEOUT_MS = 1000;   // 管线仪数据超时保护
    private static final long ANGLE_TIMEOUT_MS = 1000;      // 罗盘角度数据超时保护
    private static final long S_SWEEP_PERIOD_MS = 2600;     // S 型摆动周期，越大越丝滑
    private static final long CENTER_LOST_MS = 1800;        // 超过该时间未压中电缆，认为需要扩大搜索
    private static final float MIN_AUTO_FORWARD_SPEED = 0.08f;
    private static final float S_BASE_YAW = 0.22f;
    private static final float S_CORNER_YAW = 0.48f;
    private static final float SIGNAL_YAW_BIAS = 0.18f;
    private static final float CORNER_SIGNAL_YAW_BIAS = 0.30f;
    private static final float YAW_SMOOTHING = 0.22f;
    private static final float VY_SMOOTHING = 0.25f;
    private static final float PATH_TURN_ENTER_DEG = 46.0f;
    private static final float PATH_TURN_SUPPORT_DEG = 32.0f;
    private static final float PATH_TURN_EXIT_DEG = 18.0f;
    private static final float MAX_POINT_TURN_DEG = 18.0f;
    private static final float MAX_SIGNAL_FUTURE_BIAS_DEG = 16.0f;
    private static final long SIGNAL_FUTURE_BIAS_MS = 1800L;
    private static final long CORNER_MODE_MIN_DWELL_MS = 2200L;
    private static final long CORNER_STRAFE_PERIOD_MS = 1800L;
    private static final float MIN_CORNER_STRAFE = 0.10f;

    // ✅ S 型巡线状态
    private int currentPosture = 0;       // 0=初始平行/未知, 1=车头已偏右, 2=车头已偏左
    private long turnEndTime = 0;         // 记录本次转向结束的时间戳
    private int currentTurnDirection = 0; // 当前正在执行的转向：1(右), 2(左), 0(无/直行)
    private long lastCrossTime = 0;       // 记录上一次成功跨越(收到0)或调整姿态的时间
    private long autoTrackStartTime = 0L;
    private long lastCenterHitTime = 0L;
    private long lastSignalDirectionTime = 0L;
    private int lastSignalDirection = 0;
    private float lastAutoYawCmd = 0.0f;
    private float lastAutoVyCmd = 0.0f;
    private boolean autoCornerStrafeMode = false;
    private long lastAutoModeSwitchTime = 0L;
    private int lastSignalForPrediction = -1;
    private long signalStableSince = 0L;
    private volatile float predictedCableYawDeg = Float.NaN;
    private volatile float pathTurnDeg = 0.0f;
    private volatile float signalFutureBiasDeg = 0.0f;
    private volatile float cornerConfidence = 0.0f;
    private volatile boolean predictedCornerArea = false;
    private volatile int syncedCablePathPointCount = 0;
    private volatile long lastPathPredictionTime = 0L;
    
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
     * @param vy S 型摆动强度
     */
    public void setAutoSpeeds(float vx, float vy) {
        this.autoForwardSpeed = vx;
        this.autoStrafeSpeed = vy;
        System.out.println("⚙️ [后端] 自动巡线速度更新: Forward=" + vx + ", Strafe=" + vy);
    }

    public void updateCablePathPrediction(List<CablePoint> path) {
        if (path == null || path.size() < 4) {
            this.syncedCablePathPointCount = path == null ? 0 : path.size();
            this.pathTurnDeg = 0.0f;
            this.cornerConfidence = 0.0f;
            this.predictedCornerArea = false;
            return;
        }

        List<CablePoint> valid = new ArrayList<>();
        for (CablePoint p : path) {
            if (p == null || p.getX() == null || p.getY() == null) {
                continue;
            }
            double x = p.getX();
            double y = p.getY();
            if (!Double.isNaN(x) && !Double.isNaN(y) && !Double.isInfinite(x) && !Double.isInfinite(y)) {
                valid.add(p);
            }
        }

        int n = valid.size();
        this.syncedCablePathPointCount = n;
        if (n < 4) {
            this.pathTurnDeg = 0.0f;
            this.cornerConfidence = 0.0f;
            this.predictedCornerArea = false;
            return;
        }

        List<Float> segmentYaws = new ArrayList<>();
        int from = Math.max(0, n - 18);
        for (int i = from; i < n - 1; i++) {
            CablePoint a = valid.get(i);
            CablePoint b = valid.get(i + 1);
            double dx = b.getX() - a.getX();
            double dy = b.getY() - a.getY();
            if (Math.hypot(dx, dy) >= 0.04) {
                segmentYaws.add(yawDeg(a, b));
            }
        }

        if (segmentYaws.size() < 3) {
            this.pathTurnDeg = 0.0f;
            this.cornerConfidence = 0.0f;
            this.predictedCornerArea = false;
            return;
        }

        float limitedContinuousTurn = 0.0f;
        float smoothedYaw = segmentYaws.get(0);
        int validTurnSteps = 0;
        for (int i = 1; i < segmentYaws.size(); i++) {
            float rawStepTurn = angleDiffDeg(segmentYaws.get(i), segmentYaws.get(i - 1));
            float limitedStepTurn = clamp(rawStepTurn, -MAX_POINT_TURN_DEG, MAX_POINT_TURN_DEG);
            limitedContinuousTurn += limitedStepTurn;
            smoothedYaw += angleDiffDeg(segmentYaws.get(i), smoothedYaw) * 0.35f;
            validTurnSteps++;
        }

        int earlyCount = Math.min(4, segmentYaws.size());
        int lateStart = Math.max(0, segmentYaws.size() - earlyCount);
        float earlyYaw = averageYawDeg(segmentYaws, 0, earlyCount);
        float lateYaw = averageYawDeg(segmentYaws, lateStart, segmentYaws.size());
        float windowTurn = Math.abs(angleDiffDeg(lateYaw, earlyYaw));
        float maxPlausibleTurn = Math.max(PATH_TURN_ENTER_DEG, validTurnSteps * MAX_POINT_TURN_DEG);
        // 小弧度长弯不能靠逐段累加直接判成 90 度大转角；以窗口首尾方向为主，
        // 只保留少量连续转向贡献，用于真实圆角刚进入时的提前感知。
        float continuousTurnHint = Math.abs(limitedContinuousTurn) * 0.35f;
        float physicallyPlausibleTurn = Math.min(Math.max(windowTurn, continuousTurnHint), maxPlausibleTurn);
        float signalBias = estimateSignalFutureBiasDeg();
        float futureYaw = smoothedYaw + signalBias;

        this.signalFutureBiasDeg = signalBias;
        this.predictedCableYawDeg = normalizeDeg(futureYaw);
        this.pathTurnDeg = physicallyPlausibleTurn;
        this.cornerConfidence = clamp((physicallyPlausibleTurn - 18.0f) / 52.0f, 0.0f, 1.0f);
        this.predictedCornerArea = physicallyPlausibleTurn >= 35.0f && n >= 8;
        this.lastPathPredictionTime = System.currentTimeMillis();
    }

    /**
     * 接收管线仪数据的接口 (供 Controller 调用)
     * @param signal 0=正中，1=向右找线，2=向左找线
     * @param depth 管线仪深度，可为空
     * @param angleDeg 管线仪罗盘角度；90°=对齐，>90°=车头偏左，<90°=车头偏右
     */
    public void updatePipelineSignal(int signal, Float depth, Float angleDeg) {
        this.pipelineSignal = signal;
        long now = System.currentTimeMillis();
        this.lastSignalTime = now;
        if (signal != this.lastSignalForPrediction) {
            this.lastSignalForPrediction = signal;
            this.signalStableSince = now;
        }

        // 角度做一次轻量低通滤波，避免视觉识别偶发跳变直接造成机器狗抖动
        if (angleDeg != null && !Float.isNaN(angleDeg) && angleDeg >= 0.0f && angleDeg <= 180.0f) {
            if (this.cableAngleDeg == null) {
                this.cableAngleDeg = angleDeg;
            } else {
                this.cableAngleDeg = this.cableAngleDeg * 0.70f + angleDeg * 0.30f;
            }
            this.lastAngleTime = now;
        }

        // 推送到 /topic/pipeline，将单值改为对象
        if (template != null) {
            Map<String, Object> payload = new HashMap<>();
            payload.put("signal", signal);
            payload.put("depth", depth);
            payload.put("angle", this.cableAngleDeg);
            payload.put("rawAngle", angleDeg);
            payload.put("predictedYaw", this.predictedCableYawDeg);
            payload.put("pathTurnDeg", this.pathTurnDeg);
            payload.put("signalFutureBiasDeg", this.signalFutureBiasDeg);
            payload.put("cornerConfidence", this.cornerConfidence);
            payload.put("cornerArea", this.predictedCornerArea);
            payload.put("cornerStrafeMode", this.autoCornerStrafeMode);
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
            // ✅ [修改] 每次启动时重置姿态机和计时器
            this.currentPosture = 0; 
            this.currentTurnDirection = 0;
            this.turnEndTime = 0;
            long now = System.currentTimeMillis();
            this.lastCrossTime = now; // 启动时初始化跨越时间
            this.autoTrackStartTime = now;
            this.lastCenterHitTime = now;
            this.lastSignalDirectionTime = now;
            this.lastSignalDirection = 0;
            this.lastAutoYawCmd = 0.0f;
            this.lastAutoVyCmd = 0.0f;
            this.autoCornerStrafeMode = false;
            this.lastAutoModeSwitchTime = now;
            this.lastAngleControlTime = 0L;
            this.lastAngleErrorDeg = 0.0f;

            updateCommand(0, 0, 0, 0, 3); 
            System.out.println("🚀 自动巡线已启动：S 型前进扫描 + 90度圆角扩大搜索");
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

    private float clamp(float v, float min, float max) {
        return Math.max(min, Math.min(max, v));
    }

    private float yawDeg(CablePoint a, CablePoint b) {
        double dx = b.getX() - a.getX();
        double dy = b.getY() - a.getY();
        return (float) Math.toDegrees(Math.atan2(dy, dx));
    }

    private float angleDiffDeg(float a, float b) {
        float diff = a - b;
        while (diff > 180.0f) diff -= 360.0f;
        while (diff < -180.0f) diff += 360.0f;
        return diff;
    }

    private float averageYawDeg(List<Float> yaws, int startInclusive, int endExclusive) {
        double sx = 0.0;
        double sy = 0.0;
        for (int i = startInclusive; i < endExclusive; i++) {
            double rad = Math.toRadians(yaws.get(i));
            sx += Math.cos(rad);
            sy += Math.sin(rad);
        }
        return (float) Math.toDegrees(Math.atan2(sy, sx));
    }

    private float normalizeDeg(float deg) {
        while (deg > 180.0f) deg -= 360.0f;
        while (deg < -180.0f) deg += 360.0f;
        return deg;
    }

    private float estimateSignalFutureBiasDeg() {
        long now = System.currentTimeMillis();
        boolean signalFresh = now - lastSignalTime <= PIPELINE_TIMEOUT_MS;
        boolean directionFresh = now - lastSignalDirectionTime <= SIGNAL_FUTURE_BIAS_MS;
        int signalForBias = pipelineSignal != 0 ? pipelineSignal : (directionFresh ? lastSignalDirection : 0);

        if (!signalFresh || signalForBias == 0) {
            return 0.0f;
        }

        float stableRatio = clamp((now - signalStableSince) / (float) SIGNAL_FUTURE_BIAS_MS, 0.35f, 1.0f);
        if (signalForBias == 1) {
            return -MAX_SIGNAL_FUTURE_BIAS_DEG * stableRatio;
        }
        if (signalForBias == 2) {
            return MAX_SIGNAL_FUTURE_BIAS_DEG * stableRatio;
        }
        return 0.0f;
    }

    private float smoothYaw(float targetYaw) {
        this.lastAutoYawCmd += (targetYaw - this.lastAutoYawCmd) * YAW_SMOOTHING;
        return this.lastAutoYawCmd;
    }

    private float smoothVy(float targetVy) {
        this.lastAutoVyCmd += (targetVy - this.lastAutoVyCmd) * VY_SMOOTHING;
        return this.lastAutoVyCmd;
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
            // --- A. S 型自动巡线：持续前进，通过平滑偏航摆动扫线 ---
            long now = System.currentTimeMillis();

            // 1. 管线仪信号超时保护：超过 1 秒没有新数据，立即停车
            if (now - lastSignalTime > PIPELINE_TIMEOUT_MS) {
                finalVx = 0.0f;
                finalVy = 0.0f;
                finalYaw = 0.0f;
                this.lastAutoYawCmd = 0.0f;
            } else {
                // 2. 记录最近一次压中中心线和左右方向；圆角处通常会长时间压不中中心线
                if (pipelineSignal == 0) {
                    lastCenterHitTime = now;
                } else if (pipelineSignal == 1 || pipelineSignal == 2) {
                    lastSignalDirection = pipelineSignal;
                    lastSignalDirectionTime = now;
                }

                // 3. 罗盘角度只做柔性修正，不再硬闭环；让机器狗边前进边摆动探测
                boolean angleFresh = (this.cableAngleDeg != null) && (now - lastAngleTime <= ANGLE_TIMEOUT_MS);
                float absAngleErrorDeg = 0.0f;
                float angleYawTrim = 0.0f;

                if (angleFresh) {
                    float angleErrorDeg = this.cableAngleDeg - TARGET_CABLE_ANGLE_DEG;

                    // 小角度死区，避免 88~92° 附近反复抖动
                    if (Math.abs(angleErrorDeg) < ANGLE_DEADBAND_DEG) {
                        angleErrorDeg = 0.0f;
                    }

                    long dtMs = (lastAngleControlTime > 0L) ? Math.max(now - lastAngleControlTime, 20L) : 50L;
                    float dt = dtMs / 1000.0f;

                    float dErrorDeg = (angleErrorDeg - lastAngleErrorDeg) / dt;
                    lastAngleErrorDeg = angleErrorDeg;
                    lastAngleControlTime = now;

                    float errorRad = (float) Math.toRadians(angleErrorDeg);
                    float dErrorRad = (float) Math.toRadians(dErrorDeg);

                    // 注意符号：angleErrorDeg > 0 表示车头偏左，需要右转；本机右转 Yaw 为负
                    angleYawTrim = clamp(-(ANGLE_KP * errorRad + ANGLE_KD * dErrorRad), -0.28f, 0.28f);
                    absAngleErrorDeg = Math.abs(angleErrorDeg);
                }

                // 4. 融合路径预测与当前管线仪状态，判断是否进入大转角区域
                boolean centerLost = now - lastCenterHitTime > CENTER_LOST_MS;
                boolean pathPredictionFresh = now - lastPathPredictionTime <= 3000L;
                boolean pathPredictsCorner = pathPredictionFresh && pathTurnDeg >= PATH_TURN_ENTER_DEG;
                boolean pathPredictsSupportedTurn = pathPredictionFresh && pathTurnDeg >= PATH_TURN_SUPPORT_DEG;
                boolean pathPredictsStraight = pathPredictionFresh && pathTurnDeg <= PATH_TURN_EXIT_DEG;
                boolean oneSideSignalLong = pipelineSignal != 0 && now - signalStableSince > 1500L;
                boolean signalSupportsTurn = oneSideSignalLong && Math.abs(signalFutureBiasDeg) >= 12.0f;
                boolean angleShowsCorner = angleFresh && absAngleErrorDeg > 34.0f;
                boolean strongAngleCorner = angleFresh && absAngleErrorDeg > 42.0f;
                boolean nearStraight = pathPredictionFresh
                        && pathTurnDeg <= PATH_TURN_EXIT_DEG
                        && (!angleFresh || absAngleErrorDeg <= 24.0f)
                        && !centerLost;
                boolean likelyCorner = pathPredictsCorner
                        || (pathPredictsSupportedTurn && signalSupportsTurn && (centerLost || angleShowsCorner))
                        || (pathPredictsSupportedTurn && cornerConfidence >= 0.45f && (centerLost || strongAngleCorner))
                        || (!pathPredictionFresh && centerLost && strongAngleCorner);

                boolean canSwitchMode = now - lastAutoModeSwitchTime >= CORNER_MODE_MIN_DWELL_MS;
                boolean shouldExitCornerMode = autoCornerStrafeMode
                        && canSwitchMode
                        && nearStraight
                        && !oneSideSignalLong
                        && Math.abs(signalFutureBiasDeg) < 10.0f
                        && (!angleFresh || absAngleErrorDeg <= 22.0f);
                boolean shouldEnterCornerMode = !autoCornerStrafeMode
                        && canSwitchMode
                        && !nearStraight
                        && likelyCorner;

                if (shouldEnterCornerMode) {
                    autoCornerStrafeMode = true;
                    lastAutoModeSwitchTime = now;
                    System.out.println("🟡 自动巡线切换：大转角横移前进模式");
                } else if (shouldExitCornerMode) {
                    autoCornerStrafeMode = false;
                    lastAutoModeSwitchTime = now;
                    System.out.println("🟢 自动巡线切换：S 型丝滑前进模式");
                }

                if (autoCornerStrafeMode) {
                    this.cornerConfidence = Math.max(this.cornerConfidence, 0.75f);
                }
                this.predictedCornerArea = autoCornerStrafeMode || likelyCorner;

                // 5. 根据模式生成速度：直线/小弧度走 S 型，大转角走横移前进
                float sweepStrength = clamp(this.autoStrafeSpeed, 0.0f, MAX_AUTO_STRAFE);
                int activeDirection = pipelineSignal;
                if (activeDirection == 0 && now - lastSignalDirectionTime < CENTER_LOST_MS) {
                    activeDirection = lastSignalDirection;
                }

                if (autoCornerStrafeMode) {
                    float strafeCmd = Math.max(MIN_CORNER_STRAFE, sweepStrength);
                    float targetVy;
                    if (activeDirection == 1) {
                        targetVy = -strafeCmd;     // 向右横移找线
                    } else if (activeDirection == 2) {
                        targetVy = strafeCmd;      // 向左横移找线
                    } else {
                        double strafePhase = ((now - autoTrackStartTime) % CORNER_STRAFE_PERIOD_MS) / (double) CORNER_STRAFE_PERIOD_MS;
                        targetVy = (float) Math.sin(strafePhase * Math.PI * 2.0) * strafeCmd;
                    }

                    finalVy = smoothVy(clamp(targetVy, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE));
                    finalYaw = smoothYaw(clamp(angleYawTrim * 0.55f, -0.32f, 0.32f));
                } else {
                    float baseSweepYaw = S_BASE_YAW + sweepStrength;
                    double phase = ((now - autoTrackStartTime) % S_SWEEP_PERIOD_MS) / (double) S_SWEEP_PERIOD_MS;
                    float sWaveYaw = (float) Math.sin(phase * Math.PI * 2.0) * baseSweepYaw;

                    float signalBias = 0.0f;
                    if (activeDirection == 1) {
                        signalBias = -SIGNAL_YAW_BIAS;
                    } else if (activeDirection == 2) {
                        signalBias = SIGNAL_YAW_BIAS;
                    }

                    finalVy = smoothVy(0.0f);
                    finalYaw = smoothYaw(clamp(sWaveYaw + signalBias + angleYawTrim, -MAX_YAW_CMD, MAX_YAW_CMD));
                }

                // 6. 自适应前进速度：始终保留前进底速，圆角时慢走扩大探测时间
                float baseForward = this.autoForwardSpeed <= 0.01f
                        ? 0.0f
                        : Math.max(this.autoForwardSpeed, MIN_AUTO_FORWARD_SPEED);
                if (!angleFresh) {
                    finalVx = baseForward * 0.65f;
                } else if (autoCornerStrafeMode && absAngleErrorDeg > 45.0f) {
                    finalVx = baseForward * 0.35f;
                } else if (autoCornerStrafeMode) {
                    finalVx = baseForward * 0.50f;
                } else if (absAngleErrorDeg > 45.0f) {
                    finalVx = baseForward * 0.45f;
                } else if (absAngleErrorDeg > 25.0f) {
                    finalVx = baseForward * 0.65f;
                } else if (absAngleErrorDeg > 10.0f) {
                    finalVx = baseForward * 0.82f;
                } else {
                    finalVx = baseForward;
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
