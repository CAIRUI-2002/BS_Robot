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

@Service
public class VirtualGamepadService implements InitializingBean, DisposableBean {

    // ✅ M20 Pro 运动主机 IP (请确保 IP 正确)
    private static final String ROBOT_IP = "10.21.31.103";
    private static final int ROBOT_PORT = 30000;

    @Autowired
    private SimpMessagingTemplate template;

    private DatagramSocket socket;
    private ScheduledExecutorService scheduler;

    // 运动参数 (手动模式使用 - 由前端 /app/move 实时更新)
    private float vx = 0;   // 前后
    private float vy = 0;   // 左右
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

    @Override
    public void afterPropertiesSet() throws Exception {
        socket = new DatagramSocket();
        scheduler = Executors.newSingleThreadScheduledExecutor();

        // 1. 心跳 (10Hz) - 保持连接必须
        scheduler.scheduleAtFixedRate(this::sendHeartbeat, 0, 100, TimeUnit.MILLISECONDS);

        // 2. 运动指令 (20Hz) - 负责发送移动控制
        scheduler.scheduleAtFixedRate(this::sendMotionLoop, 0, 50, TimeUnit.MILLISECONDS);

        System.out.println("✅ M20 Pro 服务启动 - 已集成动态速度调节功能");
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
            // 开启时，强制切入 Mode 3 (足式模式)，因为只有它能横移
            updateCommand(0, 0, 0, 3);
            System.out.println("🚀 自动巡线已启动 (等待信号...)");
        } else {
            // 关闭时，速度归零
            this.vx = 0;
            this.vy = 0;
            this.vyaw = 0;
            System.out.println("🛑 自动巡线已停止");
        }
    }

    /**
     * 更新控制指令 (手动/模式切换)
     */
    public void updateCommand(float v, float y, float w, int mode) {
        this.vx = v;
        this.vy = y;
        this.vyaw = w;

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
        // 拦截非运动模式
        if (currentMode == 0 || currentMode == 1 || currentMode == 2 || currentMode == 4) {
            return;
        }

        float finalVx = 0;
        float finalVy = 0;
        float finalYaw = 0;

        if (isAutoTracking) {
            // --- A. 自动巡线逻辑 ---

            // 1. 安全保护：如果超过 2秒 没收到信号，自动停车
            if (System.currentTimeMillis() - lastSignalTime > 2000) {
                finalVx = 0;
                finalVy = 0;
            } else {
                // ✅ [修改点 3] 使用成员变量，不再使用局部硬编码值

                // 2. 根据管线仪信号决定怎么走
                if (pipelineSignal == 0) {
                    // [信号 0] 正上方: 只直行
                    finalVx = this.autoForwardSpeed;
                    finalVy = 0;

                } else if (pipelineSignal == 1) {
                    // [信号 1] 向右走: 右平移 (Vy < 0)
                    finalVx = this.autoForwardSpeed * 0.8f; // 纠偏时稍微减速
                    finalVy = -this.autoStrafeSpeed;        // 使用设定的横移速度

                } else if (pipelineSignal == 2) {
                    // [信号 2] 向左走: 左平移 (Vy > 0)
                    finalVx = this.autoForwardSpeed * 0.8f;
                    finalVy = this.autoStrafeSpeed;         // 使用设定的横移速度
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

        // 构建 Items JSON
        String items = String.format(
                "\"X\": %.4f, \"Y\": %.4f, \"Z\": 0.0, \"Roll\": 0.0, \"Pitch\": 0.0, \"Yaw\": %.4f",
                finalVx * scaleX,
                finalVy * scaleY,
                finalYaw * scaleYaw
        );

        sendJsonCommand(2, 21, items);
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
        if (socket != null) socket.close();
    }
}