package com.example.demo1.service;

import com.example.demo1.model.Point3D;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Service;

import jakarta.annotation.PostConstruct;
import jakarta.annotation.PreDestroy;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

@Service
public class SlamCacheService {

    private static final Logger log = LoggerFactory.getLogger(SlamCacheService.class);

    private static final int MAX_POINT_COUNT = 2_000_000;
    private static final String ROBOT_WS_URL = "ws://10.21.31.104:9090";

    private final List<Point3D> pointBuffer = Collections.synchronizedList(new ArrayList<>());
    private final ObjectMapper objectMapper = new ObjectMapper();

    private final AtomicBoolean connected = new AtomicBoolean(false);
    private final AtomicBoolean reconnecting = new AtomicBoolean(false);
    private final AtomicLong messageCount = new AtomicLong(0);

    private volatile WebSocketClient wsClient;

    @PostConstruct
    public void initWebSocketClient() {
        connectAsync();
    }

    private void connectAsync() {
        try {
            WebSocketClient old = wsClient;
            if (old != null && old.isOpen()) {
                return;
            }

            wsClient = new WebSocketClient(new URI(ROBOT_WS_URL)) {
                @Override
                public void onOpen(ServerHandshake handshakedata) {
                    connected.set(true);
                    reconnecting.set(false);
                    log.info("✅ Java Backend 成功连接到机器狗 SLAM WebSocket: {}", ROBOT_WS_URL);
                }

                @Override
                public void onMessage(String message) {
                    messageCount.incrementAndGet();
                    handleWsMessage(message);
                }

                @Override
                public void onClose(int code, String reason, boolean remote) {
                    connected.set(false);
                    log.warn("⚠️ SLAM WebSocket 连接断开, code={}, reason={}, remote={}", code, reason, remote);
                    scheduleReconnect();
                }

                @Override
                public void onError(Exception ex) {
                    connected.set(false);
                    log.error("❌ SLAM WebSocket 发生错误: {}", ex.getMessage());
                    scheduleReconnect();
                }
            };

            wsClient.connect();

        } catch (URISyntaxException e) {
            connected.set(false);
            reconnecting.set(false);
            log.error("WebSocket URI 格式错误: {}", ROBOT_WS_URL, e);
        } catch (Exception e) {
            connected.set(false);
            reconnecting.set(false);
            log.error("启动 SLAM WebSocket 连接失败: {}", e.getMessage(), e);
            scheduleReconnect();
        }
    }

    private void scheduleReconnect() {
        if (!reconnecting.compareAndSet(false, true)) {
            return;
        }

        Thread t = new Thread(() -> {
            try {
                Thread.sleep(3000);
            } catch (InterruptedException ignored) {
                Thread.currentThread().interrupt();
            }

            try {
                WebSocketClient old = wsClient;
                if (old != null) {
                    try {
                        old.close();
                    } catch (Exception ignored) {
                    }
                }
                log.info("🔄 正在重连 SLAM WebSocket: {}", ROBOT_WS_URL);
                connectAsync();
            } finally {
                reconnecting.set(false);
            }
        }, "slam-ws-reconnect");

        t.setDaemon(true);
        t.start();
    }

    private void handleWsMessage(String message) {
        try {
            JsonNode rootNode = objectMapper.readTree(message);
            if (!rootNode.has("type") || !"slam_data".equals(rootNode.get("type").asText())) {
                return;
            }

            JsonNode scanNode = rootNode.get("scan");
            if (scanNode == null || !scanNode.isArray() || scanNode.isEmpty()) {
                return;
            }

            // 兼容两种格式：
            // 1. 扁平数组：[x, y, z, r, g, b, x, y, z, r, g, b...]
            // 2. 嵌套点数组：[[x, y, z, r, g, b], ...]
            if (scanNode.get(0).isArray()) {
                for (JsonNode p : scanNode) {
                    if (p != null && p.isArray() && p.size() >= 3) {
                        addPoint(
                                p.get(0).asDouble(),
                                p.get(1).asDouble(),
                                p.get(2).asDouble(),
                                p.size() > 3 ? p.get(3).asInt(255) : 255,
                                p.size() > 4 ? p.get(4).asInt(255) : 255,
                                p.size() > 5 ? p.get(5).asInt(255) : 255
                        );
                    }
                }
            } else {
                for (int i = 0; i + 2 < scanNode.size(); i += 6) {
                    addPoint(
                            scanNode.get(i).asDouble(),
                            scanNode.get(i + 1).asDouble(),
                            scanNode.get(i + 2).asDouble(),
                            i + 3 < scanNode.size() ? scanNode.get(i + 3).asInt(255) : 255,
                            i + 4 < scanNode.size() ? scanNode.get(i + 4).asInt(255) : 255,
                            i + 5 < scanNode.size() ? scanNode.get(i + 5).asInt(255) : 255
                    );
                }
            }

            long count = getPointCount();
            if (count > 0 && count % 100000 == 0) {
                log.info("📌 SLAM 点云缓存数量: {}", count);
            }

        } catch (Exception e) {
            log.error("解析 SLAM 数据异常: {}", e.getMessage());
        }
    }

    private void addPoint(double x, double y, double z, int r, int g, int b) {
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(z)) {
            return;
        }

        synchronized (pointBuffer) {
            if (pointBuffer.size() >= MAX_POINT_COUNT) {
                return;
            }
            pointBuffer.add(new Point3D(
                    x,
                    y,
                    z,
                    clampColor(r),
                    clampColor(g),
                    clampColor(b)
            ));
        }
    }

    private int clampColor(int v) {
        return Math.max(0, Math.min(255, v));
    }

    public List<Point3D> getAllPoints() {
        synchronized (pointBuffer) {
            return new ArrayList<>(pointBuffer);
        }
    }

    public int getPointCount() {
        synchronized (pointBuffer) {
            return pointBuffer.size();
        }
    }

    public boolean isConnected() {
        WebSocketClient client = wsClient;
        return connected.get() && client != null && client.isOpen();
    }

    public long getMessageCount() {
        return messageCount.get();
    }

    public void clearCache() {
        synchronized (pointBuffer) {
            pointBuffer.clear();
        }
        log.info("🧹 SLAM 点云缓存已清空");
    }

    @PreDestroy
    public void closeConnection() {
        WebSocketClient client = wsClient;
        if (client != null) {
            client.close();
        }
    }
}
