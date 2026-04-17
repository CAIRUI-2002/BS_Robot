package com.example.demo1.service;

import com.example.demo1.model.Point3D; // 引入刚才建的实体类
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

@Service
public class SlamCacheService {

    private static final Logger log = LoggerFactory.getLogger(SlamCacheService.class);
    
    // 线程安全的 List，用于缓存点云
    private final List<Point3D> pointBuffer = Collections.synchronizedList(new ArrayList<>());
    
    private WebSocketClient wsClient;
    private final ObjectMapper objectMapper = new ObjectMapper();

    // 替换为你的机器狗实际 WebSocket IP 和端口
    private final String ROBOT_WS_URL = "ws://10.21.31.104:9090";

    @PostConstruct
    public void initWebSocketClient() {
        try {
            wsClient = new WebSocketClient(new URI(ROBOT_WS_URL)) {
                @Override
                public void onOpen(ServerHandshake handshakedata) {
                    log.info("✅ Java Backend 成功连接到机器狗 SLAM WebSocket!");
                }

                @Override
                public void onMessage(String message) {
                    try {
                        JsonNode rootNode = objectMapper.readTree(message);
                        // 过滤只处理 slam_data
                        if (rootNode.has("type") && "slam_data".equals(rootNode.get("type").asText())) {
                            JsonNode scanNode = rootNode.get("scan");
                            if (scanNode != null && scanNode.isArray()) {
                                // 按照前端的逻辑，数组结构为 [x, y, z, r, g, b, x, y, z, r, g, b...] 步长为 6
                                for (int i = 0; i < scanNode.size(); i += 6) {
                                    // 防止越界
                                    if (i + 5 >= scanNode.size()) break;

                                    double x = scanNode.get(i).asDouble();
                                    double y = scanNode.get(i + 1).asDouble();
                                    double z = scanNode.get(i + 2).asDouble();
                                    int r = scanNode.get(i + 3).asInt();
                                    int g = scanNode.get(i + 4).asInt();
                                    int b = scanNode.get(i + 5).asInt();

                                    // 设置一个缓存上限防内存溢出 (例如 200万个点)
                                    if (pointBuffer.size() < 2000000) {
                                        pointBuffer.add(new Point3D(x, y, z, r, g, b));
                                    }
                                }
                            }
                        }
                    } catch (Exception e) {
                        log.error("解析 SLAM 数据异常: {}", e.getMessage());
                    }
                }

                @Override
                public void onClose(int code, String reason, boolean remote) {
                    log.warn("⚠️ SLAM WebSocket 连接断开, 原因: {}", reason);
                    // 可以尝试在此处添加重连逻辑
                }

                @Override
                public void onError(Exception ex) {
                    log.error("❌ SLAM WebSocket 发生错误: {}", ex.getMessage());
                }
            };
            wsClient.connect();
            
        } catch (URISyntaxException e) {
            log.error("WebSocket URI 格式错误", e);
        }
    }

    /**
     * 供 Controller 调用：获取全部缓存点
     */
    public List<Point3D> getAllPoints() {
        return new ArrayList<>(pointBuffer);
    }

    /**
     * 供 Controller 调用：清空缓存
     */
    public void clearCache() {
        pointBuffer.clear();
        log.info("🧹 SLAM 点云缓存已清空");
    }

    @PreDestroy
    public void closeConnection() {
        if (wsClient != null) {
            wsClient.close();
        }
    }
}