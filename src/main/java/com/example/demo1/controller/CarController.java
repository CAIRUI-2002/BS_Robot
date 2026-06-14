package com.example.demo1.controller;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.util.Map;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

import com.fasterxml.jackson.databind.ObjectMapper;

import jakarta.annotation.PostConstruct;
import jakarta.annotation.PreDestroy;

@RestController
@RequestMapping("/api/car")
public class CarController {

    // 履带小车连接机器狗 WiFi 后分配的 IP 地址
    // 建议在 application.yml / application.properties 中配置 car.udp.ip=192.168.x.x
    // 如果没有配置，默认使用 192.168.123.161（请自行修改为你小车的实际 IP）
    @Value("${car.udp.ip:10.21.31.65}") 
    private String carIp;

    // 与 Python 脚本中 UDP_PORT 对应的端口
    @Value("${car.udp.port:8888}")
    private int carPort;

    private DatagramSocket udpSocket;
    private InetAddress carAddress;
    private final ObjectMapper objectMapper = new ObjectMapper();

    @PostConstruct
    public void init() {
        try {
            // 初始化复用同一个 UDP Socket 对象，避免高频创建销毁导致资源枯竭
            udpSocket = new DatagramSocket();
            carAddress = InetAddress.getByName(carIp);
            System.out.println("✅ 履带车 UDP 转发服务已启动，目标指向 -> " + carIp + ":" + carPort);
        } catch (Exception e) {
            System.err.println("❌ 履带车 UDP 服务初始化失败: " + e.getMessage());
        }
    }

    @PreDestroy
    public void destroy() {
        if (udpSocket != null && !udpSocket.isClosed()) {
            udpSocket.close();
            System.out.println("🛑 履带车 UDP 服务已安全关闭");
        }
    }

    /**
     * 修改现有的 /move 接口，加入 "type": "move" 标识
     */
    @GetMapping("/move")
    public String moveCar(@RequestParam int x, @RequestParam int y) {
        if (udpSocket == null || carAddress == null) {
            return "error: udp socket not initialized";
        }

        try {
            // 加入 type 字段
            String jsonCmd = String.format("{\"type\": \"move\", \"x\": %d, \"y\": %d}", x, y);
            byte[] sendData = jsonCmd.getBytes("UTF-8");

            DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, carAddress, carPort);
            System.out.println("🎮 [HTTP->UDP] 收到履带车摇杆指令: x=" + x + ", y=" + y);
System.out.println("🎮 [HTTP->UDP] 正在发送到履带车: " + carIp + ":" + carPort + " 数据=" + jsonCmd);

udpSocket.send(sendPacket);

System.out.println("✅ [HTTP->UDP] UDP发送完成");
return "success";
        } catch (Exception e) {
            System.err.println("⚠️ UDP 指令下发小车失败: " + e.getMessage());
            return "error";
        }
    }

    /**
     * 新增：接收前端的破损点坐标并下发给履带小车
     * API 路径: /api/car/target?lat=xxx&lon=xxx&yaw=xxx
     */
    @GetMapping("/target")
    public String sendTarget(@RequestParam double lat, @RequestParam double lon, @RequestParam double yaw) {
        if (udpSocket == null || carAddress == null) {
            return "error: udp socket not initialized";
        }

        try {
            // 组装目标点 JSON，标识类型为 "target"
            String jsonCmd = String.format("{\"type\": \"target\", \"lat\": %f, \"lon\": %f, \"yaw\": %f}", lat, lon, yaw);
            byte[] sendData = jsonCmd.getBytes("UTF-8");

            DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, carAddress, carPort);
            udpSocket.send(sendPacket);
            
            System.out.println("📍 已转发破损点坐标至履带车");
            return "success";
        } catch (Exception e) {
            System.err.println("⚠️ UDP 目标下发失败: " + e.getMessage());
            return "error";
        }
    }

    /**
     * 新增：接收前端已经规划好的扫描路径并下发给履带小车。
     * 履带小车只负责按 waypoints 执行导航、扫描和 GPR 数据采集，不再自行生成 Z 字形路径。
     * API 路径: POST /api/car/scan-path
     */
    @PostMapping("/scan-path")
    public String sendScanPath(@RequestBody Map<String, Object> body) {
        if (udpSocket == null || carAddress == null) {
            return "error: udp socket not initialized";
        }

        try {
            body.put("type", "scan_path");
            byte[] sendData = objectMapper.writeValueAsBytes(body);

            DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, carAddress, carPort);
            udpSocket.send(sendPacket);

            System.out.println("🧭 已转发上位机规划扫描路径至履带车，UDP字节数=" + sendData.length);
            return "success";
        } catch (Exception e) {
            System.err.println("⚠️ UDP 扫描路径下发失败: " + e.getMessage());
            return "error: " + e.getMessage();
        }
    }

}