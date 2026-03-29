package com.example.demo1.controller;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

import jakarta.annotation.PostConstruct;
import jakarta.annotation.PreDestroy;

@RestController
@RequestMapping("/api/car")
public class CarController {

    // 履带小车连接机器狗 WiFi 后分配的 IP 地址
    // 建议在 application.yml / application.properties 中配置 car.udp.ip=192.168.x.x
    // 如果没有配置，默认使用 192.168.123.161（请自行修改为你小车的实际 IP）
    @Value("${car.udp.ip:10.21.41.65}") 
    private String carIp;

    // 与 Python 脚本中 UDP_PORT 对应的端口
    @Value("${car.udp.port:8888}")
    private int carPort;

    private DatagramSocket udpSocket;
    private InetAddress carAddress;

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
     * 接收前端摇杆发送的 x 和 y 坐标 (-100 到 100)
     * API 路径: /api/car/move?x=50&y=-20
     */
    @GetMapping("/move")
    public String moveCar(@RequestParam int x, @RequestParam int y) {
        if (udpSocket == null || carAddress == null) {
            return "error: udp socket not initialized";
        }

        try {
            // 将坐标组装成 Python 端需要解析的 JSON 字符串
            String jsonCmd = String.format("{\"x\": %d, \"y\": %d}", x, y);
            byte[] sendData = jsonCmd.getBytes("UTF-8");

            // 打包并发送 UDP 报文
            DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, carAddress, carPort);
            udpSocket.send(sendPacket);
            
            return "success";
        } catch (Exception e) {
            System.err.println("⚠️ UDP 指令下发小车失败: " + e.getMessage());
            return "error";
        }
    }
}