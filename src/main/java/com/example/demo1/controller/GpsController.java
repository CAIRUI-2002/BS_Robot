package com.example.demo1.controller; // 🌟 修复 1：添加正确的包路径声明

import java.util.Map;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.messaging.simp.SimpMessagingTemplate;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequestMapping("/api/gps")
public class GpsController {

    @Autowired
    private SimpMessagingTemplate messagingTemplate;

    @PostMapping("/update")
    public String updateGps(@RequestBody Map<String, Object> gpsData) {
        
        // 🌟 修复 2：将 gpsData 强制转换为 Object，明确告诉编译器调用哪个方法，消除红叉
        messagingTemplate.convertAndSend("/topic/gps", (Object) gpsData);

        return "success";
    }
}