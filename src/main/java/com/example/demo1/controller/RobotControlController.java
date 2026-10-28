package com.example.demo1.controller;

import java.util.Map;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.messaging.handler.annotation.MessageMapping;
import org.springframework.stereotype.Controller;

import com.example.demo1.service.VirtualGamepadService;

@Controller
public class RobotControlController {

    @Autowired
    private VirtualGamepadService gamepadService;

    /**
     * 接收前端 home.js 发送的指令
     * payload: {'v': 0.5, 'w': 0, 'y': 0, 'z': 6.0}
     */
    @MessageMapping("/move")
public void handleMove(Map<String, Double> cmd) {
    float v_forward = cmd.getOrDefault("v", 0.0).floatValue();
    float w_turn    = cmd.getOrDefault("w", 0.0).floatValue();
    float y_strafe  = cmd.getOrDefault("y", 0.0).floatValue();

    // ✅ 修正：z 轴现在代表真实的高度速度
    float v_height  = cmd.getOrDefault("z", 0.0).floatValue(); 

    // ✅ 修正：从新的 mode 字段读取模式，不再从 z 读取
    float modeVal   = cmd.getOrDefault("mode", 0.0).floatValue();
    int mode = (int) modeVal;
    boolean holdControl = cmd.getOrDefault("holdControl", 0.0) > 0.5;

    // 调用 Service
    gamepadService.updateCommand(v_forward, y_strafe, w_turn, v_height, mode, holdControl);
}
}
