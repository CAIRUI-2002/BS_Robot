package com.example.demo1.controller;

import com.example.demo1.service.VirtualGamepadService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.messaging.handler.annotation.MessageMapping;
import org.springframework.stereotype.Controller;

import java.util.Map;

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
        // 1. 解析运动参数 (默认为 0)
        float v_forward = cmd.getOrDefault("v", 0.0).floatValue(); // 前后
        float w_turn    = cmd.getOrDefault("w", 0.0).floatValue(); // 转向
        float y_strafe  = cmd.getOrDefault("y", 0.0).floatValue(); // 横移

        // 2. 解析模式参数 (z 轴被你用作模式 Mode)
        // home.js 定义: STAND=1.0, WALK=6.0
        float modeVal = cmd.getOrDefault("z", 0.0).floatValue();
        int mode = (int) modeVal;

        // 3. 调用 Service 发送 UDP
        // 注意：如果 mode 为 0，表示单纯移动，不切换模式
        gamepadService.updateCommand(v_forward, y_strafe, w_turn, mode);
    }
}