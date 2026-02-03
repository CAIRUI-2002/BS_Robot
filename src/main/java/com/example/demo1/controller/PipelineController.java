package com.example.demo1.controller;

import com.example.demo1.service.VirtualGamepadService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.*;

@RestController
@RequestMapping("/api/pipeline")
public class PipelineController {

    @Autowired
    private VirtualGamepadService gamepadService;

    /**
     * ✅ 树莓派调用的接口
     * URL: http://<电脑IP>:8080/api/pipeline/update?signal=1
     * signal: 0=正中, 1=偏左, 2=偏右
     */
    @GetMapping("/update")
    public String updateSignal(@RequestParam int signal) {
        gamepadService.updatePipelineSignal(signal);
        return "success: " + signal;
    }

    /**
     * ✅ 前端网页调用的接口：开启/关闭自动模式
     */
    @PostMapping("/toggle")
    public String toggleAuto(@RequestParam boolean enable) {
        gamepadService.setAutoTracking(enable);
        return enable ? "Auto Mode ON" : "Auto Mode OFF";
    }

    /**
     * ✅ [新增] 前端网页滑条调用的接口：设置自动巡线速度
     * URL: POST /api/pipeline/speed?vx=0.2&vy=0.2
     */
    @PostMapping("/speed")
    public String setPipelineSpeed(@RequestParam float vx, @RequestParam float vy) {
        // 调用 Service 更新成员变量
        gamepadService.setAutoSpeeds(vx, vy);
        return "Speed updated: vx=" + vx + ", vy=" + vy;
    }
}