package com.example.demo1.controller;

import java.util.List;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

import com.example.demo1.model.CablePoint;
import com.example.demo1.service.VirtualGamepadService;

@RestController
@RequestMapping("/api/pipeline")
public class PipelineController {

    @Autowired
    private VirtualGamepadService gamepadService;

    /**
     * 管线探测模块数据入口。
     * depth/rawDepthCm 的单位均为 cm；rawDepthCm 仅作为旧发送端的参数别名保留。
     */
    @GetMapping("/update")
    public String updateSignal(@RequestParam int signal,
            @RequestParam(required = false) Float depth,
            @RequestParam(required = false) Float rawDepthCm,
            @RequestParam(required = false, name = "angle") Float angle) {
        Float depthCm = rawDepthCm != null ? rawDepthCm : depth;
        gamepadService.updatePipelineSignal(signal, depthCm, angle);
        return "success: signal=" + signal
                + ", depthCm=" + depthCm
                + ", angle=" + angle;
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

    /**
     * 前端同步已经探测出的电缆路径点，用于预测地下电缆走向和大转角区域。
     */
    @PostMapping("/path")
    public String updateCablePath(@RequestBody List<CablePoint> path) {
        gamepadService.updateCablePathPrediction(path);
        return "Path prediction updated: points=" + (path == null ? 0 : path.size());
    }
}
