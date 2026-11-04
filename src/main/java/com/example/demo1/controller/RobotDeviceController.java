package com.example.demo1.controller;

import java.util.Map;

import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import com.example.demo1.service.VirtualGamepadService;

@RestController
@RequestMapping("/api/robot")
public class RobotDeviceController {

    private final VirtualGamepadService gamepadService;

    public RobotDeviceController(VirtualGamepadService gamepadService) {
        this.gamepadService = gamepadService;
    }

    @PostMapping("/lights")
    public ResponseEntity<Map<String, Object>> setLights(@RequestBody LightControlRequest request) {
        if (request == null || !isBinary(request.front()) || !isBinary(request.back())) {
            return ResponseEntity.badRequest().body(Map.of(
                    "success", false,
                    "message", "front 和 back 必须同时提供，且只能是 0 或 1"));
        }

        boolean sent = gamepadService.setLights(request.front(), request.back());
        if (!sent) {
            return ResponseEntity.status(HttpStatus.BAD_GATEWAY).body(Map.of(
                    "success", false,
                    "message", "照明灯控制指令发送失败"));
        }

        return ResponseEntity.ok(Map.of(
                "success", true,
                "message", "照明灯控制指令已发送，等待设备状态确认",
                "front", request.front(),
                "back", request.back()));
    }

    private boolean isBinary(Integer value) {
        return value != null && (value == 0 || value == 1);
    }

    public record LightControlRequest(Integer front, Integer back) {
    }
}
