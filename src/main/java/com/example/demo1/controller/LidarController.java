package com.example.demo1.controller;

import com.example.demo1.service.LidarControlService;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import java.io.IOException;
import java.util.Map;

@RestController
@RequestMapping("/api/lidar")
public class LidarController {

    private final LidarControlService lidarControlService;

    public LidarController(LidarControlService lidarControlService) {
        this.lidarControlService = lidarControlService;
    }

    @PostMapping("/start")
    public ResponseEntity<Map<String, Object>> start() {
        try {
            return ResponseEntity.ok(lidarControlService.startRecording());
        } catch (IOException e) {
            return ResponseEntity.status(HttpStatus.BAD_GATEWAY).body(Map.of(
                    "success", false,
                    "message", e.getMessage()
            ));
        }
    }

    @PostMapping("/stop")
    public ResponseEntity<Map<String, Object>> stop() {
        try {
            return ResponseEntity.ok(lidarControlService.stopRecording());
        } catch (IOException e) {
            return ResponseEntity.status(HttpStatus.BAD_GATEWAY).body(Map.of(
                    "success", false,
                    "message", e.getMessage()
            ));
        }
    }
}
