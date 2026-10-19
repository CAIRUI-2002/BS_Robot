package com.example.demo1.controller;

import com.example.demo1.model.CablePoint;
import com.example.demo1.service.PipelineDepthService;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import java.util.List;
import java.util.Map;

@RestController
@RequestMapping("/api/pipeline-depth")
public class PipelineDepthController {

    private final PipelineDepthService depthService;

    public PipelineDepthController(PipelineDepthService depthService) {
        this.depthService = depthService;
    }

    @PostMapping("/session/start")
    public ResponseEntity<Map<String, Object>> start(@RequestBody(required = false) SessionRequest request) {
        String sessionId = request == null ? null : request.sessionId();
        String reason = request == null ? null : request.reason();
        return response(depthService.startSession(sessionId, reason));
    }

    @PostMapping("/pose")
    public ResponseEntity<Map<String, Object>> pose(@RequestBody PoseRequest request) {
        if (request == null || request.xM() == null || request.yM() == null) {
            return ResponseEntity.badRequest().body(Map.of("ok", false, "error", "xM and yM are required"));
        }
        return response(depthService.addPose(request.xM(), request.yM()));
    }

    @PostMapping("/cable-point")
    public ResponseEntity<Map<String, Object>> cablePoint(@RequestBody CablePointRequest request) {
        if (request == null || request.xM() == null || request.yM() == null) {
            return ResponseEntity.badRequest().body(Map.of("ok", false, "error", "xM and yM are required"));
        }
        return response(depthService.addCablePoint(request.pointId(), request.xM(), request.yM()));
    }

    @PostMapping("/cable-path")
    public ResponseEntity<Map<String, Object>> cablePath(@RequestBody List<CablePoint> points) {
        return response(depthService.replaceCablePath(points));
    }

    @PostMapping("/session/finalize")
    public ResponseEntity<Map<String, Object>> finalizeSession() {
        return response(depthService.finalizeSession());
    }

    @GetMapping("/map")
    public ResponseEntity<Map<String, Object>> map() {
        return response(depthService.currentMap());
    }

    @GetMapping("/health")
    public ResponseEntity<Map<String, Object>> health() {
        return response(depthService.health());
    }

    private ResponseEntity<Map<String, Object>> response(Map<String, Object> result) {
        HttpStatus status = Boolean.TRUE.equals(result.get("ok")) ? HttpStatus.OK : HttpStatus.SERVICE_UNAVAILABLE;
        return ResponseEntity.status(status).body(result);
    }

    public record SessionRequest(String sessionId, String reason) {}

    public record PoseRequest(Double xM, Double yM) {}

    public record CablePointRequest(String pointId, Double xM, Double yM) {}
}
