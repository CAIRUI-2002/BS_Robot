package com.example.demo1.controller;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.TimeUnit;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequestMapping("/api/pipeline-key")
public class PipelineKeyController {

    private static final Set<String> ALLOWED_KEYS = Set.of("POWER", "INFO", "FREQ", "MODE");

    @Value("${pipeline.key-control.python:python3}")
    private String pythonCommand;

    @Value("${pipeline.key-control.script:linux_key_control.py}")
    private String scriptPath;

    @Value("${pipeline.key-control.timeout-ms:8000}")
    private long timeoutMs;

    @PostMapping("/key")
    public ResponseEntity<Map<String, Object>> pressKey(@RequestParam String name) {
        String keyName = name == null ? "" : name.trim().toUpperCase(Locale.ROOT);
        if (!ALLOWED_KEYS.contains(keyName)) {
            return ResponseEntity.badRequest().body(Map.of(
                    "success", false,
                    "message", "不支持的管线仪按键: " + name
            ));
        }

        Process process = null;
        try {
            ProcessBuilder builder = new ProcessBuilder(List.of(pythonCommand, scriptPath, "key", keyName));
            builder.redirectErrorStream(true);

            process = builder.start();
            boolean finished = process.waitFor(timeoutMs, TimeUnit.MILLISECONDS);
            if (!finished) {
                process.destroyForcibly();
                return ResponseEntity.status(HttpStatus.GATEWAY_TIMEOUT).body(Map.of(
                        "success", false,
                        "key", keyName,
                        "message", "管线仪按键指令执行超时"
                ));
            }

            String output = new String(process.getInputStream().readAllBytes(), StandardCharsets.UTF_8).trim();
            int exitCode = process.exitValue();
            if (exitCode == 0) {
                return ResponseEntity.ok(Map.of(
                        "success", true,
                        "key", keyName,
                        "message", "管线仪按键指令已发送",
                        "output", output
                ));
            }

            return ResponseEntity.status(HttpStatus.BAD_GATEWAY).body(Map.of(
                    "success", false,
                    "key", keyName,
                    "message", "管线仪按键指令执行失败，退出码: " + exitCode,
                    "output", output
            ));
        } catch (IOException e) {
            return ResponseEntity.status(HttpStatus.BAD_GATEWAY).body(Map.of(
                    "success", false,
                    "key", keyName,
                    "message", "无法启动管线仪控制脚本: " + e.getMessage()
            ));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return ResponseEntity.status(HttpStatus.INTERNAL_SERVER_ERROR).body(Map.of(
                    "success", false,
                    "key", keyName,
                    "message", "管线仪按键指令被中断"
            ));
        } finally {
            if (process != null && process.isAlive()) {
                process.destroyForcibly();
            }
        }
    }
}
