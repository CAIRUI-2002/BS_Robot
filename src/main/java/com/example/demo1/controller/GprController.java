package com.example.demo1.controller;

import java.io.File;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.core.io.FileSystemResource;
import org.springframework.core.io.Resource;
import org.springframework.http.HttpHeaders;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

import com.example.demo1.gpr.model.GprConfig;
import com.example.demo1.gpr.model.GprProcessResult;
import com.example.demo1.gpr.service.GprProcessingService;

/**
 * 对前端暴露的接口：
 *   POST /api/gpr/process?folderPath=...
 * 返回处理后的所有数据（JSON）
 */
@RestController
@RequestMapping("/api/gpr")
public class GprController {

    private static final long PROGRESS_RETENTION_MILLIS = 10 * 60 * 1000L;

    private final ConcurrentMap<String, ProgressEntry> progressByTaskId = new ConcurrentHashMap<>();

    @Autowired
    private GprProcessingService gprProcessingService;

    /**
     * 处理服务器本地某个目录下的全部 .dzt 文件
     *
     * 用法示例：
     *   POST http://localhost:8080/api/gpr/process?folderPath=D:/data/dzt
     *
     * body（可选 JSON）：
     *   {
     *     "migrationEnabled": false,
     *     "ns": 512,
     *     "dtNs": 0.1,
     *     "dxM": 0.02,
     *     "vSearchMin": 0.04,
     *     "vSearchMax": 0.09
     *   }
     */
    @PostMapping("/process")
    public GprProcessResult processFolder(
            @RequestParam("folderPath") String folderPath,
            @RequestParam(value = "taskId", required = false) String taskId,
            @RequestBody(required = false) GprConfig config
    ) throws Exception {
        if (config == null) {
            config = new GprConfig(); // 使用默认参数
        }

        String normalizedTaskId = normalizeTaskId(taskId);
        if (normalizedTaskId == null) {
            return gprProcessingService.processFolder(folderPath, config);
        }

        cleanupExpiredProgress();
        ProgressEntry entry = new ProgressEntry();
        progressByTaskId.put(normalizedTaskId, entry);

        try {
            GprProcessResult result = gprProcessingService.processFolder(folderPath, config, entry::update);
            entry.update(100);
            return result;
        } catch (Exception ex) {
            progressByTaskId.remove(normalizedTaskId, entry);
            throw ex;
        }
    }

    /**
     * 查询指定 GPR 处理任务的百分比进度。
     */
    @GetMapping("/progress")
    public ResponseEntity<Map<String, Integer>> getProgress(@RequestParam("taskId") String taskId) {
        cleanupExpiredProgress();
        ProgressEntry entry = progressByTaskId.get(taskId);
        if (entry == null) {
            return ResponseEntity.notFound().build();
        }
        return ResponseEntity.ok(Map.of("progress", entry.getProgress()));
    }

    private String normalizeTaskId(String taskId) {
        if (taskId == null || taskId.isBlank()) {
            return null;
        }
        String normalized = taskId.trim();
        if (normalized.length() > 128) {
            throw new IllegalArgumentException("taskId 长度不能超过 128 个字符");
        }
        return normalized;
    }

    private void cleanupExpiredProgress() {
        long cutoff = System.currentTimeMillis() - PROGRESS_RETENTION_MILLIS;
        progressByTaskId.entrySet().removeIf(item -> item.getValue().getUpdatedAt() < cutoff);
    }

    private static final class ProgressEntry {
        private volatile int progress;
        private volatile long updatedAt = System.currentTimeMillis();

        private void update(int value) {
            progress = Math.max(progress, Math.max(0, Math.min(100, value)));
            updatedAt = System.currentTimeMillis();
        }

        private int getProgress() {
            return progress;
        }

        private long getUpdatedAt() {
            return updatedAt;
        }
    }

    /**
     * 读取绝对路径的点云文件并返回流给 Three.js PCDLoader
     * 用法示例： GET /api/gpr/pcd?filePath=/mnt/e/data/map.pcd
     */
    @GetMapping("/pcd")
    public ResponseEntity<Resource> getPcdFile(@RequestParam("filePath") String filePath) {
        File file = new File(filePath);
        if (!file.exists() || !file.isFile()) {
            return ResponseEntity.notFound().build();
        }
        
        Resource resource = new FileSystemResource(file);
        return ResponseEntity.ok()
                // 告诉浏览器这是一个附件流
                .header(HttpHeaders.CONTENT_DISPOSITION, "attachment; filename=\"" + file.getName() + "\"")
                .contentType(MediaType.APPLICATION_OCTET_STREAM)
                .body(resource);
    }
}
