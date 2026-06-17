package com.example.demo1.controller;

import com.example.demo1.model.Point3D;
import com.example.demo1.service.SlamCacheService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.ContentDisposition;
import org.springframework.http.HttpHeaders;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import java.nio.charset.StandardCharsets;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.List;
import java.util.Locale;
import java.util.Map;

@RestController
@RequestMapping("/api/slam")
public class SlamController {

    @Autowired
    private SlamCacheService slamCacheService;

    /**
     * 查询后端 SLAM 缓存状态，现场排查“保存出来只有 1KB”时可以先看 pointCount 是否为 0。
     */
    @GetMapping("/status")
    public ResponseEntity<Map<String, Object>> status() {
        return ResponseEntity.ok(Map.of(
                "connected", slamCacheService.isConnected(),
                "pointCount", slamCacheService.getPointCount(),
                "messageCount", slamCacheService.getMessageCount()
        ));
    }

    /**
     * 保存点云数据为 PCD 格式并下载。
     * 注意：如果前端已经改成“所见即所得导出”，该接口只是保留为后端兜底。
     */
    @PostMapping("/save_pcd")
    public ResponseEntity<byte[]> savePcdFile() {
        List<Point3D> points = slamCacheService.getAllPoints();

        if (points == null || points.isEmpty()) {
            byte[] msg = "当前后端 SLAM 点云缓存为空，已拒绝生成空 PCD。请确认 Java 后端已连接 10.21.31.104:9090，并且正在收到 slam_data。".getBytes(StandardCharsets.UTF_8);
            return ResponseEntity.status(HttpStatus.CONFLICT)
                    .contentType(new MediaType("text", "plain", StandardCharsets.UTF_8))
                    .body(msg);
        }

        StringBuilder pcdContent = new StringBuilder(points.size() * 45 + 256);

        pcdContent.append("# .PCD v0.7 - Point Cloud Data file format\n");
        pcdContent.append("VERSION 0.7\n");
        pcdContent.append("FIELDS x y z rgb\n");
        pcdContent.append("SIZE 4 4 4 4\n");
        pcdContent.append("TYPE F F F U\n");
        pcdContent.append("COUNT 1 1 1 1\n");
        pcdContent.append("WIDTH ").append(points.size()).append("\n");
        pcdContent.append("HEIGHT 1\n");
        pcdContent.append("VIEWPOINT 0 0 0 1 0 0 0\n");
        pcdContent.append("POINTS ").append(points.size()).append("\n");
        pcdContent.append("DATA ascii\n");

        for (Point3D p : points) {
            if (p == null
                    || !Double.isFinite(p.getX())
                    || !Double.isFinite(p.getY())
                    || !Double.isFinite(p.getZ())) {
                continue;
            }

            int r = clampColor(p.getR());
            int g = clampColor(p.getG());
            int b = clampColor(p.getB());
            long rgb = ((long) r << 16) | ((long) g << 8) | (long) b;

            // Locale.US 保证小数点始终为 "."，避免某些系统区域设置导致 PCD 解析失败。
            pcdContent.append(String.format(Locale.US, "%.5f %.5f %.5f %d%n",
                    p.getX(), p.getY(), p.getZ(), rgb));
        }

        byte[] fileBytes = pcdContent.toString().getBytes(StandardCharsets.UTF_8);

        String fileName = "slam_map_" +
                LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss")) +
                ".pcd";

        HttpHeaders headers = new HttpHeaders();
        headers.setContentType(MediaType.APPLICATION_OCTET_STREAM);
        headers.setContentDisposition(ContentDisposition.attachment().filename(fileName, StandardCharsets.UTF_8).build());
        headers.setContentLength(fileBytes.length);

        return new ResponseEntity<>(fileBytes, headers, HttpStatus.OK);
    }

    /**
     * 清除点云缓存接口。
     */
    @GetMapping("/clear_cache")
    public ResponseEntity<String> clearCache() {
        slamCacheService.clearCache();
        return ResponseEntity.ok("success");
    }

    private int clampColor(int v) {
        return Math.max(0, Math.min(255, v));
    }
}
