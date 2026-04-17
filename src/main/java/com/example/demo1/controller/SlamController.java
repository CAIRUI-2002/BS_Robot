package com.example.demo1.controller;

import com.example.demo1.model.Point3D;
import com.example.demo1.service.SlamCacheService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpHeaders;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import java.nio.charset.StandardCharsets;
import java.util.List;

@RestController
@RequestMapping("/api/slam")
public class SlamController {

    @Autowired
    private SlamCacheService slamCacheService;

    /**
     * 保存点云数据为 PCD 格式并下载
     */
    @PostMapping("/save_pcd")
    public ResponseEntity<byte[]> savePcdFile() {
        // 1. 获取当前缓存的所有有效点
        List<Point3D> points = slamCacheService.getAllPoints();

        // 2. 拼接 PCD Header (使用标准规范，RGB 以 Unsigned Int 形式存储)
        StringBuilder pcdContent = new StringBuilder();
        pcdContent.append("# .PCD v0.7 - Point Cloud Data file format\n");
        pcdContent.append("VERSION 0.7\n");
        pcdContent.append("FIELDS x y z rgb\n");
        pcdContent.append("SIZE 4 4 4 4\n");
        pcdContent.append("TYPE F F F U\n"); // U 代表 Unsigned 整型，适配 PCL 读取 RGB
        pcdContent.append("COUNT 1 1 1 1\n");
        pcdContent.append("WIDTH ").append(points.size()).append("\n");
        pcdContent.append("HEIGHT 1\n");
        pcdContent.append("VIEWPOINT 0 0 0 1 0 0 0\n");
        pcdContent.append("POINTS ").append(points.size()).append("\n");
        pcdContent.append("DATA ascii\n");

        // 3. 遍历拼接点云数据
        for (Point3D p : points) {
            // 将 0-255 的 R G B 打包成一个 32位 整数 (PCL 和 Three.js 标准读取方式)
            long rgb = ((long)p.getR() << 16) | ((long)p.getG() << 8) | (long)p.getB();
            pcdContent.append(String.format("%.5f %.5f %.5f %d\n", p.getX(), p.getY(), p.getZ(), rgb));
        }

        // 4. 将字符串转换为字节数组流返回
        byte[] fileBytes = pcdContent.toString().getBytes(StandardCharsets.UTF_8);

        HttpHeaders headers = new HttpHeaders();
        headers.setContentType(MediaType.APPLICATION_OCTET_STREAM);
        headers.setContentDispositionFormData("attachment", "slam_map.pcd");

        return new ResponseEntity<>(fileBytes, headers, HttpStatus.OK);
    }

    /**
     * 清除点云缓存接口 (可选：配合前端的 btnClearCloud 使用)
     */
    @GetMapping("/clear_cache")
    public ResponseEntity<String> clearCache() {
        slamCacheService.clearCache();
        return ResponseEntity.ok("success");
    }
}