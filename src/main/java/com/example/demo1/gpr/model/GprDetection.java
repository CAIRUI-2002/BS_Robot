package com.example.demo1.gpr.model;

import lombok.AllArgsConstructor;
import lombok.Data;

/**
 * YOLO 在偏移前原始 B-Scan 上识别出的管道双曲线目标。
 * 坐标使用 rawData 的原始矩阵坐标：x 对应道号，y 对应采样行。
 */
@Data
@AllArgsConstructor
public class GprDetection {

    private int classId;
    private String label;
    private double confidence;

    private double x1;
    private double y1;
    private double x2;
    private double y2;
}
