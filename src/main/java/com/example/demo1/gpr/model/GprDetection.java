package com.example.demo1.gpr.model;

import lombok.Data;

/**
 * YOLO 在偏移前原始 B-Scan 上识别出的管道双曲线目标。
 * 坐标使用 rawData 的原始矩阵坐标：x 对应道号，y 对应采样行。
 */
@Data
public class GprDetection {

    private int classId;
    private String label;
    private double confidence;

    private double x1;
    private double y1;
    private double x2;
    private double y2;

    // 原始矩阵坐标中的波形顶点；定位不可靠时为 null。
    private Double apexCol;
    private Double apexRow;
    // 模板匹配强度的 0～1 归一化值，不是统计校准过的概率。
    private Double apexScore;
    // bilateral 或 singleWing，供界面区分单侧推定的顶点。
    private String apexMethod;

    public GprDetection(int classId, String label, double confidence,
                        double x1, double y1, double x2, double y2) {
        this.classId = classId;
        this.label = label;
        this.confidence = confidence;
        this.x1 = x1;
        this.y1 = y1;
        this.x2 = x2;
        this.y2 = y2;
    }
}
