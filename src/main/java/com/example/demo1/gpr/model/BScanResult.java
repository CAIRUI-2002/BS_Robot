package com.example.demo1.gpr.model;

import lombok.Data;

/**
 * 每个 .dzt B 扫对应的结果：
 * - 预处理后的原始数据（rawData）
 * - 在最优速度下的偏移结果（migratedData）
 * - 对应几何信息以及最优速度、熵值
 */
@Data
public class BScanResult {

    private String fileName;

    private int nz;          // 垂直采样点数
    private int nu;          // 道数

    // 注意：这是二维数组，前端拿到就是 [[...],[...],...]
    private double[][] rawData;
    private double[][] migratedData;

    private double vBest;    // 该段最优速度 (m/ns)
    private double hBest;    // 最小熵

    private SegmentGeometry geometry;
}
