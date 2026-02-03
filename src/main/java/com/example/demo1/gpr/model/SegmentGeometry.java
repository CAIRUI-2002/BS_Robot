package com.example.demo1.gpr.model;

import lombok.Data;

/**
 * 对应 MATLAB 里的 segPhi, segLen, segStartXY 等，
 * 方便前端计算每段的 X/Y/Z 坐标。
 */
@Data
public class SegmentGeometry {

    private double phiRad;   // 当前段方位角（弧度）
    private double length;   // 段长度 (m)
    private double startX;   // 段起点 X
    private double startY;   // 段起点 Y

    // 一维坐标，用来生成 3D 网格
    private double[] uAxis;  // 0:dx:(nu-1)*dx
    private double[] zAxis;  // 0.5*v_depth*t
}
