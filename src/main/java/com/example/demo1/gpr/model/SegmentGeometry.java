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

    // 同名 CSV 的 RTK 用来拟合整张剖面的中心和方向；剖面内部各道仍按
    // dztTraceSpacingM 均匀排列，避免逐道RTK噪声将雷达波形拉弯或错位。
    // traceX / traceY 是刚体校准后的每道位置，数组下标与 B-scan 列一一对应。
    private boolean rtkPositioned;
    private String rtkLayoutMethod;
    private double dztTraceSpacingM;
    private String rtkCsvFileName;
    private int rtkSourceRowCount;
    private int rtkValidCoordinateCount;
    private double rtkOriginLatitudeDeg;
    private double rtkOriginLongitudeDeg;
    private double[] traceLatitudeDeg;
    private double[] traceLongitudeDeg;
    private double[] traceX;
    private double[] traceY;
    private int rtkFitInlierCount;
    private int rtkFitSampleCount;
    private double rtkFitRmseM;
}
