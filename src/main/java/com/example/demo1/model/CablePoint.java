package com.example.demo1.model;

import lombok.Data;

@Data
public class CablePoint {
    private Double x;
    private Double y;
    private Double z;
    private Double lat;
    private Double lon;
    private Double depth;
    private Double depthM;
    private Double depthUncertaintyM;
    private String depthConfidence;
    private String depthSource;
    private Double sM;
    private Integer depthMeasurementCount;
    private Boolean depthFinalized;
}
