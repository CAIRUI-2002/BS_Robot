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
    private Double rawDepthCm;
    private Double estimatedDepthM;
    private Double depthSigmaM;
    private String depthConfidence;
    private String depthSource;
    private Double onlineDepthM;
    private Double onlineDepthSigmaM;
    private Double finalDepthM;
    private Double finalDepthSigmaM;
    private String finalDepthConfidence;
    private String finalDepthSource;
    private String depthObservationChannel;
    private String depthSignalContext;
    private Double cableSM;
    private Long depthUpdatedAtMs;
}
