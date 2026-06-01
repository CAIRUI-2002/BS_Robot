package com.example.demo1.model;

import lombok.Data;

@Data
public class FaultPoint {
    private String faultId;
    private Double x;
    private Double y;
    private Double z;
    private FaultImages images;
}