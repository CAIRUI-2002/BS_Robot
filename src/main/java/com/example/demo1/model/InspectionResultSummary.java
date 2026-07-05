package com.example.demo1.model;

import lombok.Data;

@Data
public class InspectionResultSummary {
    private Long id;
    private String name;
    private String savedAt;
    private int pathPointCount;
    private Double pathLengthMeters;
    private int faultCount;
}
