package com.example.demo1.model;

import lombok.Data;

import java.util.List;

@Data
public class InspectionResultDetail {
    private Long id;
    private String name;
    private String savedAt;
    private List<CablePoint> cablePath;
    private List<FaultPoint> faults;
}