package com.example.demo1.model;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;

import lombok.Data;

@Data
@JsonIgnoreProperties(ignoreUnknown = true)
public class CablePoint {
    private Double x;
    private Double y;
    private Double z;
    private Double lat;
    private Double lon;
    private Double depth;
}
