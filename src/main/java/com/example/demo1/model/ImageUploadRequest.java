package com.example.demo1.model;

import lombok.Data;

@Data
public class ImageUploadRequest {
    private String faultId;
    private String direction;
    private String imageBase64;
}