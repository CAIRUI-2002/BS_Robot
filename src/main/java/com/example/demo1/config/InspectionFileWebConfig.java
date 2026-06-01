package com.example.demo1.config;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Configuration;
import org.springframework.web.servlet.config.annotation.ResourceHandlerRegistry;
import org.springframework.web.servlet.config.annotation.WebMvcConfigurer;

import java.nio.file.Paths;

@Configuration
public class InspectionFileWebConfig implements WebMvcConfigurer {

    @Value("${inspection.storage-dir}")
    private String storageDir;

    @Value("${inspection.public-prefix}")
    private String publicPrefix;

    @Override
    public void addResourceHandlers(ResourceHandlerRegistry registry) {
        String location = Paths.get(storageDir).toUri().toString();

        registry.addResourceHandler(publicPrefix + "/**")
                .addResourceLocations(location);
    }
}