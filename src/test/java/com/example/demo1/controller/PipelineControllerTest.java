package com.example.demo1.controller;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.verify;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import com.example.demo1.service.VirtualGamepadService;

@ExtendWith(MockitoExtension.class)
class PipelineControllerTest {

    @Mock
    private VirtualGamepadService gamepadService;

    @InjectMocks
    private PipelineController controller;

    @Test
    void rawDepthCmAliasTakesPriority() {
        String response = controller.updateSignal(0, 98.0f, 101.0f, 90.0f);

        verify(gamepadService).updatePipelineSignal(0, 101.0f, 90.0f);
        assertTrue(response.contains("depthCm=101.0"));
    }

    @Test
    void legacyDepthParameterRemainsCompatible() {
        String response = controller.updateSignal(1, 98.0f, null, null);

        verify(gamepadService).updatePipelineSignal(1, 98.0f, null);
        assertTrue(response.contains("depthCm=98.0"));
    }
}
