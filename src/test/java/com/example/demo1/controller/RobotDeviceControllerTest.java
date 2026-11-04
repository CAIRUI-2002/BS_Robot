package com.example.demo1.controller;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;

import com.example.demo1.controller.RobotDeviceController.LightControlRequest;
import com.example.demo1.service.VirtualGamepadService;

@ExtendWith(MockitoExtension.class)
class RobotDeviceControllerTest {

    @Mock
    private VirtualGamepadService gamepadService;

    private RobotDeviceController controller;

    @BeforeEach
    void setUp() {
        controller = new RobotDeviceController(gamepadService);
    }

    @Test
    void sendsBothLightTargetsAndWaitsForDeviceConfirmation() {
        when(gamepadService.setLights(1, 0)).thenReturn(true);

        ResponseEntity<?> response = controller.setLights(new LightControlRequest(1, 0));

        assertEquals(HttpStatus.OK, response.getStatusCode());
        verify(gamepadService).setLights(1, 0);
    }

    @Test
    void rejectsMissingOrInvalidLightTargets() {
        ResponseEntity<?> response = controller.setLights(new LightControlRequest(1, 2));

        assertEquals(HttpStatus.BAD_REQUEST, response.getStatusCode());
        verify(gamepadService, never()).setLights(1, 2);
    }

    @Test
    void reportsUdpSendFailure() {
        when(gamepadService.setLights(0, 1)).thenReturn(false);

        ResponseEntity<?> response = controller.setLights(new LightControlRequest(0, 1));

        assertEquals(HttpStatus.BAD_GATEWAY, response.getStatusCode());
    }
}
