package com.example.demo1.service;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

import java.util.Map;

import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;
import org.springframework.messaging.simp.SimpMessagingTemplate;
import org.springframework.test.util.ReflectionTestUtils;

class VirtualGamepadServicePipelineDepthTest {

    @Test
    void websocketPayloadContainsRawDepthUnitAndAllTimeLayers() {
        VirtualGamepadService service = new VirtualGamepadService();
        SimpMessagingTemplate template = mock(SimpMessagingTemplate.class);
        ReflectionTestUtils.setField(service, "template", template);

        long before = System.currentTimeMillis();
        service.updatePipelineSignal(0, 98.0f, 90.0f, 1000L, 1050L);
        long after = System.currentTimeMillis();

        ArgumentCaptor<Object> payloadCaptor = ArgumentCaptor.forClass(Object.class);
        verify(template).convertAndSend(eq("/topic/pipeline"), payloadCaptor.capture());

        @SuppressWarnings("unchecked")
        Map<String, Object> payload = (Map<String, Object>) payloadCaptor.getValue();
        assertEquals(0, payload.get("signal"));
        assertEquals(98.0f, payload.get("depth"));
        assertEquals(98.0f, payload.get("rawDepthCm"));
        assertEquals("cm", payload.get("depthUnit"));
        assertEquals(1000L, payload.get("captureTimeMs"));
        assertEquals(1050L, payload.get("backendReceiveTimeMs"));
        assertEquals(90.0f, payload.get("angle"));

        long serverReceiveTimeMs = ((Number) payload.get("serverReceiveTimeMs")).longValue();
        assertTrue(serverReceiveTimeMs >= before && serverReceiveTimeMs <= after);
    }
}
