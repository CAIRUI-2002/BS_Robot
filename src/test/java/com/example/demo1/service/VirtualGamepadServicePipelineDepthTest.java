package com.example.demo1.service;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import java.util.Map;

import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;
import org.springframework.messaging.simp.SimpMessagingTemplate;
import org.springframework.test.util.ReflectionTestUtils;

class VirtualGamepadServicePipelineDepthTest {

    @Test
    void websocketPayloadContainsValidCentimeterDepthOnly() {
        VirtualGamepadService service = new VirtualGamepadService();
        SimpMessagingTemplate template = mock(SimpMessagingTemplate.class);
        ReflectionTestUtils.setField(service, "template", template);

        service.updatePipelineSignal(0, 98.0f, 90.0f);

        ArgumentCaptor<Object> payloadCaptor = ArgumentCaptor.forClass(Object.class);
        verify(template).convertAndSend(eq("/topic/pipeline"), payloadCaptor.capture());

        @SuppressWarnings("unchecked")
        Map<String, Object> payload = (Map<String, Object>) payloadCaptor.getValue();
        assertEquals(0, payload.get("signal"));
        assertEquals(98.0f, payload.get("depth"));
        assertEquals("cm", payload.get("depthUnit"));
        assertEquals(90.0f, payload.get("angle"));
        assertEquals(false, payload.containsKey("rawDepthCm"));
        assertEquals(false, payload.containsKey("captureTimeMs"));
        assertEquals(false, payload.containsKey("backendReceiveTimeMs"));
    }

    @Test
    void outOfRangeDepthIsDiscardedWithoutDiscardingSignal() {
        VirtualGamepadService service = new VirtualGamepadService();
        SimpMessagingTemplate template = mock(SimpMessagingTemplate.class);
        ReflectionTestUtils.setField(service, "template", template);

        service.updatePipelineSignal(2, 9.0f, null);
        service.updatePipelineSignal(2, 501.0f, null);

        ArgumentCaptor<Object> payloadCaptor = ArgumentCaptor.forClass(Object.class);
        verify(template, times(2)).convertAndSend(eq("/topic/pipeline"), payloadCaptor.capture());
        for (Object captured : payloadCaptor.getAllValues()) {
            @SuppressWarnings("unchecked")
            Map<String, Object> payload = (Map<String, Object>) captured;
            assertEquals(2, payload.get("signal"));
            assertEquals(null, payload.get("depth"));
        }
    }
}
