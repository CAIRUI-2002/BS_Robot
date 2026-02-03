package com.example.demo1.config;

import org.springframework.context.annotation.Configuration;
import org.springframework.messaging.simp.config.MessageBrokerRegistry;
import org.springframework.web.socket.config.annotation.EnableWebSocketMessageBroker;
import org.springframework.web.socket.config.annotation.StompEndpointRegistry;
import org.springframework.web.socket.config.annotation.WebSocketMessageBrokerConfigurer;

@Configuration
@EnableWebSocketMessageBroker
public class WebSocketConfig implements WebSocketMessageBrokerConfigurer {

    @Override
    public void configureMessageBroker(MessageBrokerRegistry config) {
        // 开启代理，前端订阅通常以 /topic 开头
        config.enableSimpleBroker("/topic");
        // 全局应用前缀，这对应了前端代码里的 "/app/move"
        config.setApplicationDestinationPrefixes("/app");
    }

    @Override
    public void registerStompEndpoints(StompEndpointRegistry registry) {
        // 注册端点，前端 home.js 里 new SockJS('/ws') 连的就是这
        // setAllowedOriginPatterns("*") 是为了防止跨域拦截
        registry.addEndpoint("/ws").setAllowedOriginPatterns("*").withSockJS();
    }
}