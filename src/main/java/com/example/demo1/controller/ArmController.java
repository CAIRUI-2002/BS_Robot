package com.example.demo1.controller;

import java.util.Locale;

import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;
import org.springframework.web.client.RestTemplate;

@RestController
@RequestMapping("/arm")
public class ArmController {

    
    // 务必确保机器狗能 Ping 通这个 IP
    private static final String ARM_IP = "10.21.31.104";
    private static final String ARM_PORT = "5000";

    // Spring 自带的 HTTP 请求工具
    private final RestTemplate restTemplate = new RestTemplate();

    /**
     * 1. 绝对移动接口 (保留用于一键测试和归零)
     * URL: /arm/move?j1=0&...&speed=50
     */
    @GetMapping("/move")
    public String moveRobot(
            @RequestParam(defaultValue = "0") double j1,
            @RequestParam(defaultValue = "0") double j2,
            @RequestParam(defaultValue = "0") double j3,
            @RequestParam(defaultValue = "0") double j4,
            @RequestParam(defaultValue = "0") double j5,
            @RequestParam(defaultValue = "0") double j6,
            @RequestParam(defaultValue = "10") int speed
    ) {
        try {
            String url = String.format(
                    "http://%s:%s/move?j1=%f&j2=%f&j3=%f&j4=%f&j5=%f&j6=%f&speed=%d",
                    ARM_IP, ARM_PORT, j1, j2, j3, j4, j5, j6, speed
            );

            System.out.println("📡 [Move] 发送远程指令: " + url);
            String response = restTemplate.getForObject(url, String.class);
            return "机械臂响应: " + response;

        } catch (Exception e) {
            e.printStackTrace();
            return "通信失败 (请检查树莓派IP和网络): " + e.getMessage();
        }
    }

    /**
     * 2. ✅ 点动开始接口 (按住按钮)
     * URL: /arm/jog?index=0&direction=1
     * 前端 index 传 0-5，direction 传 1(加) 或 -1(减)
     */
    @GetMapping("/jog")
    public String jogRobot(
            @RequestParam int index,
            @RequestParam int direction
    ) {
        try {
            // 参数转换适配 pymycobot 协议
            // 1. 关节号：前端(0-5) -> Python(1-6)
            int jointId = index; // 适配 Piper Python 脚本从 0 开始的数组索引

            // 2. 方向：前端(1/-1) -> Python(1=正向, 0=反向)
            int dir = (direction > 0) ? 1 : 0;

            // 默认点动速度设为 50，可根据需要调整
            int speed = 10;

            String url = String.format(
                    "http://%s:%s/jog?index=%d&direction=%d&speed=%d",
                    ARM_IP, ARM_PORT, jointId, dir, speed
            );

            System.out.println("🚀 [Jog] Start: " + url);
            restTemplate.getForObject(url, String.class);
            return "Jog Started";

        } catch (Exception e) {
            System.err.println("Jog Error: " + e.getMessage());
            return "Jog Error: " + e.getMessage();
        }
    }

    /**
     * 3. ✅ 点动停止接口 (松开按钮)
     * URL: /arm/stop
     */
    @GetMapping("/stop")
    public String stopRobot() {
        try {
            String url = String.format("http://%s:%s/stop", ARM_IP, ARM_PORT);

            System.out.println("🛑 [Jog] Stop");
            restTemplate.getForObject(url, String.class);
            return "Jog Stopped";

        } catch (Exception e) {
            System.err.println("Stop Error: " + e.getMessage());
            return "Stop Error: " + e.getMessage();
        }
    }
    /**
     * ✅ 新增：末端位姿控制 (XYZ + RX RY RZ)
     * URL: /arm/move_end?x=150&y=0&z=150&rx=0&ry=90&rz=0
     */
    @GetMapping("/move_end")
    public String moveEndPose(
            @RequestParam(defaultValue = "0") double x,
            @RequestParam(defaultValue = "0") double y,
            @RequestParam(defaultValue = "0") double z,
            @RequestParam(defaultValue = "0") double rx,
            @RequestParam(defaultValue = "0") double ry,
            @RequestParam(defaultValue = "0") double rz
    ) {
        try {
            String url = String.format(Locale.US,
                    "http://%s:%s/move_end?x=%f&y=%f&z=%f&rx=%f&ry=%f&rz=%f",
                    ARM_IP, ARM_PORT, x, y, z, rx, ry, rz
            );
            System.out.println("🚀 [Move End Pose]: " + url);
            restTemplate.getForObject(url, String.class);
            return "End Pose Command Sent";
        } catch (Exception e) {
            return "Move End Error: " + e.getMessage();
        }
    }
    /**
     * ✅ 新增：末端点动开始接口 (连续平滑移动)
     * URL: /arm/jog_end?axis=0&direction=1
     * axis: 0=X, 1=Y, 2=Z, 3=RX, 4=RY, 5=RZ
     */
    @GetMapping("/jog_end")
    public String jogEndRobot(
            @RequestParam int axis,
            @RequestParam int direction
    ) {
        try {
            int dir = (direction > 0) ? 1 : -1;
            String url = String.format("http://%s:%s/jog_end?axis=%d&direction=%d", ARM_IP, ARM_PORT, axis, dir);
            restTemplate.getForObject(url, String.class);
            return "Cartesian Jog Started";
        } catch (Exception e) {
            return "Jog End Error: " + e.getMessage();
        }
    }

    /**
     * 4. ✅ 夹爪控制相关接口
     */
    @GetMapping("/gripper")
    public String setGripper(@RequestParam double value) {
        try {
            String url = String.format(Locale.US, "http://%s:%s/gripper?value=%f", ARM_IP, ARM_PORT, value);
            restTemplate.getForObject(url, String.class);
            return "Gripper Set";
        } catch (Exception e) {
            return "Gripper Error: " + e.getMessage();
        }
    }

    @GetMapping("/gripper_jog")
    public String jogGripper(@RequestParam int dir) {
        try {
            String url = String.format("http://%s:%s/gripper_jog?dir=%d", ARM_IP, ARM_PORT, dir);
            restTemplate.getForObject(url, String.class);
            return "Gripper Jog Start";
        } catch (Exception e) {
            return "Gripper Jog Error";
        }
    }

    @GetMapping("/gripper_stop")
    public String stopGripper() {
        try {
            String url = String.format("http://%s:%s/gripper_stop", ARM_IP, ARM_PORT);
            restTemplate.getForObject(url, String.class);
            return "Gripper Stopped";
        } catch (Exception e) {
            return "Gripper Stop Error";
        }
    }
}