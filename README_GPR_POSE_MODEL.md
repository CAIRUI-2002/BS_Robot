# GPR 双曲线顶点模型集成

上位机当前使用 `src/main/resources/models/gpr-hyperbola-apex-yolo11n-pose.onnx`。
该 YOLO11 Pose 模型同时预测 `cable` 检测框和 `hyperbola_apex` 顶点，输出张量
为 `[1, 8, 8400]`。`GprYoloDetectionService` 完成置信度过滤、NMS、检测框和
顶点的 letterbox 逆映射，不再在运行链路中调用传统 `GprApexLocator`。

主要配置位于 `src/main/resources/application.properties`：

```properties
gpr.yolo.model=classpath:/models/gpr-hyperbola-apex-yolo11n-pose.onnx
gpr.yolo.input-size=640
gpr.yolo.confidence-threshold=0.50
gpr.yolo.keypoint-threshold=0.25
gpr.yolo.iou-threshold=0.45
```

模型训练与评估产物位于
`D:/Desktop/YOLO训练/runs_pose/gpr_hyperbola_apex_yolo11n_pose/`。模型元数据、
测试指标和 SHA-256 记录在同目录资源文件
`gpr-hyperbola-apex-yolo11n-pose.json` 中。
