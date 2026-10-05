// home.js
// 功能：
// 1. GPR 探地雷达数据处理与三维可视化
// 2. ROS 机器人运动控制 (WebSocket + Stomp + 长按连续发送)
// 3. 多面板切换 (GPR / Robot)

(function () {
    // ==========================================
    // 0. 全局配置与状态
    // ==========================================

    // ✅ 资深写法（推荐）：自动跟随当前网页的 IP 地址
    // 这样无论你是用 localhost、127.0.0.1 还是 172.x.x.x 访问，
    // 它都会自动去连同一个 IP 的 8000 端口。
    const ARM_BACKEND_URL = "http://" + window.location.hostname + ":8080";
    const ARM_API_BASE = "http://10.21.31.104:5005";
    // 机械臂末端摄像头 Flask 服务端口，跟随当前访问上位机的机器狗 IP。
    const ARM_CAMERA_BASE = "http://" + (window.location.hostname || "10.21.31.104") + ":5001";
    const ROBOT_CAMERA_WEBRTC_BASE = "http://10.21.31.103:8889";
    const ROBOT_CAMERA_WHEP_URLS = {
        front: ROBOT_CAMERA_WEBRTC_BASE + "/video1/whep",
        back: ROBOT_CAMERA_WEBRTC_BASE + "/video2/whep"
    };
    const ROBOT_CAMERA_CALIBRATION_URL = "/config/robot-camera-calibration.json";
    const ROBOT_CAMERA_DIRECTIONS = ["front", "back"];
    const robotCameraReaders = {
        front: null,
        back: null
    };
    const robotCameraStreams = {
        front: null,
        back: null
    };
    const robotCameraSourceVideos = {
        front: null,
        back: null
    };
    const robotCameraCanvases = {
        front: null,
        back: null
    };
    const robotCameraRenderers = {
        front: null,
        back: null
    };
    const robotCameraRenderLoopTokens = {
        front: 0,
        back: 0
    };
    const robotCameraFrameRequests = {
        front: null,
        back: null
    };
    const robotCameraLastResizeChecks = {
        front: 0,
        back: 0
    };
    const robotCameraLabelStates = {
        front: "",
        back: ""
    };
    let robotCameraCalibrationConfig = null;
    let robotCameraCorrectionMode = "inspection";

    // ====== 🌍 GPS 与 SLAM 坐标映射全局变量 ======
    window.gpsAnchor = null;        // 起点真实的 GPS 坐标 {lat, lon}
    window.slamAnchor = null;       // 起点对应的 SLAM 坐标 {x, y}
    window.headingOffset = 0;       // 两个坐标系的角度差 (弧度)
    window.isGpsCalibrated = false; // 是否已完成标定
    window.currentGps = null;       // 机器狗当前的最新 GPS
    window.geoTransform = null;     // ✅ 多点轨迹标定结果：SLAM -> GPS局部米制坐标的二维相似变换
    window.geoCalibSamples = [];    // ✅ 多点轨迹标定采样缓存 [{x,y,lat,lon,status,t}]

    // ====== 🌊 自适应检测全局变量 ======
    const HISTORY_SIZE = 50;  // 记录最近 50 帧的数据作为背景参考
    let signalBaselineQueue = []; // 存储历史信号强度
    // ====== 🌊 [新] AUC 算法全局配置 ======
    // 对应 Python 代码中的核心参数
    const AUC_CONFIG = {
        SAMPLING_RATE: 10000,   // 采样率
        PULSE_THRESHOLD: 1200,   // 粗筛：主脉冲识别阈值 (绝对值 > 500)
        DISTANCE_MIN: 1000,     // 粗筛：最小脉冲间隔 (防止重复触发)

        // --- 核心判定参数 ---
        NOISE_THRESHOLD: 3.5,   // Z-score 阈值 (由滑条控制)
        BURST_THRESHOLD: 1,     // 异常点数量阈值
        AUC_THRESHOLD: 2500.0,  // 能量积分阈值 (幅度 * 数量)

        // --- 窗口设置 ---
        WINDOW_BEFORE: 2000,    // 峰值前取点
        WINDOW_AFTER: 5000      // 峰值后取点 (等待数据攒够)
    };

    // ==========================================
    // ✅ 新增：AUC 算法参数微调控制
    // ==========================================
    window.adjustAucParam = function (type, delta) {
        // 1. 获取显示元素
        const elPulse = document.getElementById('disp_pulse_threshold');
        const elAuc = document.getElementById('disp_auc_threshold');
        const elZ = document.getElementById('sensitivityDisplay'); // ✅ 新增：获取 Z-score 显示元素

        if (type === 'PULSE') {
            // --- 调整 Pulse Threshold ---
            AUC_CONFIG.PULSE_THRESHOLD += delta;

            // 限制最小值为 0
            if (AUC_CONFIG.PULSE_THRESHOLD < 0) AUC_CONFIG.PULSE_THRESHOLD = 0;

            // 更新 UI
            if (elPulse) elPulse.textContent = AUC_CONFIG.PULSE_THRESHOLD;
            console.log(`🎛️ [参数调整] 脉冲阈值 -> ${AUC_CONFIG.PULSE_THRESHOLD}`);

        } else if (type === 'AUC') {
            // --- 调整 AUC Threshold ---
            AUC_CONFIG.AUC_THRESHOLD += delta;

            // 限制最小值为 0
            if (AUC_CONFIG.AUC_THRESHOLD < 0) AUC_CONFIG.AUC_THRESHOLD = 0;

            // 更新 UI
            if (elAuc) elAuc.textContent = AUC_CONFIG.AUC_THRESHOLD.toFixed(0);
            console.log(`🎛️ [参数调整] 能量积分阈值 -> ${AUC_CONFIG.AUC_THRESHOLD}`);

        } else if (type === 'ZSCORE') {
            // ✅✅✅ 补全了这部分逻辑 ✅✅✅

            // --- 调整 Z-score Threshold ---
            AUC_CONFIG.NOISE_THRESHOLD += delta;

            // 限制最小值 (防止设为负数或0)
            if (AUC_CONFIG.NOISE_THRESHOLD < 0.1) AUC_CONFIG.NOISE_THRESHOLD = 0.1;

            // 更新 UI (保留1位小数)
            if (elZ) elZ.textContent = AUC_CONFIG.NOISE_THRESHOLD.toFixed(1);
            console.log(`🎛️ [参数调整] Z-score 阈值 -> ${AUC_CONFIG.NOISE_THRESHOLD.toFixed(1)}`);
        }
    };

    // ====== 🌊 [新] FFT 频段控制配置 ======
    const FFT_CONFIG = {
        FREQ_MIN: 300,  // 默认显示频率下限
        FREQ_MAX: 600   // 默认显示频率上限
    };

    // 全局存储最新一次触发的 FFT 频谱数据
    window.latestAcousticFFT = null;


    // ====== 🌊 峰值监测全局变量 ======
    // 存储最近 50 帧的 { deviation: 偏离度, val: 信号强度 }
    let mutationWindow = [];
    // 用于界面滚动显示的队列 [历史2, 历史1, 当前]
    let mutationDisplayHistory = [0, 0, 0];


    // GPR 相关
    let gprMigrationEnabled = false; // 偏移成像默认关闭；只影响下一次“开始处理”
    let showAnnotation = false;      // 标注开关状态
    let annotation3DGroups = [];     // 存储 3D 标注对象
    let gprResult = null;            // 缓存 GPR 结果
    let gprPipePopup3DPos = null;    // 三维重建页地下管道点击弹窗对应的世界坐标

    // ✅ GPR 标注校准持久化状态
    // 说明：以“GPR 数据文件夹路径”为依据，把人工校准和彩色倾斜管道标注保存到机器狗主机的 GPR 数据文件夹中。
    // 文件名固定为 gpr_annotation_calibration.json；下次再次处理同一个文件夹时，会由后端自动读取并恢复上次校准结果。
    let currentGprFolderPath = "";
    let currentGprAnnotationStorageKey = "";
    const GPR_ANNOTATION_FILE_NAME = "gpr_annotation_calibration.json";
    const GPR_ANNOTATION_API = {
        load: (folderPath) => "/api/gpr/annotation?folderPath=" + encodeURIComponent(folderPath),
        save: "/api/gpr/annotation",
        remove: (folderPath) => "/api/gpr/annotation?folderPath=" + encodeURIComponent(folderPath)
    };

    // ✅ 偏移后二维剖面手动标注相关状态
    let currentGprSegmentIndex = 0;
    let manualMigAnnotationMode = false;
    const manualMigAnnotationsBySegment = new Map(); // segIndex -> [{row,col,radius,width,height,manual}]
    const gpr2DRenderInfo = {};                      // canvasId -> {rows,cols,stepX,stepY,w,h}，step 可为小数
    const MANUAL_MIG_ANNOTATION = {
        displayRadiusPx: 6,
        deleteHitRadiusPx: 18
    };

    // ✅ 倾斜/交叉管道彩色强制连接标注状态
    // 说明：每次点击“倾斜管道”进入标注，都会开启一个新的管道组，自动换一种颜色。
    // 同一组颜色的圆圈在相邻剖面之间会强制连接，不再受 maxZDiff / maxYDiff 阈值限制。
    let forcedPipeAnnotationMode = false;
    let activeForcedPipeGroupId = null;
    let activeForcedPipeColorHex = 0x0099ff;
    let activeForcedPipeColorCss = "#0099ff";
    let activeForcedPipeColorName = "蓝色";
    let forcedPipeGroupCounter = 0;
    let forcedPipeAnnotationOrder = 0;
    const forcedPipeAnnotationsBySegment = new Map(); // segIndex -> [{row,col,radius,pipeGroupId,pipeColorHex,...}]
    const FORCED_PIPE_ANNOTATION = {
        displayRadiusPx: 6,
        deleteHitRadiusPx: 18,
        colors: [
            { hex: 0x0099ff, css: "#0099ff", name: "蓝色" },
            { hex: 0x00ffcc, css: "#00ffcc", name: "青色" },
            { hex: 0xffcc00, css: "#ffcc00", name: "黄色" },
            { hex: 0xff66ff, css: "#ff66ff", name: "紫色" },
            { hex: 0x00ff66, css: "#00ff66", name: "绿色" },
            { hex: 0xff8800, css: "#ff8800", name: "橙色" }
        ]
    };

    // ✅ 地下管道材质颜色指定状态
    // 说明：点击三维重建页中已经连接好的地下管道后，可将其指定为强电电缆/PVC管/金属管道等材质颜色。
    // 指定结果与手动标注校准一起保存到 gpr_annotation_calibration.json；下次读取同一GPR文件夹时自动恢复。
    const GPR_PIPE_DEFAULT_COLOR_HEX = 0xffff00;
    const GPR_PIPE_MATERIAL_TYPES = [
        { type: "cable", name: "地下电缆", title: "高压电缆测点信息", colorHex: 0x050505, colorCss: "#050505" },
        { type: "pvc", name: "PVC管", title: "PVC管道测点信息", colorHex: 0xffffff, colorCss: "#ffffff" },
        { type: "metal", name: "金属管道", title: "金属管道测点信息", colorHex: 0xc0c0c0, colorCss: "#c0c0c0" }
    ];
    const gprPipeColorAssignments = new Map(); // pipeColorKey -> {materialType, materialName, colorHex, colorCss, updatedAt}
    let selectedGprPipeObject = null;

    // ✅ 人工标注圆圈拖动相关状态（自动红圈功能已移除）
    // 支持拖动对象：人工校准红圈、倾斜管道彩色圆圈。
    let gprAnnotationDragState = null;
    let gprAnnotationPointerDownState = null;
    // ✅ 删除操作发生在 pointerdown 阶段，浏览器随后仍会触发 pointerup。
    // 用该标记屏蔽本次 pointerup，避免右键/Ctrl/Shift 删除后又被当成“空白点击新增圆圈”。
    let suppressNextMigAnnotationPointerUp = false;
    const GPR_ANNOTATION_DRAG = {
        startThresholdPx: 3,
        refreshIntervalMs: 16
    };

    // ROS 机器人相关
    let stompClient = null;          // WebSocket 客户端
    let motionTimer = null;          // ✅ 运动控制定时器 (用于实现长按连续移动)
    let isArmBusy = false;           // ✅ 补充定义：机械臂忙碌状态锁

    // ⚙️ 机器人运动参数配置
    const SPEEDS = {
        LINEAR: 0.5,   // 前进/后退线速度 (m/s)
        ANGULAR: 1.0,   // 转向角速度 (rad/s)
        STRAFE: 0.3    // ✅ 新增：横向移动速度 (m/s) (螃蟹步通常慢一点)
    };
    // ✅ 新增：机器狗模式定义 (通过 linear.z 传递)
    // 对应 C++ enum StateName: kStandUp=1, kRLControl=6
    const MODES = {
        IDLE: 0.0,      // 空闲
        STAND: 1.0,     // 站立
        DAMPING: 2.0,   // 阻尼
        WALK: 3.0,      // ✅ 常规足式模式 (之前验证过能横移)
        LIE_DOWN: 4.0   // 趴下
    };

    // GPR 算法配置
    const CONFIG = {
        minRadius: 3,    //噪点过滤阈值
        maxRadius: 15,       // ✅ 新增：最大半径阈值（如大于 30 像素则认为是地面/连续地层，不予标注）
        radiusPadding: 6    //会在算出的真实亮斑半径基础上加上这个数值
    };

    // 管道连接配置
    const LINK_CONFIG = {
        maxZDiff: 0.15,
        maxYDiff: 0.4
    };

    // ✅ 管道三维连接平滑配置
    // 说明：相邻剖面圆圈连接时，若同一条管道跨越 3 个及以上剖面，
    // 不再逐段生成硬直圆柱，而是根据多个圆心生成 Catmull-Rom 平滑管道。
    const PIPE_SMOOTH_CONFIG = {
        enabled: true,
        minPointsForSmooth: 3,
        curveType: "centripetal",
        tension: 0.35,
        tubularSegmentsPerSpan: 18,
        radialSegments: 12
    };

    // ✅ 地上黄色电缆路径 -> 地下 GPR 管道自动校正配置
    // 校正只改变地下管道中心线的水平 X/Z 位置；GPR 检出的埋深 Y、管径、材质和原始红圈均保留。
    // 多条地下管道同时存在时，只选择与黄色路径最接近的一条，避免把其它管线错误吸附到电缆路径上。
    const GPR_CABLE_PATH_ALIGNMENT_CONFIG = {
        enabled: true,
        minMatchDistanceM: 0.65,
        maxMatchDistanceM: 2.0,
        scanWidthRatio: 0.55,
        outlierDistanceRatio: 1.35,
        minMatchedPoints: 3,
        minAlongPathSpanM: 0.35,
        sampleStepM: 0.08
    };

    // ==========================================
    // 1. 辅助算法与工具函数
    // ==========================================

    function $(id) { return document.getElementById(id); }

    const OPERATION_STATUS_HISTORY_LIMIT = 3;
    const GPR_PROGRESS_STATUS_KEY = "gpr-processing-progress";
    let operationStatusHistory = [];

    function isTrajectoryGeoCalibrationActive() {
        return Array.isArray(window.geoCalibSamples) &&
            window.geoCalibSamples.length > 0 &&
            !window.isGpsCalibrated &&
            !window.geoTransform;
    }

    function isTrajectoryGeoCalibrationStatus(message) {
        return message.includes("轨迹标定") || message.includes("GPS/SLAM轨迹标定");
    }

    function normalizeStatusType(type) {
        const statusType = String(type || "");
        return ["error", "success", "warning"].includes(statusType) ? statusType : "";
    }

    function renderOperationStatusEntries(entries) {
        const statusText = $("operationStatusText");
        const list = Array.isArray(entries) ? entries.filter(item => item && item.message) : [];

        if (!statusText) {
            list.forEach(item => console.log(`[系统状态] ${item.message}`));
            return;
        }

        statusText.innerHTML = "";
        statusText.classList.remove("error", "success", "warning");
        statusText.title = list.length ? list.map(item => item.message).join("\n") : "等待操作...";

        if (list.length === 0) {
            statusText.textContent = "等待操作...";
            return;
        }

        list.forEach(item => {
            const line = document.createElement("div");
            line.className = "global-operation-log-line";
            if (item.type) line.classList.add(item.type);
            line.textContent = item.message;
            statusText.appendChild(line);
        });
        statusText.scrollTop = 0;
    }

    function setStatus(text, type) {
        const message = text == null ? "" : String(text);
        if (!message.trim()) return;

        const statusType = normalizeStatusType(type);
        const isCalibrationActive = isTrajectoryGeoCalibrationActive();
        const isCalibrationStatus = isTrajectoryGeoCalibrationStatus(message);

        if (isCalibrationActive && !isCalibrationStatus) {
            return;
        }

        if (isCalibrationActive) {
            renderOperationStatusEntries([{ message, type: statusType }]);
            return;
        }

        operationStatusHistory.unshift({ message, type: statusType });
        operationStatusHistory = operationStatusHistory.slice(0, OPERATION_STATUS_HISTORY_LIMIT);
        renderOperationStatusEntries(operationStatusHistory);
    }

    function setGprProcessingProgress(value) {
        const percentage = Math.max(0, Math.min(100, Math.round(Number(value) || 0)));
        operationStatusHistory = operationStatusHistory.filter(item => item.key !== GPR_PROGRESS_STATUS_KEY);
        operationStatusHistory.unshift({
            key: GPR_PROGRESS_STATUS_KEY,
            message: `处理进度：${percentage}%`,
            type: ""
        });
        operationStatusHistory = operationStatusHistory.slice(0, OPERATION_STATUS_HISTORY_LIMIT);
        renderOperationStatusEntries(operationStatusHistory);
    }

    function clearGprProcessingProgress() {
        operationStatusHistory = operationStatusHistory.filter(item => item.key !== GPR_PROGRESS_STATUS_KEY);
        renderOperationStatusEntries(operationStatusHistory);
    }

    const SYSTEM_MESSAGE_LIMIT = 30;
    let systemMessages = [];
    let unreadSystemMessageCount = 0;
    let systemMessageToastTimer = null;

    function formatSystemMessageTime(timestamp) {
        const date = new Date(timestamp);
        const pad = value => String(value).padStart(2, "0");
        return `${pad(date.getHours())}:${pad(date.getMinutes())}:${pad(date.getSeconds())}`;
    }

    function updateSystemMessageBadge() {
        const badge = $("systemMessageBadge");
        if (!badge) return;

        if (unreadSystemMessageCount <= 0) {
            badge.classList.add("hidden");
            badge.textContent = "0";
            return;
        }

        badge.textContent = unreadSystemMessageCount > 99 ? "99+" : String(unreadSystemMessageCount);
        badge.classList.remove("hidden");
    }

    function renderSystemMessages() {
        const list = $("systemMessageList");
        if (!list) return;

        list.innerHTML = "";
        if (!systemMessages.length) {
            const empty = document.createElement("div");
            empty.className = "system-message-empty";
            empty.textContent = "暂无系统消息";
            list.appendChild(empty);
            return;
        }

        systemMessages.forEach(item => {
            const row = document.createElement("div");
            row.className = "system-message-item";
            if (item.type) row.classList.add(item.type);

            const time = document.createElement("div");
            time.className = "system-message-item-time";
            time.textContent = formatSystemMessageTime(item.timestamp);

            const text = document.createElement("div");
            text.className = "system-message-item-text";
            text.textContent = item.message;

            row.appendChild(time);
            row.appendChild(text);
            list.appendChild(row);
        });
    }

    function showSystemMessageToast(message, type = "warning") {
        const toast = $("systemMessageToast");
        if (!toast) return;

        const statusType = normalizeStatusType(type);
        toast.className = "system-message-toast";
        if (statusType) toast.classList.add(statusType);
        toast.textContent = message;

        if (systemMessageToastTimer !== null) {
            window.clearTimeout(systemMessageToastTimer);
            systemMessageToastTimer = null;
        }

        // 触发布局刷新，使连续消息也能重新播放淡入效果。
        void toast.offsetWidth;
        toast.classList.add("visible");

        systemMessageToastTimer = window.setTimeout(() => {
            toast.classList.remove("visible");
            systemMessageToastTimer = null;
        }, 2000);
    }

    function setSystemMessagePanelOpen(open) {
        const panel = $("systemMessagePanel");
        if (!panel) return;

        panel.classList.toggle("hidden", !open);
        if (open) {
            unreadSystemMessageCount = 0;
            updateSystemMessageBadge();
        }
    }

    function addSystemMessage(message, type = "warning") {
        const text = message == null ? "" : String(message).trim();
        if (!text) return;

        const panel = $("systemMessagePanel");
        const isPanelOpen = panel && !panel.classList.contains("hidden");
        systemMessages.unshift({
            message: text,
            type: normalizeStatusType(type),
            timestamp: Date.now()
        });
        systemMessages = systemMessages.slice(0, SYSTEM_MESSAGE_LIMIT);

        if (!isPanelOpen) {
            unreadSystemMessageCount += 1;
        }

        renderSystemMessages();
        updateSystemMessageBadge();
        showSystemMessageToast(text, type);
    }

    function initSystemMessageCenter() {
        const wrapper = $("systemMessageWrapper");
        const button = $("btnSystemMessages");
        const panel = $("systemMessagePanel");
        if (!wrapper || !button || !panel || button.dataset.bound === "true") return;

        button.dataset.bound = "true";
        button.addEventListener("click", function (e) {
            e.preventDefault();
            e.stopPropagation();
            setSystemMessagePanelOpen(panel.classList.contains("hidden"));
        });

        panel.addEventListener("click", function (e) {
            e.stopPropagation();
        });

        document.addEventListener("click", function (e) {
            if (!wrapper.contains(e.target)) {
                setSystemMessagePanelOpen(false);
            }
        });

        document.addEventListener("keydown", function (e) {
            if (e.key === "Escape") {
                setSystemMessagePanelOpen(false);
            }
        });

        renderSystemMessages();
        updateSystemMessageBadge();
    }

    // ==========================================
    // ✅ 点云地图经纬度元数据工具函数
    // 说明：PCD 只保存 SLAM 点云本体；同名 .geo.json 保存 GPS/SLAM 标定参数。
    // 三维重建页读取 .geo.json 后会恢复 window.gpsAnchor / window.slamAnchor / window.headingOffset，
    // 从而复用现有 slamToGps() 生成履带小车经纬度航点。
    // ==========================================
    function roundNumber(value, digits = 8) {
        const n = Number(value);
        if (!Number.isFinite(n)) return null;
        return Number(n.toFixed(digits));
    }

    function clonePlainGps(gps) {
        if (!gps) return null;
        const lat = Number(gps.lat);
        const lon = Number(gps.lon);
        if (!Number.isFinite(lat) || !Number.isFinite(lon)) return null;
        return {
            lat: roundNumber(lat, 10),
            lon: roundNumber(lon, 10)
        };
    }

    function clonePlainSlam(pt) {
        if (!pt) return null;
        const x = Number(pt.x);
        const y = Number(pt.y);
        const z = Number(pt.z ?? 0);
        if (!Number.isFinite(x) || !Number.isFinite(y)) return null;
        return {
            x: roundNumber(x, 5),
            y: roundNumber(y, 5),
            z: roundNumber(z, 5)
        };
    }


    // ==========================================
    // ✅ 多点轨迹 GPS/SLAM 标定工具函数
    // 适配激光雷达经纬度只有小数点后 6 位的情况：
    // 不再依赖机器狗直线移动 1 米，而是采集一段不规则轨迹，
    // 用 RANSAC + 二维相似变换同时求解 scale / 旋转角 / 平移量。
    // ==========================================
    const TRAJECTORY_GEO_CALIB_CONFIG = {
        enabled: true,
        gpsDecimals: 6,
        minSampleDistanceM: 0.35,      // 6位经纬度量化约0.1m，采样不宜过密
        minSamples: 30,
        minTotalTravelM: 8.0,
        minSpatialSpanM: 3.0,
        maxRmsErrorM: 0.45,
        maxPointErrorM: 1.20,
        ransacIterations: 90,
        ransacInlierThresholdM: 0.60,
        statusLogIntervalMs: 1200
    };

    function normalizeAngleRad(rad) {
        let v = Number(rad) || 0;
        while (v > Math.PI) v -= 2 * Math.PI;
        while (v < -Math.PI) v += 2 * Math.PI;
        return v;
    }

    function parseGpsNumber(value) {
        if (typeof value === "number") return value;
        if (value === null || value === undefined) return NaN;
        return parseFloat(String(value).replace(/[^\d.\-+]/g, ""));
    }

    function gpsToLocalMeters(lat, lon, originLat, originLon) {
        const lat0 = Number(originLat);
        const lon0 = Number(originLon);
        const north = (Number(lat) - lat0) * 111320;
        const east = (Number(lon) - lon0) * (111320 * Math.cos(lat0 * Math.PI / 180));
        return { east, north };
    }

    function localMetersToGps(east, north, originLat, originLon) {
        const lat0 = Number(originLat);
        const lon0 = Number(originLon);
        return {
            lat: lat0 + Number(north) / 111320,
            lon: lon0 + Number(east) / (111320 * Math.cos(lat0 * Math.PI / 180))
        };
    }

    function transformPointByGeoTransform(transform, x, y) {
        if (!transform) return null;
        const scale = Number(transform.scale ?? 1);
        const theta = Number(transform.theta ?? transform.headingOffset ?? 0);
        const tx = Number(transform.tx ?? 0);
        const ty = Number(transform.ty ?? 0);
        const cos = Math.cos(theta);
        const sin = Math.sin(theta);
        return {
            east: scale * (cos * Number(x) - sin * Number(y)) + tx,
            north: scale * (sin * Number(x) + cos * Number(y)) + ty
        };
    }

    function inverseTransformPointByGeoTransform(transform, east, north) {
        if (!transform) return null;
        const scale = Number(transform.scale ?? 1);
        if (!Number.isFinite(scale) || Math.abs(scale) < 1e-9) return null;
        const theta = Number(transform.theta ?? transform.headingOffset ?? 0);
        const tx = Number(transform.tx ?? 0);
        const ty = Number(transform.ty ?? 0);
        const dx = Number(east) - tx;
        const dy = Number(north) - ty;
        const cos = Math.cos(theta);
        const sin = Math.sin(theta);
        return {
            x: (cos * dx + sin * dy) / scale,
            y: (-sin * dx + cos * dy) / scale
        };
    }

    function normalizeLoadedGeoTransform(transform) {
        if (!transform || typeof transform !== "object") return null;
        const originLat = Number(transform.originLat ?? transform.gpsOrigin?.lat ?? transform.origin?.lat);
        const originLon = Number(transform.originLon ?? transform.gpsOrigin?.lon ?? transform.origin?.lon);
        const scale = Number(transform.scale ?? 1);
        const theta = Number(transform.theta ?? transform.headingOffset ?? transform.yawRad ?? 0);
        const tx = Number(transform.tx ?? transform.translationEast ?? 0);
        const ty = Number(transform.ty ?? transform.translationNorth ?? 0);
        if (![originLat, originLon, scale, theta, tx, ty].every(Number.isFinite)) return null;
        return {
            ...transform,
            method: transform.method || "trajectory_similarity_2d",
            originLat,
            originLon,
            scale,
            theta,
            tx,
            ty
        };
    }

    function resetTrajectoryGeoCalibration(reason = "manual") {
        window.geoCalibSamples = [];
        window.geoTransform = null;
        window.isGpsCalibrated = false;
        window.gpsAnchor = null;
        window.slamAnchor = null;
        window.headingOffset = 0;
        window.__lastGeoCalibStatusAt = 0;
        console.log("🔄 已重置多点轨迹GPS/SLAM标定：", reason);
    }
    window.resetTrajectoryGeoCalibration = resetTrajectoryGeoCalibration;

    function computeTrajectoryQuality(samples) {
        const arr = Array.isArray(samples) ? samples : [];
        if (arr.length < 2) {
            return { ready: false, totalTravelM: 0, spatialSpanM: 0, uniqueGpsCount: arr.length };
        }

        let totalTravelM = 0;
        let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
        const gpsKeys = new Set();

        arr.forEach((s, i) => {
            minX = Math.min(minX, s.x);
            maxX = Math.max(maxX, s.x);
            minY = Math.min(minY, s.y);
            maxY = Math.max(maxY, s.y);
            gpsKeys.add(`${Number(s.lat).toFixed(6)},${Number(s.lon).toFixed(6)}`);
            if (i > 0) {
                const p = arr[i - 1];
                totalTravelM += Math.hypot(s.x - p.x, s.y - p.y);
            }
        });

        const spatialSpanM = Math.hypot(maxX - minX, maxY - minY);
        return {
            ready: arr.length >= TRAJECTORY_GEO_CALIB_CONFIG.minSamples &&
                totalTravelM >= TRAJECTORY_GEO_CALIB_CONFIG.minTotalTravelM &&
                spatialSpanM >= TRAJECTORY_GEO_CALIB_CONFIG.minSpatialSpanM &&
                gpsKeys.size >= Math.max(6, Math.floor(arr.length * 0.35)),
            totalTravelM,
            spatialSpanM,
            uniqueGpsCount: gpsKeys.size
        };
    }

    function fitSimilarityTransform2D(pairs) {
        if (!Array.isArray(pairs) || pairs.length < 2) return null;
        let meanPx = 0, meanPy = 0, meanQx = 0, meanQy = 0;
        pairs.forEach(pair => {
            meanPx += pair.px;
            meanPy += pair.py;
            meanQx += pair.qx;
            meanQy += pair.qy;
        });
        const n = pairs.length;
        meanPx /= n; meanPy /= n; meanQx /= n; meanQy /= n;

        let a = 0, b = 0, denom = 0;
        pairs.forEach(pair => {
            const px = pair.px - meanPx;
            const py = pair.py - meanPy;
            const qx = pair.qx - meanQx;
            const qy = pair.qy - meanQy;
            a += px * qx + py * qy;
            b += px * qy - py * qx;
            denom += px * px + py * py;
        });
        if (denom < 1e-9) return null;

        const theta = Math.atan2(b, a);
        const scale = Math.sqrt(a * a + b * b) / denom;
        const cos = Math.cos(theta);
        const sin = Math.sin(theta);
        const tx = meanQx - scale * (cos * meanPx - sin * meanPy);
        const ty = meanQy - scale * (sin * meanPx + cos * meanPy);

        const transform = { method: "trajectory_similarity_2d", scale, theta, tx, ty, sampleCount: n };
        const errors = pairs.map(pair => {
            const pred = transformPointByGeoTransform(transform, pair.px, pair.py);
            return Math.hypot(pred.east - pair.qx, pred.north - pair.qy);
        });
        transform.rmsErrorM = Math.sqrt(errors.reduce((sum, e) => sum + e * e, 0) / errors.length);
        transform.maxErrorM = Math.max(...errors);
        return transform;
    }

    function randomPickTwo(pairs) {
        const i = Math.floor(Math.random() * pairs.length);
        let j = Math.floor(Math.random() * pairs.length);
        while (j === i) j = Math.floor(Math.random() * pairs.length);
        return [pairs[i], pairs[j]];
    }

    function fitSimilarityTransform2DRansac(pairs) {
        if (!Array.isArray(pairs) || pairs.length < 6) return fitSimilarityTransform2D(pairs);
        let bestInliers = [];
        for (let k = 0; k < TRAJECTORY_GEO_CALIB_CONFIG.ransacIterations; k++) {
            const candidate = fitSimilarityTransform2D(randomPickTwo(pairs));
            if (!candidate) continue;
            const inliers = pairs.filter(pair => {
                const pred = transformPointByGeoTransform(candidate, pair.px, pair.py);
                const err = Math.hypot(pred.east - pair.qx, pred.north - pair.qy);
                return err <= TRAJECTORY_GEO_CALIB_CONFIG.ransacInlierThresholdM;
            });
            if (inliers.length > bestInliers.length) bestInliers = inliers;
        }
        const refined = bestInliers.length >= 6 ? fitSimilarityTransform2D(bestInliers) : fitSimilarityTransform2D(pairs);
        if (!refined) return null;
        refined.inlierCount = bestInliers.length >= 6 ? bestInliers.length : pairs.length;
        refined.outlierCount = pairs.length - refined.inlierCount;
        refined.ransacThresholdM = TRAJECTORY_GEO_CALIB_CONFIG.ransacInlierThresholdM;
        refined.rawSampleCount = pairs.length;
        return refined;
    }

    function finishTrajectoryGeoCalibration() {
        const samples = Array.isArray(window.geoCalibSamples) ? window.geoCalibSamples : [];
        const quality = computeTrajectoryQuality(samples);
        if (!quality.ready) return false;

        const origin = samples[0];
        const pairs = samples.map(s => {
            const q = gpsToLocalMeters(s.lat, s.lon, origin.lat, origin.lon);
            return {
                px: Number(s.x),
                py: Number(s.y),
                qx: q.east,
                qy: q.north,
                lat: s.lat,
                lon: s.lon,
                t: s.t
            };
        }).filter(p => [p.px, p.py, p.qx, p.qy].every(Number.isFinite));

        const transform = fitSimilarityTransform2DRansac(pairs);
        if (!transform) {
            setStatus("❌ 轨迹标定失败：有效样本不足或轨迹退化。", "error");
            return false;
        }

        if (transform.rmsErrorM > TRAJECTORY_GEO_CALIB_CONFIG.maxRmsErrorM ||
            transform.maxErrorM > TRAJECTORY_GEO_CALIB_CONFIG.maxPointErrorM) {
            console.warn("⚠️ 轨迹标定误差过大，继续采样：", transform);
            setStatus(
                `⚠️ 轨迹标定误差偏大：RMS=${transform.rmsErrorM.toFixed(2)}m，最大=${transform.maxErrorM.toFixed(2)}m，请继续移动扩大轨迹范围。`,
                "error"
            );
            return false;
        }

        window.geoTransform = {
            ...transform,
            originLat: origin.lat,
            originLon: origin.lon,
            gpsDecimals: TRAJECTORY_GEO_CALIB_CONFIG.gpsDecimals,
            totalTravelM: Number(quality.totalTravelM.toFixed(3)),
            spatialSpanM: Number(quality.spatialSpanM.toFixed(3)),
            uniqueGpsCount: quality.uniqueGpsCount,
            calibratedAt: new Date().toISOString()
        };

        // 兼容旧代码：仍同步写入 gpsAnchor / slamAnchor / headingOffset。
        window.gpsAnchor = { lat: origin.lat, lon: origin.lon };
        window.slamAnchor = { x: 0, y: 0, z: 0 };
        window.headingOffset = normalizeAngleRad(window.geoTransform.theta);
        window.isGpsCalibrated = true;

        console.log("✅ 多点轨迹GPS/SLAM标定完成：", window.geoTransform);
        setStatus(
            `✅ 轨迹标定完成：样本${transform.rawSampleCount || transform.sampleCount}个，内点${transform.inlierCount || transform.sampleCount}个，RMS=${transform.rmsErrorM.toFixed(2)}m，scale=${transform.scale.toFixed(4)}，角度=${(transform.theta * 180 / Math.PI).toFixed(2)}°`,
            "success"
        );
        return true;
    }

    function addGeoCalibrationSample(rawLat, rawLon, pose, status = "") {
        if (!TRAJECTORY_GEO_CALIB_CONFIG.enabled) return;
        if (window.isGpsCalibrated || window.geoTransform) return;
        if (!pose || pose.x === -999) return;

        const lat = parseGpsNumber(rawLat);
        const lon = parseGpsNumber(rawLon);
        const x = Number(pose.x);
        const y = Number(pose.y);
        if (![lat, lon, x, y].every(Number.isFinite)) return;

        if (!Array.isArray(window.geoCalibSamples)) window.geoCalibSamples = [];
        const samples = window.geoCalibSamples;
        const sample = { lat, lon, x, y, status: status || "", t: Date.now() };

        if (samples.length === 0) {
            samples.push(sample);
            window.gpsAnchor = { lat, lon };
            window.slamAnchor = { x, y, z: Number(pose.z ?? 0) };
            setStatus("📍 开始采集多点GPS/SLAM轨迹标定样本，请正常移动机器狗，不要求直线。", "warning");
            return;
        }

        const last = samples[samples.length - 1];
        const moveDist = Math.hypot(x - last.x, y - last.y);
        if (moveDist < TRAJECTORY_GEO_CALIB_CONFIG.minSampleDistanceM) return;

        // 6位经纬度会出现格点重复；完全重复的GPS点不参与拟合，避免把量化平台期当成真实轨迹。
        if (Number(lat).toFixed(6) === Number(last.lat).toFixed(6) &&
            Number(lon).toFixed(6) === Number(last.lon).toFixed(6)) {
            return;
        }

        samples.push(sample);
        const quality = computeTrajectoryQuality(samples);
        const now = Date.now();
        if (now - (window.__lastGeoCalibStatusAt || 0) > TRAJECTORY_GEO_CALIB_CONFIG.statusLogIntervalMs) {
            window.__lastGeoCalibStatusAt = now;
            setStatus(
                `📍 轨迹标定采样中：${samples.length}/${TRAJECTORY_GEO_CALIB_CONFIG.minSamples}点，累计${quality.totalTravelM.toFixed(1)}m，跨度${quality.spatialSpanM.toFixed(1)}m，GPS格点${quality.uniqueGpsCount}个`,
                "warning"
            );
        }

        if (quality.ready) finishTrajectoryGeoCalibration();
    }
    window.addGeoCalibrationSample = addGeoCalibrationSample;

    function applyLoadedMapGeoMetadata(meta, sourceName = "") {
        if (!meta || typeof meta !== "object") return false;

        const calibration = meta.calibration || meta.gpsCalibration || meta;
        const loadedTransform = normalizeLoadedGeoTransform(calibration.geoTransform || meta.geoTransform || calibration.transform);
        const gpsAnchor = clonePlainGps(calibration.gpsAnchor) || (loadedTransform ? { lat: loadedTransform.originLat, lon: loadedTransform.originLon } : null);
        const slamAnchor = clonePlainSlam(calibration.slamAnchor) || (loadedTransform ? { x: 0, y: 0, z: 0 } : null);
        const headingOffset = Number(calibration.headingOffset ?? calibration.headingOffsetRad ?? loadedTransform?.theta ?? 0);

        if (!loadedTransform && (!gpsAnchor || !slamAnchor || !Number.isFinite(headingOffset))) {
            console.warn("⚠️ 点云经纬度元数据缺少 geoTransform 或 gpsAnchor / slamAnchor / headingOffset，已跳过恢复：", meta);
            return false;
        }

        window.geoTransform = loadedTransform;
        window.gpsAnchor = gpsAnchor;
        window.slamAnchor = slamAnchor;
        window.headingOffset = normalizeAngleRad(headingOffset);
        window.isGpsCalibrated = true;

        if (calibration.currentGps) {
            const currentGps = clonePlainGps(calibration.currentGps);
            if (currentGps) window.currentGps = currentGps;
        }

        window.loadedPointCloudGeoMetadata = meta;
        window.loadedPointCloudGeoSource = sourceName || meta.pcdFileName || "loaded-map";

        console.log("✅ 已恢复点云地图经纬度标定：", {
            source: window.loadedPointCloudGeoSource,
            gpsAnchor: window.gpsAnchor,
            slamAnchor: window.slamAnchor,
            headingOffset: window.headingOffset,
            geoTransform: window.geoTransform
        });

        return true;
    }

    function inferGeoJsonPathFromPcdPath(pcdPath) {
        const path = String(pcdPath || "").trim();
        if (!path) return "";
        if (/\.pcd$/i.test(path)) return path.replace(/\.pcd$/i, ".geo.json");
        return path + ".geo.json";
    }

    async function tryLoadServerPcdGeoMetadata(pcdPath) {
        const geoPath = inferGeoJsonPathFromPcdPath(pcdPath);
        if (!geoPath) return null;

        try {
            const url = "/api/gpr/pcd?filePath=" + encodeURIComponent(geoPath);
            const res = await fetch(url, { cache: "no-store" });
            if (!res.ok) return null;

            const text = await res.text();
            return JSON.parse(text);
        } catch (err) {
            console.warn("未读取到同名点云经纬度元数据：", geoPath, err);
            return null;
        }
    }

    function readFileAsText(file) {
        return new Promise((resolve, reject) => {
            const reader = new FileReader();
            reader.onload = e => resolve(String(e.target.result || ""));
            reader.onerror = () => reject(new Error("读取文件失败：" + file.name));
            reader.readAsText(file);
        });
    }

    function readFileAsArrayBuffer(file) {
        return new Promise((resolve, reject) => {
            const reader = new FileReader();
            reader.onload = e => resolve(e.target.result);
            reader.onerror = () => reject(new Error("读取文件失败：" + file.name));
            reader.readAsArrayBuffer(file);
        });
    }

    // ==========================================
    // ✅ 机械臂末端摄像头实时视频流
    // ==========================================
    function initArmEndCameraStream() {
        const img = $("armEndCameraStream");
        const placeholder = $("armEndCameraPlaceholder");
        if (!img || !placeholder) return;

        const streamUrl = ARM_CAMERA_BASE + "/arm_feed";
        let retryTimer = null;

        function showPlaceholder(text) {
            img.style.display = "none";
            placeholder.style.display = "block";
            placeholder.innerHTML = `
                <span style="font-size: 24px; display: block; margin-bottom: 8px;">📡</span>
                ${text || "末端视频流未连接"}<br>
                <span style="font-size: 11px; color: rgba(135, 200, 255, 0.25); font-family: monospace;">${streamUrl}</span>
            `;
        }

        function connectStream() {
            if (retryTimer) {
                clearTimeout(retryTimer);
                retryTimer = null;
            }

            showPlaceholder("正在连接末端视频流...");

            img.onload = function () {
                placeholder.style.display = "none";
                img.style.display = "block";
            };

            img.onerror = function () {
                showPlaceholder("末端视频流连接失败，3秒后重试");
                img.removeAttribute("src");
                retryTimer = setTimeout(connectStream, 3000);
            };

            // 加时间戳避免浏览器缓存旧连接。
            img.src = streamUrl + "?t=" + Date.now();
        }

        connectStream();

        // 便于现场调试时在浏览器控制台手动重连：window.reconnectArmEndCameraStream()
        window.reconnectArmEndCameraStream = connectStream;
    }

    function getRobotCameraSourceVideo(direction) {
        if (robotCameraSourceVideos[direction]) return robotCameraSourceVideos[direction];
        const id = direction === "front" ? "robotCameraSourceFront" : "robotCameraSourceBack";
        const video = document.getElementById(id);
        if (!video) return null;

        video.muted = true;
        video.autoplay = true;
        video.playsInline = true;
        video.classList.add("stream-img", "robot-camera-video-fallback");
        video.setAttribute("playsinline", "");
        video.setAttribute("webkit-playsinline", "");
        video.title = ROBOT_CAMERA_WHEP_URLS[direction] || "";
        robotCameraSourceVideos[direction] = video;
        return video;
    }

    function ensureRobotCameraCanvas(direction) {
        if (robotCameraCanvases[direction]) return robotCameraCanvases[direction];
        const canvas = document.createElement("canvas");
        canvas.className = "stream-img robot-camera-canvas";
        canvas.dataset.camera = direction;
        canvas.width = 960;
        canvas.height = 540;
        canvas.setAttribute("aria-label", direction === "front" ? "机器狗前摄像头画面" : "机器狗后摄像头画面");
        robotCameraCanvases[direction] = canvas;
        return canvas;
    }

    function getRobotCameraSlot(panel, direction) {
        return panel && panel.querySelector(`[data-camera-slot="${direction}"]`);
    }

    function getRobotCameraOutputElement(direction) {
        return robotCameraRenderers[direction]
            ? ensureRobotCameraCanvas(direction)
            : getRobotCameraSourceVideo(direction);
    }

    function moveRobotCameraOutputsToPanel(target) {
        const panelId = target === "pathFault"
            ? "pathFaultPanel"
            : target === "robot"
                ? "robotNavPanel"
                : null;
        if (!panelId) return;

        const panel = document.getElementById(panelId);
        if (!panel) return;
        const sourceContainer = document.querySelector(".robot-camera-source-container");

        ROBOT_CAMERA_DIRECTIONS.forEach(direction => {
            const slot = getRobotCameraSlot(panel, direction);
            const video = getRobotCameraSourceVideo(direction);
            const output = getRobotCameraOutputElement(direction);
            if (!slot || !output) return;

            if (robotCameraRenderers[direction] && sourceContainer && video && video.parentElement !== sourceContainer) {
                sourceContainer.appendChild(video);
            }
            if (output.parentElement !== slot) slot.appendChild(output);
        });

        requestAnimationFrame(() => {
            ROBOT_CAMERA_DIRECTIONS.forEach(direction => {
                resizeRobotCameraCanvas(direction, true);
                renderRobotCameraFrame(direction, true);
            });
        });
    }

    function createDisabledRobotCameraCalibration() {
        return {
            version: "unavailable",
            model: "opencv-fisheye",
            preview: {
                maxWidth: 1280,
                maxHeight: 720,
                maxDevicePixelRatio: 1.5
            },
            front: { enabled: false, inputSize: [1920, 1080] },
            back: { enabled: false, inputSize: [1920, 1080] }
        };
    }

    async function loadRobotCameraCalibration() {
        try {
            const response = await fetch(ROBOT_CAMERA_CALIBRATION_URL, { cache: "no-store" });
            if (!response.ok) {
                throw new Error(`HTTP ${response.status}`);
            }
            const config = await response.json();
            if (!config || config.model !== "opencv-fisheye") {
                throw new Error("不支持的标定配置格式");
            }
            return config;
        } catch (error) {
            console.warn("机器狗摄像头标定配置加载失败，将显示原始画面:", error);
            return createDisabledRobotCameraCalibration();
        }
    }

    function initializeRobotCameraRenderer(direction) {
        const canvas = ensureRobotCameraCanvas(direction);
        const video = getRobotCameraSourceVideo(direction);
        if (!canvas || !video || !window.FisheyeWebGLRenderer) {
            console.warn(`机器狗${direction}摄像头无法初始化 WebGL，回退原始视频。`);
            robotCameraRenderers[direction] = null;
            return;
        }

        try {
            const renderer = new window.FisheyeWebGLRenderer(
                canvas,
                robotCameraCalibrationConfig && robotCameraCalibrationConfig[direction],
                {
                    profile: robotCameraCorrectionMode,
                    correctionEnabled: robotCameraCorrectionMode !== "raw"
                }
            );
            renderer.setVideo(video);
            robotCameraRenderers[direction] = renderer;
        } catch (error) {
            robotCameraRenderers[direction] = null;
            console.warn(`机器狗${direction}摄像头 WebGL 初始化失败，回退原始视频:`, error);
        }
        updateRobotCameraLabels(direction);
    }

    function resizeRobotCameraCanvas(direction, force = false) {
        const renderer = robotCameraRenderers[direction];
        const canvas = robotCameraCanvases[direction];
        if (!renderer || !canvas || !canvas.parentElement) return;

        const now = performance.now();
        if (!force && now - robotCameraLastResizeChecks[direction] < 250) return;
        robotCameraLastResizeChecks[direction] = now;

        const rect = canvas.parentElement.getBoundingClientRect();
        if (!rect.width || !rect.height) return;

        const preview = robotCameraCalibrationConfig && robotCameraCalibrationConfig.preview || {};
        const maxWidth = Number(preview.maxWidth) > 0 ? Number(preview.maxWidth) : 1280;
        const maxHeight = Number(preview.maxHeight) > 0 ? Number(preview.maxHeight) : 720;
        const maxDpr = Number(preview.maxDevicePixelRatio) > 0
            ? Number(preview.maxDevicePixelRatio)
            : 1.5;
        const pixelRatio = Math.min(window.devicePixelRatio || 1, maxDpr);
        const video = getRobotCameraSourceVideo(direction);
        const aspect = video && video.videoWidth && video.videoHeight
            ? video.videoWidth / video.videoHeight
            : 16 / 9;

        let width = Math.max(rect.width * pixelRatio, rect.height * pixelRatio * aspect);
        let height = width / aspect;
        const scale = Math.min(1, maxWidth / width, maxHeight / height);
        width *= scale;
        height *= scale;
        renderer.resize(width, height);
    }

    function isRobotCameraPanelActive() {
        const active = window.currentActivePanel || "robot";
        return active === "robot" || active === "pathFault";
    }

    function renderRobotCameraFrame(direction, force = false) {
        const renderer = robotCameraRenderers[direction];
        if (!renderer || (!force && !isRobotCameraPanelActive())) return false;
        resizeRobotCameraCanvas(direction);
        const rendered = renderer.render();
        if (rendered) updateRobotCameraLabels(direction);
        return rendered;
    }

    function stopRobotCameraRenderLoop(direction) {
        robotCameraRenderLoopTokens[direction] += 1;
        const request = robotCameraFrameRequests[direction];
        if (!request) return;

        if (
            request.type === "video" &&
            request.video &&
            typeof request.video.cancelVideoFrameCallback === "function"
        ) {
            request.video.cancelVideoFrameCallback(request.id);
        } else if (request.type === "animation") {
            cancelAnimationFrame(request.id);
        }
        robotCameraFrameRequests[direction] = null;
    }

    function startRobotCameraRenderLoop(direction) {
        stopRobotCameraRenderLoop(direction);
        const video = getRobotCameraSourceVideo(direction);
        if (!video) return;
        const token = robotCameraRenderLoopTokens[direction];

        const scheduleNext = () => {
            if (token !== robotCameraRenderLoopTokens[direction]) return;
            if (typeof video.requestVideoFrameCallback === "function") {
                const id = video.requestVideoFrameCallback(() => {
                    if (token !== robotCameraRenderLoopTokens[direction]) return;
                    renderRobotCameraFrame(direction);
                    scheduleNext();
                });
                robotCameraFrameRequests[direction] = { type: "video", id, video };
            } else {
                const id = requestAnimationFrame(() => {
                    if (token !== robotCameraRenderLoopTokens[direction]) return;
                    renderRobotCameraFrame(direction);
                    scheduleNext();
                });
                robotCameraFrameRequests[direction] = { type: "animation", id, video };
            }
        };

        renderRobotCameraFrame(direction, true);
        scheduleNext();
    }

    function updateRobotCameraLabels(direction) {
        const renderer = robotCameraRenderers[direction];
        const baseLabel = direction === "front" ? "● 前视广角" : "● 后视广角";
        let suffix = " · 连接中";
        let title = "";

        if (renderer) {
            if (robotCameraCorrectionMode === "raw") {
                suffix = " · 原始";
            } else if (renderer.isCorrectionActive()) {
                suffix = robotCameraCorrectionMode === "wide" ? " · 广视野矫正" : " · 巡检矫正";
            } else {
                suffix = " · 原始（待标定）";
                title = renderer.getCorrectionReason();
            }
        } else if (getRobotCameraSourceVideo(direction)) {
            suffix = " · 原始（无WebGL）";
            title = "当前设备未启用 WebGL，已回退原始视频";
        }

        const stateKey = baseLabel + suffix + "|" + title;
        if (robotCameraLabelStates[direction] === stateKey) return;
        robotCameraLabelStates[direction] = stateKey;
        document.querySelectorAll(`[data-camera-label="${direction}"]`).forEach(label => {
            label.textContent = baseLabel;
            label.title = title || suffix.replace(/^\s*·\s*/, "");
        });
    }

    function setRobotCameraCorrectionMode(mode) {
        const normalizedMode = ["inspection", "wide", "raw"].includes(mode) ? mode : "inspection";
        robotCameraCorrectionMode = normalizedMode;
        ROBOT_CAMERA_DIRECTIONS.forEach(direction => {
            const renderer = robotCameraRenderers[direction];
            if (!renderer) return;
            renderer.setProfile(normalizedMode === "raw" ? "inspection" : normalizedMode);
            renderer.setCorrectionEnabled(normalizedMode !== "raw");
            renderRobotCameraFrame(direction, true);
            updateRobotCameraLabels(direction);
        });
        return robotCameraCorrectionMode;
    }

    function attachRobotCameraStream(direction, stream) {
        const video = getRobotCameraSourceVideo(direction);
        if (!video) return;

        video.srcObject = stream;
        video.onloadedmetadata = () => {
            resizeRobotCameraCanvas(direction, true);
            startRobotCameraRenderLoop(direction);
            updateRobotCameraLabels(direction);
        };

        const playPromise = video.play();
        if (playPromise && typeof playPromise.then === "function") {
            playPromise
                .then(() => startRobotCameraRenderLoop(direction))
                .catch(err => {
                    console.warn(`机器狗${direction}摄像头自动播放失败:`, err);
                });
        }
    }

    function closeRobotCameraReader(direction) {
        stopRobotCameraRenderLoop(direction);
        const reader = robotCameraReaders[direction];
        if (reader && typeof reader.close === "function") {
            reader.close();
        }
        robotCameraReaders[direction] = null;
        robotCameraStreams[direction] = null;
    }

    function startRobotCameraReader(direction) {
        const url = ROBOT_CAMERA_WHEP_URLS[direction];
        if (!url || !getRobotCameraSourceVideo(direction)) return;

        closeRobotCameraReader(direction);

        robotCameraReaders[direction] = new window.MediaMTXWebRTCReader({
            url,
            onTrack: evt => {
                if (!evt.track || evt.track.kind !== "video") return;

                const stream = evt.streams && evt.streams[0]
                    ? evt.streams[0]
                    : new MediaStream([evt.track]);

                robotCameraStreams[direction] = stream;
                attachRobotCameraStream(direction, stream);

                evt.track.onended = () => {
                    if (robotCameraStreams[direction] !== stream) return;
                    stopRobotCameraRenderLoop(direction);
                    robotCameraStreams[direction] = null;
                    console.warn(`机器狗${direction}摄像头视频轨道已结束`);
                };
            },
            onError: err => {
                console.warn(`机器狗${direction}摄像头 WHEP 连接异常:`, err);
            }
        });
    }

    async function initRobotWideCameraStreams() {
        if (!window.MediaMTXWebRTCReader) {
            console.error("MediaMTX reader.js 未加载，无法连接机器狗前后广角 WebRTC 视频。");
            return;
        }

        robotCameraCalibrationConfig = await loadRobotCameraCalibration();
        ROBOT_CAMERA_DIRECTIONS.forEach(direction => {
            ensureRobotCameraCanvas(direction);
            getRobotCameraSourceVideo(direction);
            initializeRobotCameraRenderer(direction);
        });
        moveRobotCameraOutputsToPanel(window.currentActivePanel || "robot");

        startRobotCameraReader("front");
        startRobotCameraReader("back");

        window.reconnectRobotWideCameraStreams = function () {
            startRobotCameraReader("front");
            startRobotCameraReader("back");
        };
        window.setRobotCameraCorrectionMode = setRobotCameraCorrectionMode;
        window.refreshRobotCameraOutputs = function () {
            moveRobotCameraOutputsToPanel(window.currentActivePanel || "robot");
        };
    }

    function captureRobotCameraFrame(direction, quality = 0.92) {
        const video = getRobotCameraSourceVideo(direction);

        if (
            !video ||
            video.readyState < HTMLMediaElement.HAVE_CURRENT_DATA ||
            video.videoWidth <= 0 ||
            video.videoHeight <= 0
        ) {
            throw new Error(direction === "front"
                ? "前摄像头画面尚未就绪"
                : "后摄像头画面尚未就绪");
        }

        const renderer = robotCameraRenderers[direction];
        if (renderer) {
            return renderer.captureDataUrl(video.videoWidth, video.videoHeight, quality);
        }

        // WebGL 不可用时保留原始抓拍能力。
        const canvas = document.createElement("canvas");
        canvas.width = video.videoWidth;
        canvas.height = video.videoHeight;

        const context = canvas.getContext("2d");
        if (!context) {
            throw new Error("浏览器无法创建抓拍画布");
        }

        context.drawImage(video, 0, 0, canvas.width, canvas.height);
        return canvas.toDataURL("image/jpeg", quality);
    }

    async function captureFaultRobotCamera(direction, label) {
        if (!window.lastFaultMarker || !window.lastFaultMarker.userData.faultId) {
            alert(`⚠️ 当前没有检测到破损点，无法进行${label}抓拍绑定！`);
            return;
        }

        const faultId = window.lastFaultMarker.userData.faultId;

        try {
            console.log(`📸 开始截取机器狗【${label}】摄像头画面...`);
            const imageBase64 = captureRobotCameraFrame(direction);

            if (!window.faultImagesMap.has(faultId)) {
                window.faultImagesMap.set(faultId, { left: "", right: "", front: "", back: "" });
            }

            await bindCapturedImageToFault(faultId, direction, imageBase64);
            console.log(`✅ ${label}抓拍完成并已绑定`);
        } catch (err) {
            console.error(`${label}抓拍失败:`, err);
            alert(`${label}抓拍失败：${err.message || err}`);
        }
    }


    // ==========================================
    // ✅ 三维重建页：从“路径查找及故障定点”三维视图加载当前点云地图
    // 说明：路径页采用 Z 轴向上，GPR 三维重建页采用 Y 轴向上，因此导入时统一做坐标变换：
    //      路径页 (x, y, z)  ->  三维重建页 (x, z, y)
    // ==========================================
    function disposeThreeObjectForReuse(obj) {
        if (!obj) return;

        obj.traverse(child => {
            if (child.geometry) {
                child.geometry.dispose();
            }

            if (child.material) {
                const mats = Array.isArray(child.material) ? child.material : [child.material];
                mats.forEach(mat => {
                    if (!mat) return;
                    if (mat.map) mat.map.dispose();
                    mat.dispose();
                });
            }
        });
    }

    function clearCurrentGprExternalMapLayer() {
        if (window.gprScene && window.currentPcdGroup) {
            window.gprScene.remove(window.currentPcdGroup);
            disposeThreeObjectForReuse(window.currentPcdGroup);
        } else if (window.gprScene && window.currentPcdMesh) {
            window.gprScene.remove(window.currentPcdMesh);
            disposeThreeObjectForReuse(window.currentPcdMesh);
        }

        window.currentPcdMesh = null;
        window.currentPcdGroup = null;
        window.currentImportedInspectionMapGroup = null;
        window.gprDataWorldGroup = null;
        window.gprDepthDisplayScale = 1;
        window.gprHasInspectionMapLayerLoaded = false;
    }

    function clearGpr2DCanvasById(canvasId) {
        const canvas = $(canvasId);
        if (!canvas) return;
        const ctx = canvas.getContext && canvas.getContext("2d");
        if (ctx) {
            ctx.clearRect(0, 0, canvas.width || 0, canvas.height || 0);
        }
        // 重新设置尺寸可以同步清掉浏览器保留的旧图像缓存。
        canvas.width = canvas.width || 1;
        canvas.height = canvas.height || 1;
        delete gpr2DRenderInfo[canvasId];
    }

    function clearGprReconstructionSceneCompletely() {
        const container = $("threeContainer");

        // 1) 停止旧 Three.js 动画循环，防止清空后仍继续 render 旧场景。
        if (window.gprAnimId) {
            cancelAnimationFrame(window.gprAnimId);
            window.gprAnimId = null;
        }

        // 2) 关闭弹窗和临时交互提示。
        try { if (typeof hideGprPipeInfoPopup === "function") hideGprPipeInfoPopup(); } catch (e) { }
        try { if (typeof hideManualScanTip === "function") hideManualScanTip(); } catch (e) { }

        // 3) 释放整套三维重建场景资源：点云、地下环境盒体、GPR剖面、管道、坐标轴、扫描路径。
        if (window.gprScene) {
            disposeThreeObjectForReuse(window.gprScene);
            // 从 scene.children 中移除对象，避免 Three.js 仍保留旧层级引用。
            while (window.gprScene.children && window.gprScene.children.length) {
                window.gprScene.remove(window.gprScene.children[0]);
            }
        }

        if (window.gprRenderer) {
            try { window.gprRenderer.dispose(); } catch (e) { }
            if (typeof window.gprRenderer.forceContextLoss === "function") {
                try { window.gprRenderer.forceContextLoss(); } catch (e) { }
            }
            if (window.gprRenderer.domElement && window.gprRenderer.domElement.parentNode) {
                window.gprRenderer.domElement.parentNode.removeChild(window.gprRenderer.domElement);
            }
        }

        if (container) {
            container.innerHTML = "";
        }

        // 4) 清空 3D 全局引用。
        window.gprScene = null;
        window.gprCamera = null;
        window.gprRenderer = null;
        window.gprSetViewToBox = null;
        window.gprOrbitTarget = null;

        window.currentPcdMesh = null;
        window.currentPcdGroup = null;
        window.currentImportedInspectionMapGroup = null;
        window.gprHasInspectionMapLayerLoaded = false;

        window.gprScanPlanGroup = null;
        window.gprManualScanPickGroup = null;
        window.gprSlicesGroup = null;
        window.gprAxesGroup = null;
        window.gprZPathLine = null;
        window.boxFaces = null;
        window.boxCenter = null;

        window.gprPipePickObjects = [];
        window.gprCablePathAlignmentStats = null;
        selectedGprPipeObject = null;
        gprPipePopup3DPos = null;
        annotation3DGroups = [];
        updateGprCableAlignmentBadge("idle", "路径校正：待加载");

        // 5) 清空本次 GPR 处理结果和二维剖面显示，避免旧结果按钮再次触发旧管道重绘。
        gprResult = null;
        currentGprSegmentIndex = 0;
        manualMigAnnotationMode = false;
        forcedPipeAnnotationMode = false;
        activeForcedPipeGroupId = null;
        forcedPipeGroupCounter = 0;
        forcedPipeAnnotationOrder = 0;
        manualMigAnnotationsBySegment.clear();
        forcedPipeAnnotationsBySegment.clear();
        gprPipeColorAssignments.clear();

        clearGpr2DCanvasById("raw2dCanvas");
        clearGpr2DCanvasById("mig2dCanvas");

        const segmentSelect = $("segmentSelect");
        if (segmentSelect) {
            segmentSelect.innerHTML = "";
            segmentSelect.disabled = true;
            segmentSelect.onchange = null;
        }

        const btnAnno = $("btnToggleAnnotation");
        if (btnAnno) btnAnno.style.display = "none";
        try { if (typeof setAnnotationButtonState === "function") setAnnotationButtonState(false); } catch (e) { }
        try { if (typeof updateManualMigAnnotationUI === "function") updateManualMigAnnotationUI(); } catch (e) { }
        try { if (typeof updateForcedPipeAnnotationUI === "function") updateForcedPipeAnnotationUI(); } catch (e) { }

        // 6) 清空规划数据，避免下次处理 GPR 时继续沿用已经被用户清掉的旧扫描路径。
        window.currentGprScanPlan = null;
        window.currentGprScanPlanSavedAt = 0;
    }

    function clearGprPointCloudMapFromPage() {
        const hadAnyContent = !!(
            window.currentPcdGroup ||
            window.currentPcdMesh ||
            window.currentImportedInspectionMapGroup ||
            window.gprHasInspectionMapLayerLoaded ||
            window.gprScene ||
            window.gprSlicesGroup ||
            window.gprAxesGroup ||
            (window.gprPipePickObjects && window.gprPipePickObjects.length) ||
            gprResult
        );

        clearGprReconstructionSceneCompletely();
        resetGprLoadedMapFreezeState();
        window.loadedPointCloudGeoMetadata = null;
        window.loadedPointCloudGeoSource = "";

        if (hadAnyContent) {
            setStatus("✅ 已清除三维重建页面中的点云地图、地下环境、GPR剖面和管道渲染。", "success");
            console.log("🧹 已清除三维重建页面全部三维显示内容：点云地图、地下环境、GPR剖面、管道、坐标轴和扫描路径。");
        } else {
            setStatus("当前三维重建页面没有已加载的点云地图或地下渲染内容。", "warning");
        }
    }
    window.clearGprPointCloudMapFromPage = clearGprPointCloudMapFromPage;

    // ==========================================
    // ✅ 三维重建页点云地图冻结与恢复
    // 说明：GPR 三维渲染会重建 threeContainer / scene，原来的点云对象会被销毁。
    // 因此无论加载的是“当前点云地图”还是“本地点云文件”，都必须保存一份冻结快照，
    // 渲染 GPR 剖面后再从快照恢复，避免重新读取实时点云导致地图自动变成扫描后的最新地图。
    // ==========================================
    window.gprLoadedMapSource = "";                 // "current" | "local" | ""
    window.gprFrozenLocalPcdSnapshot = null;        // 本地点云文件冻结快照

    function resetGprLoadedMapFreezeState() {
        window.gprLoadedMapSource = "";
        window.gprFrozenLocalPcdSnapshot = null;
        window.latestImportedInspectionMapSnapshot = null;
    }

    function clonePlainObjectForGpr(value) {
        if (value === null || value === undefined) return value;
        try {
            if (typeof structuredClone === "function") return structuredClone(value);
        } catch (e) {
            // 某些浏览器对 Three.js 对象或特殊字段 structuredClone 会失败，继续走 JSON 兜底。
        }
        try {
            return JSON.parse(JSON.stringify(value));
        } catch (e) {
            return value;
        }
    }

    function cloneInspectionMapSnapshotForGpr(snapshot) {
        if (!snapshot || typeof snapshot !== "object") return snapshot;

        const pointCloud = snapshot.pointCloud || {};
        const cloned = {
            ...snapshot,
            pointCloud: {
                ...pointCloud,
                count: Number(pointCloud.count || 0),
                positions: pointCloud.positions ? new Float32Array(pointCloud.positions) : new Float32Array(0),
                colors: pointCloud.colors ? new Float32Array(pointCloud.colors) : new Float32Array(0)
            },
            cablePath: Array.isArray(snapshot.cablePath)
                ? snapshot.cablePath.map(p => clonePlainObjectForGpr(p))
                : [],
            faults: Array.isArray(snapshot.faults)
                ? snapshot.faults.map(f => clonePlainObjectForGpr(f))
                : []
        };

        cloned.__frozenForGpr = true;
        cloned.__frozenAt = Date.now();
        return cloned;
    }

    function getVectorSnapshot(v, fallback = { x: 0, y: 0, z: 0 }) {
        return {
            x: Number(v && Number.isFinite(v.x) ? v.x : fallback.x),
            y: Number(v && Number.isFinite(v.y) ? v.y : fallback.y),
            z: Number(v && Number.isFinite(v.z) ? v.z : fallback.z)
        };
    }

    function applyVectorSnapshot(target, value) {
        if (!target || !value) return;
        target.set(Number(value.x || 0), Number(value.y || 0), Number(value.z || 0));
    }

    function freezeLocalPcdMapForGpr(points, pcdGroup, sourceName, geoMeta = null) {
        if (!points || !points.geometry || !pcdGroup) return null;

        const geom = points.geometry;
        const posAttr = geom.getAttribute && geom.getAttribute("position");
        if (!posAttr || !posAttr.array || posAttr.count <= 0) return null;

        const colAttr = geom.getAttribute && geom.getAttribute("color");
        const hasColors = !!(colAttr && colAttr.array && colAttr.array.length >= posAttr.array.length);

        const snapshot = {
            version: 1,
            sourceName: sourceName || pcdGroup.name || "本地点云文件",
            pointCount: posAttr.count,
            positions: new Float32Array(posAttr.array),
            colors: hasColors ? new Float32Array(colAttr.array) : null,
            hasColors,
            geoMeta: geoMeta ? clonePlainObjectForGpr(geoMeta) : null,
            cablePath: extractCablePathFromGprGeoMetadata(geoMeta),
            group: {
                position: getVectorSnapshot(pcdGroup.position),
                rotation: getVectorSnapshot(pcdGroup.rotation),
                scale: getVectorSnapshot(pcdGroup.scale, { x: 1, y: 1, z: 1 })
            },
            points: {
                position: getVectorSnapshot(points.position),
                rotation: getVectorSnapshot(points.rotation),
                scale: getVectorSnapshot(points.scale, { x: 1, y: 1, z: 1 })
            },
            material: {
                size: Number(points.material && Number.isFinite(points.material.size) ? points.material.size : 0.05),
                sizeAttenuation: points.material ? points.material.sizeAttenuation !== false : true,
                vertexColors: !!hasColors
            },
            frozenAt: Date.now()
        };

        window.gprLoadedMapSource = "local";
        window.gprFrozenLocalPcdSnapshot = snapshot;
        window.latestImportedInspectionMapSnapshot = null;
        return snapshot;
    }

    function restoreFrozenLocalPcdMapToGprScene(options = {}) {
        const snapshot = window.gprFrozenLocalPcdSnapshot;
        if (!snapshot || !snapshot.positions || !snapshot.positions.length) {
            console.warn("⚠️ 未找到可恢复的本地点云冻结快照，已跳过恢复本地点云。");
            return false;
        }

        const fitCamera = options.fitCamera !== false;
        ensureStandaloneGprSceneForMap();
        clearCurrentGprExternalMapLayer();

        const geom = new THREE.BufferGeometry();
        geom.setAttribute("position", new THREE.BufferAttribute(new Float32Array(snapshot.positions), 3));
        if (snapshot.hasColors && snapshot.colors) {
            geom.setAttribute("color", new THREE.BufferAttribute(new Float32Array(snapshot.colors), 3));
        }
        geom.computeBoundingBox();

        const mat = new THREE.PointsMaterial({
            vertexColors: !!snapshot.hasColors,
            size: Number(snapshot.material && snapshot.material.size ? snapshot.material.size : 0.05),
            sizeAttenuation: !(snapshot.material && snapshot.material.sizeAttenuation === false)
        });

        const points = new THREE.Points(geom, mat);
        points.frustumCulled = false;
        applyVectorSnapshot(points.position, snapshot.points && snapshot.points.position);
        applyVectorSnapshot(points.rotation, snapshot.points && snapshot.points.rotation);
        applyVectorSnapshot(points.scale, snapshot.points && snapshot.points.scale);

        const pcdGroup = new THREE.Group();
        pcdGroup.name = snapshot.sourceName || "本地点云文件";
        pcdGroup.add(points);

        const cableMesh = createGprSurfaceCablePathMesh(snapshot.cablePath);
        if (cableMesh) pcdGroup.add(cableMesh);
        applyVectorSnapshot(pcdGroup.position, snapshot.group && snapshot.group.position);
        applyVectorSnapshot(pcdGroup.rotation, snapshot.group && snapshot.group.rotation);
        applyVectorSnapshot(pcdGroup.scale, snapshot.group && snapshot.group.scale);

        if (snapshot.geoMeta) {
            pcdGroup.userData.geoMetadata = snapshot.geoMeta;
            applyLoadedMapGeoMetadata(snapshot.geoMeta, snapshot.sourceName || "本地点云文件");
        }

        window.gprScene.add(pcdGroup);
        window.currentPcdMesh = points;
        window.currentPcdGroup = pcdGroup;
        window.currentImportedInspectionMapGroup = pcdGroup;
        window.gprHasInspectionMapLayerLoaded = true;
        window.gprLoadedMapSource = "local";
        window.latestImportedInspectionMapSnapshot = null;

        if (fitCamera) {
            const planLikeSpan = Math.max(DEFAULT_SCAN_PARAMS.length, DEFAULT_SCAN_PARAMS.width, 2.5);
            const mapOriginCenter = pcdGroup.localToWorld(new THREE.Vector3(0, 0, 0));
            const planLikeBox = new THREE.Box3(
                new THREE.Vector3(mapOriginCenter.x - planLikeSpan / 2, mapOriginCenter.y - 0.25, mapOriginCenter.z - planLikeSpan / 2),
                new THREE.Vector3(mapOriginCenter.x + planLikeSpan / 2, mapOriginCenter.y + 0.25, mapOriginCenter.z + planLikeSpan / 2)
            );
            updateGprCameraByBox(planLikeBox);
        }

        console.log("✅ 已从冻结快照恢复本地点云地图：", {
            sourceName: snapshot.sourceName,
            pointCount: snapshot.pointCount,
            hasGeoMeta: !!snapshot.geoMeta
        });
        return true;
    }

    window.restoreFrozenLocalPcdMapToGprScene = restoreFrozenLocalPcdMapToGprScene;

    function updateGprCameraByBox(box) {
        if (!box || box.isEmpty() || !window.gprCamera) return;

        if (typeof window.gprSetViewToBox === "function") {
            window.gprSetViewToBox(box);
            return;
        }

        const center = new THREE.Vector3();
        const size = new THREE.Vector3();
        box.getCenter(center);
        box.getSize(size);

        const span = Math.max(size.x, size.y, size.z, 1.0);
        window.gprCamera.position.set(
            center.x + span * 1.6,
            center.y + span * 1.2,
            center.z + span * 1.6
        );
        window.gprCamera.lookAt(center);
        window.gprCamera.updateProjectionMatrix();
    }

    function ensureStandaloneGprSceneForMap() {
        const container = $("threeContainer");
        if (!container) {
            throw new Error("未找到三维重建容器 threeContainer。");
        }
        if (typeof THREE === "undefined") {
            throw new Error("three.js 未加载，无法创建三维场景。");
        }

        if (window.gprScene && window.gprRenderer && window.gprCamera) {
            return;
        }

        if (window.gprAnimId) {
            cancelAnimationFrame(window.gprAnimId);
            window.gprAnimId = null;
        }

        if (window.gprRenderer) {
            window.gprRenderer.dispose();
            if (typeof window.gprRenderer.forceContextLoss === "function") {
                window.gprRenderer.forceContextLoss();
            }
            window.gprRenderer = null;
        }

        container.innerHTML = "";

        let width = container.clientWidth || 800;
        let height = container.clientHeight || 500;
        if (!container.clientWidth || !container.clientHeight) {
            container.style.minWidth = "800px";
            container.style.minHeight = "500px";
        }

        const scene = new THREE.Scene();
        scene.background = new THREE.Color(0x000000);

        const camera = new THREE.PerspectiveCamera(45, width / height, 0.01, 10000);
        // ✅ 三维重建页采用 Y 轴向上，这里显式固定 camera.up，避免平移/旋转过程中的轴向漂移。
        camera.up.set(0, 1, 0);

        const renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(width, height);
        container.appendChild(renderer.domElement);

        const worldGrid = new THREE.GridHelper(50, 50, 0x444444, 0x222222);
        worldGrid.position.y = 0;
        scene.add(worldGrid);

        // 简单方向参考：红 X、绿 Y(高度)、蓝 Z
        const axes = new THREE.AxesHelper(3);
        scene.add(axes);

        // ✅ 与“路径查找及故障定点”页面保持一致：
        // orbitTarget 是真正的视角中心。中键/右键平移时移动 orbitTarget，
        // 左键旋转时始终围绕最新 orbitTarget 旋转。
        const orbitTarget = new THREE.Vector3(0, 0, 0);
        let radius = 20;
        let theta = Math.PI / 4;
        let phi = Math.PI / 3.5;

        // ✅ 根据点云地图整体尺寸动态控制缩放范围
        let controlSpan = 20;

        function updateCamera() {
            // 路径页是 Z 轴向上：x/y 为水平面，z 为高度；
            // 三维重建页是 Y 轴向上：x/z 为水平面，y 为高度。
            const x = orbitTarget.x + radius * Math.sin(phi) * Math.cos(theta);
            const y = orbitTarget.y + radius * Math.cos(phi);
            const z = orbitTarget.z + radius * Math.sin(phi) * Math.sin(theta);

            camera.up.set(0, 1, 0);
            camera.position.set(x, y, z);
            camera.lookAt(orbitTarget);
        }

        window.gprSetViewToBox = function (box) {
            if (!box || box.isEmpty()) return;

            const size = new THREE.Vector3();
            box.getCenter(orbitTarget);
            box.getSize(size);

            const span = Math.max(size.x, size.y, size.z, 1.0);
            controlSpan = span;

            // 初始视角拉开到能完整看到地图，后续滚轮仍允许继续放大/缩小。
            radius = Math.max(span * 1.8, 3.0);
            theta = Math.PI / 4;
            phi = Math.PI / 3.5;

            updateCamera();
        };
        updateCamera();

        const dom = renderer.domElement;
        let isRotating = false;
        let isPanning = false;
        let prevPos = { x: 0, y: 0 };
        let clickStartX = 0;
        let clickStartY = 0;
        let hasDraggedForView = false;

        dom.addEventListener("mousedown", e => {
            e.preventDefault();

            if (e.button === 0) {
                // 左键旋转
                isRotating = true;
                isPanning = false;
            } else if (e.button === 1 || e.button === 2) {
                // 中键/右键平移
                isPanning = true;
                isRotating = false;
            } else {
                return;
            }

            prevPos = { x: e.offsetX, y: e.offsetY };
            clickStartX = e.clientX;
            clickStartY = e.clientY;
            hasDraggedForView = false;
        });

        window.addEventListener("mouseup", () => {
            isRotating = false;
            isPanning = false;
        });

        dom.addEventListener("contextmenu", e => e.preventDefault());

        dom.addEventListener("click", e => {
            if (hasDraggedForView) return;
            handleGprPipeClick(e, camera, dom);
        });

        dom.addEventListener("mousemove", e => {
            const dx = e.offsetX - prevPos.x;
            const dy = e.offsetY - prevPos.y;
            prevPos = { x: e.offsetX, y: e.offsetY };

            if (Math.hypot(e.clientX - clickStartX, e.clientY - clickStartY) > 4) {
                hasDraggedForView = true;
            }

            if (isRotating) {
                // ✅ 与路径查找页保持一致：左右拖动改变水平角，上下拖动改变俯仰角。
                theta += dx * 0.01;
                phi -= dy * 0.01;
                phi = Math.max(0.1, Math.min(Math.PI / 2 - 0.1, phi));
                updateCamera();
            } else if (isPanning) {
                // ✅ 与路径查找页一致：平移只改变水平面的 orbitTarget，
                // 不让目标点沿高度方向漂移，这样后续左键旋转会围绕当前视图中心旋转。
                const speed = Math.max(radius * 0.002, 0.001);

                const forward = new THREE.Vector3();
                camera.getWorldDirection(forward);

                const right = new THREE.Vector3()
                    .crossVectors(forward, camera.up)
                    .normalize();

                const forwardXZ = new THREE.Vector3(forward.x, 0, forward.z);
                if (forwardXZ.lengthSq() > 1e-8) {
                    forwardXZ.normalize();
                    orbitTarget.addScaledVector(right, -dx * speed);
                    orbitTarget.addScaledVector(forwardXZ, dy * speed);
                } else {
                    // 极端俯视角下的兜底处理
                    const camUp = new THREE.Vector3(0, 0, 1);
                    orbitTarget.addScaledVector(right, -dx * speed);
                    orbitTarget.addScaledVector(camUp, dy * speed);
                }

                updateCamera();
            }

            updateGprPipeHoverCursor(e, camera, dom, isRotating || isPanning);
        });

        dom.addEventListener("mouseleave", () => resetGprPipeHoverCursor(dom));

        dom.addEventListener("wheel", e => {
            e.preventDefault();

            radius *= (e.deltaY > 0 ? 1.1 : 0.9);

            // 动态范围：小点云可继续放大，大点云可继续拉远。
            const minRadius = 0.02;
            const maxRadius = Math.max(controlSpan * 80.0, 1000.0);
            radius = Math.max(minRadius, Math.min(maxRadius, radius));

            updateCamera();
        });

        function resizeGprScene() {
            const w = container.clientWidth || 800;
            const h = container.clientHeight || 500;
            camera.aspect = w / h;
            camera.updateProjectionMatrix();
            renderer.setSize(w, h);
        }
        window.addEventListener("resize", resizeGprScene);
        if (typeof ResizeObserver !== "undefined") {
            const resizeObserver = new ResizeObserver(resizeGprScene);
            resizeObserver.observe(container);
        }

        function animateStandaloneGprScene() {
            window.gprAnimId = requestAnimationFrame(animateStandaloneGprScene);
            if (window.currentActivePanel !== "gpr") return;
            renderer.render(scene, camera);
        }
        animateStandaloneGprScene();

        window.gprScene = scene;
        window.gprCamera = camera;
        window.gprRenderer = renderer;
        window.gprOrbitTarget = orbitTarget;
    }

    function slamPointToGprVector(pt) {
        // 路径页坐标：X/Y 为水平面，Z 为高度。
        // 三维重建页坐标：X/Z 为水平面，Y 为高度。
        // 注意：单纯 (x, z, y) 会改变坐标系手性，表现为镜像；这里使用 -y 保持方向一致。
        return new THREE.Vector3(
            Number(pt.x || 0),
            Number(pt.z ?? 0.08),
            -Number(pt.y || 0)
        );
    }

    function normalizePlainCablePathForGpr(path) {
        if (!Array.isArray(path)) return [];

        const result = [];
        path.forEach(pt => {
            const x = Number(pt?.x);
            const y = Number(pt?.y);
            const z = Number(pt?.z ?? 0.08);
            if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) return;

            const item = {
                x,
                y,
                z,
                depth: pt?.userData ? (pt.userData.depth ?? null) : (pt?.depth ?? null)
            };
            const prev = result[result.length - 1];
            if (!prev || Math.hypot(item.x - prev.x, item.y - prev.y) > 1e-5) {
                result.push(item);
            }
        });
        return result;
    }

    function extractCablePathFromGprGeoMetadata(meta) {
        if (!meta || typeof meta !== "object") return [];
        const path = meta.inspectionOverlays?.cablePath ||
            meta.overlays?.cablePath ||
            meta.cablePath ||
            [];
        return normalizePlainCablePathForGpr(path);
    }

    function createGprSurfaceCablePathMesh(path) {
        const plainPath = normalizePlainCablePathForGpr(path);
        if (plainPath.length < 2 || typeof THREE === "undefined") return null;

        const cablePoints = plainPath.map(slamPointToGprVector);
        const cableCurve = new THREE.CatmullRomCurve3(cablePoints, false, "centripetal", 0.35);
        const cableTube = new THREE.TubeGeometry(
            cableCurve,
            Math.max(cablePoints.length * 8, 16),
            0.045,
            10,
            false
        );
        const cableMesh = new THREE.Mesh(
            cableTube,
            new THREE.MeshBasicMaterial({ color: 0xffff00 })
        );
        cableMesh.name = "地上黄色电缆路径";
        cableMesh.userData.isGprSurfaceCablePath = true;
        cableMesh.userData.slamCablePath = plainPath;
        return cableMesh;
    }

    function updateGprCableAlignmentBadge(state, text, title = "") {
        const badge = $("gprCableAlignmentBadge");
        if (!badge) return;

        badge.classList.remove("pending", "aligned", "unavailable", "unmatched");
        if (state && state !== "idle") badge.classList.add(state);
        badge.textContent = text || "路径校正：待加载";
        badge.title = title || badge.textContent;
    }

    function createGprFaultMarker(fault) {
        const faultGroup = new THREE.Group();
        const pos = slamPointToGprVector(fault);
        faultGroup.position.copy(pos);
        faultGroup.userData.faultId = fault.faultId || fault.id || "";

        const sphere = new THREE.Mesh(
            new THREE.SphereGeometry(0.14, 16, 16),
            new THREE.MeshBasicMaterial({ color: 0xff0000 })
        );
        faultGroup.add(sphere);

        const canvas = document.createElement("canvas");
        canvas.width = 128;
        canvas.height = 128;
        const ctx = canvas.getContext("2d");
        ctx.clearRect(0, 0, 128, 128);
        ctx.font = "bold 86px Arial";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillStyle = "#ffdd00";
        ctx.shadowColor = "#ff0000";
        ctx.shadowBlur = 14;
        ctx.fillText("⚡", 64, 64);

        const texture = new THREE.CanvasTexture(canvas);
        const sprite = new THREE.Sprite(new THREE.SpriteMaterial({
            map: texture,
            transparent: true,
            depthTest: false
        }));
        sprite.position.set(0, 0.45, 0);
        sprite.scale.set(0.65, 0.65, 1);
        faultGroup.add(sprite);

        return faultGroup;
    }

    function loadCurrentPathFaultMapToGpr(options = {}) {
        // 默认手动点击“加载当前点云地图”时自动调整视角；
        // GPR 剖面渲染完成后重叠加地图时传入 { fitCamera: false }，避免相机被拉回整张点云地图，导致地下剖面/管道看起来消失、拖动变生硬。
        const fitCamera = options.fitCamera !== false;
        const btn = $("btnLoadCurrentPointCloudMap");
        const oldText = btn ? btn.textContent : "";

        try {
            if (btn) {
                btn.disabled = true;
                btn.classList.remove("loaded");
                btn.textContent = "⏳ 正在加载地图...";
            }

            if (typeof window.exportCurrentPathFaultMapForGpr !== "function") {
                throw new Error("路径查找及故障定点页面的三维地图尚未初始化，请先进入一次该页面。 ");
            }

            const rawSnapshot = window.exportCurrentPathFaultMapForGpr();
            const snapshot = cloneInspectionMapSnapshotForGpr(rawSnapshot);
            const pointCount = snapshot?.pointCloud?.count || 0;
            const cableCount = Array.isArray(snapshot?.cablePath) ? snapshot.cablePath.length : 0;
            const faultCount = Array.isArray(snapshot?.faults) ? snapshot.faults.length : 0;

            if (pointCount === 0 && cableCount < 2 && faultCount === 0) {
                throw new Error("当前路径查找页面还没有可加载的点云地图、电缆路径或破损点。 ");
            }

            ensureStandaloneGprSceneForMap();
            clearCurrentGprExternalMapLayer();

            const group = new THREE.Group();
            group.name = "当前点云地图_电缆路径_破损点";

            if (pointCount > 0) {
                const geom = new THREE.BufferGeometry();
                geom.setAttribute("position", new THREE.BufferAttribute(snapshot.pointCloud.positions, 3));
                geom.setAttribute("color", new THREE.BufferAttribute(snapshot.pointCloud.colors, 3));

                const mat = new THREE.PointsMaterial({
                    vertexColors: true,
                    size: 0.035,
                    sizeAttenuation: true
                });

                const points = new THREE.Points(geom, mat);
                points.frustumCulled = false;
                group.add(points);
                window.currentPcdMesh = points;
            }

            if (cableCount >= 2) {
                const cableMesh = createGprSurfaceCablePathMesh(snapshot.cablePath);
                if (cableMesh) group.add(cableMesh);
            }

            if (faultCount > 0) {
                snapshot.faults.forEach(fault => {
                    group.add(createGprFaultMarker(fault));
                });
            }

            window.gprScene.add(group);
            window.currentPcdGroup = group;
            window.currentImportedInspectionMapGroup = group;
            window.gprHasInspectionMapLayerLoaded = true;
            window.gprLoadedMapSource = "current";
            window.gprFrozenLocalPcdSnapshot = null;
            window.latestImportedInspectionMapSnapshot = snapshot;
            updateGprCableAlignmentBadge(
                cableCount >= 2 ? "pending" : "unavailable",
                cableCount >= 2 ? "路径校正：待匹配" : "路径校正：无黄色路径",
                cableCount >= 2
                    ? `已加载 ${cableCount} 个黄色电缆路径点；生成地下管道后将自动匹配校正。`
                    : "当前地图快照中没有可用于地下管道校正的黄色电缆路径。"
            );

            // ✅ 当前点云地图单独加载完成后，不再使用整幅点云/电缆/破损点包围盒取景。
            // 原来的 setFromObject(group) 会把整张地图范围作为控制范围，导致 radius / controlSpan 过大，
            // 进而让拖拽和平移手感变得生硬。
            // 这里改成本地点云加载后的同款逻辑：以当前地图自身坐标原点/中心原点为中心，
            // 构造一个接近扫描区域大小的小 box，再调用 updateGprCameraByBox()。
            if (fitCamera) {
                const planLikeSpan = Math.max(DEFAULT_SCAN_PARAMS.length, DEFAULT_SCAN_PARAMS.width, 2.5);
                const mapOriginCenter = group.localToWorld(new THREE.Vector3(0, 0, 0));
                const planLikeBox = new THREE.Box3(
                    new THREE.Vector3(mapOriginCenter.x - planLikeSpan / 2, mapOriginCenter.y - 0.25, mapOriginCenter.z - planLikeSpan / 2),
                    new THREE.Vector3(mapOriginCenter.x + planLikeSpan / 2, mapOriginCenter.y + 0.25, mapOriginCenter.z + planLikeSpan / 2)
                );
                updateGprCameraByBox(planLikeBox);
                console.log("✅ 当前点云地图已按地图中心原点构造小 box 触发相机控制：", {
                    center: {
                        x: Number(mapOriginCenter.x.toFixed(3)),
                        y: Number(mapOriginCenter.y.toFixed(3)),
                        z: Number(mapOriginCenter.z.toFixed(3))
                    },
                    span: planLikeSpan
                });
            }

            setStatus(`✅ 当前点云地图已加载：点云 ${pointCount} 点，电缆路径 ${cableCount} 点，破损点 ${faultCount} 个。`, "success");
            addSystemMessage("当前点云地图已加载。", "success");

            if (btn) {
                btn.classList.add("loaded");
                btn.textContent = "✅ 当前地图已加载";
                setTimeout(() => {
                    if (btn) btn.textContent = "🗺️ 加载当前点云地图";
                }, 1800);
            }

        } catch (err) {
            console.error("加载当前点云地图失败：", err);
            alert("❌ " + err.message);
            setStatus("加载当前点云地图失败：" + err.message, "error");
        } finally {
            if (btn) {
                btn.disabled = false;
                if (!btn.classList.contains("loaded")) {
                    btn.textContent = oldText || "🗺️ 加载当前点云地图";
                }
            }
        }
    }

    // ==========================================
    // ✅ 巡检结果正式 B/S 部署接口
    // ==========================================
    const INSPECTION_API = {
        uploadImage: "/api/inspection/images/upload-base64",
        saveResult: "/api/inspection/results",
        listResults: "/api/inspection/results?limit=5",
        detail: (id) => "/api/inspection/results/" + encodeURIComponent(id)
    };

    function getFullImageUrl(url) {
        if (!url) return "/img/placeholder.png";
        if (url.startsWith("http://") || url.startsWith("https://")) return url;
        if (url.startsWith("/")) return url;
        return "/" + url;
    }

    function ensureFaultImageItem(faultId) {
        if (!window.faultImagesMap) {
            window.faultImagesMap = new Map();
        }

        if (!window.faultImagesMap.has(faultId)) {
            window.faultImagesMap.set(faultId, {
                left: "",
                right: "",
                front: "",
                back: ""
            });
        }

        return window.faultImagesMap.get(faultId);
    }

    async function uploadFaultImageBase64(faultId, direction, imageBase64) {
        if (!imageBase64) return "";

        // 已经是 URL 的，不重复上传
        if (
            !imageBase64.startsWith("data:") &&
            (
                imageBase64.startsWith("/") ||
                imageBase64.startsWith("http://") ||
                imageBase64.startsWith("https://")
            )
        ) {
            return imageBase64;
        }

        const res = await fetch(INSPECTION_API.uploadImage, {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({
                faultId: faultId,
                direction: direction,
                imageBase64: imageBase64
            })
        });

        if (!res.ok) {
            throw new Error("图片上传失败：" + res.status);
        }

        const data = await res.json();

        if (!data.url) {
            throw new Error("后端未返回图片 URL");
        }

        return data.url;
    }

    function setFaultImageUrl(faultId, direction, url) {
        const imgs = ensureFaultImageItem(faultId);
        imgs[direction] = url;
        return imgs;
    }

    function updateCaptureImageUI(direction, url) {
        const idMap = {
            front: ["imgCaptureFront", "dispCaptureFront"],
            back: ["imgCaptureBack", "dispCaptureBack"],
            left: ["imgCaptureLeft", "dispCaptureLeft"],
            right: ["imgCaptureRight", "dispCaptureRight"]
        };

        const ids = idMap[direction] || [];
        ids.forEach(id => {
            const el = document.getElementById(id);
            if (el) {
                el.src = getFullImageUrl(url);
            }
        });
    }

    function showFaultImagesInModal(imgs) {
        const safeImgs = imgs || {};

        updateCaptureImageUI("front", safeImgs.front || "");
        updateCaptureImageUI("back", safeImgs.back || "");
        updateCaptureImageUI("left", safeImgs.left || "");
        updateCaptureImageUI("right", safeImgs.right || "");

        const modal = document.getElementById("captureImageModal");
        if (modal) {
            modal.style.display = "block";
        }
    }

    async function bindCapturedImageToFault(faultId, direction, imageBase64) {
        const imageUrl = await uploadFaultImageBase64(faultId, direction, imageBase64);
        setFaultImageUrl(faultId, direction, imageUrl);
        updateCaptureImageUI(direction, imageUrl);
        console.log(`✅ ${direction} 实景已上传后端并绑定 URL：`, imageUrl);
        return imageUrl;
    }


    // ==========================================
    // ✅ 上位机规划履带小车矩形扫描范围与 Z 字形路径
    // 说明：履带小车只接收规划好的 GPS 航点，不再自行生成扫描路径。
    // ==========================================
    const DEFAULT_SCAN_PARAMS = {
        length: 8,
        width: 3.5,
        angleDeg: 23.0
    };
    const FAULT_SEARCH_WAIT_STORAGE_KEY = "faultSearchWaitSeconds";
    const DEFAULT_FAULT_SEARCH_WAIT_SECONDS = 30;
    let faultSearchWaitSeconds = DEFAULT_FAULT_SEARCH_WAIT_SECONDS;

    function normalizeRad(rad) {
        while (rad > Math.PI) rad -= 2 * Math.PI;
        while (rad < -Math.PI) rad += 2 * Math.PI;
        return rad;
    }

    function clampNumber(value, min, max, fallback) {
        const n = Number(value);
        if (!Number.isFinite(n)) return fallback;
        return Math.max(min, Math.min(max, n));
    }

    function normalizeFaultSearchWaitSeconds(value) {
        if (value === null || value === undefined || value === "") {
            return DEFAULT_FAULT_SEARCH_WAIT_SECONDS;
        }
        return Math.round(clampNumber(value, 1, 120, DEFAULT_FAULT_SEARCH_WAIT_SECONDS));
    }

    function loadFaultSearchWaitSeconds() {
        try {
            const saved = localStorage.getItem(FAULT_SEARCH_WAIT_STORAGE_KEY);
            return normalizeFaultSearchWaitSeconds(saved);
        } catch (e) {
            return DEFAULT_FAULT_SEARCH_WAIT_SECONDS;
        }
    }

    function saveFaultSearchWaitSeconds(value) {
        faultSearchWaitSeconds = normalizeFaultSearchWaitSeconds(value);
        try {
            localStorage.setItem(FAULT_SEARCH_WAIT_STORAGE_KEY, String(faultSearchWaitSeconds));
        } catch (e) {
            console.warn("保存寻找破损点等待间隔失败:", e);
        }
        syncFaultWaitSettingsUi();
        return faultSearchWaitSeconds;
    }

    function getFaultSearchWaitMs() {
        return normalizeFaultSearchWaitSeconds(faultSearchWaitSeconds) * 1000;
    }

    function getFaultSearchWaitLabel() {
        return normalizeFaultSearchWaitSeconds(faultSearchWaitSeconds) + " 秒";
    }

    function syncFaultWaitSettingsUi() {
        const input = document.getElementById("faultWaitSecondsInput");
        const text = document.getElementById("faultWaitCurrentText");
        const value = normalizeFaultSearchWaitSeconds(faultSearchWaitSeconds);
        if (input) input.value = String(value);
        if (text) text.textContent = value + " 秒";
    }

    function openFaultWaitParamModal() {
        const modal = document.getElementById("faultWaitParamModal");
        const input = document.getElementById("faultWaitSecondsInput");
        const btnOk = document.getElementById("btnFaultWaitParamConfirm");
        const btnCancel = document.getElementById("btnFaultWaitParamCancel");
        const btnClose = document.getElementById("btnFaultWaitParamClose");

        if (!modal || !input || !btnOk) {
            const value = prompt("自动寻找破损点等待间隔（秒）", String(faultSearchWaitSeconds));
            if (value !== null) {
                const saved = saveFaultSearchWaitSeconds(value);
                setStatus("自动寻找破损点等待间隔已设置为 " + saved + " 秒", "success");
            }
            return;
        }

        syncFaultWaitSettingsUi();

        function close() {
            modal.style.display = "none";
            btnOk.onclick = null;
            if (btnCancel) btnCancel.onclick = null;
            if (btnClose) btnClose.onclick = null;
            modal.onclick = null;
        }

        btnOk.onclick = function () {
            const saved = saveFaultSearchWaitSeconds(input.value);
            setStatus("自动寻找破损点等待间隔已设置为 " + saved + " 秒", "success");
            close();
        };
        if (btnCancel) btnCancel.onclick = close;
        if (btnClose) btnClose.onclick = close;
        modal.onclick = function (e) {
            if (e.target === modal) close();
        };

        modal.style.display = "flex";
        setTimeout(() => input.focus(), 30);
    }

    faultSearchWaitSeconds = loadFaultSearchWaitSeconds();

    function openScanParamModal(defaults = DEFAULT_SCAN_PARAMS, options = {}) {
        const modal = document.getElementById("scanParamModal");
        const inputL = document.getElementById("scanRectLengthInput");
        const inputW = document.getElementById("scanRectWidthInput");
        const inputA = document.getElementById("scanZAngleInput");
        const btnOk = document.getElementById("btnScanParamConfirm");
        const btnCancel = document.getElementById("btnScanParamCancel");
        const btnClose = document.getElementById("btnScanParamClose");

        const titleText = options.title || "🧭 履带小车扫描范围参数设置";
        const descText = options.desc || "设置完成后，上位机会生成矩形扫描范围和 Z 字形扫描路径，并将规划好的 GPS 航点发送给履带小车。";
        const confirmText = options.confirmText || "确定并规划";

        if (!modal || !inputL || !inputW || !inputA || !btnOk) {
            // 如果 HTML 没有同步更新，使用浏览器 prompt 兜底，避免功能完全不可用。
            const l = prompt("矩形长度 L（米）", String(defaults.length ?? DEFAULT_SCAN_PARAMS.length));
            if (l === null) return Promise.resolve(null);
            const w = prompt("矩形宽度 W（米）", String(defaults.width ?? DEFAULT_SCAN_PARAMS.width));
            if (w === null) return Promise.resolve(null);
            const a = prompt("Z 字折线与矩形短边夹角（度）", String(defaults.angleDeg ?? DEFAULT_SCAN_PARAMS.angleDeg));
            if (a === null) return Promise.resolve(null);
            return Promise.resolve({
                length: clampNumber(l, 0.2, 50, DEFAULT_SCAN_PARAMS.length),
                width: clampNumber(w, 0.2, 20, DEFAULT_SCAN_PARAMS.width),
                angleDeg: clampNumber(a, 5, 85, DEFAULT_SCAN_PARAMS.angleDeg)
            });
        }

        const titleEl = modal.querySelector(".cyber-modal-header span");
        const descEl = modal.querySelector(".scan-param-desc");
        const oldTitle = titleEl ? titleEl.textContent : "";
        const oldDesc = descEl ? descEl.textContent : "";
        const oldConfirmText = btnOk.textContent;

        if (titleEl) titleEl.textContent = titleText;
        if (descEl) descEl.textContent = descText;
        btnOk.textContent = confirmText;

        inputL.value = String(defaults.length ?? DEFAULT_SCAN_PARAMS.length);
        inputW.value = String(defaults.width ?? DEFAULT_SCAN_PARAMS.width);
        inputA.value = String(defaults.angleDeg ?? DEFAULT_SCAN_PARAMS.angleDeg);

        return new Promise(resolve => {
            let resolved = false;

            function close(value) {
                if (resolved) return;
                resolved = true;
                modal.style.display = "none";
                btnOk.onclick = null;
                if (btnCancel) btnCancel.onclick = null;
                if (btnClose) btnClose.onclick = null;
                modal.onclick = null;
                if (titleEl) titleEl.textContent = oldTitle;
                if (descEl) descEl.textContent = oldDesc;
                btnOk.textContent = oldConfirmText;
                resolve(value);
            }

            btnOk.onclick = function () {
                const params = {
                    length: clampNumber(inputL.value, 0.2, 50, DEFAULT_SCAN_PARAMS.length),
                    width: clampNumber(inputW.value, 0.2, 20, DEFAULT_SCAN_PARAMS.width),
                    angleDeg: clampNumber(inputA.value, 5, 85, DEFAULT_SCAN_PARAMS.angleDeg)
                };
                DEFAULT_SCAN_PARAMS.length = params.length;
                DEFAULT_SCAN_PARAMS.width = params.width;
                DEFAULT_SCAN_PARAMS.angleDeg = params.angleDeg;
                close(params);
            };

            if (btnCancel) btnCancel.onclick = () => close(null);
            if (btnClose) btnClose.onclick = () => close(null);
            modal.onclick = function (e) {
                if (e.target === modal) close(null);
            };

            modal.style.display = "flex";
            setTimeout(() => inputL.focus(), 30);
        });
    }

    function getCableSlamYawAtFault(faultPos) {
        if (!faultPos || !window.savedCablePath || window.savedCablePath.length < 2) {
            return 0.0;
        }

        let bestYaw = 0.0;
        let minDist = Infinity;
        const fp = new THREE.Vector3(Number(faultPos.x), Number(faultPos.y), Number(faultPos.z ?? 0));

        for (let i = 0; i < window.savedCablePath.length - 1; i++) {
            const p1 = window.savedCablePath[i];
            const p2 = window.savedCablePath[i + 1];
            const a = new THREE.Vector3(Number(p1.x), Number(p1.y), Number(p1.z ?? 0));
            const b = new THREE.Vector3(Number(p2.x), Number(p2.y), Number(p2.z ?? 0));
            const ab = new THREE.Vector3().subVectors(b, a);
            const lenSq = ab.lengthSq();
            if (lenSq < 1e-8) continue;

            const t = Math.max(0, Math.min(1, new THREE.Vector3().subVectors(fp, a).dot(ab) / lenSq));
            const proj = a.clone().add(ab.multiplyScalar(t));
            const dist = fp.distanceTo(proj);

            if (dist < minDist) {
                minDist = dist;
                bestYaw = Math.atan2(Number(p2.y) - Number(p1.y), Number(p2.x) - Number(p1.x));
            }
        }

        return normalizeRad(bestYaw);
    }

    function slamYawToGeoYawRad(slamYaw) {
        // SLAM 数学角：0=局部X正向；headingOffset 转为真实 ENU 数学角：0=东，90=北；
        // 履带车 RTK 航向角约定：0=北，90=东，因此用 PI/2 - mathYaw 转换。
        const transformYaw = window.geoTransform ? Number(window.geoTransform.theta || 0) : (window.headingOffset || 0);
        const mathGlobalYaw = normalizeRad(slamYaw + transformYaw);
        return normalizeRad((Math.PI / 2) - mathGlobalYaw);
    }

    function localScanPointToSlam(center, slamYaw, lx, ly, z = 0.10) {
        const cos = Math.cos(slamYaw);
        const sin = Math.sin(slamYaw);
        return {
            x: Number(center.x) + lx * cos - ly * sin,
            y: Number(center.y) + lx * sin + ly * cos,
            z: Number(center.z ?? z)
        };
    }

    function generateLocalZigzagPlan(length, width, angleDeg) {
        const L = Math.max(0.2, Number(length));
        const W = Math.max(0.2, Number(width));
        const safeAngle = clampNumber(angleDeg, 5, 85, DEFAULT_SCAN_PARAMS.angleDeg);
        const dxStep = Math.max(0.05, W * Math.tan(safeAngle * Math.PI / 180));

        // 原始矩形的起点和终点。后续只允许向前方自动延长终点，不改变起点位置，
        // 避免破损点附近的扫描起始位置被整体平移。
        const startX = -L / 2;
        const requestedEndX = L / 2;

        let x = startX;
        let y = W / 2;
        let direction = -1;
        const points = [{ x, y }];

        // ✅ 优化点：不再把最后一条斜线裁剪到原始矩形终点。
        // 当下一条完整 Z 字斜线会超过原始矩形长度时，直接让它完整走到对侧边，
        // 并把矩形终点同步延长到这条完整斜线的末端。
        while (points.length < 1000) {
            const targetY = direction === -1 ? -W / 2 : W / 2;
            const nextX = x + dxStep;

            points.push({ x: nextX, y: targetY });

            x = nextX;
            y = targetY;
            direction *= -1;

            if (x >= requestedEndX - 1e-9) {
                break;
            }
        }

        const endX = Math.max(requestedEndX, x);
        const effectiveLength = endX - startX;
        const extensionLength = Math.max(0, endX - requestedEndX);

        return {
            points,
            startX,
            endX,
            width: W,
            requestedLength: L,
            effectiveLength,
            extensionLength,
            dxStep,
            angleDeg: safeAngle,
            autoExtended: extensionLength > 1e-6
        };
    }

    function buildUpperComputerScanPlan(centerSlamPos, params, options = {}) {
        if (!centerSlamPos) {
            throw new Error("扫描中心点无效，无法生成扫描路径。");
        }

        const scanCenter = {
            x: Number(centerSlamPos.x),
            y: Number(centerSlamPos.y),
            z: Number(centerSlamPos.z ?? 0.10)
        };
        if (!Number.isFinite(scanCenter.x) || !Number.isFinite(scanCenter.y)) {
            throw new Error("扫描中心点坐标无效，无法生成扫描路径。");
        }

        const centerGps = window.slamToGps(scanCenter.x, scanCenter.y);
        if (!centerGps) {
            throw new Error("GPS 尚未标定完成，无法将扫描路径转换为经纬度。");
        }

        // 破损点扫描：不传 options.slamYaw 时，沿用电缆局部切向。
        // 手动指定扫描区域：传入用户在三维视图中点击得到的 slamYaw。
        const slamYaw = Number.isFinite(Number(options.slamYaw))
            ? normalizeRad(Number(options.slamYaw))
            : getCableSlamYawAtFault(scanCenter);
        const geoYawRad = slamYawToGeoYawRad(slamYaw);
        const zigzagPlan = generateLocalZigzagPlan(params.length, params.width, params.angleDeg);
        const localZigzag = zigzagPlan.points;

        const rectangleLocal = [
            { x: zigzagPlan.startX, y: zigzagPlan.width / 2 },
            { x: zigzagPlan.endX, y: zigzagPlan.width / 2 },
            { x: zigzagPlan.endX, y: -zigzagPlan.width / 2 },
            { x: zigzagPlan.startX, y: -zigzagPlan.width / 2 },
            { x: zigzagPlan.startX, y: zigzagPlan.width / 2 }
        ];

        const zigzagSlam = localZigzag.map(p => localScanPointToSlam(scanCenter, slamYaw, p.x, p.y));
        const rectangleSlam = rectangleLocal.map(p => localScanPointToSlam(scanCenter, slamYaw, p.x, p.y));

        const waypoints = zigzagSlam.map((p, idx) => {
            const gps = window.slamToGps(p.x, p.y);
            if (!gps) throw new Error("GPS 标定失效，无法转换第 " + (idx + 1) + " 个航点。");
            return {
                seq: idx + 1,
                lat: Number(gps.lat.toFixed(8)),
                lon: Number(gps.lon.toFixed(8)),
                x: Number(p.x.toFixed(4)),
                y: Number(p.y.toFixed(4))
            };
        });

        const source = options.source || "fault";
        const taskPrefix = options.taskPrefix || (source === "manual_area" ? "manual_scan" : "scan");

        return {
            type: "scan_path",
            source,
            taskId: taskPrefix + "_" + Date.now(),
            center: {
                lat: Number(centerGps.lat.toFixed(8)),
                lon: Number(centerGps.lon.toFixed(8)),
                x: Number(scanCenter.x.toFixed(4)),
                y: Number(scanCenter.y.toFixed(4)),
                z: Number((scanCenter.z ?? 0.10).toFixed(4)),
                yawRad: Number(geoYawRad.toFixed(6)),
                yawDeg: Number((geoYawRad * 180 / Math.PI).toFixed(3)),
                slamYawRad: Number(slamYaw.toFixed(6))
            },
            // ✅ 用于三维重建页规划后微调：保留可重新生成扫描路径的原始参数。
            edit: {
                centerSlam: {
                    x: Number(scanCenter.x.toFixed(6)),
                    y: Number(scanCenter.y.toFixed(6)),
                    z: Number((scanCenter.z ?? 0.10).toFixed(6))
                },
                slamYaw: Number(slamYaw.toFixed(8)),
                length: Number(zigzagPlan.requestedLength.toFixed(4)),
                width: Number(zigzagPlan.width.toFixed(4)),
                angleDeg: Number(zigzagPlan.angleDeg.toFixed(4)),
                source,
                taskPrefix
            },
            params: {
                // length 为最终实际扫描矩形长度；requestedLength 为弹窗中用户输入的原始长度。
                // 当最后一条 Z 字路线需要越过原始矩形边界才能画完整时，length 会自动延长。
                length: Number(zigzagPlan.effectiveLength.toFixed(4)),
                requestedLength: Number(zigzagPlan.requestedLength.toFixed(4)),
                extensionLength: Number(zigzagPlan.extensionLength.toFixed(4)),
                autoExtended: zigzagPlan.autoExtended,
                width: Number(zigzagPlan.width.toFixed(4)),
                angleDeg: Number(zigzagPlan.angleDeg.toFixed(3)),
                stepX: Number(zigzagPlan.dxStep.toFixed(4))
            },
            waypoints,
            preview: {
                rectangleSlam,
                zigzagSlam
            }
        };
    }

    function clearGprScanPlanOverlay() {
        if (window.gprScene && window.gprScanPlanGroup) {
            window.gprScene.remove(window.gprScanPlanGroup);
            disposeThreeObjectForReuse(window.gprScanPlanGroup);
        }
        window.gprScanPlanGroup = null;
    }

    function makeGprDashedLine(points, color, name) {
        const geom = new THREE.BufferGeometry().setFromPoints(points.map(slamPointToGprVector));
        const mat = new THREE.LineDashedMaterial({
            color,
            dashSize: 0.18,
            gapSize: 0.12,
            linewidth: 2,
            depthTest: false
        });
        const line = new THREE.Line(geom, mat);
        line.name = name;
        line.computeLineDistances();
        line.renderOrder = 20;
        line.frustumCulled = false;
        return line;
    }

    function addGprTextSprite(text, slamPoint) {
        const canvas = document.createElement("canvas");
        canvas.width = 256;
        canvas.height = 96;
        const ctx = canvas.getContext("2d");
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.font = "bold 28px Microsoft YaHei, Arial";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillStyle = "#ffffff";
        ctx.shadowColor = "#00e5ff";
        ctx.shadowBlur = 10;
        ctx.fillText(text, canvas.width / 2, canvas.height / 2);

        const texture = new THREE.CanvasTexture(canvas);
        const sprite = new THREE.Sprite(new THREE.SpriteMaterial({
            map: texture,
            transparent: true,
            depthTest: false
        }));
        const pos = slamPointToGprVector(slamPoint);
        sprite.position.set(pos.x, pos.y + 0.35, pos.z);
        sprite.scale.set(1.8, 0.68, 1);
        sprite.renderOrder = 25;
        return sprite;
    }

    function renderScanPlanOnGpr(plan, options = {}) {
        if (!plan || !plan.preview) return;

        // 默认首次规划时自动把视角对准扫描区域；
        // 微调过程中传入 { fitCamera: false }，只刷新矩形/Z字路径，不改变用户当前视角。
        const fitCamera = options.fitCamera !== false;

        // 保存最近一次由上位机规划的扫描路径。
        // 后续点击“开始处理”生成多个 GPR 剖面时，会优先用这条 Z 字路径来摆放剖面，
        // 从而让地下剖面直接落到点云地图中规划好的扫描区域下面。
        window.currentGprScanPlan = plan;
        window.currentGprScanPlanSavedAt = Date.now();

        ensureStandaloneGprSceneForMap();

        // 如果三维重建页还没加载当前点云地图，自动加载一次，保证矩形和 Z 字路线显示在同一地图中。
        if (!window.currentPcdGroup && !window.currentImportedInspectionMapGroup && typeof loadCurrentPathFaultMapToGpr === "function") {
            try {
                loadCurrentPathFaultMapToGpr();
            } catch (e) {
                console.warn("⚠️ 自动加载当前点云地图失败，仅绘制扫描路径预览：", e);
            }
        }

        clearGprScanPlanOverlay();

        const group = new THREE.Group();
        group.name = "上位机规划扫描范围与Z字路径";

        const rect = makeGprDashedLine(plan.preview.rectangleSlam, 0x00e5ff, "扫描矩形范围");
        const zigzag = makeGprDashedLine(plan.preview.zigzagSlam, 0xffa500, "Z字形扫描路径");
        group.add(rect);
        group.add(zigzag);

        if (plan.preview.zigzagSlam.length > 0) {
            group.add(addGprTextSprite("起点", plan.preview.zigzagSlam[0]));
            group.add(addGprTextSprite("终点", plan.preview.zigzagSlam[plan.preview.zigzagSlam.length - 1]));
        }

        window.gprScene.add(group);
        window.gprScanPlanGroup = group;

        const box = new THREE.Box3().setFromObject(group);
        if (fitCamera && box && !box.isEmpty()) {
            updateGprCameraByBox(box);
        }
    }

    async function sendScanPlanToCrawler(plan) {
        const payload = {
            type: "scan_path",
            source: plan.source || "fault",
            taskId: plan.taskId,
            center: plan.center,
            params: plan.params,
            waypoints: plan.waypoints
        };

        const res = await fetch("/api/car/scan-path", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify(payload)
        });

        const text = await res.text();
        if (!res.ok || !/^success/i.test(text.trim())) {
            throw new Error("后端返回：" + text);
        }
        return text;
    }


    // ==========================================
    // ✅ 扫描路径规划后微调：先预览，不立即下发
    // 支持对规划矩形区域进行旋转、长度拉伸、宽度拉伸；
    // 用户点击“确认发送”后，才把最终生成的 GPS 航点发送给履带小车。
    // ==========================================
    let scanPlanAdjustState = null;

    function radToDeg(rad) {
        return Number(rad || 0) * 180 / Math.PI;
    }

    function degToRad(deg) {
        return Number(deg || 0) * Math.PI / 180;
    }

    function getScanPlanAdjustPanel() {
        const container = document.getElementById("threeContainer");
        if (!container) return null;

        let panel = document.getElementById("gprScanAdjustPanel");
        if (!panel) {
            panel = document.createElement("div");
            panel.id = "gprScanAdjustPanel";
            panel.className = "gpr-scan-adjust-panel";
            panel.innerHTML = `
                <div class="scan-adjust-header">
                    <span>🛠️ 扫描区域微调</span>
                    <button type="button" class="scan-adjust-close" data-scan-adjust-action="cancel" title="取消本次规划">×</button>
                </div>
                <div class="scan-adjust-desc">路径尚未发送，微调后点击确认发送。</div>
                <div class="scan-adjust-metrics compact">
                    <span>L=<b data-scan-adjust-field="length">--</b>m</span>
                    <span>W=<b data-scan-adjust-field="width">--</b>m</span>
                    <span>角度=<b data-scan-adjust-field="yaw">--</b>°</span>
                    <span>中心=(<b data-scan-adjust-field="centerX">--</b>, <b data-scan-adjust-field="centerY">--</b>)</span>
                    <span style="display:none;">航点=<b data-scan-adjust-field="count">--</b></span>
                </div>
                <div class="scan-adjust-section">
                    <div class="scan-adjust-label">沿扫描方向平移</div>
                    <div class="scan-adjust-buttons compact">
                        <button type="button" data-scan-adjust-action="translate" data-local-x="-0.1" data-local-y="0">后退0.1m</button>
                        <button type="button" data-scan-adjust-action="translate" data-local-x="0.1" data-local-y="0">前进0.1m</button>
                    </div>
                </div>
                <div class="scan-adjust-section">
                    <div class="scan-adjust-label">横向平移</div>
                    <div class="scan-adjust-buttons compact">
                        <button type="button" data-scan-adjust-action="translate" data-local-x="0" data-local-y="0.1">左移0.1m</button>
                        <button type="button" data-scan-adjust-action="translate" data-local-x="0" data-local-y="-0.1">右移0.1m</button>
                    </div>
                </div>
                <div class="scan-adjust-section">
                    <div class="scan-adjust-label">旋转微调</div>
                    <div class="scan-adjust-buttons compact">
                        <button type="button" data-scan-adjust-action="rotate" data-value="-1">左转1°</button>
                        <button type="button" data-scan-adjust-action="rotate" data-value="1">右转1°</button>
                    </div>
                </div>
                <div class="scan-adjust-section">
                    <div class="scan-adjust-label">长度拉伸</div>
                    <div class="scan-adjust-buttons compact">
                        <button type="button" data-scan-adjust-action="length" data-value="-0.1">缩短0.1m</button>
                        <button type="button" data-scan-adjust-action="length" data-value="0.1">加长0.1m</button>
                    </div>
                </div>
                <div class="scan-adjust-section">
                    <div class="scan-adjust-label">宽度拉伸</div>
                    <div class="scan-adjust-buttons compact">
                        <button type="button" data-scan-adjust-action="width" data-value="-0.1">变窄0.1m</button>
                        <button type="button" data-scan-adjust-action="width" data-value="0.1">加宽0.1m</button>
                    </div>
                </div>
                <div class="scan-adjust-footer">
                    <button type="button" class="scan-adjust-secondary" data-scan-adjust-action="reset">恢复初始</button>
                    <button type="button" class="scan-adjust-secondary" data-scan-adjust-action="cancel">取消</button>
                    <button type="button" class="scan-adjust-primary" data-scan-adjust-action="confirm">确认发送</button>
                </div>
            `;
            container.appendChild(panel);

            panel.addEventListener("click", function (event) {
                const btn = event.target.closest("[data-scan-adjust-action]");
                if (!btn) return;
                event.preventDefault();
                event.stopPropagation();

                const action = btn.dataset.scanAdjustAction;
                const value = Number(btn.dataset.value || 0);

                if (action === "rotate") {
                    adjustCurrentScanPlan({ rotateDeg: value });
                } else if (action === "translate") {
                    adjustCurrentScanPlan({
                        localXDelta: Number(btn.dataset.localX || 0),
                        localYDelta: Number(btn.dataset.localY || 0)
                    });
                } else if (action === "length") {
                    adjustCurrentScanPlan({ lengthDelta: value });
                } else if (action === "width") {
                    adjustCurrentScanPlan({ widthDelta: value });
                } else if (action === "reset") {
                    resetCurrentScanPlanAdjustment();
                } else if (action === "confirm") {
                    confirmCurrentScanPlanAdjustment();
                } else if (action === "cancel") {
                    cancelCurrentScanPlanAdjustment();
                }
            });
        } else if (panel.parentElement !== container) {
            container.appendChild(panel);
        }

        return panel;
    }

    function normalizeScanPlanEdit(plan) {
        const edit = plan && plan.edit ? plan.edit : {};
        const centerSlam = edit.centerSlam || plan?.center || {};
        const source = edit.source || plan?.source || "manual_area";
        const taskPrefix = edit.taskPrefix || (source === "manual_area" ? "manual_scan" : "scan");
        return {
            centerSlam: {
                x: Number(centerSlam.x),
                y: Number(centerSlam.y),
                z: Number(centerSlam.z ?? 0.10)
            },
            slamYaw: normalizeRad(Number(edit.slamYaw ?? plan?.center?.slamYawRad ?? 0)),
            length: clampNumber(edit.length ?? plan?.params?.requestedLength ?? plan?.params?.length, 0.2, 50, DEFAULT_SCAN_PARAMS.length),
            width: clampNumber(edit.width ?? plan?.params?.width, 0.2, 20, DEFAULT_SCAN_PARAMS.width),
            angleDeg: clampNumber(edit.angleDeg ?? plan?.params?.angleDeg, 5, 85, DEFAULT_SCAN_PARAMS.angleDeg),
            source,
            taskPrefix,
            taskId: plan?.taskId || (taskPrefix + "_" + Date.now())
        };
    }

    function buildPlanFromAdjustState(state) {
        if (!state) return null;
        const plan = buildUpperComputerScanPlan(state.centerSlam, {
            length: state.length,
            width: state.width,
            angleDeg: state.angleDeg
        }, {
            slamYaw: state.slamYaw,
            source: state.source,
            taskPrefix: state.taskPrefix
        });

        // 微调过程中保持同一个 taskId，避免每点一次按钮就生成新任务号。
        plan.taskId = state.taskId;
        if (plan.edit) {
            plan.edit.taskId = state.taskId;
        }
        return plan;
    }

    function updateScanPlanAdjustPanel(plan) {
        const panel = getScanPlanAdjustPanel();
        if (!panel || !scanPlanAdjustState || !plan) return;

        const lengthEl = panel.querySelector('[data-scan-adjust-field="length"]');
        const widthEl = panel.querySelector('[data-scan-adjust-field="width"]');
        const yawEl = panel.querySelector('[data-scan-adjust-field="yaw"]');
        const countEl = panel.querySelector('[data-scan-adjust-field="count"]');
        const centerXEl = panel.querySelector('[data-scan-adjust-field="centerX"]');
        const centerYEl = panel.querySelector('[data-scan-adjust-field="centerY"]');

        if (lengthEl) lengthEl.textContent = Number(scanPlanAdjustState.length).toFixed(2);
        if (widthEl) widthEl.textContent = Number(scanPlanAdjustState.width).toFixed(2);
        if (yawEl) yawEl.textContent = radToDeg(scanPlanAdjustState.slamYaw).toFixed(1);
        if (countEl) countEl.textContent = String(plan.waypoints ? plan.waypoints.length : 0);
        if (centerXEl) centerXEl.textContent = Number(scanPlanAdjustState.centerSlam.x).toFixed(2);
        if (centerYEl) centerYEl.textContent = Number(scanPlanAdjustState.centerSlam.y).toFixed(2);
    }

    function refreshAdjustedScanPlanPreview() {
        if (!scanPlanAdjustState) return null;
        const plan = buildPlanFromAdjustState(scanPlanAdjustState);
        // 微调按钮会频繁重绘扫描预览，这里不再自动调整相机，避免视角跳变。
        renderScanPlanOnGpr(plan, { fitCamera: false });
        updateScanPlanAdjustPanel(plan);
        setStatus(
            `🛠️ 扫描区域待确认：中心=(${scanPlanAdjustState.centerSlam.x.toFixed(2)}, ${scanPlanAdjustState.centerSlam.y.toFixed(2)})，L=${scanPlanAdjustState.length.toFixed(2)}m，W=${scanPlanAdjustState.width.toFixed(2)}m，角度=${radToDeg(scanPlanAdjustState.slamYaw).toFixed(1)}°。确认后才会发送给履带小车。`,
            "warning"
        );
        return plan;
    }

    function adjustCurrentScanPlan(delta = {}) {
        if (!scanPlanAdjustState) return;

        if (Number.isFinite(Number(delta.rotateDeg))) {
            scanPlanAdjustState.slamYaw = normalizeRad(scanPlanAdjustState.slamYaw + degToRad(delta.rotateDeg));
        }
        if (Number.isFinite(Number(delta.localXDelta)) || Number.isFinite(Number(delta.localYDelta))) {
            const localXDelta = Number.isFinite(Number(delta.localXDelta)) ? Number(delta.localXDelta) : 0;
            const localYDelta = Number.isFinite(Number(delta.localYDelta)) ? Number(delta.localYDelta) : 0;
            const cos = Math.cos(scanPlanAdjustState.slamYaw);
            const sin = Math.sin(scanPlanAdjustState.slamYaw);

            // 与 localScanPointToSlam() 保持一致：localX 为扫描前进方向，localY 为矩形横向。
            scanPlanAdjustState.centerSlam.x += localXDelta * cos - localYDelta * sin;
            scanPlanAdjustState.centerSlam.y += localXDelta * sin + localYDelta * cos;
        }
        if (Number.isFinite(Number(delta.lengthDelta))) {
            scanPlanAdjustState.length = clampNumber(scanPlanAdjustState.length + Number(delta.lengthDelta), 0.2, 50, scanPlanAdjustState.length);
        }
        if (Number.isFinite(Number(delta.widthDelta))) {
            scanPlanAdjustState.width = clampNumber(scanPlanAdjustState.width + Number(delta.widthDelta), 0.2, 20, scanPlanAdjustState.width);
        }

        refreshAdjustedScanPlanPreview();
    }

    function resetCurrentScanPlanAdjustment() {
        if (!scanPlanAdjustState || !scanPlanAdjustState.initial) return;
        const initial = scanPlanAdjustState.initial;
        scanPlanAdjustState.centerSlam = {
            x: initial.centerSlam.x,
            y: initial.centerSlam.y,
            z: initial.centerSlam.z
        };
        scanPlanAdjustState.slamYaw = initial.slamYaw;
        scanPlanAdjustState.length = initial.length;
        scanPlanAdjustState.width = initial.width;
        refreshAdjustedScanPlanPreview();
    }

    function hideScanPlanAdjustPanel() {
        const panel = document.getElementById("gprScanAdjustPanel");
        if (panel) panel.style.display = "none";
    }

    function confirmCurrentScanPlanAdjustment() {
        if (!scanPlanAdjustState) return;
        const finalPlan = buildPlanFromAdjustState(scanPlanAdjustState);
        // 确认发送时也保持当前视角，只更新最终预览层。
        renderScanPlanOnGpr(finalPlan, { fitCamera: false });

        const resolver = scanPlanAdjustState.resolve;
        scanPlanAdjustState = null;
        hideScanPlanAdjustPanel();

        if (typeof resolver === "function") resolver(finalPlan);
    }

    function cancelCurrentScanPlanAdjustment() {
        if (!scanPlanAdjustState) return;
        const resolver = scanPlanAdjustState.resolve;
        scanPlanAdjustState = null;
        hideScanPlanAdjustPanel();
        clearGprScanPlanOverlay();
        setStatus("已取消扫描路径规划，未向履带小车发送航点。", "warning");
        if (typeof resolver === "function") resolver(null);
    }

    function enterScanPlanAdjustmentMode(plan, options = {}) {
        if (!plan) return Promise.resolve(null);

        if (scanPlanAdjustState && typeof scanPlanAdjustState.resolve === "function") {
            scanPlanAdjustState.resolve(null);
        }

        const edit = normalizeScanPlanEdit(plan);
        if (!Number.isFinite(edit.centerSlam.x) || !Number.isFinite(edit.centerSlam.y)) {
            return Promise.reject(new Error("扫描路径缺少可微调的中心点坐标。"));
        }

        scanPlanAdjustState = {
            ...edit,
            initial: {
                centerSlam: {
                    x: edit.centerSlam.x,
                    y: edit.centerSlam.y,
                    z: edit.centerSlam.z
                },
                slamYaw: edit.slamYaw,
                length: edit.length,
                width: edit.width
            },
            resolve: null,
            options
        };

        ensureStandaloneGprSceneForMap();
        const panel = getScanPlanAdjustPanel();
        if (panel) panel.style.display = "block";
        const previewPlan = refreshAdjustedScanPlanPreview() || plan;

        return new Promise(resolve => {
            if (!scanPlanAdjustState) {
                resolve(previewPlan);
                return;
            }
            scanPlanAdjustState.resolve = resolve;
        });
    }


    // ==========================================
    // ✅ 手动指定履带小车扫描区域
    // 交互流程：点击“指定扫描区域” -> 设置 L/W/夹角 -> 在三维视图依次点击中心点和方向点 -> 上位机规划并下发航点。
    // ==========================================
    let manualScanPickState = null;

    function gprVectorToSlamPoint(v) {
        // slamPointToGprVector 的逆变换：GPR页 (x, y高度, z) -> 路径页 SLAM (x, -z, y高度)
        return {
            x: Number(v.x),
            y: -Number(v.z),
            z: Number(v.y || 0.10)
        };
    }

    function getManualScanTipEl() {
        const container = document.getElementById("threeContainer");
        if (!container) return null;
        let el = document.getElementById("gprManualScanTip");
        if (!el) {
            el = document.createElement("div");
            el.id = "gprManualScanTip";
            el.className = "gpr-manual-scan-tip";
            container.appendChild(el);
        }
        return el;
    }

    function showManualScanTip(html) {
        const el = getManualScanTipEl();
        if (!el) return;
        el.innerHTML = html;
        el.style.display = "block";
    }

    function hideManualScanTip() {
        const el = document.getElementById("gprManualScanTip");
        if (el) el.style.display = "none";
    }

    function clearManualScanPickOverlay() {
        if (window.gprScene && window.gprManualScanPickGroup) {
            window.gprScene.remove(window.gprManualScanPickGroup);
            disposeThreeObjectForReuse(window.gprManualScanPickGroup);
        }
        window.gprManualScanPickGroup = null;
    }

    function addManualScanTextSprite(text, slamPoint, colorCss = "#ffffff") {
        const canvas = document.createElement("canvas");
        canvas.width = 192;
        canvas.height = 80;
        const ctx = canvas.getContext("2d");
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.font = "bold 26px Microsoft YaHei, Arial";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillStyle = colorCss;
        ctx.shadowColor = "#000000";
        ctx.shadowBlur = 8;
        ctx.fillText(text, canvas.width / 2, canvas.height / 2);

        const texture = new THREE.CanvasTexture(canvas);
        const sprite = new THREE.Sprite(new THREE.SpriteMaterial({
            map: texture,
            transparent: true,
            depthTest: false
        }));
        const pos = slamPointToGprVector(slamPoint);
        sprite.position.set(pos.x, pos.y + 0.35, pos.z);
        sprite.scale.set(1.25, 0.52, 1);
        sprite.renderOrder = 35;
        return sprite;
    }

    function renderManualScanPickOverlay(centerSlam, directionSlam = null) {
        if (!window.gprScene || typeof THREE === "undefined") return;

        clearManualScanPickOverlay();

        const group = new THREE.Group();
        group.name = "手动扫描区域选点辅助";

        if (centerSlam) {
            const centerPos = slamPointToGprVector(centerSlam);
            const centerMarker = new THREE.Mesh(
                new THREE.SphereGeometry(0.12, 16, 16),
                new THREE.MeshBasicMaterial({ color: 0x18a058, depthTest: false })
            );
            centerMarker.position.copy(centerPos);
            centerMarker.renderOrder = 32;
            group.add(centerMarker);
            group.add(addManualScanTextSprite("中心", centerSlam, "#8bff8b"));
        }

        if (centerSlam && directionSlam) {
            const dirPos = slamPointToGprVector(directionSlam);
            const dirMarker = new THREE.Mesh(
                new THREE.SphereGeometry(0.10, 16, 16),
                new THREE.MeshBasicMaterial({ color: 0xffa500, depthTest: false })
            );
            dirMarker.position.copy(dirPos);
            dirMarker.renderOrder = 32;
            group.add(dirMarker);
            group.add(addManualScanTextSprite("方向", directionSlam, "#ffd166"));

            const lineGeom = new THREE.BufferGeometry().setFromPoints([
                slamPointToGprVector(centerSlam),
                dirPos
            ]);
            const line = new THREE.Line(
                lineGeom,
                new THREE.LineBasicMaterial({ color: 0xffa500, depthTest: false })
            );
            line.renderOrder = 31;
            line.frustumCulled = false;
            group.add(line);
        }

        window.gprScene.add(group);
        window.gprManualScanPickGroup = group;
    }

    // 采集顺序视图仅镜像 WebGL 画面；点选时须把屏幕横坐标映回未镜像的相机坐标。
    function getGprCanvasPointerNdcX(event, dom, rect = dom.getBoundingClientRect()) {
        const ndcX = ((event.clientX - rect.left) / rect.width) * 2 - 1;
        return dom.dataset.gprMirroredView === "true" ? -ndcX : ndcX;
    }

    function getSlamPointFromGprMouseEvent(event) {
        if (!window.gprRenderer || !window.gprCamera || typeof THREE === "undefined") return null;

        const dom = window.gprRenderer.domElement;
        const rect = dom.getBoundingClientRect();
        if (!rect.width || !rect.height) return null;

        const mouse = new THREE.Vector2(
            getGprCanvasPointerNdcX(event, dom, rect),
            -((event.clientY - rect.top) / rect.height) * 2 + 1
        );

        const raycaster = new THREE.Raycaster();
        raycaster.setFromCamera(mouse, window.gprCamera);

        // 三维重建页以 Y 为高度轴；用 y=0 的水平面承接点击，适合指定地面扫描区域。
        const groundPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
        const hit = new THREE.Vector3();
        const ok = raycaster.ray.intersectPlane(groundPlane, hit);
        if (!ok) return null;

        const slam = gprVectorToSlamPoint(hit);
        slam.z = 0.10;
        if (!Number.isFinite(slam.x) || !Number.isFinite(slam.y)) return null;
        return slam;
    }

    function cleanupManualScanPicking() {
        if (!manualScanPickState) return;

        const dom = manualScanPickState.dom;
        if (dom && manualScanPickState.onPointerDown) {
            dom.removeEventListener("pointerdown", manualScanPickState.onPointerDown, true);
            dom.removeEventListener("pointermove", manualScanPickState.onPointerMove, true);
        }
        if (manualScanPickState.onKeyDown) {
            window.removeEventListener("keydown", manualScanPickState.onKeyDown, true);
        }
        if (dom) {
            dom.style.cursor = manualScanPickState.oldCursor || "";
        }

        hideManualScanTip();
        manualScanPickState = null;
    }

    function pickManualScanAreaPlan(params) {
        ensureStandaloneGprSceneForMap();

        if (!window.gprRenderer || !window.gprCamera) {
            return Promise.reject(new Error("三维视图尚未初始化，无法指定扫描区域。"));
        }

        if (manualScanPickState) {
            cleanupManualScanPicking();
        }

        clearManualScanPickOverlay();

        const dom = window.gprRenderer.domElement;
        const oldCursor = dom.style.cursor;
        dom.style.cursor = "crosshair";

        return new Promise((resolve, reject) => {
            const state = {
                dom,
                oldCursor,
                step: "center",
                centerSlam: null,
                latestDirectionSlam: null,
                resolved: false
            };

            function finish(value, isError = false) {
                if (state.resolved) return;
                state.resolved = true;
                cleanupManualScanPicking();
                if (isError) reject(value);
                else resolve(value);
            }

            state.onPointerDown = function (event) {
                event.preventDefault();
                event.stopPropagation();

                if (event.button !== undefined && event.button !== 0) return;

                const slam = getSlamPointFromGprMouseEvent(event);
                if (!slam) {
                    alert("当前点击位置无法投影到地面平面，请调整视角后重试。 ");
                    return;
                }

                if (state.step === "center") {
                    state.centerSlam = slam;
                    state.step = "direction";
                    renderManualScanPickOverlay(state.centerSlam);
                    showManualScanTip("已选择扫描中心点。请继续在三维视图中点击 <b>扫描前进方向点</b>。<span class='hotkey'>Esc</span> 取消");
                    return;
                }

                const dx = slam.x - state.centerSlam.x;
                const dy = slam.y - state.centerSlam.y;
                const dist = Math.hypot(dx, dy);
                if (dist < 0.15) {
                    alert("方向点距离中心点太近，请重新点击一个更远的方向点。 ");
                    return;
                }

                renderManualScanPickOverlay(state.centerSlam, slam);
                const slamYaw = Math.atan2(dy, dx);
                const plan = buildUpperComputerScanPlan(state.centerSlam, params, {
                    slamYaw,
                    source: "manual_area",
                    taskPrefix: "manual_scan"
                });
                finish(plan, false);
            };

            state.onPointerMove = function (event) {
                if (state.step !== "direction" || !state.centerSlam) return;
                const slam = getSlamPointFromGprMouseEvent(event);
                if (!slam) return;
                state.latestDirectionSlam = slam;
                renderManualScanPickOverlay(state.centerSlam, slam);
            };

            state.onKeyDown = function (event) {
                if (event.key === "Escape") {
                    clearManualScanPickOverlay();
                    finish(new Error("已取消手动指定扫描区域。"), true);
                }
            };

            manualScanPickState = state;
            dom.addEventListener("pointerdown", state.onPointerDown, true);
            dom.addEventListener("pointermove", state.onPointerMove, true);
            window.addEventListener("keydown", state.onKeyDown, true);

            showManualScanTip("请在三维视图中点击 <b>扫描区域中心点</b>，再点击 <b>扫描前进方向点</b>。<span class='hotkey'>Esc</span> 取消");
        });
    }

    async function startManualScanAreaPlanning(btnManualScan) {
        const originText = btnManualScan ? btnManualScan.innerText : "";

        try {
            if (!window.isGpsCalibrated || !window.gpsAnchor) {
                alert("⚠️ GPS 尚未标定完成，无法把手动扫描区域转换为履带小车经纬度航点！\n(可按 Shift+C 触发室内虚拟标定)");
                return;
            }

            const params = await openScanParamModal(DEFAULT_SCAN_PARAMS, {
                title: "🧭 手动指定扫描区域参数设置",
                desc: "设置矩形长度、宽度和 Z 字路径夹角。点击确定后，请在三维视图中依次点击扫描中心点和扫描前进方向点；该流程不依赖电缆破损点。",
                confirmText: "下一步：选中心和方向"
            });
            if (!params) return;

            ensureStandaloneGprSceneForMap();

            // 有当前点云地图时尽量自动加载，便于用户在点云地图上选扫描区域；没有地图也允许在网格平面上选点。
            if (!window.currentPcdGroup && !window.currentImportedInspectionMapGroup && typeof loadCurrentPathFaultMapToGpr === "function") {
                try {
                    loadCurrentPathFaultMapToGpr();
                } catch (e) {
                    console.warn("⚠️ 当前没有可加载的点云地图，允许在三维网格平面上手动指定扫描区域：", e);
                }
            }

            if (btnManualScan) {
                btnManualScan.disabled = true;
                btnManualScan.classList.add("picking");
                btnManualScan.innerText = "🎯 选中心/方向";
            }

            const plan = await pickManualScanAreaPlan(params);

            if (btnManualScan) btnManualScan.innerText = "🛠️ 等待微调...";
            clearManualScanPickOverlay();
            renderScanPlanOnGpr(plan);

            const finalPlan = await enterScanPlanAdjustmentMode(plan, { source: "manual_area" });
            if (!finalPlan) {
                if (btnManualScan) btnManualScan.innerText = "已取消";
                return;
            }

            if (btnManualScan) btnManualScan.innerText = "⏳ 发送路径...";
            await sendScanPlanToCrawler(finalPlan);

            const extendTip = finalPlan.params.autoExtended
                ? `，末段自动延长 ${finalPlan.params.extensionLength}m，实际L=${finalPlan.params.length}m`
                : "";
            setStatus(`✅ 已确认微调并下发履带小车路径：设置L=${finalPlan.params.requestedLength}m${extendTip}，W=${finalPlan.params.width}m，夹角=${finalPlan.params.angleDeg}°，航点${finalPlan.waypoints.length}个。`, "success");
            console.log("🧭 手动指定扫描区域路径已微调并下发：", finalPlan);

            if (btnManualScan) btnManualScan.innerText = `✅ 已发送${finalPlan.waypoints.length}个航点`;
        } catch (err) {
            if (err && String(err.message || "").includes("已取消")) {
                setStatus("已取消手动指定扫描区域。", "");
            } else {
                console.error("⚠️ 手动指定扫描区域失败：", err);
                alert("❌ 手动指定扫描区域失败：" + (err.message || err));
                setStatus("手动指定扫描区域失败：" + (err.message || err), "error");
                if (btnManualScan) btnManualScan.innerText = "❌ 指定失败";
            }
        } finally {
            cleanupManualScanPicking();
            if (btnManualScan) {
                setTimeout(() => {
                    btnManualScan.innerText = originText || "🧭 指定扫描区域";
                    btnManualScan.disabled = false;
                    btnManualScan.classList.remove("picking");
                }, 1500);
            }
        }
    }
    // ==========================================
    // ✅ 新增：精确计算滑块 Thumb 实时偏移量的浮动标签函数
    // ==========================================
    window.updateFloatingThumb = function (inputEl, labelId) {
        const labelEl = document.getElementById(labelId);
        if (!labelEl || !inputEl) return;

        // 1. 获取当前原始浮点数值 (用于后续内部偏移量计算)
        const val = parseFloat(inputEl.value);

        // 🌟 核心修改：生成强制保留 1 位小数的字符串格式 (例如 2 -> "2.0")
        const displayVal = val.toFixed(1);

        // 更新当前悬浮气泡的文本显示
        labelEl.textContent = displayVal;

        // 2. 顺便同步更新原有的自动巡线数字显示（统一显示保留一位小数的版本）
        if (labelId === 'valAutoX') document.getElementById('valAutoX').textContent = displayVal;
        if (labelId === 'valAutoY') document.getElementById('valAutoY').textContent = displayVal;

        // 同步更新信息栏中的机器狗前进速度
        if (inputEl.id === 'manualSpeedX') {
            const infoSpeedEl = document.getElementById('infoForwardSpeedValue');
            if (infoSpeedEl) {
                infoSpeedEl.textContent = displayVal + ' m/s';
            }
        }

        // 3. 核心物理几何对齐计算 (继续使用原始 val 进行数学计算，确保几何精度不变)
        const min = parseFloat(inputEl.min) || 0;
        const max = parseFloat(inputEl.max) || 3.0;

        // 计算当前值在总射程中的百分比 (0.0 ~ 1.0)
        const percent = (val - min) / (max - min);

        // 💡 视觉修正魔法：补偿 range thumb 圆点在边缘的动态像素偏移
        const thumbWidth = 14;
        const offsetPx = (0.5 - percent) * thumbWidth;

        // 应用计算结果到绝对定位的 left 属性
        labelEl.style.left = `calc(${percent * 100}% + ${offsetPx}px)`;
    };



    // ✅ 新增：发送自动巡线速度给后端 (挂载到 window 以便 HTML 调用)
    window.updateAutoSpeed = function () {
        const vx = document.getElementById('autoSpeedX').value;
        const vy = document.getElementById('autoSpeedY').value;

        // 更新UI数字
        document.getElementById('valAutoX').textContent = vx;
        document.getElementById('valAutoY').textContent = vy;

        // 发送 fetch 请求
        const url = `/api/pipeline/speed?vx=${vx}&vy=${vy}`;
        console.log(`📡 更新自动巡线速度: vx=${vx}, vy=${vy}`);

        fetch(url, { method: "POST" })
            .then(res => res.text()) // 假设后端返回字符串 "ok"
            .then(msg => console.log("速度更新反馈:", msg))
            .catch(err => console.error("速度更新失败:", err));
    };

    let lastCablePathPredictionSyncAt = 0;
    let lastCablePathPredictionSyncCount = 0;
    window.syncCablePathPrediction = function (force = false) {
        const path = window.savedCablePath;
        if (!Array.isArray(path) || path.length < 4) return;

        const now = Date.now();
        if (!force && now - lastCablePathPredictionSyncAt < 1000 && path.length === lastCablePathPredictionSyncCount) {
            return;
        }

        lastCablePathPredictionSyncAt = now;
        lastCablePathPredictionSyncCount = path.length;

        const recentPath = path.slice(-80).map(p => ({
            x: Number(p.x),
            y: Number(p.y),
            z: Number(p.z ?? 0.08),
            depth: p.userData && p.userData.depth !== undefined ? p.userData.depth : null
        })).filter(p => Number.isFinite(p.x) && Number.isFinite(p.y));

        if (recentPath.length < 4) return;

        fetch('/api/pipeline/path', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(recentPath)
        }).catch(err => console.warn("电缆走向预测路径同步失败:", err));
    };

    // ==========================================
    // 核心：基于 Radix-2 的轻量级快速傅里叶变换 (FFT)
    // ==========================================
    function computeMagnitudeFFT(realInput) {
        const N = realInput.length; // 必须是 2 的整数次幂 (例如 16384)
        const real = new Float32Array(realInput);
        const imag = new Float32Array(N);

        // 位反转 (Bit Reversal)
        let j = 0;
        for (let i = 0; i < N - 1; i++) {
            if (i < j) {
                let tr = real[i]; real[i] = real[j]; real[j] = tr;
            }
            let k = N >> 1;
            while (k <= j) { j -= k; k >>= 1; }
            j += k;
        }

        // 蝶形运算 (Cooley-Tukey)
        for (let L = 2; L <= N; L <<= 1) {
            const halfL = L >> 1;
            const theta = -2 * Math.PI / L;
            const w_r_step = Math.cos(theta);
            const w_i_step = Math.sin(theta);

            for (let k = 0; k < N; k += L) {
                let w_r = 1, w_i = 0;
                for (let step = 0; step < halfL; step++) {
                    const idx = k + step;
                    const idx2 = idx + halfL;

                    const t_r = w_r * real[idx2] - w_i * imag[idx2];
                    const t_i = w_r * imag[idx2] + w_i * real[idx2];

                    real[idx2] = real[idx] - t_r;
                    imag[idx2] = imag[idx] - t_i;
                    real[idx] += t_r;
                    imag[idx] += t_i;

                    const next_w_r = w_r * w_r_step - w_i * w_i_step;
                    const next_w_i = w_r * w_i_step + w_i * w_r_step;
                    w_r = next_w_r; w_i = next_w_i;
                }
            }
        }

        // 计算幅值 (Magnitude) -> 只取前半部分（Nyquist 频率内）
        const mag = new Float32Array(N / 2);
        for (let i = 0; i < N / 2; i++) {
            mag[i] = Math.sqrt(real[i] * real[i] + imag[i] * imag[i]);
        }
        return mag;
    }

    // ==========================================
    // ✅ GPR 标注校准持久化工具函数
    // ==========================================
    function normalizeGprFolderPath(path) {
        const normalized = String(path || "")
            .trim()
            .replace(/\\+/g, "/")
            .replace(/\/+$/g, "");

        // Windows 路径大小写不敏感，统一小写；Linux 路径保持原样，避免不同大小写目录被误认为同一个。
        if (/^[a-zA-Z]:\//.test(normalized)) {
            return normalized.toLowerCase();
        }
        return normalized;
    }

    function simpleHashForStorage(text) {
        // FNV-1a 32bit，保留为旧版兼容工具；当前版本不再使用浏览器 localStorage 保存校准结果。
        let hash = 2166136261;
        const str = String(text || "");
        for (let i = 0; i < str.length; i++) {
            hash ^= str.charCodeAt(i);
            hash = Math.imul(hash, 16777619);
        }
        return (hash >>> 0).toString(16).padStart(8, "0");
    }

    function getGprFolderAnnotationKey(folderPath) {
        const normalized = normalizeGprFolderPath(folderPath || currentGprFolderPath);
        if (!normalized) return "";
        // 这里不再生成浏览器缓存 key，只作为日志/状态显示使用。
        return normalized + "/" + GPR_ANNOTATION_FILE_NAME;
    }

    function getGprSegmentStableName(seg, idx) {
        const fallback = "segment_" + (Number.isFinite(Number(idx)) ? Number(idx) : 0);
        if (!seg || typeof seg !== "object") return fallback;
        return String(
            seg.fileName ||
            seg.name ||
            seg.sourceFileName ||
            seg.originalFileName ||
            seg.path ||
            seg.filePath ||
            fallback
        );
    }

    function getGprSegmentSignature(result) {
        const segments = result && Array.isArray(result.segments) ? result.segments : [];
        return {
            segmentCount: segments.length,
            segmentNames: segments.map((seg, idx) => getGprSegmentStableName(seg, idx))
        };
    }

    function isSameGprSegmentSignature(savedSig, result) {
        if (!savedSig) return true;
        const currentSig = getGprSegmentSignature(result);
        if (Number(savedSig.segmentCount) !== Number(currentSig.segmentCount)) return false;

        const savedNames = Array.isArray(savedSig.segmentNames) ? savedSig.segmentNames : [];
        const currentNames = Array.isArray(currentSig.segmentNames) ? currentSig.segmentNames : [];
        // 只有两边都有文件名时才严格比对，避免后端没有 fileName 时误判。
        if (savedNames.length === currentNames.length && savedNames.length > 0) {
            for (let i = 0; i < savedNames.length; i++) {
                if (String(savedNames[i] || "") && String(currentNames[i] || "") && savedNames[i] !== currentNames[i]) {
                    return false;
                }
            }
        }
        return true;
    }

    function mapToStorageArray(map, type) {
        const out = [];
        map.forEach((value, key) => {
            if (type === "set") {
                out.push([Number(key), Array.from(value || [])]);
            } else {
                out.push([Number(key), Array.isArray(value) ? value : []]);
            }
        });
        return out;
    }

    function restoreArrayMap(targetMap, entries, valueType) {
        targetMap.clear();
        if (!Array.isArray(entries)) return;

        entries.forEach(item => {
            if (!Array.isArray(item) || item.length < 2) return;
            const key = Number(item[0]);
            if (!Number.isFinite(key)) return;

            if (valueType === "set") {
                targetMap.set(key, new Set(Array.isArray(item[1]) ? item[1] : []));
            } else {
                targetMap.set(key, Array.isArray(item[1]) ? item[1] : []);
            }
        });
    }


    function normalizeGprPipeColorHex(value, fallback = GPR_PIPE_DEFAULT_COLOR_HEX) {
        if (typeof value === "number" && Number.isFinite(value)) {
            return Math.max(0, Math.min(0xffffff, Math.round(value)));
        }

        if (typeof value === "string") {
            let text = value.trim();
            if (!text) return fallback;
            if (text.startsWith("#")) text = text.slice(1);
            if (/^0x/i.test(text)) text = text.slice(2);
            if (/^[0-9a-fA-F]{3}$/.test(text)) {
                text = text.split("").map(ch => ch + ch).join("");
            }
            if (/^[0-9a-fA-F]{6}$/.test(text)) {
                return parseInt(text, 16);
            }
        }

        return fallback;
    }

    function gprPipeColorHexToCss(colorHex) {
        return "#" + normalizeGprPipeColorHex(colorHex).toString(16).padStart(6, "0");
    }

    function normalizeGprPipeMaterialType(type) {
        const raw = String(type || "").trim();
        // 兼容上一版保存过的 “steel/钢管” 标识，后续统一按 metal（金属管道）处理。
        if (raw === "steel" || raw === "钢管") return "metal";
        return raw;
    }

    function getGprPipeMaterialByType(type) {
        const normalizedType = normalizeGprPipeMaterialType(type);
        return GPR_PIPE_MATERIAL_TYPES.find(item => item.type === normalizedType) || null;
    }

    function getGprPipePopupTitleByAssignment(assignment) {
        if (!assignment) return "🟡 地下管道测点信息";
        const material = getGprPipeMaterialByType(assignment.materialType);
        return material && material.title ? material.title : "地下管道测点信息";
    }

    function updateGprPipePopupTitle(assignment) {
        const titleEl = $("gprPipePopupTitle");
        if (titleEl) {
            titleEl.textContent = getGprPipePopupTitleByAssignment(assignment);
        }
    }

    function serializeGprPipeColorAssignments() {
        return Array.from(gprPipeColorAssignments.entries()).map(([key, value]) => {
            const colorHex = normalizeGprPipeColorHex(value && value.colorHex);
            return [String(key), {
                pipeColorKey: String(key),
                materialType: normalizeGprPipeMaterialType(value && value.materialType ? String(value.materialType) : "custom"),
                materialName: value && value.materialName ? String(value.materialName) : "自定义颜色",
                colorHex,
                colorCss: value && value.colorCss ? String(value.colorCss) : gprPipeColorHexToCss(colorHex),
                updatedAt: value && value.updatedAt ? Number(value.updatedAt) : Date.now()
            }];
        });
    }

    function restoreGprPipeColorAssignments(entries) {
        gprPipeColorAssignments.clear();
        if (!Array.isArray(entries)) return;

        entries.forEach(item => {
            if (!Array.isArray(item) || item.length < 2) return;
            const key = String(item[0] || "");
            if (!key) return;

            const value = item[1] || {};
            const colorHex = normalizeGprPipeColorHex(value.colorHex ?? value.hex ?? value.color);
            const rawMaterialType = String(value.materialType || value.pipeType || "custom");
            const matchedMaterial = getGprPipeMaterialByType(rawMaterialType);
            const materialType = matchedMaterial ? matchedMaterial.type : normalizeGprPipeMaterialType(rawMaterialType);

            gprPipeColorAssignments.set(key, {
                pipeColorKey: key,
                materialType,
                // 已指定过的旧版“钢管”统一显示为“金属管道”。
                materialName: matchedMaterial ? matchedMaterial.name : String(value.materialName || value.pipeTypeName || "自定义颜色"),
                colorHex,
                colorCss: String(value.colorCss || gprPipeColorHexToCss(colorHex)),
                updatedAt: Number(value.updatedAt || Date.now())
            });
        });
    }

    function roundGprPipeCoord(value) {
        const n = Number(value);
        return Number.isFinite(n) ? n.toFixed(3) : "0.000";
    }

    function roundGprIdentityNumber(value, digits = 1) {
        const n = Number(value);
        return Number.isFinite(n) ? n.toFixed(digits) : "0.0";
    }

    function sanitizeGprPipeKeyPart(value) {
        const raw = String(value ?? "").trim();
        if (!raw) return "none";
        return raw
            .replace(/\\+/g, "/")
            .split("/")
            .pop()
            .replace(/[^a-zA-Z0-9_\-.一-龥]/g, "_")
            .slice(0, 80) || "none";
    }

    function makeGprSegmentIdentityToken(segmentName, segIndex) {
        const idx = Number.isFinite(Number(segIndex)) ? Number(segIndex) : 0;
        const safeName = sanitizeGprPipeKeyPart(segmentName || ("segment_" + idx));
        const hash = simpleHashForStorage(String(segmentName || safeName));
        return `seg${idx}_${safeName}_${hash}`;
    }

    function normalizeGprBlobSource(source, linkMode) {
        const raw = String(source || "").trim();
        if (raw === "forcedPipe" || raw === "forced") return "forced";
        if (raw === "detectedApex" || raw === "apex") return "apex";
        if (raw === "manual") return "manual";
        if (raw === "auto") return "auto";
        if (linkMode === "forced") return "forced";
        return raw || "auto";
    }

    function makeGprAnnotationIdentityFromBlob(blob, seg, segIndex) {
        const segmentName = getGprSegmentStableName(seg, segIndex);
        const segmentToken = makeGprSegmentIdentityToken(segmentName, segIndex);
        const source = normalizeGprBlobSource(blob && blob.source, blob && blob.linkMode);
        const row = roundGprIdentityNumber(blob && blob.row, 1);
        const col = roundGprIdentityNumber(blob && blob.col, 1);
        const radius = roundGprIdentityNumber(blob && blob.radius, 1);

        let localId = "";
        if (source === "apex") {
            localId = sanitizeGprPipeKeyPart(blob && blob.apexKey ? blob.apexKey : `${row}_${col}_${radius}`);
        } else if (source === "auto") {
            localId = sanitizeGprPipeKeyPart(blob && blob.autoKey ? blob.autoKey : `${row}_${col}_${radius}`);
        } else if (source === "manual") {
            const manualIndex = Number.isFinite(Number(blob && blob.manualIndex)) ? Number(blob.manualIndex) : -1;
            // 手动红圈以下次保存的二维剖面 row/col/radius 为主，manualIndex 只作为同一剖面多点时的辅助区分。
            localId = `m${manualIndex}_${row}_${col}_${radius}`;
        } else if (source === "forced") {
            const groupId = sanitizeGprPipeKeyPart(blob && (blob.pipeGroupId || blob.groupId) ? (blob.pipeGroupId || blob.groupId) : "forced_group");
            const pipeOrder = Number.isFinite(Number(blob && blob.pipeOrder)) ? Number(blob.pipeOrder) : 0;
            const forcedIndex = Number.isFinite(Number(blob && blob.forcedIndex)) ? Number(blob.forcedIndex) : -1;
            localId = `${groupId}_o${pipeOrder}_f${forcedIndex}_${row}_${col}_${radius}`;
        } else {
            localId = `${sanitizeGprPipeKeyPart(source)}_${row}_${col}_${radius}`;
        }

        const annotationStableKey = `${segmentToken}:${source}:${localId}`;
        return {
            segmentName,
            segmentToken,
            blobSource: source,
            annotationStableKey,
            annotationLocalId: localId,
            annotationRow: row,
            annotationCol: col,
            annotationRadius: radius
        };
    }

    // 旧版兼容：历史版本使用三维世界坐标 hash 保存材质。新版本不再把它作为主 key，
    // 但仍保留为 legacyColorKey，方便同一布局下读取旧标注文件时继续显示。
    function makeGprPipeColorKeyFromPoints(points, meta = {}) {
        const cleanPoints = (Array.isArray(points) ? points : [])
            .map(p => p && p.clone ? p : new THREE.Vector3(Number(p.x), Number(p.y), Number(p.z)))
            .filter(p => Number.isFinite(p.x) && Number.isFinite(p.y) && Number.isFinite(p.z));

        const pointPart = cleanPoints
            .map(p => `${roundGprPipeCoord(p.x)},${roundGprPipeCoord(p.y)},${roundGprPipeCoord(p.z)}`)
            .join("|");
        const source = sanitizeGprPipeKeyPart(meta.source || "pipe");
        const groupId = sanitizeGprPipeKeyPart(meta.pipeGroupId || meta.groupId || "");
        return `legacy:${source}:${groupId}:${simpleHashForStorage(pointPart)}`;
    }

    function makeGprPipeLegacyColorKeyFromCenters(centers, meta = {}) {
        const points = (Array.isArray(centers) ? centers : [])
            .map(item => item && item.center ? item.center : item)
            .filter(Boolean);
        return makeGprPipeColorKeyFromPoints(points, meta);
    }

    function makeGprPipeColorKeyFromCenters(centers, meta = {}) {
        const arr = (Array.isArray(centers) ? centers : []).filter(Boolean);
        const stableParts = arr
            .map(item => item && item.annotationStableKey ? String(item.annotationStableKey) : "")
            .filter(Boolean);

        const source = sanitizeGprPipeKeyPart(meta.source || (arr[0] && arr[0].blobSource) || "pipe");
        const groupId = sanitizeGprPipeKeyPart(meta.pipeGroupId || meta.groupId || (arr[0] && arr[0].pipeGroupId) || "");

        if (stableParts.length >= 2) {
            // 稳定 key 只依赖原始 GPR 剖面身份 + 二维圆圈身份，不依赖本次三维场景坐标。
            const identityPart = stableParts.join("|");
            return `gpr-pipe-v2:${source}:${groupId}:${arr.length}:${simpleHashForStorage(identityPart)}`;
        }

        return makeGprPipeLegacyColorKeyFromCenters(arr, meta);
    }

    function normalizeGprPipeKeyList(keys) {
        const out = [];
        (Array.isArray(keys) ? keys : [keys]).forEach(key => {
            const text = String(key || "").trim();
            if (text && !out.includes(text)) out.push(text);
        });
        return out;
    }

    function getGprPipeAssignmentByKey(pipeColorKey, fallbackKeys = []) {
        const keys = normalizeGprPipeKeyList([pipeColorKey].concat(fallbackKeys || []));
        for (const key of keys) {
            const assignment = gprPipeColorAssignments.get(key);
            if (assignment) return assignment;
        }
        return null;
    }

    function resolveGprPipeColorHexByKey(pipeColorKey, fallbackColorHex = GPR_PIPE_DEFAULT_COLOR_HEX, fallbackKeys = []) {
        const assignment = getGprPipeAssignmentByKey(pipeColorKey, fallbackKeys);
        if (assignment) return normalizeGprPipeColorHex(assignment.colorHex, fallbackColorHex);
        return normalizeGprPipeColorHex(fallbackColorHex);
    }

    function getGprPipeColorKeysForObject(obj) {
        if (!obj || !obj.userData) return [];
        const meta = obj.userData.gprPipeMeta || {};
        return normalizeGprPipeKeyList([
            obj.userData.gprPipeColorKey,
            meta.colorKey,
            meta.pipeColorKey,
            obj.userData.gprPipeLegacyColorKey,
            meta.legacyColorKey,
            meta.coordColorKey,
            meta.oldColorKey
        ]);
    }

    function getGprPipeAssignmentForObject(obj) {
        const keys = getGprPipeColorKeysForObject(obj);
        return getGprPipeAssignmentByKey(keys[0], keys.slice(1));
    }

    function applyGprPipeColorToObject(obj, colorHex) {
        if (!obj) return;
        const safeColorHex = normalizeGprPipeColorHex(colorHex);
        obj.traverse(child => {
            if (!child.material) return;
            const materials = Array.isArray(child.material) ? child.material : [child.material];
            materials.forEach(mat => {
                if (mat && mat.color && typeof mat.color.setHex === "function") {
                    mat.color.setHex(safeColorHex);
                    mat.needsUpdate = true;
                }
            });
        });
    }

    function applyGprPipeSavedColorIfAny(mesh) {
        if (!mesh || !mesh.userData) return;
        const meta = mesh.userData.gprPipeMeta || {};
        const keys = getGprPipeColorKeysForObject(mesh);
        const pipeColorKey = keys[0] || "";
        const renderColorHex = resolveGprPipeColorHexByKey(
            pipeColorKey,
            meta.colorHex || GPR_PIPE_DEFAULT_COLOR_HEX,
            keys.slice(1)
        );
        applyGprPipeColorToObject(mesh, renderColorHex);
    }

    function applyGprPipeColorAssignmentToScene(pipeColorKey) {
        const key = String(pipeColorKey || "");
        if (!key || !window.gprPipePickObjects) return;

        window.gprPipePickObjects.forEach(obj => {
            if (!obj || !obj.userData) return;
            const keys = getGprPipeColorKeysForObject(obj);
            if (!keys.includes(key)) return;
            const meta = obj.userData.gprPipeMeta || {};
            const assignment = getGprPipeAssignmentByKey(keys[0], keys.slice(1));
            const colorHex = assignment
                ? assignment.colorHex
                : (meta.colorHex || GPR_PIPE_DEFAULT_COLOR_HEX);
            applyGprPipeColorToObject(obj, colorHex);
        });
    }

    function ensureGprPipeColorControls(popup) {
        if (!popup) return;

        // 1) 标题区改成“标题 + 指定材质下拉按钮”同一行。
        //    下方不再显示“管道材质：xxx”这一行，材质类型由标题自动体现。
        let titleEl = popup.querySelector("#gprPipePopupTitle") || popup.querySelector(".gpr-pipe-popup-title");
        if (!titleEl) {
            titleEl = document.createElement("div");
            titleEl.id = "gprPipePopupTitle";
            titleEl.className = "gpr-pipe-popup-title";
            titleEl.textContent = "🟡 地下管道测点信息";
            const closeBtn = popup.querySelector(".gpr-pipe-popup-close");
            popup.insertBefore(titleEl, closeBtn ? closeBtn.nextSibling : popup.firstChild);
        }
        titleEl.id = "gprPipePopupTitle";

        let titleRow = popup.querySelector(".gpr-pipe-popup-title-row");
        if (!titleRow) {
            titleRow = document.createElement("div");
            titleRow.className = "gpr-pipe-popup-title-row";
            titleEl.parentNode.insertBefore(titleRow, titleEl);
            titleRow.appendChild(titleEl);
        } else if (titleEl.parentElement !== titleRow) {
            titleRow.insertBefore(titleEl, titleRow.firstChild);
        }

        let toggleBtn = popup.querySelector("#gprPipeColorToggleBtn");
        if (!toggleBtn) {
            toggleBtn = document.createElement("button");
            toggleBtn.type = "button";
            toggleBtn.id = "gprPipeColorToggleBtn";
            toggleBtn.className = "pipe-color-toggle-btn";
            toggleBtn.textContent = "▼";
        }
        if (toggleBtn.parentElement !== titleRow) {
            titleRow.appendChild(toggleBtn);
        }

        // 2) 移除旧版弹窗下方的“管道材质：当前类型 + 色块”行，以及旧版局部坐标行。
        popup.querySelectorAll(".pipe-material-row").forEach(row => row.remove());
        const legacyMaterialEl = popup.querySelector("#gprPipePopupMaterial");
        if (legacyMaterialEl) {
            const row = legacyMaterialEl.closest(".gpr-pipe-popup-item");
            if (row) row.remove();
            else legacyMaterialEl.remove();
        }
        const legacySwatchEl = popup.querySelector("#gprPipePopupColorSwatch");
        if (legacySwatchEl) legacySwatchEl.remove();
        const legacyLocalEl = popup.querySelector("#gprPipePopupLocal");
        if (legacyLocalEl) {
            const legacyRow = legacyLocalEl.closest(".gpr-pipe-popup-item");
            if (legacyRow) legacyRow.remove();
            else legacyLocalEl.remove();
        }

        // 3) 下拉按钮只控制材质按钮组；按钮组默认折叠。
        let section = popup.querySelector("#gprPipeColorSection");
        if (!section) {
            section = document.createElement("div");
            section.id = "gprPipeColorSection";
            section.className = "gpr-pipe-color-section";
            popup.appendChild(section);
        }

        let actionPanel = section.querySelector(".gpr-pipe-color-actions");
        if (!actionPanel) {
            actionPanel = document.createElement("div");
            actionPanel.className = "gpr-pipe-color-actions collapsed";
            section.appendChild(actionPanel);
        }
        actionPanel.innerHTML = `
            <button type="button" data-gpr-pipe-material="cable" class="pipe-color-btn pipe-color-cable"><span class="pipe-color-dot"></span>地下电缆</button>
            <button type="button" data-gpr-pipe-material="pvc" class="pipe-color-btn pipe-color-pvc"><span class="pipe-color-dot"></span>PVC管</button>
            <button type="button" data-gpr-pipe-material="metal" class="pipe-color-btn pipe-color-metal"><span class="pipe-color-dot"></span>金属管道</button>
            <button type="button" data-gpr-pipe-material-reset="1" class="pipe-color-btn pipe-color-reset">恢复默认</button>
        `;

        if (toggleBtn.dataset.bound !== "1") {
            toggleBtn.dataset.bound = "1";
            toggleBtn.addEventListener("click", function (event) {
                event.preventDefault();
                event.stopPropagation();
                const collapsed = actionPanel.classList.toggle("collapsed");
                toggleBtn.textContent = collapsed ? "▼" : "▲";
                refreshGprPipeMaterialButtonActiveState(popup, selectedGprPipeObject);
            });
        }

        actionPanel.querySelectorAll("[data-gpr-pipe-material]").forEach(btn => {
            btn.addEventListener("click", function (event) {
                event.preventDefault();
                event.stopPropagation();
                setSelectedGprPipeMaterialColor(this.dataset.gprPipeMaterial);
            });
        });

        const resetBtn = actionPanel.querySelector("[data-gpr-pipe-material-reset]");
        if (resetBtn) {
            resetBtn.addEventListener("click", function (event) {
                event.preventDefault();
                event.stopPropagation();
                resetSelectedGprPipeMaterialColor();
            });
        }
    }

    function refreshGprPipeMaterialButtonActiveState(popup, obj) {
        if (!popup) return;
        const assignment = getGprPipeAssignmentForObject(obj);
        const activeType = assignment ? normalizeGprPipeMaterialType(assignment.materialType) : "";

        popup.querySelectorAll("[data-gpr-pipe-material]").forEach(btn => {
            const btnType = normalizeGprPipeMaterialType(btn.dataset.gprPipeMaterial);
            const isActive = !!activeType && activeType === btnType;
            btn.classList.toggle("active", isActive);
            btn.setAttribute("aria-pressed", isActive ? "true" : "false");
        });

        const resetBtn = popup.querySelector("[data-gpr-pipe-material-reset]");
        if (resetBtn) {
            const isDefault = !assignment;
            resetBtn.classList.toggle("active", isDefault);
            resetBtn.setAttribute("aria-pressed", isDefault ? "true" : "false");
        }
    }

    function updateGprPipeColorControls(obj) {
        const popup = ensureGprPipeInfoPopup();
        ensureGprPipeColorControls(popup);
        if (!popup) return;

        const assignment = getGprPipeAssignmentForObject(obj);
        const meta = obj && obj.userData ? (obj.userData.gprPipeMeta || {}) : {};
        const fallbackColorHex = normalizeGprPipeColorHex(meta.colorHex || GPR_PIPE_DEFAULT_COLOR_HEX);
        const activeColorHex = assignment ? normalizeGprPipeColorHex(assignment.colorHex) : fallbackColorHex;

        updateGprPipePopupTitle(assignment);

        const materialEl = $("gprPipePopupMaterial");
        if (materialEl) {
            materialEl.textContent = assignment ? assignment.materialName : "未指定（默认识别色）";
        }

        const swatchEl = $("gprPipePopupColorSwatch");
        if (swatchEl) {
            swatchEl.style.background = gprPipeColorHexToCss(activeColorHex);
            swatchEl.title = gprPipeColorHexToCss(activeColorHex);
        }

        refreshGprPipeMaterialButtonActiveState(popup, obj);
    }

    function setSelectedGprPipeMaterialColor(materialType) {
        const material = getGprPipeMaterialByType(materialType);
        if (!material) return;
        if (!selectedGprPipeObject || !selectedGprPipeObject.userData) {
            alert("请先点击三维视图中已经连接好的地下管道。 ");
            return;
        }

        const keys = getGprPipeColorKeysForObject(selectedGprPipeObject);
        const key = keys[0];
        if (!key) {
            alert("当前管道缺少可保存的颜色标识，请重新处理GPR数据后再试。 ");
            return;
        }

        // 只把新版本稳定 key 作为正式保存键；旧版坐标 hash 作为兼容别名时，删除旧记录避免同一管道出现两份材质配置。
        keys.slice(1).forEach(aliasKey => gprPipeColorAssignments.delete(String(aliasKey)));
        gprPipeColorAssignments.set(String(key), {
            pipeColorKey: String(key),
            materialType: material.type,
            materialName: material.name,
            colorHex: material.colorHex,
            colorCss: material.colorCss,
            updatedAt: Date.now()
        });

        applyGprPipeColorAssignmentToScene(key);
        updateGprPipeColorControls(selectedGprPipeObject);
        saveGprAnnotationCalibration("pipe-material-color-change");
        // 不在页面状态栏显示“已指定管道颜色”的提示，避免占用三维重建顶部提示区域。
        setStatus("", "");
        console.log(`✅ 已将选中地下管道指定为${material.name}（${material.colorCss}），并保存到GPR校准文件。`);
    }

    function resetSelectedGprPipeMaterialColor() {
        if (!selectedGprPipeObject || !selectedGprPipeObject.userData) return;
        const keys = getGprPipeColorKeysForObject(selectedGprPipeObject);
        const key = keys[0];
        if (!key) return;

        keys.forEach(aliasKey => gprPipeColorAssignments.delete(String(aliasKey)));
        applyGprPipeColorAssignmentToScene(key);
        updateGprPipeColorControls(selectedGprPipeObject);
        saveGprAnnotationCalibration("pipe-material-color-reset");
        // 不在页面状态栏显示“已恢复默认颜色”的提示，避免占用三维重建顶部提示区域。
        setStatus("", "");
        console.log("✅ 已恢复选中地下管道的默认识别颜色，并同步保存到GPR校准文件。");
    }

    function recomputeForcedPipeCountersFromLoadedData() {
        let maxGroupNo = 0;
        let maxPipeOrder = -1;

        forcedPipeAnnotationsBySegment.forEach(arr => {
            if (!Array.isArray(arr)) return;
            arr.forEach(item => {
                const groupNo = Number(item.pipeGroupNo || 0);
                if (Number.isFinite(groupNo)) maxGroupNo = Math.max(maxGroupNo, groupNo);
                const pipeOrder = Number(item.pipeOrder || 0);
                if (Number.isFinite(pipeOrder)) maxPipeOrder = Math.max(maxPipeOrder, pipeOrder);
            });
        });

        forcedPipeGroupCounter = Math.max(forcedPipeGroupCounter || 0, maxGroupNo);
        forcedPipeAnnotationOrder = Math.max(forcedPipeAnnotationOrder || 0, maxPipeOrder + 1);
    }

    function buildGprAnnotationCalibrationPayload(reason = "auto") {
        const folderInput = $("folderPathInput");
        const folderPath = normalizeGprFolderPath(currentGprFolderPath || (folderInput ? folderInput.value : ""));
        if (!folderPath || !gprResult || !Array.isArray(gprResult.segments) || !gprResult.segments.length) return null;

        currentGprFolderPath = folderPath;
        currentGprAnnotationStorageKey = getGprFolderAnnotationKey(folderPath);

        return {
            version: 6,
            savedAt: new Date().toISOString(),
            reason,
            folderPath,
            storage: "gpr-folder",
            annotationFileName: GPR_ANNOTATION_FILE_NAME,
            signature: getGprSegmentSignature(gprResult),
            manualMigAnnotationsBySegment: mapToStorageArray(manualMigAnnotationsBySegment, "array"),
            forcedPipeAnnotationsBySegment: mapToStorageArray(forcedPipeAnnotationsBySegment, "array"),
            gprPipeColorAssignments: serializeGprPipeColorAssignments(),
            forcedPipeGroupCounter,
            forcedPipeAnnotationOrder
        };
    }

    function saveGprAnnotationCalibration(reason = "auto") {
        const payload = buildGprAnnotationCalibrationPayload(reason);
        if (!payload) return false;

        fetch(GPR_ANNOTATION_API.save, {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({
                folderPath: payload.folderPath,
                payload: payload
            })
        })
            .then(res => {
                if (!res.ok) {
                    return res.text().then(text => {
                        throw new Error(text || ("HTTP " + res.status));
                    });
                }
                return res.json().catch(() => ({}));
            })
            .then(data => {
                const filePath = data && data.filePath ? data.filePath : currentGprAnnotationStorageKey;
                console.log("✅ GPR 标注校准已保存到机器狗主机文件夹：", filePath, reason);
            })
            .catch(e => {
                console.warn("⚠️ GPR 标注校准保存到机器狗主机失败：", e);
                setStatus("标注校准保存失败：请确认后端已加入 GprAnnotationController，且 GPR 数据文件夹可写。" + (e.message ? " " + e.message : ""), "error");
            });

        // 异步保存，先返回 true，避免影响标注交互流畅性。
        return true;
    }

    async function loadGprAnnotationCalibration(result, folderPath) {
        const normalizedFolder = normalizeGprFolderPath(folderPath || currentGprFolderPath || ($("folderPathInput") ? $("folderPathInput").value : ""));
        currentGprFolderPath = normalizedFolder;
        currentGprAnnotationStorageKey = getGprFolderAnnotationKey(normalizedFolder);

        if (!normalizedFolder) {
            return { loaded: false, reason: "empty-folder" };
        }

        let payload = null;
        try {
            const res = await fetch(GPR_ANNOTATION_API.load(normalizedFolder), {
                method: "GET",
                cache: "no-store"
            });

            if (!res.ok) {
                const text = await res.text().catch(() => "");
                console.warn("⚠️ 读取机器狗主机 GPR 标注校准失败：", res.status, text);
                return { loaded: false, reason: "read-error" };
            }

            const data = await res.json();
            if (!data || data.exists === false) {
                return { loaded: false, reason: "not-found" };
            }

            payload = data.payload || data;
        } catch (e) {
            console.warn("⚠️ 读取机器狗主机 GPR 标注校准失败：", e);
            return { loaded: false, reason: "read-error" };
        }

        try {
            if (!isSameGprSegmentSignature(payload.signature, result)) {
                console.warn("⚠️ 已找到该文件夹的历史标注，但剖面数量或文件名不一致，已跳过加载。", payload.signature, getGprSegmentSignature(result));
                return { loaded: false, reason: "signature-mismatch" };
            }

            restoreArrayMap(manualMigAnnotationsBySegment, payload.manualMigAnnotationsBySegment, "array");
            restoreArrayMap(forcedPipeAnnotationsBySegment, payload.forcedPipeAnnotationsBySegment, "array");
            restoreGprPipeColorAssignments(payload.gprPipeColorAssignments || payload.pipeColorAssignments);

            forcedPipeGroupCounter = Number(payload.forcedPipeGroupCounter || 0);
            forcedPipeAnnotationOrder = Number(payload.forcedPipeAnnotationOrder || 0);
            recomputeForcedPipeCountersFromLoadedData();

            const manualCount = Array.from(manualMigAnnotationsBySegment.values()).reduce((sum, arr) => sum + (Array.isArray(arr) ? arr.length : 0), 0);
            const forcedCount = getTotalForcedPipeAnnotationCount();
            const pipeColorCount = gprPipeColorAssignments.size;
            const hasRelevantCalibration = manualCount > 0 || forcedCount > 0 || pipeColorCount > 0;

            console.log("✅ 已从机器狗主机 GPR 文件夹加载标注校准：", {
                manualCount,
                forcedCount,
                pipeColorCount,
                file: currentGprAnnotationStorageKey
            });

            return {
                loaded: hasRelevantCalibration,
                manualCount,
                forcedCount,
                pipeColorCount,
                savedAt: payload.savedAt || "",
                filePath: currentGprAnnotationStorageKey
            };
        } catch (e) {
            console.warn("⚠️ GPR 标注校准数据解析失败：", e);
            return { loaded: false, reason: "parse-error" };
        }
    }

    async function clearCurrentGprAnnotationCalibration() {
        const folderInput = $("folderPathInput");
        const folderPath = normalizeGprFolderPath(currentGprFolderPath || (folderInput ? folderInput.value : ""));
        if (!folderPath) return;

        if (!confirm("确定清除当前 GPR 文件夹中的历史标注校准文件吗？清除后会恢复为自动识别标注。")) {
            return;
        }

        try {
            const res = await fetch(GPR_ANNOTATION_API.remove(folderPath), { method: "DELETE" });
            if (!res.ok) {
                const text = await res.text().catch(() => "");
                throw new Error(text || ("HTTP " + res.status));
            }
            console.log("✅ 已删除机器狗主机 GPR 文件夹中的标注校准文件：", getGprFolderAnnotationKey(folderPath));
        } catch (e) {
            console.warn("清除机器狗主机 GPR 标注校准失败：", e);
            setStatus("清除标注校准失败：" + (e.message || e), "error");
            return;
        }

        manualMigAnnotationsBySegment.clear();
        forcedPipeAnnotationsBySegment.clear();
        gprPipeColorAssignments.clear();
        selectedGprPipeObject = null;
        forcedPipeGroupCounter = 0;
        forcedPipeAnnotationOrder = 0;

        refreshCurrentMig2DCanvas();
        if (gprResult && Array.isArray(gprResult.segments) && gprResult.segments.length) {
            render3DZigzagAll(gprResult);
        }
        setStatus("已清除当前 GPR 文件夹中的历史人工校准，管线连接继续使用识别顶点。", "success");
    }

    // ==========================================
    // ✅ 偏移后二维剖面：手动红圈标注工具函数
    // ==========================================
    function getManualMigAnnotations(segIndex) {
        const key = Number(segIndex) || 0;
        if (!manualMigAnnotationsBySegment.has(key)) {
            manualMigAnnotationsBySegment.set(key, []);
        }
        return manualMigAnnotationsBySegment.get(key);
    }

    function cloneManualMigAnnotation(item, manualIndex = -1) {
        return {
            row: Number(item.row),
            col: Number(item.col),
            radius: Number(item.radius),
            width: Number(item.width || item.radius * 2),
            height: Number(item.height || item.radius * 2),
            manual: true,
            source: "manual",
            manualIndex
        };
    }

    // 三维管线的普通连接点只来自抛物线/双曲线识别结果中的顶点。
    // 它不读取旧版亮斑红圈、人工红圈或历史“忽略自动红圈”校准数据。
    function getDetectedApexPipePoints(seg) {
        const apexPoints = [];
        if (seg && Array.isArray(seg.rawDetections)) {
            try {
                seg.rawDetections.forEach((detection, detectionIndex) => {
                    if (detection.apexRow == null || detection.apexCol == null) return;
                    const row = Number(detection.apexRow);
                    const col = Number(detection.apexCol);
                    if (!Number.isFinite(row) || !Number.isFinite(col)) return;

                    const rows = seg.rawData?.length || 0;
                    const cols = seg.rawData?.[0]?.length || 0;
                    if (rows < 1 || cols < 1 || row < 0 || row >= rows || col < 0 || col >= cols) return;

                    const width = Math.abs(Number(detection.x2) - Number(detection.x1));
                    const height = Math.abs(Number(detection.y2) - Number(detection.y1));
                    const finiteWidth = Number.isFinite(width) ? width : 0;
                    const finiteHeight = Number.isFinite(height) ? height : 0;
                    const radius = Math.max(
                        CONFIG.minRadius + 1,
                        Math.min(CONFIG.maxRadius - 1, Math.min(finiteWidth, finiteHeight) / 4 || CONFIG.minRadius + 1)
                    );
                    apexPoints.push({
                        row,
                        col,
                        radius,
                        width: finiteWidth,
                        height: finiteHeight,
                        detectionIndex,
                        confidence: Number(detection.confidence),
                        apexScore: Number(detection.apexScore),
                        apexMethod: detection.apexMethod || "",
                        manual: false,
                        source: "detectedApex",
                        apexKey: `d${detectionIndex}_${row.toFixed(1)}_${col.toFixed(1)}`
                    });
                });
            } catch (e) {
                console.warn("原始剖面识别顶点转换为三维管线连接点失败：", e);
            }
        }
        return apexPoints;
    }

    function getManualMigAnnotationBlobs(segIndex) {
        return getManualMigAnnotations(segIndex)
            .map((item, idx) => cloneManualMigAnnotation(item, idx))
            .filter(b => Number.isFinite(b.row) && Number.isFinite(b.col) && Number.isFinite(b.radius));
    }

    function getForcedPipeAnnotations(segIndex) {
        const key = Number(segIndex) || 0;
        if (!forcedPipeAnnotationsBySegment.has(key)) {
            forcedPipeAnnotationsBySegment.set(key, []);
        }
        return forcedPipeAnnotationsBySegment.get(key);
    }

    function getForcedPipeColorByGroupNo(groupNo) {
        const colors = FORCED_PIPE_ANNOTATION.colors;
        const idx = Math.max(0, (Number(groupNo) || 1) - 1) % colors.length;
        return colors[idx];
    }

    function cloneForcedPipeAnnotation(item, forcedIndex = -1) {
        const colorHex = Number.isFinite(Number(item.pipeColorHex)) ? Number(item.pipeColorHex) : 0x0099ff;
        return {
            row: Number(item.row),
            col: Number(item.col),
            radius: Number(item.radius),
            width: Number(item.width || item.radius * 2),
            height: Number(item.height || item.radius * 2),
            manual: true,
            source: "forcedPipe",
            linkMode: "forced",
            forcedIndex,
            pipeGroupId: item.pipeGroupId || "forced_pipe_1",
            pipeGroupNo: Number(item.pipeGroupNo || 1),
            pipeColorHex: colorHex,
            pipeColorCss: item.pipeColorCss || ("#" + colorHex.toString(16).padStart(6, "0")),
            pipeColorName: item.pipeColorName || "彩色",
            pipeOrder: Number(item.pipeOrder || 0)
        };
    }

    function getForcedPipeAnnotationBlobs(segIndex) {
        return getForcedPipeAnnotations(segIndex)
            .map((item, idx) => cloneForcedPipeAnnotation(item, idx))
            .filter(b => Number.isFinite(b.row) && Number.isFinite(b.col) && Number.isFinite(b.radius));
    }

    function getTotalForcedPipeAnnotationCount() {
        return Array.from(forcedPipeAnnotationsBySegment.values()).reduce((sum, arr) => sum + arr.length, 0);
    }

    function updateGprMigrationUI() {
        const btn = $("btnToggleMigration");
        if (btn) {
            btn.textContent = "偏移成像: " + (gprMigrationEnabled ? "ON" : "OFF");
            btn.classList.toggle("active", gprMigrationEnabled);
            btn.setAttribute("aria-pressed", gprMigrationEnabled ? "true" : "false");
            btn.title = gprMigrationEnabled
                ? "已开启：下一次GPR处理将执行速度搜索和 Stolt F-K 偏移"
                : "已关闭：跳过速度搜索和 Stolt F-K 偏移（默认）";
        }

        const view = $("mig2dView");
        if (view) view.hidden = !gprMigrationEnabled;

        const views = document.querySelector(".gpr-views");
        if (views) views.classList.toggle("migration-disabled", !gprMigrationEnabled);

        const title = $("mig2dTitle");
        if (title) title.textContent = "偏移后二维剖面";

        if (!gprMigrationEnabled) {
            manualMigAnnotationMode = false;
            forcedPipeAnnotationMode = false;
            activeForcedPipeGroupId = null;
            clearGpr2DCanvasById("mig2dCanvas");
            updateManualMigAnnotationUI();
            updateForcedPipeAnnotationUI();
        }
    }

    function setGprMigrationEnabled(enabled, announce = false) {
        gprMigrationEnabled = !!enabled;
        updateGprMigrationUI();
        if (announce) {
            setStatus(
                gprMigrationEnabled
                    ? "偏移成像已开启，将在下一次点击“开始处理”时执行。"
                    : "偏移成像已关闭，下一次处理将直接进行双曲线及顶点识别。",
                "success"
            );
        }
    }

    function setAnnotationButtonState(enabled) {
        showAnnotation = !!enabled;
        const btnAnno = $("btnToggleAnnotation");
        if (btnAnno) {
            btnAnno.textContent = "显示标注: " + (showAnnotation ? "ON" : "OFF");
            btnAnno.classList.toggle("active", showAnnotation);
        }

        annotation3DGroups.forEach(g => g.visible = showAnnotation);
        if (window.gprSlicesGroup) {
            // 原始剖面上的识别框和顶点始终可见；按钮只控制管线连接标注。
            window.gprSlicesGroup.visible = true;
        }
    }

    function refreshCurrentMig2DCanvas() {
        if (!gprResult || !Array.isArray(gprResult.segments)) return;
        const seg = gprResult.segments[currentGprSegmentIndex];
        if (gprMigrationEnabled && seg && seg.migratedData) {
            render2DCanvas("mig2dCanvas", seg.migratedData);
        } else {
            clearGpr2DCanvasById("mig2dCanvas");
        }
    }

    function updateManualMigAnnotationUI() {
        const btn = $("btnManualMigAnnotation");
        const view = $("mig2dView");

        if (btn) {
            btn.textContent = manualMigAnnotationMode ? "结束标注" : "手动标注";
            btn.classList.toggle("active", manualMigAnnotationMode);
        }

        if (view) {
            view.classList.toggle("manual-annotation-active", manualMigAnnotationMode);
        }
    }

    function updateForcedPipeAnnotationUI() {
        const btn = $("btnForcedPipeAnnotation");
        const view = $("mig2dView");

        if (btn) {
            btn.textContent = forcedPipeAnnotationMode ? "结束倾斜标注" : "倾斜管道";
            btn.classList.toggle("active", forcedPipeAnnotationMode);
            if (forcedPipeAnnotationMode) {
                btn.style.borderColor = activeForcedPipeColorCss;
                btn.style.boxShadow = `0 0 12px ${activeForcedPipeColorCss}`;
            } else {
                btn.style.borderColor = "";
                btn.style.boxShadow = "";
            }
        }

        if (view) {
            view.classList.toggle("forced-pipe-annotation-active", forcedPipeAnnotationMode);
        }
    }

    function setManualMigAnnotationMode(enabled) {
        const nextEnabled = !!enabled;

        if (nextEnabled && (!gprResult || !Array.isArray(gprResult.segments) || !gprResult.segments.length)) {
            alert("请先点击“开始处理”并加载探地雷达剖面数据。 ");
            return;
        }

        if (nextEnabled && forcedPipeAnnotationMode) {
            forcedPipeAnnotationMode = false;
            activeForcedPipeGroupId = null;
            updateForcedPipeAnnotationUI();
        }

        manualMigAnnotationMode = nextEnabled;
        updateManualMigAnnotationUI();

        if (manualMigAnnotationMode) {
            // 手动标注时自动打开显示，确保新增的人工圆圈可见。
            setAnnotationButtonState(true);
            refreshCurrentMig2DCanvas();
            console.log("[GPR标注] 手动标注模式已开启");
        } else {
            refreshCurrentMig2DCanvas();

            // 人工红圈只用于二维校准展示，不参与普通管线的顶点连接。
            if (gprResult && Array.isArray(gprResult.segments) && gprResult.segments.length) {
                render3DZigzagAll(gprResult);
                saveGprAnnotationCalibration("manual-finish");
                const totalManual = Array.from(manualMigAnnotationsBySegment.values()).reduce((sum, arr) => sum + arr.length, 0);
                console.log("[GPR标注] 手动标注结束，当前手动红圈 " + totalManual + " 个");
            }
        }
    }

    function setForcedPipeAnnotationMode(enabled) {
        const nextEnabled = !!enabled;

        if (nextEnabled && (!gprResult || !Array.isArray(gprResult.segments) || !gprResult.segments.length)) {
            alert("请先点击“开始处理”并加载探地雷达剖面数据。 ");
            return;
        }

        if (nextEnabled) {
            // 倾斜管道模式与普通红圈手动模式互斥，避免同一次点击同时创建两类圆圈。
            manualMigAnnotationMode = false;
            updateManualMigAnnotationUI();

            forcedPipeGroupCounter += 1;
            const colorItem = getForcedPipeColorByGroupNo(forcedPipeGroupCounter);
            activeForcedPipeGroupId = "forced_pipe_" + forcedPipeGroupCounter;
            activeForcedPipeColorHex = colorItem.hex;
            activeForcedPipeColorCss = colorItem.css;
            activeForcedPipeColorName = colorItem.name;
        }

        forcedPipeAnnotationMode = nextEnabled;
        if (!forcedPipeAnnotationMode) {
            activeForcedPipeGroupId = null;
        }
        updateForcedPipeAnnotationUI();

        if (forcedPipeAnnotationMode) {
            setAnnotationButtonState(true);
            refreshCurrentMig2DCanvas();
            console.log(`[GPR标注] 倾斜管道标注模式已开启，当前颜色：${activeForcedPipeColorName}`);
        } else {
            refreshCurrentMig2DCanvas();
            if (gprResult && Array.isArray(gprResult.segments) && gprResult.segments.length) {
                render3DZigzagAll(gprResult);
                saveGprAnnotationCalibration("forced-pipe-finish");
                console.log("[GPR标注] 倾斜管道标注结束，当前彩色圆圈 " + getTotalForcedPipeAnnotationCount() + " 个");
            }
        }
    }

    function getCanvasContentPoint(canvas, event) {
        if (!canvas) return null;
        const rect = canvas.getBoundingClientRect();
        if (!rect.width || !rect.height || !canvas.width || !canvas.height) return null;

        // canvas 使用 object-fit: contain 时，真实图像可能有上下/左右留黑边。
        const scale = Math.min(rect.width / canvas.width, rect.height / canvas.height);
        const displayW = canvas.width * scale;
        const displayH = canvas.height * scale;
        const offsetX = (rect.width - displayW) / 2;
        const offsetY = (rect.height - displayH) / 2;

        const x = (event.clientX - rect.left - offsetX) / scale;
        const y = (event.clientY - rect.top - offsetY) / scale;

        if (x < 0 || y < 0 || x > canvas.width || y > canvas.height) return null;
        return { x, y };
    }

    function addManualMigAnnotationAt(canvasPoint) {
        const info = gpr2DRenderInfo.mig2dCanvas;
        if (!info) return false;

        const segIndex = currentGprSegmentIndex;
        const col = Math.max(0, Math.min(info.cols - 1, canvasPoint.x * info.stepX));
        const row = Math.max(0, Math.min(info.rows - 1, canvasPoint.y * info.stepY));

        // 让手动红圈在当前显示画布中保持约 12px 的视觉半径，同时在 3D 中能转换为合理物理半径。
        const pixelScale = Math.max(info.stepX, info.stepY, 1);
        const matrixRadius = Math.max(
            CONFIG.minRadius + 1,
            MANUAL_MIG_ANNOTATION.displayRadiusPx * pixelScale - CONFIG.radiusPadding
        );

        getManualMigAnnotations(segIndex).push({
            row,
            col,
            radius: matrixRadius,
            width: matrixRadius * 2,
            height: matrixRadius * 2,
            manual: true,
            createdAt: Date.now()
        });

        saveGprAnnotationCalibration("manual-add");
        refreshCurrentMig2DCanvas();
        console.log("[GPR标注] 已添加 1 个手动红圈");
        return true;
    }

    function removeNearestMigAnnotation(canvasPoint) {
        const info = gpr2DRenderInfo.mig2dCanvas;
        if (!info) return false;

        const candidates = getManualMigAnnotationBlobs(currentGprSegmentIndex);
        if (!candidates.length) {
            console.log("[GPR标注] 当前剖面没有可删除的红圈");
            return false;
        }

        let best = null;
        let bestDist = Infinity;

        candidates.forEach(item => {
            const x = Number(item.col) / info.stepX;
            const y = Number(item.row) / info.stepY;
            const dist = Math.hypot(x - canvasPoint.x, y - canvasPoint.y);
            if (dist < bestDist) {
                bestDist = dist;
                best = item;
            }
        });

        if (!best || bestDist > MANUAL_MIG_ANNOTATION.deleteHitRadiusPx) {
            console.log("[GPR标注] 附近没有可删除的红圈");
            return false;
        }

        if (best.source === "manual") {
            const arr = getManualMigAnnotations(currentGprSegmentIndex);
            const idx = Number(best.manualIndex);
            if (idx >= 0 && idx < arr.length) {
                arr.splice(idx, 1);
                saveGprAnnotationCalibration("manual-delete");
                refreshCurrentMig2DCanvas();
                console.log("[GPR标注] 已删除最近的 1 个手动红圈");
                return true;
            }
        }

        console.log("[GPR标注] 删除失败：没有识别到红圈来源");
        return false;
    }

    function addForcedPipeAnnotationAt(canvasPoint) {
        const info = gpr2DRenderInfo.mig2dCanvas;
        if (!info || !activeForcedPipeGroupId) return false;

        const segIndex = currentGprSegmentIndex;
        const col = Math.max(0, Math.min(info.cols - 1, canvasPoint.x * info.stepX));
        const row = Math.max(0, Math.min(info.rows - 1, canvasPoint.y * info.stepY));

        const pixelScale = Math.max(info.stepX, info.stepY, 1);
        const matrixRadius = Math.max(
            CONFIG.minRadius + 1,
            FORCED_PIPE_ANNOTATION.displayRadiusPx * pixelScale - CONFIG.radiusPadding
        );

        getForcedPipeAnnotations(segIndex).push({
            row,
            col,
            radius: matrixRadius,
            width: matrixRadius * 2,
            height: matrixRadius * 2,
            manual: true,
            pipeGroupId: activeForcedPipeGroupId,
            pipeGroupNo: forcedPipeGroupCounter,
            pipeColorHex: activeForcedPipeColorHex,
            pipeColorCss: activeForcedPipeColorCss,
            pipeColorName: activeForcedPipeColorName,
            pipeOrder: forcedPipeAnnotationOrder++,
            createdAt: Date.now()
        });

        saveGprAnnotationCalibration("forced-pipe-add");
        refreshCurrentMig2DCanvas();
        console.log(`[GPR标注] 已添加 1 个${activeForcedPipeColorName}倾斜管道圆圈`);
        return true;
    }

    function removeNearestForcedPipeAnnotation(canvasPoint) {
        const info = gpr2DRenderInfo.mig2dCanvas;
        if (!info) return false;

        const candidates = getForcedPipeAnnotationBlobs(currentGprSegmentIndex);
        if (!candidates.length) {
            console.log("[GPR标注] 当前剖面没有可删除的彩色倾斜管道圆圈");
            return false;
        }

        let best = null;
        let bestDist = Infinity;

        candidates.forEach(item => {
            const x = Number(item.col) / info.stepX;
            const y = Number(item.row) / info.stepY;
            const dist = Math.hypot(x - canvasPoint.x, y - canvasPoint.y);
            if (dist < bestDist) {
                bestDist = dist;
                best = item;
            }
        });

        if (!best || bestDist > FORCED_PIPE_ANNOTATION.deleteHitRadiusPx) {
            console.log("[GPR标注] 附近没有可删除的彩色圆圈");
            return false;
        }

        const arr = getForcedPipeAnnotations(currentGprSegmentIndex);
        const idx = Number(best.forcedIndex);
        if (idx >= 0 && idx < arr.length) {
            arr.splice(idx, 1);
            saveGprAnnotationCalibration("forced-pipe-delete");
            refreshCurrentMig2DCanvas();
            console.log("[GPR标注] 已删除最近的 1 个彩色倾斜管道圆圈");
            return true;
        }

        console.log("[GPR标注] 删除失败：没有识别到彩色圆圈来源");
        return false;
    }

    function clampCanvasPointToMatrix(canvasPoint) {
        const info = gpr2DRenderInfo.mig2dCanvas;
        if (!info || !canvasPoint) return null;

        return {
            col: Math.max(0, Math.min(info.cols - 1, canvasPoint.x * info.stepX)),
            row: Math.max(0, Math.min(info.rows - 1, canvasPoint.y * info.stepY))
        };
    }

    function getNearestNormalMigAnnotationTarget(canvasPoint) {
        const info = gpr2DRenderInfo.mig2dCanvas;
        if (!info) return null;

        const candidates = getManualMigAnnotationBlobs(currentGprSegmentIndex);
        if (!candidates.length) return null;

        let best = null;
        let bestDist = Infinity;

        candidates.forEach(item => {
            const x = Number(item.col) / info.stepX;
            const y = Number(item.row) / info.stepY;
            const dist = Math.hypot(x - canvasPoint.x, y - canvasPoint.y);
            if (dist < bestDist) {
                bestDist = dist;
                best = item;
            }
        });

        if (!best || bestDist > MANUAL_MIG_ANNOTATION.deleteHitRadiusPx) return null;
        return { item: best, distance: bestDist };
    }

    function getNearestForcedPipeAnnotationTarget(canvasPoint) {
        const info = gpr2DRenderInfo.mig2dCanvas;
        if (!info) return null;

        const candidates = getForcedPipeAnnotationBlobs(currentGprSegmentIndex);
        if (!candidates.length) return null;

        let best = null;
        let bestDist = Infinity;

        candidates.forEach(item => {
            const x = Number(item.col) / info.stepX;
            const y = Number(item.row) / info.stepY;
            const dist = Math.hypot(x - canvasPoint.x, y - canvasPoint.y);
            if (dist < bestDist) {
                bestDist = dist;
                best = item;
            }
        });

        if (!best || bestDist > FORCED_PIPE_ANNOTATION.deleteHitRadiusPx) return null;
        return { item: best, distance: bestDist };
    }

    function setMigAnnotationCursor(cursor) {
        const canvas = $("mig2dCanvas");
        if (canvas) canvas.style.cursor = cursor || "";
    }

    function updateManualAnnotationByDragState(canvasPoint) {
        if (!gprAnnotationDragState) return false;

        const matrixPoint = clampCanvasPointToMatrix(canvasPoint);
        if (!matrixPoint) return false;

        const segIndex = gprAnnotationDragState.segIndex;

        if (gprAnnotationDragState.type === "manual") {
            const arr = getManualMigAnnotations(segIndex);
            const idx = gprAnnotationDragState.index;
            if (idx < 0 || idx >= arr.length) return false;
            arr[idx].row = matrixPoint.row;
            arr[idx].col = matrixPoint.col;
            arr[idx].updatedAt = Date.now();
            return true;
        }

        if (gprAnnotationDragState.type === "forcedPipe") {
            const arr = getForcedPipeAnnotations(segIndex);
            const idx = gprAnnotationDragState.index;
            if (idx < 0 || idx >= arr.length) return false;
            arr[idx].row = matrixPoint.row;
            arr[idx].col = matrixPoint.col;
            arr[idx].updatedAt = Date.now();
            return true;
        }

        return false;
    }

    function beginNormalMigAnnotationDrag(canvasPoint) {
        const target = getNearestNormalMigAnnotationTarget(canvasPoint);
        if (!target) return false;

        const item = target.item;
        const segIndex = currentGprSegmentIndex;

        if (item.source === "manual") {
            const idx = Number(item.manualIndex);
            if (idx < 0) return false;
            gprAnnotationDragState = {
                type: "manual",
                segIndex,
                index: idx,
                startPoint: canvasPoint,
                moved: false,
                lastRefreshAt: 0
            };
            return true;
        }

        return false;
    }

    function beginForcedPipeAnnotationDrag(canvasPoint) {
        const target = getNearestForcedPipeAnnotationTarget(canvasPoint);
        if (!target) return false;

        const item = target.item;
        const idx = Number(item.forcedIndex);
        if (idx < 0) return false;

        gprAnnotationDragState = {
            type: "forcedPipe",
            segIndex: currentGprSegmentIndex,
            index: idx,
            startPoint: canvasPoint,
            moved: false,
            lastRefreshAt: 0
        };
        return true;
    }

    function handleMigAnnotationPointerDown(event) {
        if (!manualMigAnnotationMode && !forcedPipeAnnotationMode) return;

        event.preventDefault();
        event.stopPropagation();

        const canvas = $("mig2dCanvas");
        const pt = getCanvasContentPoint(canvas, event);
        if (!pt) return;

        const isDeleteAction = event.ctrlKey || event.metaKey || event.shiftKey || event.button === 2;
        if (isDeleteAction) {
            if (forcedPipeAnnotationMode) {
                removeNearestForcedPipeAnnotation(pt);
            } else {
                removeNearestMigAnnotation(pt);
            }
            gprAnnotationDragState = null;
            gprAnnotationPointerDownState = null;
            suppressNextMigAnnotationPointerUp = true;
            return;
        }

        gprAnnotationPointerDownState = {
            startPoint: pt,
            latestPoint: pt,
            moved: false
        };

        const started = forcedPipeAnnotationMode
            ? beginForcedPipeAnnotationDrag(pt)
            : beginNormalMigAnnotationDrag(pt);

        if (started) {
            if (canvas && typeof canvas.setPointerCapture === "function" && event.pointerId !== undefined) {
                try { canvas.setPointerCapture(event.pointerId); } catch (e) { }
            }
            setMigAnnotationCursor("grabbing");
        }
    }

    function handleMigAnnotationPointerMove(event) {
        if (!manualMigAnnotationMode && !forcedPipeAnnotationMode) return;

        const canvas = $("mig2dCanvas");
        const pt = getCanvasContentPoint(canvas, event);
        if (!pt) return;

        if (!gprAnnotationDragState && !gprAnnotationPointerDownState) {
            // 悬停到已有圆圈附近时给出 move 光标提示。
            const hoverTarget = forcedPipeAnnotationMode
                ? getNearestForcedPipeAnnotationTarget(pt)
                : getNearestNormalMigAnnotationTarget(pt);
            setMigAnnotationCursor(hoverTarget ? "grab" : "crosshair");
            return;
        }

        event.preventDefault();
        event.stopPropagation();

        if (gprAnnotationPointerDownState) {
            gprAnnotationPointerDownState.latestPoint = pt;
            const d = Math.hypot(
                pt.x - gprAnnotationPointerDownState.startPoint.x,
                pt.y - gprAnnotationPointerDownState.startPoint.y
            );
            if (d >= GPR_ANNOTATION_DRAG.startThresholdPx) {
                gprAnnotationPointerDownState.moved = true;
            }
        }

        if (!gprAnnotationDragState) return;

        const dragD = Math.hypot(pt.x - gprAnnotationDragState.startPoint.x, pt.y - gprAnnotationDragState.startPoint.y);
        if (dragD >= GPR_ANNOTATION_DRAG.startThresholdPx) {
            gprAnnotationDragState.moved = true;
        }

        if (!gprAnnotationDragState.moved) return;

        if (updateManualAnnotationByDragState(pt)) {
            const now = Date.now();
            if (now - (gprAnnotationDragState.lastRefreshAt || 0) >= GPR_ANNOTATION_DRAG.refreshIntervalMs) {
                gprAnnotationDragState.lastRefreshAt = now;
                refreshCurrentMig2DCanvas();
            }
        }
    }

    function handleMigAnnotationPointerUp(event) {
        if (!manualMigAnnotationMode && !forcedPipeAnnotationMode) return;

        event.preventDefault();
        event.stopPropagation();

        const canvas = $("mig2dCanvas");

        // ✅ 右键 / Ctrl / Shift 删除已经在 pointerdown 中完成。
        // pointerup 不应继续走“空白位置新增圆圈”的逻辑，否则会出现“右键删除后反而新增”的现象。
        const isDeletePointerUp = suppressNextMigAnnotationPointerUp
            || event.button === 2
            || event.ctrlKey
            || event.metaKey
            || event.shiftKey;
        if (isDeletePointerUp) {
            suppressNextMigAnnotationPointerUp = false;
            gprAnnotationDragState = null;
            gprAnnotationPointerDownState = null;
            setMigAnnotationCursor("crosshair");
            return;
        }

        const pt = getCanvasContentPoint(canvas, event) || (gprAnnotationPointerDownState ? gprAnnotationPointerDownState.latestPoint : null);
        const downState = gprAnnotationPointerDownState;
        const dragState = gprAnnotationDragState;

        if (canvas && typeof canvas.releasePointerCapture === "function" && event.pointerId !== undefined) {
            try { canvas.releasePointerCapture(event.pointerId); } catch (e) { }
        }

        gprAnnotationPointerDownState = null;
        setMigAnnotationCursor("crosshair");

        if (!pt) {
            gprAnnotationDragState = null;
            return;
        }

        if (dragState) {
            const reallyMoved = dragState.moved || (downState && downState.moved);

            // 只是单击已有人工圆圈但没有拖动时，不改变标注。
            if (!reallyMoved) {
                gprAnnotationDragState = null;
                return;
            }

            updateManualAnnotationByDragState(pt);
            gprAnnotationDragState = null;
            saveGprAnnotationCalibration(dragState.type === "forcedPipe" ? "forced-pipe-drag" : "manual-drag");
            refreshCurrentMig2DCanvas();
            if (dragState.type === "forcedPipe") {
                console.log("[GPR标注] 已拖动彩色倾斜管道圆圈");
            } else {
                console.log("[GPR标注] 已拖动手动红圈");
            }
            return;
        }

        // 没有拖到已有圆圈时，短按空白位置仍按原逻辑新增圆圈。
        if (downState && downState.moved) return;

        if (forcedPipeAnnotationMode) {
            addForcedPipeAnnotationAt(pt);
        } else {
            addManualMigAnnotationAt(pt);
        }
    }

    function handleMigAnnotationPointerCancel(event) {
        if (gprAnnotationDragState) {
            saveGprAnnotationCalibration(gprAnnotationDragState.type === "forcedPipe" ? "forced-pipe-drag-cancel" : "manual-drag-cancel");
            refreshCurrentMig2DCanvas();
        }
        gprAnnotationDragState = null;
        gprAnnotationPointerDownState = null;
        setMigAnnotationCursor("crosshair");
    }

    function initManualMigAnnotationFeature() {
        const btn = $("btnManualMigAnnotation");
        const btnForcedPipe = $("btnForcedPipeAnnotation");
        const btnClearCalibration = $("btnClearGprAnnotationCalibration");
        const canvas = $("mig2dCanvas");

        if (btn) {
            btn.addEventListener("click", function () {
                setManualMigAnnotationMode(!manualMigAnnotationMode);
            });
        }

        if (btnForcedPipe) {
            btnForcedPipe.addEventListener("click", function () {
                setForcedPipeAnnotationMode(!forcedPipeAnnotationMode);
            });
        }

        if (btnClearCalibration) {
            btnClearCalibration.addEventListener("click", clearCurrentGprAnnotationCalibration);
        }

        if (canvas) {
            canvas.addEventListener("pointerdown", handleMigAnnotationPointerDown);
            canvas.addEventListener("pointermove", handleMigAnnotationPointerMove);
            canvas.addEventListener("pointerup", handleMigAnnotationPointerUp);
            canvas.addEventListener("pointercancel", handleMigAnnotationPointerCancel);
            canvas.addEventListener("pointerleave", function (event) {
                if (gprAnnotationDragState) return;
                setMigAnnotationCursor("");
            });
            canvas.addEventListener("contextmenu", function (event) {
                if (!manualMigAnnotationMode && !forcedPipeAnnotationMode) return;
                event.preventDefault();
                event.stopPropagation();
            });
        }

        updateManualMigAnnotationUI();
        updateForcedPipeAnnotationUI();
    }

    // SLAM 本地坐标 (x,y) 转 全局 GPS 经纬度 (lat,lon)
    window.slamToGps = function (targetX, targetY) {
        // ✅ 新版优先：多点轨迹相似变换，包含 scale / rotation / translation。
        if (window.geoTransform) {
            const pred = transformPointByGeoTransform(window.geoTransform, targetX, targetY);
            if (!pred) return null;
            return localMetersToGps(pred.east, pred.north, window.geoTransform.originLat, window.geoTransform.originLon);
        }

        // ✅ 旧版兼容：gpsAnchor + slamAnchor + headingOffset。
        if (!window.isGpsCalibrated || !window.gpsAnchor || !window.slamAnchor) {
            return null;
        }

        const dx = Number(targetX) - Number(window.slamAnchor.x);
        const dy = Number(targetY) - Number(window.slamAnchor.y);
        const trueDx = dx * Math.cos(window.headingOffset) - dy * Math.sin(window.headingOffset);
        const trueDy = dx * Math.sin(window.headingOffset) + dy * Math.cos(window.headingOffset);
        return localMetersToGps(trueDx, trueDy, window.gpsAnchor.lat, window.gpsAnchor.lon);
    };

    // ====== 🌍 全球 GPS 经纬度转 SLAM 本地坐标 (x,y) ======
    window.gpsToSlam = function (lat, lon) {
        // ✅ 新版优先：多点轨迹相似变换的逆变换。
        if (window.geoTransform) {
            const q = gpsToLocalMeters(lat, lon, window.geoTransform.originLat, window.geoTransform.originLon);
            return inverseTransformPointByGeoTransform(window.geoTransform, q.east, q.north);
        }

        // ✅ 旧版兼容。
        if (!window.isGpsCalibrated || !window.gpsAnchor || !window.slamAnchor) {
            return null;
        }

        const q = gpsToLocalMeters(lat, lon, window.gpsAnchor.lat, window.gpsAnchor.lon);
        const cosO = Math.cos(window.headingOffset);
        const sinO = Math.sin(window.headingOffset);
        const dx = q.east * cosO + q.north * sinO;
        const dy = -q.east * sinO + q.north * cosO;
        return {
            x: window.slamAnchor.x + dx,
            y: window.slamAnchor.y + dy
        };
    };

    // ==========================================
    // 2. 页面初始化逻辑 (DOMContentLoaded)
    // ==========================================

    document.addEventListener("DOMContentLoaded", function () {

        // ✅ 启动机械臂末端摄像头实时视频流
        initArmEndCameraStream();
        initRobotWideCameraStreams().catch(error => {
            console.error("机器狗前后摄像头初始化失败:", error);
        });
        initSystemMessageCenter();

        // ✅ 初始化所有浮动滑块的位置
        ['manualSpeedX', 'manualSpeedY', 'autoSpeedX', 'autoSpeedY'].forEach(id => {
            const input = document.getElementById(id);
            if (input) {
                // 根据 input 的 id 推导对应的 tooltip id
                let tipId = '';
                if (id === 'manualSpeedX') tipId = 'valManX';
                if (id === 'manualSpeedY') tipId = 'valManY';
                if (id === 'autoSpeedX') tipId = 'valAutoX';
                if (id === 'autoSpeedY') tipId = 'valAutoY';

                window.updateFloatingThumb(input, tipId);
            }
        });
        // ==========================================
        // ✅ [修改] 初始化所有 AUC 算法参数显示 (Z-score, Pulse, AUC)
        // ==========================================
        // 说明：原有的 sensitivitySlider 滑条已被移除，现在三个参数统一使用加减按钮控制。
        // 这里只需要负责页面加载时，把 AUC_CONFIG 里的默认值填入界面即可。

        // 1. 获取显示数值的 DOM 元素
        const elZ = document.getElementById('sensitivityDisplay');       // Z-score 显示
        const elPulse = document.getElementById('disp_pulse_threshold'); // 脉冲阈值显示
        const elAuc = document.getElementById('disp_auc_threshold');     // 能量积分显示

        // 2. 将全局配置 (AUC_CONFIG) 中的默认值填入 UI
        if (elZ) {
            elZ.textContent = AUC_CONFIG.NOISE_THRESHOLD.toFixed(1); // 保留1位小数
        }
        if (elPulse) {
            elPulse.textContent = AUC_CONFIG.PULSE_THRESHOLD;        // 整数
        }
        if (elAuc) {
            elAuc.textContent = AUC_CONFIG.AUC_THRESHOLD;            // 整数
        }

        syncFaultWaitSettingsUi();
        const btnFaultWaitSettings = $("btnFaultWaitSettings");
        if (btnFaultWaitSettings) {
            btnFaultWaitSettings.onclick = openFaultWaitParamModal;
        }

        const headerDropdownGroups = Array.from(document.querySelectorAll('.header-btn-group .dropdown-group'));
        function closeHeaderDropdowns(exceptGroup) {
            headerDropdownGroups.forEach(group => {
                if (group !== exceptGroup) {
                    group.classList.remove('open');
                }
            });
        }

        headerDropdownGroups.forEach(group => {
            const toggle = group.querySelector('.dropdown-toggle');
            const content = group.querySelector('.dropdown-content');
            if (!toggle) return;

            if (content) {
                content.addEventListener('click', function (e) {
                    e.stopPropagation();
                });
            }

            toggle.addEventListener('click', function (e) {
                e.preventDefault();
                e.stopPropagation();
                const willOpen = !group.classList.contains('open');
                closeHeaderDropdowns(group);
                group.classList.toggle('open', willOpen);
            });
        });

        document.addEventListener('click', function () {
            closeHeaderDropdowns();
        });

        document.addEventListener('keydown', function (e) {
            if (e.key === 'Escape') {
                closeHeaderDropdowns();
            }
        });

        // --- A. 获取 DOM ---
        const btn3D = $("btn3DReconstruct");
        const btnRobotNav = $("btnRobotNav");
        const btnPathFault = $("btnPathFault");
        // ✅ 获取新增的机械臂切换按钮
        const btnShowArm = $("btnShowArm");

        const gprPanel = $("gprPanel");
        const robotPanel = $("robotNavPanel");
        // ✅ 获取新增的机械臂控制面板 (如果按照之前步骤添加了 id="armPanel")
        // 如果你的 HTML 中没有 id="armPanel"，请给那个包含 .control-group-box 的 div 加上 id="armPanel"
        // 建议：直接复用 robot-right-col 的父级容器作为 armPanel
        // 或者我们假设 armPanel 是一个新的独立 div，如果不存就在 switchMainPanel 里做兼容
        const armPanel = $("armPanel");
        // ✅ 新增：获取故障定点面板
        const pathFaultPanel = $("pathFaultPanel");

        const btnStart = $("btnStartGpr");
        const btnLoadCurrentPointCloudMap = $("btnLoadCurrentPointCloudMap"); // ✅ 加载路径页当前点云地图
        const btnClearGprPointCloudMap = $("btnClearGprPointCloudMap"); // ✅ 清除三维重建页点云地图

        if (btnLoadCurrentPointCloudMap) {
            btnLoadCurrentPointCloudMap.addEventListener("click", loadCurrentPathFaultMapToGpr);
        }
        if (btnClearGprPointCloudMap) {
            btnClearGprPointCloudMap.addEventListener("click", clearGprPointCloudMapFromPage);
        }

        // 👇========= 新增：读取电脑本地下载的 PCD 点云 =========👇
        const btnLoadLocalPcd = $("btnLoadLocalPcd");
        const pcdLocalFileInput = $("pcdLocalFileInput");

        function sanitizeLocalPcdPointsForGpr(points, maxDisplayPoints = 300000) {
            if (!points || !points.geometry || typeof THREE === "undefined") return points;

            const geom = points.geometry;
            const posAttr = geom.getAttribute && geom.getAttribute("position");
            if (!posAttr || !posAttr.array || posAttr.count <= 0) return points;

            const colAttr = geom.getAttribute && geom.getAttribute("color");
            const srcPos = posAttr.array;
            const srcCol = colAttr && colAttr.array ? colAttr.array : null;

            // 与路径查找页导出到三维重建页的逻辑一致：过滤无效点和极端离群点。
            // 本地点云如果直接把所有原始点都放进 GPR 场景，远处离群点会拉大包围盒，
            // 大点数也会让鼠标旋转/拖拽时重绘卡顿。
            const valid = [];
            for (let i = 0; i < srcPos.length; i += 3) {
                const x = srcPos[i];
                const y = srcPos[i + 1];
                const z = srcPos[i + 2];
                if (
                    Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(z) &&
                    Math.abs(x) < 9000 && Math.abs(y) < 9000 && Math.abs(z) < 9000
                ) {
                    valid.push(i);
                }
            }

            if (!valid.length) return points;

            const stride = Math.max(1, Math.ceil(valid.length / maxDisplayPoints));
            const keepCount = Math.ceil(valid.length / stride);
            const dstPos = new Float32Array(keepCount * 3);
            const dstCol = srcCol ? new Float32Array(keepCount * 3) : null;

            let j = 0;
            for (let k = 0; k < valid.length; k += stride) {
                const i = valid[k];
                dstPos[j] = srcPos[i];
                dstPos[j + 1] = srcPos[i + 1];
                dstPos[j + 2] = srcPos[i + 2];

                if (dstCol) {
                    dstCol[j] = srcCol[i] ?? 1.0;
                    dstCol[j + 1] = srcCol[i + 1] ?? 1.0;
                    dstCol[j + 2] = srcCol[i + 2] ?? 1.0;
                }
                j += 3;
            }

            const newGeom = new THREE.BufferGeometry();
            newGeom.setAttribute("position", new THREE.BufferAttribute(dstPos, 3));
            if (dstCol) newGeom.setAttribute("color", new THREE.BufferAttribute(dstCol, 3));

            if (geom.dispose) geom.dispose();
            points.geometry = newGeom;

            if (!points.material) {
                points.material = new THREE.PointsMaterial({ size: 0.05, sizeAttenuation: true });
            }
            points.material.size = 0.05;
            points.material.sizeAttenuation = true;
            points.material.vertexColors = !!dstCol;
            points.material.needsUpdate = true;
            points.frustumCulled = false;

            console.log(`✅ 本地点云显示优化：原始有效点 ${valid.length} 个，三维重建页显示 ${keepCount} 个。`);
            return points;
        }

        function addLoadedPcdPointsToGprScene(points, sourceName, geoMeta = null) {
            clearCurrentGprExternalMapLayer();

            points = sanitizeLocalPcdPointsForGpr(points);
            points.material.size = 0.05;
            points.geometry.computeBoundingBox();

            const hasValidGeoMeta = !!(geoMeta && typeof geoMeta === "object");

            // 与原“加载点云”按钮保持一致：SLAM 点云 Z 轴向上，GPR 三维页 Y 轴向上。
            // 有 geo.json 时不能 geometry.center()，否则原始 SLAM 坐标会被平移，后续经纬度换算会失真。
            points.rotation.x = -Math.PI / 2;

            const pcdGroup = new THREE.Group();
            pcdGroup.name = sourceName || "本地点云文件";
            pcdGroup.add(points);

            // 新版 .geo.json 会随点云保存黄色电缆矢量路径；加载本地图时一并恢复，
            // 供后续 GPR 地下管道自动校正。旧版元数据没有该字段时保持原行为。
            const localCablePath = extractCablePathFromGprGeoMetadata(geoMeta);
            const localCableMesh = createGprSurfaceCablePathMesh(localCablePath);
            if (localCableMesh) pcdGroup.add(localCableMesh);

            if (hasValidGeoMeta) {
                pcdGroup.position.set(0, 0, 0);
                pcdGroup.rotation.y = 0;
                pcdGroup.userData.geoMetadata = geoMeta;
                applyLoadedMapGeoMetadata(geoMeta, sourceName);
            } else {
                // 没有 geo.json 的旧点云只用于视觉查看，仍保持之前的居中和现场对齐参数。
                points.geometry.center();
                points.geometry.computeBoundingBox();
                const bbox = points.geometry.boundingBox;
                const zHeight = bbox ? (bbox.max.z - bbox.min.z) : 0;
                points.position.y = zHeight / 2;

                pcdGroup.position.x = 5.25;
                pcdGroup.position.z = 5.40;
                pcdGroup.rotation.y = 1.8501;
            }

            window.gprScene.add(pcdGroup);

            window.currentPcdMesh = points;
            window.currentPcdGroup = pcdGroup;
            // ✅ 关键修复：本地点云也视为三维重建页已加载的地图层。
            // 否则规划 Z 字路径时 renderScanPlanOnGpr/startManualScanAreaPlanning 会自动调用
            // loadCurrentPathFaultMapToGpr()，把本地点云清掉并换成路径查找页面的地图，
            // 造成“规划后突然变丝滑”的假象。
            window.currentImportedInspectionMapGroup = pcdGroup;
            window.gprHasInspectionMapLayerLoaded = true;
            freezeLocalPcdMapForGpr(points, pcdGroup, sourceName, hasValidGeoMeta ? geoMeta : null);
            updateGprCableAlignmentBadge(
                localCablePath.length >= 2 ? "pending" : "unavailable",
                localCablePath.length >= 2 ? "路径校正：待匹配" : "路径校正：无黄色路径",
                localCablePath.length >= 2
                    ? `本地点云元数据中已恢复 ${localCablePath.length} 个黄色电缆路径点。`
                    : "所选本地点云的元数据中没有黄色电缆矢量路径；地下管道将保留 GPR 原始位置。"
            );

            // ✅ 本地点云加载完成后，不再取整幅点云包围盒中心。
            // 直接以“当前点云地图自身坐标原点/中心原点”为中心构造一个小 box，
            // 再调用和 Z 字路径规划相同的 updateGprCameraByBox()。
            // 这样只加载本地点云时，相机的 orbitTarget / radius / controlSpan
            // 就不会被整幅点云范围或离群点放大，操作手感会接近规划 Z 字路径后的状态。
            const planLikeSpan = Math.max(DEFAULT_SCAN_PARAMS.length, DEFAULT_SCAN_PARAMS.width, 2.5);
            const mapOriginCenter = pcdGroup.localToWorld(new THREE.Vector3(0, 0, 0));
            const planLikeBox = new THREE.Box3(
                new THREE.Vector3(mapOriginCenter.x - planLikeSpan / 2, mapOriginCenter.y - 0.25, mapOriginCenter.z - planLikeSpan / 2),
                new THREE.Vector3(mapOriginCenter.x + planLikeSpan / 2, mapOriginCenter.y + 0.25, mapOriginCenter.z + planLikeSpan / 2)
            );
            updateGprCameraByBox(planLikeBox);
            console.log("✅ 本地点云已按地图中心原点构造小 box 触发相机控制：", {
                center: {
                    x: Number(mapOriginCenter.x.toFixed(3)),
                    y: Number(mapOriginCenter.y.toFixed(3)),
                    z: Number(mapOriginCenter.z.toFixed(3))
                },
                span: planLikeSpan
            });
        }

        if (btnLoadLocalPcd && pcdLocalFileInput) {
            btnLoadLocalPcd.addEventListener("click", function () {
                pcdLocalFileInput.value = "";
                pcdLocalFileInput.click();
            });

            pcdLocalFileInput.addEventListener("change", async function () {
                const files = Array.from(this.files || []);
                if (!files.length) return;

                const pcdFile = files.find(f => /\.pcd$/i.test(f.name));
                const geoFile = files.find(f => /\.geo\.json$/i.test(f.name)) || files.find(f => /\.json$/i.test(f.name));

                if (!pcdFile) {
                    alert("请选择 .pcd 点云文件；如果需要恢复经纬度，请同时选择同名 .geo.json 文件。 ");
                    return;
                }

                if (!window.gprScene) {
                    try {
                        ensureStandaloneGprSceneForMap();
                    } catch (e) {
                        console.error("三维视图初始化失败:", e);
                        alert("三维视图初始化失败：" + (e.message || e));
                        return;
                    }
                }

                try {
                    let geoMeta = null;
                    if (geoFile) {
                        const geoText = await readFileAsText(geoFile);
                        geoMeta = JSON.parse(geoText);
                    }

                    const buffer = await readFileAsArrayBuffer(pcdFile);
                    const loader = new THREE.PCDLoader();
                    const points = loader.parse(buffer, pcdFile.name);
                    addLoadedPcdPointsToGprScene(points, pcdFile.name, geoMeta);

                    const localCableCount = extractCablePathFromGprGeoMetadata(geoMeta).length;
                    if (geoMeta && window.isGpsCalibrated) {
                        const cableTip = localCableCount >= 2 ? `，已恢复黄色电缆路径 ${localCableCount} 点` : "";
                        setStatus(`✅ 本地地图加载完成：${pcdFile.name}，已恢复经纬度标定${cableTip}，可发送履带小车巡检。`, "success");
                        addSystemMessage(`本地地图加载完成：${pcdFile.name}。`, "success");
                    } else {
                        setStatus(`✅ 本地点云加载完成：${pcdFile.name}；未加载 .geo.json，经纬度不可用。`, "success");
                        addSystemMessage(`本地点云加载完成：${pcdFile.name}；未加载 geo.json。`, "success");
                    }
                } catch (err) {
                    console.error("本地地图读取失败:", err);
                    alert("本地地图读取失败：" + (err.message || err));
                    setStatus("本地地图读取失败，请检查 PCD / geo.json 文件格式。", "error");
                }
            });
        }
        // 👆=======================================================👆
        // 👆=============================================👆
        const segmentSelect = $("segmentSelect");
        const btnAnno = $("btnToggleAnnotation");
        const btnMigration = $("btnToggleMigration");
        // 👇 新增：获取新按钮和视图容器
        const btnSlices = $("btnToggleSlices");
        const gprViewsContainer = document.querySelector('.gpr-views');
        let showSlices = false; // 初始状态为隐藏

        // ✅ 初始化偏移后二维剖面的手动红圈标注按钮和鼠标事件
        initManualMigAnnotationFeature();
        updateGprMigrationUI();

        const btnDogStand = $("btnDogStand");  // 机器狗站立/趴下

        // --- B. 初始化 WebSocket 连接 (机器狗) ---
        connectWebSocket();

        // --- C. 面板切换逻辑 (一体化同步版) ---
        function switchMainPanel(target) {
            console.log("切换面板模式至:", target);
            // 记录当前激活的面板，供 Three.js 和 Canvas 节流拦截使用
            window.currentActivePanel = target;

            // 1. 隐藏所有内容面板
            if (gprPanel) { gprPanel.classList.add("hidden"); gprPanel.style.display = 'none'; }
            if (robotPanel) { robotPanel.classList.add("hidden"); robotPanel.style.display = 'none'; }
            if (armPanel) { armPanel.classList.add("hidden"); armPanel.style.display = 'none'; }
            if (pathFaultPanel) { pathFaultPanel.classList.add("hidden"); pathFaultPanel.style.display = 'none'; }
            const inspectResultPanel = $("inspectResultPanel"); // 获取预留的巡检结果面板
            if (inspectResultPanel) { inspectResultPanel.classList.add("hidden"); inspectResultPanel.style.display = 'none'; }

            // 2. 移除顶部所有导航选项卡的 active 状态
            const allNavBtns = document.querySelectorAll(".cyber-nav .nav-item");
            allNavBtns.forEach(btn => btn.classList.remove("active"));

            // 3. 针对选中的 target，同时赋予对应选项卡 active 高亮态，并展示下方内容容器
            if (target === 'robot') {
                if (btnRobotNav) btnRobotNav.classList.add("active");
                if (robotPanel) { robotPanel.classList.remove("hidden"); robotPanel.style.display = 'flex'; }
            } else if (target === 'pathFault') {
                if (btnPathFault) btnPathFault.classList.add("active");
                if (pathFaultPanel) { pathFaultPanel.classList.remove("hidden"); pathFaultPanel.style.display = 'flex'; }
            } else if (target === 'gpr') {
                if (btn3D) btn3D.classList.add("active");
                if (gprPanel) { gprPanel.classList.remove("hidden"); gprPanel.style.display = 'flex'; }
            } else if (target === 'inspectResult') {
                const btnInspect = $("btnInspectResult");
                if (btnInspect) btnInspect.classList.add("active");
                if (inspectResultPanel) { inspectResultPanel.classList.remove("hidden"); inspectResultPanel.style.display = 'flex'; }
                if (window.renderInspectionHistoryList) {
                    window.renderInspectionHistoryList();
                }
            } else if (target === 'arm') {
                // 侧边栏独立切换到机械臂面板
                if (armPanel) { armPanel.classList.remove("hidden"); armPanel.style.display = 'flex'; }
            }

            moveRobotCameraOutputsToPanel(target);
            scrollToMain();
        }

        window.switchMainPanel = switchMainPanel;

        function scrollToMain() {
            const mainArea = $("mainArea");
            if (mainArea) mainArea.scrollIntoView({ behavior: "smooth", block: "start" });
        }

        // --- D. 绑定顶部导航按钮切换事件 ---
        if (btnRobotNav) btnRobotNav.addEventListener("click", () => switchMainPanel('robot'));
        if (btnPathFault) btnPathFault.addEventListener("click", () => switchMainPanel('pathFault'));
        if (btn3D) btn3D.addEventListener("click", () => switchMainPanel('gpr'));

        // 绑定新增的“巡检结果”按钮
        const btnInspectResult = $("btnInspectResult");
        if (btnInspectResult) {
            btnInspectResult.addEventListener("click", function (e) {
                e.preventDefault();
                e.stopPropagation();

                switchMainPanel('inspectResult');
            });
        }

        // 绑定左侧系统栏“设备”按钮
        if (btnShowArm) {
            btnShowArm.addEventListener("click", () => switchMainPanel('arm'));
        }

        // 🌟 核心新增：页面加载完成后，默认自动执行一次切换操作
        // 确保一开始打开网页时，不仅显示“控制与导航”页面，且顶部按钮默认呈发光激活状态
        switchMainPanel('robot');

        // --- E. 绑定 GPR 按钮 ---
        if (btnStart) btnStart.addEventListener("click", startGprProcessing);

        if (btnMigration) {
            btnMigration.addEventListener("click", function () {
                if (btnStart && btnStart.disabled) return;
                setGprMigrationEnabled(!gprMigrationEnabled, true);
            });
        }

        if (btnAnno) {
            btnAnno.addEventListener("click", function () {
                const nextShow = !showAnnotation;
                setAnnotationButtonState(nextShow);
                if (!nextShow) {
                    if (manualMigAnnotationMode) {
                        manualMigAnnotationMode = false;
                        updateManualMigAnnotationUI();
                    }
                    if (forcedPipeAnnotationMode) {
                        forcedPipeAnnotationMode = false;
                        activeForcedPipeGroupId = null;
                        updateForcedPipeAnnotationUI();
                    }
                }
                if (segmentSelect && !segmentSelect.disabled) {
                    const idx = parseInt(segmentSelect.value, 10);
                    if (!isNaN(idx)) showSegment(idx);
                }

                // 控制 3D 顶点管线和人工强制连接标记的显示状态。
                annotation3DGroups.forEach(g => g.visible = showAnnotation);

                // 原始三维剖面及其识别框/顶点始终显示；按钮只切换管线连接及人工标记。
                if (window.gprSlicesGroup) {
                    window.gprSlicesGroup.visible = true;
                }
            });
        }
        // 👇 新增：二维剖面显隐按钮绑定逻辑
        if (btnSlices) {
            btnSlices.addEventListener("click", function () {
                showSlices = !showSlices;

                // 更新按钮 UI (文字与点亮状态)
                this.textContent = "二维剖面: " + (showSlices ? "ON" : "OFF");
                this.classList.toggle("active", showSlices);

                if (gprViewsContainer) {
                    if (showSlices) {
                        gprViewsContainer.classList.remove("hide-2d"); // 恢复三列布局
                    } else {
                        gprViewsContainer.classList.add("hide-2d");    // 单列铺满
                    }

                    // 🌟 核心点：因为容器大小瞬间发生了改变，必须通知 Three.js 更新渲染画布的尺寸，
                    // 否则 Three.js 画布会被拉伸变形或者出现大黑边。
                    setTimeout(() => {
                        window.dispatchEvent(new Event('resize'));
                    }, 50); // 延迟 50ms 等待 CSS Grid 渲染完成再触发重绘
                }
            });
        }

        // 👇========= 🌟 新增代码：网格与刻度线显隐控制交互 =========👇
        const btnToggleGrid = document.getElementById('btnToggleGrid');
        if (typeof window.isGridVisible === 'undefined') {
            window.isGridVisible = true; // 默认全局状态是显示
        }

        if (btnToggleGrid) {
            btnToggleGrid.addEventListener('click', function (e) {
                e.preventDefault();
                window.isGridVisible = !window.isGridVisible; // 切换状态

                // 切换按钮的暗化 UI 样式
                if (window.isGridVisible) {
                    btnToggleGrid.classList.remove('off-state');
                } else {
                    btnToggleGrid.classList.add('off-state');
                }

                // 1. 【核心精细化控制】：使用 traverse 递归遍历整个坐标系组的所有子孙节点
                if (window.gprAxesGroup) {
                    window.gprAxesGroup.traverse(child => {
                        // 💡 判断1：如果是土壤的面 (Mesh)，强制保持显示
                        if (child.type === 'Mesh') {
                            child.visible = true;
                        }
                        // 💡 判断2：如果是线 (长方体表面的网格、外围边线) 或 精灵/文字 (刻度数字)
                        else if (child.type === 'Line' || child.type === 'LineSegments' || child.type === 'Sprite') {
                            child.visible = window.isGridVisible;
                        }
                    });
                }

                // 2. 同步控制 Z 形折线（巡检轨迹/电缆形态）
                // 注意：这里需要你配合进行“第二步”的修改
                if (window.gprZPathLine) {
                    window.gprZPathLine.visible = window.isGridVisible;
                }
            });
        }
        // 👆=======================================================👆

        if (segmentSelect) segmentSelect.disabled = true;

        // --- F. 绑定机器人运动控制按钮 (长按连续发送) ---
        // ✅ 修改：加入了侧边栏的 ID (btnSide...)
        const motionBtnIds = [
            // 主控制台按钮
            'btnMoveForward', 'btnMoveBackward',
            'btnTurnLeft', 'btnTurnRight',
            'btnStopMove',
            'btnStrafeLeft', 'btnStrafeRight',

            // ✅ 新增：侧边栏按钮
            'btnSideMoveForward', 'btnSideMoveBackward',
            'btnSideTurnLeft', 'btnSideTurnRight',
            'btnSideStop',
            'btnSideStrafeLeft', 'btnSideStrafeRight'
        ];

        motionBtnIds.forEach(id => {
            const btn = $(id);
            if (btn) {
                // 鼠标/手指按下 -> 开始 (启动定时器)
                btn.addEventListener('mousedown', () => handleMotion(id, 'start'));
                btn.addEventListener('touchstart', (e) => { e.preventDefault(); handleMotion(id, 'start'); }, { passive: false });

                // 鼠标/手指松开或移出 -> 停止 (清除定时器)
                btn.addEventListener('mouseup', () => handleMotion(id, 'stop'));
                btn.addEventListener('mouseleave', () => handleMotion(id, 'stop'));
                btn.addEventListener('touchend', (e) => { e.preventDefault(); handleMotion(id, 'stop'); });
            }
        });
        // ✅ G. 绑定新增的模式切换按钮 (防重复终极版)
        let dogStandWalkTimer = null;
        if (btnDogStand) {
            // 使用 dataset 在 DOM 元素上存储状态，防止变量污染
            // 0 = 趴下, 1 = 站立
            btnDogStand.dataset.status = "0";

            // ⚡️ 核心修改：使用 onclick 而不是 addEventListener
            // 这样即使 JS 被加载两遍，第二次的也会覆盖第一次，保证只有一个事件生效
            btnDogStand.onclick = function () {

                // --- 1. 防止快速点击 (防抖 1秒) ---
                const now = Date.now();
                const lastTime = parseInt(this.dataset.lastClick || "0");
                if (now - lastTime < 1000) {
                    console.warn("⚠️ 点击太快，已拦截");
                    return;
                }
                this.dataset.lastClick = now;

                // --- 2. 获取当前状态 ---
                const isStanding = this.dataset.status === "1";

                if (!isStanding) {
                    // === 当前是趴下 -> 切换为站立 ===
                    console.log("🚀 [执行] 切换到: 站立 (Mode 1)，随后自动进入常规模式 (Mode 3)");

                    // 先站立，再给底层一点状态切换时间后进入常规足式模式。
                    sendMoveCommand(0.0, 0.0, 0.0, 0.0, MODES.STAND, true);
                    if (dogStandWalkTimer) clearTimeout(dogStandWalkTimer);
                    dogStandWalkTimer = setTimeout(() => {
                        if (this.dataset.status !== "1") return;
                        sendMoveCommand(0.0, 0.0, 0.0, 0.0, MODES.WALK, true);
                    }, 800);

                    // 更新 UI
                    this.textContent = "机器狗趴下";
                    this.classList.add("active");
                    setStatus("发送指令：站立", "success");

                    // 更新状态
                    this.dataset.status = "1";

                } else {
                    // === 当前是站立 -> 切换为趴下 ===
                    console.log("🚀 [执行] 切换到: 趴下 (Mode 4)");

                    // 发送指令
                    if (dogStandWalkTimer) {
                        clearTimeout(dogStandWalkTimer);
                        dogStandWalkTimer = null;
                    }
                    sendMoveCommand(0.0, 0.0, 0.0, 0.0, MODES.LIE_DOWN, true);

                    // 更新 UI
                    this.textContent = "机器狗站立";
                    this.classList.remove("active");
                    setStatus("发送指令：趴下", "success");

                    // 更新状态
                    this.dataset.status = "0";
                }
            };
        }
        // ==========================================
        // ✅ 新增：定时获取机械臂实时角度
        // ==========================================

        // 1. 确保端口正确 (你的 Python 代码是 5000)
        const ARM_API = "http://10.21.31.104:5005";
        // 🔴 状态标志位：是否已记录过错误（防止日志重复打印）
        let hasLoggedArmError = false;

        function fetchArmAngles() {
            if (typeof isArmBusy !== 'undefined' && isArmBusy) {
                setTimeout(fetchArmAngles, 100);
                return;
            }

            // 👇 核心优化：如果没有连上或者不在对应面板，直接把延迟拉长到 3 秒
            let nextDelay = (window.currentActivePanel === 'arm' || window.currentActivePanel === 'robot') ? 200 : 3000;

            const controller = new AbortController();
            const timeoutId = setTimeout(() => controller.abort(), 2000);

            Promise.all([
                fetch(ARM_API_BASE + "/get_angles", { signal: controller.signal }).then(res => res.ok ? res.json() : Promise.reject("Angles Error")),
                fetch(ARM_API_BASE + "/get_end_pose", { signal: controller.signal }).then(res => res.ok ? res.json() : Promise.reject("Pose Error")),
                fetch(ARM_API_BASE + "/gripper_status", { signal: controller.signal }).then(res => res.ok ? res.text() : Promise.reject("Gripper Error"))
            ])
                .then(([anglesData, poseData, gripperData]) => {
                    clearTimeout(timeoutId);

                    // --- 处理关节角度 ---
                    if (hasLoggedArmError) {
                        console.log("%c✅ 机械臂连接已恢复！", "color: green; font-weight: bold;");
                        hasLoggedArmError = false;
                        const container = document.getElementById('armDataDisplay');
                        if (container) container.style.opacity = "1";
                    }

                    if (Array.isArray(anglesData) && anglesData.length === 6) {
                        // ✅ 核心修复：将真实关节角度同步到全局状态，防止下次单关节控制时从旧角度起步
                        if (typeof currentAngles !== 'undefined') {
                            currentAngles = [...anglesData];
                        }

                        for (let i = 0; i < 6; i++) {
                            const el = document.getElementById('disp_j' + (i + 1));
                            if (el) {
                                let val = anglesData[i];
                                if (Math.abs(val) < 0.5) val = 0.0;
                                el.textContent = val.toFixed(1);
                            }
                        }
                    }

                    // --- 处理末端位姿 ---
                    if (Array.isArray(poseData) && poseData.length === 6) {
                        currentEndPose = [...poseData];
                        isEndPoseSynced = true;
                    }

                    // --- 处理夹爪状态 ---
                    if (gripperData) {
                        const currentGripper = parseFloat(gripperData);
                        const el = document.getElementById('disp_gripper');
                        const slider = document.getElementById('gripperSlider');
                        if (el) el.textContent = currentGripper.toFixed(1) + ' mm';
                        // 当用户没有在拖动且没有在长按点动时，同步滑条，防止跳变
                        if (slider && !isGripperJogging) {
                            slider.value = currentGripper;
                        }
                    }
                    // 数据正常获取，可以恢复 100ms 刷新
                    nextDelay = 100;
                })
                .catch(err => {
                    clearTimeout(timeoutId);
                    if (!hasLoggedArmError) {
                        console.warn("⚠️ 机械臂未连接，转为低速模式防卡顿。");
                        hasLoggedArmError = true;
                    }
                    // 👇 网络报错时，强制降速到 3000ms，绝不允许它狂刷
                    nextDelay = 3000;
                })
                .finally(() => {
                    setTimeout(fetchArmAngles, nextDelay);
                });
        }

        // 🔥 启动第一次请求 (不要再用 setInterval 了！)
        fetchArmAngles();

        // ==========================================
        // 🧪 [室内测试专用] 强制虚拟 GPS 标定 (快捷键 Shift + C)
        // ==========================================
        document.addEventListener('keydown', function (event) {
            // 忽略输入框内的按键
            if (document.activeElement.tagName === 'INPUT' || document.activeElement.tagName === 'TEXTAREA') return;

            // 按下 Shift + C 触发室内测试标定
            if (event.code === 'KeyC' && event.shiftKey) {
                // 确保机器狗已经上线并且有 SLAM 坐标
                if (window.currentRobotPose && window.currentRobotPose.x !== -999 && window.currentRobotPose.x !== undefined) {

                    // 1. 设置一个虚拟的基准 GPS 锚点
                    window.gpsAnchor = { lat: 22.60991285, lon: 113.05173552 };

                    // 2. 将机器狗当前的 SLAM 坐标与虚拟 GPS 强行绑定
                    window.slamAnchor = { x: window.currentRobotPose.x, y: window.currentRobotPose.y };

                    // 3. [修改核心] 获取并计算双天线 RTK 偏航角 (基于 0度=正北, 90度=正东)
                    // 弹窗让用户输入当前的真实物理偏航角（角度）
                    let userInput = prompt("📍 室内虚拟 GPS 标定\n\n请输入双天线 RTK 给出的真实偏航角 (单位: 度)\n参考：0=正北, 90=正东, 180=正南, 270(或-90)=正西", "0");

                    // 如果用户点击了“取消”，则中断本次标定操作
                    if (userInput === null) {
                        console.log("❌ 用户取消了虚拟 GPS 标定");
                        return;
                    }

                    // 解析输入的角度值，如果输入非数字则兜底设为 0 (正北)
                    let inputDeg = parseFloat(userInput);
                    if (isNaN(inputDeg)) inputDeg = 0.0;

                    // --- 核心数学转换 ---
                    // RTK地理角度(0北/90东) 转为 数学极坐标系角度(0东/90北)
                    let mathDeg = 90 - inputDeg;

                    // 将角度转换为弧度 (SLAM 系统和 JS Math 函数都需要弧度)
                    let rtkYawRad = mathDeg * (Math.PI / 180);

                    // 核心算法：坐标系偏角 = 真实物理航向(数学弧度) - SLAM 局部航向(弧度)
                    window.headingOffset = rtkYawRad - window.currentRobotPose.yaw;

                    // 4. 强制修改状态锁
                    window.isGpsCalibrated = true;
                    window.currentGps = window.gpsAnchor; // 伪造当前信号，防止 UI 显示“标定中”

                    console.log(`✅ [室内测试] 虚拟 GPS 标定已强制完成！`);
                    console.log(`🧭 输入 RTK 航向: ${inputDeg}° (转为内部数学弧度: ${rtkYawRad.toFixed(4)} rad)`);
                    console.log(`🧭 当前 SLAM 航向: ${window.currentRobotPose.yaw.toFixed(4)} rad`);
                    console.log(`📐 计算得出坐标系偏角 (headingOffset): ${window.headingOffset.toFixed(4)} rad`);

                    alert(`📍 虚拟 GPS 标定成功！\n\n输入航向: ${inputDeg}° (0=北, 90=东)\n内部坐标偏角已设为: ${window.headingOffset.toFixed(4)} rad\n\n现在可以点击地图上的电缆测试经纬度换算了。`);
                } else {
                    console.warn("⚠️ 尚未获取到机器狗的 SLAM 坐标，请先让机器狗连上并稍微动一下。");
                    alert("⚠️ 尚未获取到机器狗坐标，请先遥控机器狗走一步再标定。");
                }
            }
        });
        // ==========================================
        // 🌟 [新增] 点云水平位置与角度综合微调
        // ==========================================
        document.addEventListener('keydown', function (event) {
            // 如果还没加载点云，直接忽略
            if (!window.currentPcdGroup) return;

            // 忽略输入框内的按键，防止打字时触发
            if (document.activeElement.tagName === 'INPUT' || document.activeElement.tagName === 'TEXTAREA') return;

            const rotateStep = 0.02; // 每次按下按键旋转的弧度 (约 1.14 度)
            const moveStep = 0.05;   // 每次平移的距离 (0.05米 = 5厘米)
            let updated = false;

            // --- A. 旋转控制 ---
            if (event.key === '[') {
                window.currentPcdGroup.rotation.y += rotateStep;
                updated = true;
            } else if (event.key === ']') {
                window.currentPcdGroup.rotation.y -= rotateStep;
                updated = true;
            }
            // --- B. 平移控制 (使用键盘方向键) ---
            else if (event.key === 'ArrowUp') {
                event.preventDefault(); // 阻止浏览器滚动条跟着页面移动
                window.currentPcdGroup.position.z -= moveStep; // 前移
                updated = true;
            } else if (event.key === 'ArrowDown') {
                event.preventDefault();
                window.currentPcdGroup.position.z += moveStep; // 后移
                updated = true;
            } else if (event.key === 'ArrowLeft') {
                event.preventDefault();
                window.currentPcdGroup.position.x -= moveStep; // 左移
                updated = true;
            } else if (event.key === 'ArrowRight') {
                event.preventDefault();
                window.currentPcdGroup.position.x += moveStep; // 右移
                updated = true;
            }

            // --- C. 实时打印调试信息 ---
            if (updated) {
                const pos = window.currentPcdGroup.position;
                const deg = (window.currentPcdGroup.rotation.y * 180 / Math.PI).toFixed(2);

                console.log(`📍 正在微调: 位置 X=${pos.x.toFixed(2)}, Z=${pos.z.toFixed(2)} | 角度 Yaw=${deg}°`);

                // 实时更新状态栏，方便你记录数值
                setStatus(`📍 偏移: X=${pos.x.toFixed(2)}m, Z=${pos.z.toFixed(2)}m | 角度: ${deg}°`, "warning");
            }
        });
    });

    // ==========================================
    // 3. WebSocket 与 机器人控制函数
    // ==========================================

    function connectWebSocket() {
        console.log("正在尝试建立 WebSocket 连接 (机器狗)...");
        if (typeof SockJS === 'undefined' || typeof Stomp === 'undefined') {
            console.error("缺少 SockJS 或 Stomp 库");
            updateConnectionUI(false);
            return;
        }

        const socket = new SockJS('/ws');
        stompClient = Stomp.over(socket);
        stompClient.debug = null;

        stompClient.connect({}, function (frame) {
            console.log('%c✅ WebSocket 连接成功！', 'color: green; font-weight: bold;');
            updateConnectionUI(true);

            // ==================================================
            // 🌟 新增：订阅机器狗实时设备状态 (电量与照明灯)
            // ==================================================
            stompClient.subscribe('/topic/robot_status', function (message) {
                try {
                    const statusData = JSON.parse(message.body);
                    const readNumber = (value) => {
                        const n = Number(value);
                        return Number.isFinite(n) ? n : null;
                    };

                    // 👇 新增：解析并更新预计剩余续航里程 (如果有该字段) 👇
                    const remainMile = readNumber(statusData.remainMile);
                    if (remainMile !== null) {
                        const elRemainMile = document.getElementById('remainMileValue');
                        if (elRemainMile) {
                            // 保留 2 位小数显示，加上单位
                            elRemainMile.textContent = remainMile.toFixed(2) + ' km';
                        }
                    }

                    // 1. 更新左侧电池显示 (低于20%触发红色高亮预警)
                    const batteryLeft = readNumber(statusData.batteryLeft);
                    const elBatLeft = document.getElementById('batteryLeftValue');
                    if (elBatLeft && batteryLeft !== null) {
                        elBatLeft.textContent = batteryLeft.toFixed(0) + ' %';
                        elBatLeft.style.color = batteryLeft < 20 ? '#ff4444' : '#00ff00';
                    }

                    // 2. 更新右侧电池显示
                    const batteryRight = readNumber(statusData.batteryRight);
                    const elBatRight = document.getElementById('batteryRightValue');
                    if (elBatRight && batteryRight !== null) {
                        elBatRight.textContent = batteryRight.toFixed(0) + ' %';
                        elBatRight.style.color = batteryRight < 20 ? '#ff4444' : '#00ff00';
                    }

                    // 3. 更新前侧照明灯状态徽章
                    const elLightFront = document.getElementById('lightFrontBadge');
                    const lightFront = readNumber(statusData.lightFront);
                    if (elLightFront && statusData.lightFront !== undefined) {
                        if (lightFront === 1) {
                            elLightFront.textContent = '开启';
                            elLightFront.className = 'status-badge online';
                        } else {
                            elLightFront.textContent = '关闭';
                            elLightFront.className = 'status-badge offline';
                        }
                    }

                    // 4. 更新后侧照明灯状态徽章
                    const elLightBack = document.getElementById('lightBackBadge');
                    const lightBack = readNumber(statusData.lightBack);
                    if (elLightBack && statusData.lightBack !== undefined) {
                        if (lightBack === 1) {
                            elLightBack.textContent = '开启';
                            elLightBack.className = 'status-badge online';
                        } else {
                            elLightBack.textContent = '关闭';
                            elLightBack.className = 'status-badge offline';
                        }
                    }

                } catch (e) {
                    console.error("机器狗状态数据解析与更新错误:", e);
                }
            });

            // ==================================================
            // 🌟 新增：订阅管线仪数据
            // ==================================================
            stompClient.subscribe('/topic/pipeline', function (message) {
                try {
                    // 解析后端发来的 JSON {signal: 0, depth: 15.0}
                    var data = JSON.parse(message.body);
                    var signal = data.signal !== undefined ? data.signal : parseInt(message.body);
                    var depth = data.depth !== undefined ? data.depth : null;
                    var angle = data.angle !== undefined ? data.angle : null;
                    var prediction = {
                        yaw: data.predictedYaw,
                        pathTurnDeg: data.pathTurnDeg,
                        signalFutureBiasDeg: data.signalFutureBiasDeg,
                        cornerConfidence: data.cornerConfidence,
                        cornerArea: data.cornerArea,
                        cornerStrafeMode: data.cornerStrafeMode,
                        pathPointCount: data.pathPointCount
                    };

                    updatePipelineDisplay(signal);

                    // 存入全局变量
                    window.g_pipelineSignal = signal;
                    window.g_pipelinePrediction = prediction;

                    // 可选：如果页面后续添加了 id="pipelineAngleValue" 的元素，这里会自动显示罗盘角度
                    const angleEl = document.getElementById('pipelineAngleValue');
                    if (angleEl && angle !== null && angle !== undefined) {
                        angleEl.textContent = Number(angle).toFixed(1) + '°';
                        angleEl.style.color = Math.abs(Number(angle) - 90) <= 3 ? '#00ff00' : '#ffcc00';
                    }

                    // ✅ 废弃卡尔曼滤波：上位机仅在突变时发深度，用全局变量接住，等待 3D 引擎回溯绑定
                    if (depth !== null) {
                        window.g_pipelineRawDepth = depth;
                        window.g_newDepthReceived = depth; // 核心：将突变深度存入专门的回溯变量
                    }

                } catch (e) {
                    console.error("管线数据解析错误:", e);
                }
            });

            // ==================================================
            // 🌟 新增：订阅 GPS 数据，并进行自动标定
            // ==================================================
            stompClient.subscribe('/topic/gps', function (message) {
                try {
                    const gpsData = JSON.parse(message.body);
                    // 只有在信号良好 (qual > 0) 时才处理
                    if (gpsData.qual > 0) {
                        const gpsLat = parseGpsNumber(gpsData.lat);
                        const gpsLon = parseGpsNumber(gpsData.lon);
                        window.currentGps = { ...gpsData, lat: gpsLat, lon: gpsLon };

                        // ⚠️ 假设 currentRobotPose 在这里可以获取到 (它在下方定义的)
                        const pose = window.currentRobotPose;

                        // ✅ 多点轨迹标定：替代原来的“走出1米单向量标定”。
                        // 机器狗可直线、转弯或不规则移动；系统会采集多组 SLAM/GPS 样本，
                        // 用 RANSAC + 二维相似变换求解 scale / 旋转角 / 平移量。
                        if (!window.isGpsCalibrated && !window.geoTransform && pose && pose.x !== -999) {
                            addGeoCalibrationSample(gpsLat, gpsLon, pose, gpsData.status || gpsData.qual);
                        }
                    }
                } catch (e) {
                    console.error("GPS数据解析错误:", e);
                }
            });

        }, function (error) {
            console.error('WebSocket 连接失败:', error);
            updateConnectionUI(false);
        });
    }


    // ✅ 新增：专门用于更新左侧连接状态 UI 的函数
    function updateConnectionUI(isConnected) {
        const badge = $("robotConnectionBadge");
        if (!badge) return;

        if (isConnected) {
            badge.textContent = "在线";
            badge.className = "status-badge online"; // 切换为绿色样式
        } else {
            badge.textContent = "离线";
            badge.className = "status-badge offline"; // 切换为红色样式
        }
    }

    // 优化后的运动处理函数 (已集成滑条速度控制)
    function handleMotion(btnId, action) {
        // --- 停止逻辑 (最高优先级) ---
        if (action === 'stop' || btnId === 'btnStopMove' || btnId === 'btnSideStop') {
            // 1. 立即清除定时器
            if (motionTimer) {
                clearInterval(motionTimer);
                motionTimer = null;
            }
            // 2. 立即发送停止指令
            // ✅ 修复：参数(mode) 必须是数字 0.0
            sendMoveCommand(0.0, 0.0, 0.0, 0.0, 0.0, false); // 第6个参数 false 代表不打印日志

            // 20ms后再补一发，确保停止
            setTimeout(() => {
                sendMoveCommand(0.0, 0.0, 0.0, 0.0, 0.0, true);
            }, 20);
            return;
        }

        // --- 开始逻辑 ---
        if (motionTimer) return;

        // ============================================================
        // 🚀 核心修改：动态读取滑条数值
        // ============================================================

        // 1. 获取 DOM 元素 (防止页面未加载完报错，加个判断)
        const elSpeedX = document.getElementById('manualSpeedX');
        const elSpeedY = document.getElementById('manualSpeedY');

        // 2. 解析数值 (如果获取不到，使用默认兜底值 0.5 和 0.3)
        // 注意：input[type=range] 的 value 是字符串，需要 parseFloat 转换
        const manualX = elSpeedX ? parseFloat(elSpeedX.value) : 0.5;
        const manualY = elSpeedY ? parseFloat(elSpeedY.value) : 0.3;

        // 3. 转向速度 (目前保持固定，如需滑条可按同样方式添加)
        const manualW = 1.0;

        // 4. 根据按钮分配速度
        let v = 0.0;
        let w = 0.0;
        let y = 0.0;

        switch (btnId) {
            // --- 前进 ---
            case 'btnMoveForward':
            case 'btnSideMoveForward':
                v = manualX; break;

            // --- 后退 ---
            case 'btnMoveBackward':
            case 'btnSideMoveBackward':
                v = -manualX; break;

            // --- 左转 ---
            case 'btnTurnLeft':
            case 'btnSideTurnLeft':
                w = manualW; break;

            // --- 右转 ---
            case 'btnTurnRight':
            case 'btnSideTurnRight':
                w = -manualW; break;

            // --- 左横移 ---
            case 'btnStrafeLeft':
            case 'btnSideStrafeLeft':
                y = manualY; break;

            // --- 右横移 ---
            case 'btnStrafeRight':
            case 'btnSideStrafeRight':
                y = -manualY; break;
        }

        // ============================================================

        if (v !== 0 || w !== 0 || y !== 0) {
            // 5. 立即发送第一帧
            // 参数说明: v(前后), w(旋转), y(横移), mode(0.0=保持当前), showLog
            sendMoveCommand(v, w, y, 0.0, 0.0, true);

            // 6. 开启心跳包 (长按连续发送)
            motionTimer = setInterval(() => {
                sendMoveCommand(v, w, y, 0.0, 0.0, false);
            }, 150);
        }
    }

    // ✅ 修改后的发送函数：增加 mode (通过 linear.z 传递)
    // 参数：v(前后), w(旋转), y(横移), mode(站立=1/运动=6), showLog
    // ✅ 修改后的发送函数：增加了 UI 状态更新
    // 增加 z 参数，并将 mode 独立出来
    function sendMoveCommand(v, w, y, z = 0.0, mode = 0.0, showLog = true) {
        if (stompClient && stompClient.connected) {
            const payload = JSON.stringify({
                'v': v,
                'w': w,
                'y': y || 0.0,
                'z': z || 0.0,    // ✅ 这里现在是高度速度
                'mode': mode || 0.0 // ✅ 这里现在是模式指令
            });
            stompClient.send("/app/move", {}, payload);

            // ============================================
            // 🌟 新增：实时更新左侧“运行模式”文字
            // ============================================
            const modeLabel = document.getElementById("currentModeLabel");
            if (modeLabel && mode !== 0.0) {
                let text = "未知";
                switch (mode) {
                    case 1.0: text = "站立 (锁定)"; break;
                    case 3.0: text = "运动 (常规)"; break;
                    case 4.0: text = "趴下"; break;
                    case 6.0: text = "轮式/特殊"; break;
                    default: text = "未知模式";
                }
                // 自动巡线是特殊的运动模式，我们在 toggle 按钮那里单独处理，这里先更新基础模式
                modeLabel.textContent = text;

                // 给文字换个颜色提醒
                modeLabel.style.color = (mode === 3.0) ? "#00ff00" : "#eaf6ff";
            }

            if (showLog) {
                if (mode !== 0) {
                    console.log(`🚀 发送模式切换: Mode=${mode}`);
                } else {
                    console.log(`🚀 发送运动: v=${v}, w=${w}, y=${y}`);
                }
            }
        } else {
            if (showLog) console.warn("⚠️ WebSocket 未连接，无法发送指令");
        }
    }

    // 调用后端接口
    function createGprProgressTaskId() {
        if (window.crypto && typeof window.crypto.randomUUID === "function") {
            return window.crypto.randomUUID();
        }
        return `gpr-${Date.now()}-${Math.random().toString(16).slice(2)}`;
    }

    function startGprProgressPolling(taskId) {
        let stopped = false;
        let timer = null;

        const poll = function () {
            if (stopped) return;
            fetch("/api/gpr/progress?taskId=" + encodeURIComponent(taskId), { cache: "no-store" })
                .then(res => res.ok ? res.json() : null)
                .then(data => {
                    if (!stopped && data && Number.isFinite(Number(data.progress))) {
                        setGprProcessingProgress(Number(data.progress));
                    }
                })
                .catch(() => {
                    // 处理请求刚创建时进度记录可能尚未注册，下次轮询会自动补上。
                });
        };

        poll();
        timer = window.setInterval(poll, 400);

        return function stopPolling() {
            stopped = true;
            if (timer !== null) window.clearInterval(timer);
        };
    }

    function startGprProcessing() {
        const input = $("folderPathInput");
        if (!input) return;

        const folderPath = input.value.trim();
        if (!folderPath) {
            alert("请先输入服务器上的数据文件夹路径！");
            return;
        }

        currentGprFolderPath = normalizeGprFolderPath(folderPath);
        currentGprAnnotationStorageKey = getGprFolderAnnotationKey(currentGprFolderPath);

        const btnStart = $("btnStartGpr");
        const btnMigration = $("btnToggleMigration");
        if (btnStart && btnStart.disabled) return;
        if (btnStart) btnStart.disabled = true;
        if (btnMigration) btnMigration.disabled = true;

        const taskId = createGprProgressTaskId();
        setGprProcessingProgress(0);
        const stopProgressPolling = startGprProgressPolling(taskId);

        fetch("/api/gpr/process?folderPath=" + encodeURIComponent(folderPath) + "&taskId=" + encodeURIComponent(taskId), {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({ migrationEnabled: gprMigrationEnabled })
        })
            .then(function (res) {
                if (!res.ok) {
                    throw new Error("后端返回错误状态：" + res.status);
                }
                return res.json();
            })
            .then(async function (data) {
                if (!data || !data.segments || !data.segments.length) {
                    throw new Error("返回数据为空或格式不正确");
                }
                stopProgressPolling();
                clearGprProcessingProgress();
                await renderGprResults(data);
            })
            .catch(function (err) {
                console.error(err);
                clearGprProcessingProgress();
                setStatus("处理失败：" + err.message, "error");
            })
            .finally(function () {
                stopProgressPolling();
                if (btnStart) btnStart.disabled = false;
                if (btnMigration) btnMigration.disabled = false;
            });

    }

    // ==========================================
    // ✅ 三维重建页：按已规划的点云地图 Z 字路径融合摆放 GPR 剖面
    // 说明：
    // 1. 有“当前点云地图”且已经规划过扫描路径时，GPR 剖面沿规划的 zigzagSlam 逐段落位；
    // 2. 没有当前点云地图或没有规划路径时，不改动后端原始 geometry，保持原来的展示效果。
    // ==========================================
    function hasUsableGprScanPlanForFusion() {
        const plan = window.currentGprScanPlan;
        const zigzag = plan?.preview?.zigzagSlam;

        return !!(
            window.gprHasInspectionMapLayerLoaded &&
            plan &&
            Array.isArray(zigzag) &&
            zigzag.length >= 2
        );
    }

    function getGprPlanLegCount() {
        const zigzag = window.currentGprScanPlan?.preview?.zigzagSlam;
        return Array.isArray(zigzag) ? Math.max(0, zigzag.length - 1) : 0;
    }

    function cloneNumberArray(arr, fallbackLen, fallbackStep = 1) {
        if (Array.isArray(arr) && arr.length === fallbackLen) {
            return arr.map(v => Number(v));
        }
        return Array.from({ length: fallbackLen }, (_, i) => i * fallbackStep);
    }

    function rescaleUAxisToLineLength(sourceUAxis, cols, lineLength) {
        const fallback = Array.from({ length: cols }, (_, i) => i);
        const rawU = (Array.isArray(sourceUAxis) && sourceUAxis.length === cols)
            ? sourceUAxis.map(v => Number(v))
            : fallback;

        if (cols <= 1) return [0];

        let uMin = rawU[0];
        let uMax = rawU[rawU.length - 1];

        if (!Number.isFinite(uMin) || !Number.isFinite(uMax) || Math.abs(uMax - uMin) < 1e-9) {
            uMin = 0;
            uMax = cols - 1;
        }

        const span = uMax - uMin;
        return rawU.map(u => ((u - uMin) / span) * lineLength);
    }

    function hasActiveGpsSlamTransform() {
        return !!(
            window.geoTransform ||
            (window.isGpsCalibrated && window.gpsAnchor && window.slamAnchor)
        );
    }

    function hasUsableRtkSegmentGeometry(seg, mat) {
        const geo = seg?.geometry;
        const cols = mat?.[0]?.length || 0;
        if (!geo?.rtkPositioned || cols < 2) return false;
        const localX = geo.traceX;
        const localY = geo.traceY;
        return Array.isArray(localX) && Array.isArray(localY) &&
            localX.length === cols && localY.length === cols &&
            localX.every(v => Number.isFinite(Number(v))) &&
            localY.every(v => Number.isFinite(Number(v)));
    }

    function medianGprNumber(values) {
        const sorted = (values || []).map(Number).filter(Number.isFinite).sort((a, b) => a - b);
        if (!sorted.length) return NaN;
        const middle = Math.floor(sorted.length / 2);
        return sorted.length % 2
            ? sorted[middle]
            : (sorted[middle - 1] + sorted[middle]) / 2;
    }

    function fitGprRtkLine(samples) {
        if (!Array.isArray(samples) || samples.length < 2) return null;
        const centerX = samples.reduce((sum, item) => sum + item.x, 0) / samples.length;
        const centerY = samples.reduce((sum, item) => sum + item.y, 0) / samples.length;
        let covarianceXX = 0;
        let covarianceXY = 0;
        let covarianceYY = 0;
        samples.forEach(item => {
            const dx = item.x - centerX;
            const dy = item.y - centerY;
            covarianceXX += dx * dx;
            covarianceXY += dx * dy;
            covarianceYY += dy * dy;
        });
        if (covarianceXX + covarianceYY <= 1e-12) return null;
        const angle = 0.5 * Math.atan2(2 * covarianceXY, covarianceXX - covarianceYY);
        return { centerX, centerY, directionX: Math.cos(angle), directionY: Math.sin(angle) };
    }

    /**
     * RTK只拟合整张剖面的中心和方向。返回的每一道坐标严格按DZT道间距均匀排列，
     * 因此单个RTK点的横向抖动不会再把剖面网格拉弯。
     */
    function fitRigidGprProfileFromRtk(sampleX, sampleY, traceCount, traceSpacingM) {
        const samples = [];
        for (let i = 0; i < Math.min(sampleX?.length || 0, sampleY?.length || 0); i++) {
            const x = Number(sampleX[i]);
            const y = Number(sampleY[i]);
            if (Number.isFinite(x) && Number.isFinite(y)) samples.push({ order: i, x, y });
        }
        if (samples.length < 2 || traceCount < 2 || !(traceSpacingM > 0)) return null;

        const initial = fitGprRtkLine(samples);
        if (!initial) return null;
        const residualFor = (sample, line) => Math.abs(
            -line.directionY * (sample.x - line.centerX) +
            line.directionX * (sample.y - line.centerY)
        );
        const residuals = samples.map(sample => residualFor(sample, initial));
        const residualMedian = medianGprNumber(residuals);
        const mad = medianGprNumber(residuals.map(value => Math.abs(value - residualMedian)));
        const threshold = residualMedian + Math.max(0.05, 4 * 1.4826 * (Number.isFinite(mad) ? mad : 0));
        let inliers = samples.filter((sample, index) => residuals[index] <= threshold);
        if (inliers.length < Math.max(2, Math.ceil(samples.length * 0.5))) inliers = samples;

        const fitted = fitGprRtkLine(inliers);
        if (!fitted) return null;
        let directionX = fitted.directionX;
        let directionY = fitted.directionY;

        const windowSize = Math.max(1, Math.min(10, Math.floor(samples.length / 10) || 1));
        const first = samples.slice(0, windowSize);
        const last = samples.slice(-windowSize);
        const orderedX = medianGprNumber(last.map(item => item.x)) - medianGprNumber(first.map(item => item.x));
        const orderedY = medianGprNumber(last.map(item => item.y)) - medianGprNumber(first.map(item => item.y));
        if (directionX * orderedX + directionY * orderedY < 0) {
            directionX = -directionX;
            directionY = -directionY;
        }

        const centerColumn = (traceCount - 1) / 2;
        const traceX = new Array(traceCount);
        const traceY = new Array(traceCount);
        for (let c = 0; c < traceCount; c++) {
            const along = (c - centerColumn) * traceSpacingM;
            traceX[c] = fitted.centerX + along * directionX;
            traceY[c] = fitted.centerY + along * directionY;
        }
        const squaredResidualSum = inliers.reduce((sum, sample) => {
            const residual = Math.abs(
                -directionY * (sample.x - fitted.centerX) +
                directionX * (sample.y - fitted.centerY)
            );
            return sum + residual * residual;
        }, 0);

        return {
            traceX,
            traceY,
            phiRad: Math.atan2(directionY, directionX),
            length: (traceCount - 1) * traceSpacingM,
            inlierCount: inliers.length,
            sampleCount: samples.length,
            rmseM: Math.sqrt(squaredResidualSum / inliers.length)
        };
    }

    /**
     * 计算“原始二维矩阵比例”所需的深度显示倍率。
     * 水平方向每列按DZT道间距显示；纵向每行乘以该倍率后，单个矩阵单元近似为正方形。
     * 这只是显示倍率，Depth刻度和弹窗仍换算回真实米数。
     */
    function computeGprMatrixAspectDepthScale(segments, geometries) {
        const scales = [];
        (Array.isArray(segments) ? segments : []).forEach((segment, index) => {
            const matrix = segment?.rawData;
            const geometry = geometries?.[index];
            const rows = matrix?.length || 0;
            const cols = matrix?.[0]?.length || 0;
            const zAxis = geometry?.zAxis;
            if (rows < 2 || cols < 2 || !Array.isArray(zAxis) || zAxis.length !== rows) return;

            const depthSpan = Math.abs(Number(zAxis[rows - 1]) - Number(zAxis[0]));
            const depthStep = depthSpan / (rows - 1);
            let horizontalStep = Number(geometry?.dztTraceSpacingM);
            if (!(horizontalStep > 0)) {
                const uAxis = geometry?.uAxis;
                if (Array.isArray(uAxis) && uAxis.length === cols) {
                    horizontalStep = Math.abs(Number(uAxis[cols - 1]) - Number(uAxis[0])) / (cols - 1);
                }
            }

            const scale = horizontalStep / depthStep;
            if (Number.isFinite(scale) && scale >= 0.25 && scale <= 4) scales.push(scale);
        });

        const scale = medianGprNumber(scales);
        return Number.isFinite(scale) ? scale : 1;
    }

    /**
     * 将后端返回的RTK测点拟合为整张剖面的刚体位姿。
     * 已加载GPS/SLAM标定时先映射到全局地图再拟合；否则使用后端局部米制拟合结果。
     */
    function createRtkSegmentGeometry(seg, mat) {
        if (!hasUsableRtkSegmentGeometry(seg, mat)) return null;

        const source = seg.geometry || {};
        const rows = mat.length;
        const cols = mat[0].length;
        let rtkSampleX = source.traceX.map(Number);
        let rtkSampleY = source.traceY.map(Number);
        let projectedToSlam = false;

        const latitude = source.traceLatitudeDeg;
        const longitude = source.traceLongitudeDeg;
        if (
            hasActiveGpsSlamTransform() &&
            Array.isArray(latitude) && Array.isArray(longitude) &&
            latitude.length === cols && longitude.length === cols &&
            typeof window.gpsToSlam === "function"
        ) {
            const mappedX = [];
            const mappedY = [];
            let mappingValid = true;
            for (let c = 0; c < cols; c++) {
                const slam = window.gpsToSlam(Number(latitude[c]), Number(longitude[c]));
                if (!slam || !Number.isFinite(Number(slam.x)) || !Number.isFinite(Number(slam.y))) {
                    mappingValid = false;
                    break;
                }
                const point = slamPointToGprVector({ x: Number(slam.x), y: Number(slam.y), z: 0 });
                mappedX.push(point.x);
                mappedY.push(point.z);
            }
            if (mappingValid && mappedX.length === cols) {
                rtkSampleX = mappedX;
                rtkSampleY = mappedY;
                projectedToSlam = true;
            }
        }

        const configuredSpacing = Number(source.dztTraceSpacingM);
        const traceSpacingM = Number.isFinite(configuredSpacing) && configuredSpacing > 0
            ? configuredSpacing
            : 0.01;
        const rigidFit = fitRigidGprProfileFromRtk(rtkSampleX, rtkSampleY, cols, traceSpacingM);
        if (!rigidFit || rigidFit.length <= 1e-6) return null;
        const uAxis = Array.from({ length: cols }, (_, c) => c * traceSpacingM);
        const backendFitRmse = Number(source.rtkFitRmseM);
        const fitRmseM = projectedToSlam || !Number.isFinite(backendFitRmse)
            ? rigidFit.rmseM
            : backendFitRmse;

        return {
            ...source,
            startX: rigidFit.traceX[0],
            startY: rigidFit.traceY[0],
            phiRad: rigidFit.phiRad,
            length: rigidFit.length,
            uAxis,
            zAxis: cloneNumberArray(source.zAxis, rows, 0.02),
            traceX: rigidFit.traceX,
            traceY: rigidFit.traceY,
            _fromRtk: true,
            _rtkProjectedToSlam: projectedToSlam,
            _rtkRigidProfile: true,
            _rtkFitInlierCount: rigidFit.inlierCount,
            _rtkFitSampleCount: rigidFit.sampleCount,
            _rtkFitRmseM: fitRmseM
        };
    }

    function getGprTraceHorizontalPoint(geometry, column, cols) {
        const c = Math.min(Math.max(0, Math.round(Number(column) || 0)), Math.max(0, cols - 1));
        const traceX = geometry?.traceX;
        const traceY = geometry?.traceY;
        if (
            Array.isArray(traceX) && Array.isArray(traceY) &&
            traceX.length === cols && traceY.length === cols &&
            Number.isFinite(Number(traceX[c])) && Number.isFinite(Number(traceY[c]))
        ) {
            let left = Math.max(0, c - 1);
            let right = Math.min(cols - 1, c + 1);
            while (left > 0 && Math.hypot(
                Number(traceX[right]) - Number(traceX[left]),
                Number(traceY[right]) - Number(traceY[left])
            ) < 1e-8) left--;
            while (right < cols - 1 && Math.hypot(
                Number(traceX[right]) - Number(traceX[left]),
                Number(traceY[right]) - Number(traceY[left])
            ) < 1e-8) right++;
            const phi = Math.atan2(
                Number(traceY[right]) - Number(traceY[left]),
                Number(traceX[right]) - Number(traceX[left])
            );
            return { x: Number(traceX[c]), z: Number(traceY[c]), phi };
        }

        const uAxis = (Array.isArray(geometry?.uAxis) && geometry.uAxis.length === cols)
            ? geometry.uAxis
            : Array.from({ length: cols }, (_, i) => i);
        const phi = Number(geometry?.phiRad) || 0;
        const u = Number(uAxis[c]) || 0;
        const startX = Number(geometry?.startX) || 0;
        const startY = Number(geometry?.startY) || 0;
        return {
            x: startX + u * Math.cos(phi),
            z: startY + u * Math.sin(phi),
            phi
        };
    }

    function interpolateGprAxisValue(axis, index, expectedLength, fallbackStep) {
        const safeIndex = Math.max(0, Math.min(expectedLength - 1, Number(index) || 0));
        if (!Array.isArray(axis) || axis.length !== expectedLength) return safeIndex * fallbackStep;
        const lower = Math.floor(safeIndex);
        const upper = Math.min(expectedLength - 1, Math.ceil(safeIndex));
        const fraction = safeIndex - lower;
        const lowerValue = Number(axis[lower]);
        const upperValue = Number(axis[upper]);
        if (!Number.isFinite(lowerValue) || !Number.isFinite(upperValue)) return safeIndex * fallbackStep;
        return lowerValue + fraction * (upperValue - lowerValue);
    }

    function getGprProfileWorldPoint(geometry, column, row, rows, cols, groundY, normalOffset = 0.006) {
        const safeColumn = Math.max(0, Math.min(cols - 1, Number(column) || 0));
        const leftColumn = Math.floor(safeColumn);
        const rightColumn = Math.min(cols - 1, Math.ceil(safeColumn));
        const fraction = safeColumn - leftColumn;
        const left = getGprTraceHorizontalPoint(geometry, leftColumn, cols);
        const right = getGprTraceHorizontalPoint(geometry, rightColumn, cols);
        const phi = Number.isFinite(Number(geometry?.phiRad)) ? Number(geometry.phiRad) : left.phi;
        const depth = interpolateGprAxisValue(geometry?.zAxis, row, rows, 0.02);
        const x = left.x + fraction * (right.x - left.x) - Math.sin(phi) * normalOffset;
        const z = left.z + fraction * (right.z - left.z) + Math.cos(phi) * normalOffset;
        return new THREE.Vector3(x, groundY - depth, z);
    }

    /** 在原始剖面曲面上直接绘制YOLO识别框和波形顶点。 */
    function createGprRawDetectionOverlay(seg, geometry, groundY) {
        const group = new THREE.Group();
        group.name = "GPR原始剖面识别框与顶点";
        const matrix = seg?.rawData;
        const detections = Array.isArray(seg?.rawDetections) ? seg.rawDetections : [];
        const rows = matrix?.length || 0;
        const cols = matrix?.[0]?.length || 0;
        if (rows < 1 || cols < 1 || !detections.length) return group;

        const boxMaterial = new THREE.LineBasicMaterial({
            color: 0xff2b2b,
            // 使用正常深度测试：后方剖面的识别框应被前方剖面遮挡。
            depthTest: true,
            depthWrite: false,
            transparent: true,
            opacity: 0.95
        });
        const profileLength = Number(geometry?.length) || ((cols - 1) * 0.01);
        const markerSize = Math.max(0.018, Math.min(0.05, profileLength * 0.008));

        detections.forEach((detection, detectionIndex) => {
            const x1 = Math.max(0, Math.min(cols - 1, Number(detection.x1)));
            const x2 = Math.max(0, Math.min(cols - 1, Number(detection.x2)));
            const y1 = Math.max(0, Math.min(rows - 1, Number(detection.y1)));
            const y2 = Math.max(0, Math.min(rows - 1, Number(detection.y2)));
            if (![x1, x2, y1, y2].every(Number.isFinite) || x2 <= x1 || y2 <= y1) return;

            const boxPoints = [
                getGprProfileWorldPoint(geometry, x1, y1, rows, cols, groundY),
                getGprProfileWorldPoint(geometry, x2, y1, rows, cols, groundY),
                getGprProfileWorldPoint(geometry, x2, y2, rows, cols, groundY),
                getGprProfileWorldPoint(geometry, x1, y2, rows, cols, groundY)
            ];
            const box = new THREE.LineLoop(
                new THREE.BufferGeometry().setFromPoints(boxPoints),
                boxMaterial
            );
            box.renderOrder = 20;
            box.userData.gprDetectionIndex = detectionIndex;
            group.add(box);

            if (detection.apexCol == null || detection.apexRow == null) return;
            const apexCol = Number(detection.apexCol);
            const apexRow = Number(detection.apexRow);
            if (!Number.isFinite(apexCol) || !Number.isFinite(apexRow)) return;

            const center = getGprProfileWorldPoint(geometry, apexCol, apexRow, rows, cols, groundY, 0.009);
            const phi = Number(geometry?.phiRad) || 0;
            const tangent = new THREE.Vector3(Math.cos(phi), 0, Math.sin(phi));
            const vertical = new THREE.Vector3(0, 1, 0);
            const apexColor = detection.apexMethod === "singleWing" ? 0xffd64a : 0x00ffff;
            const crossMaterial = new THREE.LineBasicMaterial({
                color: apexColor,
                depthTest: true,
                depthWrite: false
            });
            const cross = new THREE.LineSegments(
                new THREE.BufferGeometry().setFromPoints([
                    center.clone().addScaledVector(tangent, -markerSize),
                    center.clone().addScaledVector(tangent, markerSize),
                    center.clone().addScaledVector(vertical, -markerSize),
                    center.clone().addScaledVector(vertical, markerSize)
                ]),
                crossMaterial
            );
            cross.renderOrder = 21;
            group.add(cross);

            const vertex = new THREE.Mesh(
                new THREE.SphereGeometry(markerSize * 0.28, 10, 10),
                new THREE.MeshBasicMaterial({ color: apexColor, depthTest: true, depthWrite: false })
            );
            vertex.position.copy(center);
            vertex.renderOrder = 22;
            vertex.userData.gprDetectionIndex = detectionIndex;
            group.add(vertex);
        });

        return group;
    }

    function createPlannedSegmentGeometry(seg, segIndex, mat) {
        if (!hasUsableGprScanPlanForFusion()) return null;

        const zigzag = window.currentGprScanPlan.preview.zigzagSlam;
        if (!Array.isArray(zigzag) || segIndex >= zigzag.length - 1) return null;

        const p0 = slamPointToGprVector(zigzag[segIndex]);
        const p1 = slamPointToGprVector(zigzag[segIndex + 1]);

        const dx = p1.x - p0.x;
        const dz = p1.z - p0.z;
        const lineLength = Math.hypot(dx, dz);
        if (!Number.isFinite(lineLength) || lineLength <= 1e-6) return null;

        const rows = mat?.length || 0;
        const cols = mat?.[0]?.length || 0;
        if (rows <= 0 || cols <= 0) return null;

        const srcGeo = seg.geometry || {};
        const phiRad = Math.atan2(dz, dx);

        return {
            ...srcGeo,
            startX: p0.x,
            startY: p0.z,
            phiRad: phiRad,
            uAxis: rescaleUAxisToLineLength(srcGeo.uAxis, cols, lineLength),
            zAxis: cloneNumberArray(srcGeo.zAxis, rows, 0.02),
            traceX: null,
            traceY: null,
            _fromPlannedScanPath: true,
            _plannedLegIndex: segIndex,
            _plannedLegLength: lineLength
        };
    }

    function getLoadedSurfaceCablePathForGprAlignment() {
        // 规划路径或RTK经GPS/SLAM标定投影到地图后，才允许做地表电缆路径校正。
        if (!hasUsableGprScanPlanForFusion() && !window.gprRtkProjectedToSlamForCurrentRender) return [];

        let plainPath = [];

        if (window.gprLoadedMapSource === "current") {
            plainPath = normalizePlainCablePathForGpr(window.latestImportedInspectionMapSnapshot?.cablePath);
        } else if (window.gprLoadedMapSource === "local") {
            plainPath = normalizePlainCablePathForGpr(window.gprFrozenLocalPcdSnapshot?.cablePath);
            if (plainPath.length < 2) {
                plainPath = extractCablePathFromGprGeoMetadata(window.gprFrozenLocalPcdSnapshot?.geoMeta);
            }
        }

        if (plainPath.length < 2) return [];
        return plainPath.map(slamPointToGprVector);
    }

    function buildGprCableHorizontalMeasure(points) {
        const clean = [];
        (Array.isArray(points) ? points : []).forEach(pt => {
            if (!pt) return;
            const p = pt.clone ? pt.clone() : new THREE.Vector3(Number(pt.x), Number(pt.y), Number(pt.z));
            if (!Number.isFinite(p.x) || !Number.isFinite(p.z)) return;
            const prev = clean[clean.length - 1];
            if (!prev || Math.hypot(p.x - prev.x, p.z - prev.z) > 1e-5) clean.push(p);
        });
        if (clean.length < 2) return null;

        const cumulative = [0];
        for (let i = 1; i < clean.length; i++) {
            cumulative.push(cumulative[i - 1] + Math.hypot(
                clean[i].x - clean[i - 1].x,
                clean[i].z - clean[i - 1].z
            ));
        }
        const totalLength = cumulative[cumulative.length - 1];
        return totalLength > 1e-5 ? { points: clean, cumulative, totalLength } : null;
    }

    function projectGprPointToMeasuredCableXZ(point, measure) {
        if (!point || !measure) return null;

        let best = null;
        for (let i = 0; i < measure.points.length - 1; i++) {
            const a = measure.points[i];
            const b = measure.points[i + 1];
            const dx = b.x - a.x;
            const dz = b.z - a.z;
            const lenSq = dx * dx + dz * dz;
            if (lenSq < 1e-12) continue;

            const t = Math.max(0, Math.min(1,
                ((point.x - a.x) * dx + (point.z - a.z) * dz) / lenSq
            ));
            const x = a.x + dx * t;
            const z = a.z + dz * t;
            const distance = Math.hypot(point.x - x, point.z - z);

            if (!best || distance < best.distance) {
                const segmentLength = measure.cumulative[i + 1] - measure.cumulative[i];
                best = {
                    x,
                    z,
                    distance,
                    along: measure.cumulative[i] + segmentLength * t
                };
            }
        }
        return best;
    }

    function getGprCablePointAtDistance(measure, along) {
        const s = Math.max(0, Math.min(measure.totalLength, Number(along) || 0));
        for (let i = 0; i < measure.points.length - 1; i++) {
            const s0 = measure.cumulative[i];
            const s1 = measure.cumulative[i + 1];
            if (s > s1 && i < measure.points.length - 2) continue;
            const span = s1 - s0;
            const t = span > 1e-12 ? Math.max(0, Math.min(1, (s - s0) / span)) : 0;
            const a = measure.points[i];
            const b = measure.points[i + 1];
            return new THREE.Vector3(
                a.x + (b.x - a.x) * t,
                0,
                a.z + (b.z - a.z) * t
            );
        }
        const last = measure.points[measure.points.length - 1];
        return new THREE.Vector3(last.x, 0, last.z);
    }

    function interpolateGprPipeYAtCableDistance(depthSamples, along) {
        if (!depthSamples.length) return 0;
        if (along <= depthSamples[0].along) return depthSamples[0].y;
        const last = depthSamples[depthSamples.length - 1];
        if (along >= last.along) return last.y;

        for (let i = 0; i < depthSamples.length - 1; i++) {
            const a = depthSamples[i];
            const b = depthSamples[i + 1];
            if (along > b.along) continue;
            const span = b.along - a.along;
            const t = span > 1e-9 ? (along - a.along) / span : 0;
            return a.y + (b.y - a.y) * t;
        }
        return last.y;
    }

    function getGprCableAlignmentMatchDistance() {
        const scanWidth = Number(window.currentGprScanPlan?.params?.width);
        const dynamic = Number.isFinite(scanWidth)
            ? scanWidth * GPR_CABLE_PATH_ALIGNMENT_CONFIG.scanWidthRatio
            : 1.25;
        return Math.max(
            GPR_CABLE_PATH_ALIGNMENT_CONFIG.minMatchDistanceM,
            Math.min(GPR_CABLE_PATH_ALIGNMENT_CONFIG.maxMatchDistanceM, dynamic)
        );
    }

    function buildGprPipeCableAlignment(centerItems, cableMeasure, maxDistance) {
        if (!Array.isArray(centerItems) || centerItems.length < GPR_CABLE_PATH_ALIGNMENT_CONFIG.minMatchedPoints) {
            return null;
        }

        const projected = centerItems.map(item => {
            const hit = projectGprPointToMeasuredCableXZ(item.center, cableMeasure);
            return hit ? { ...hit, y: Number(item.center.y), source: item } : null;
        }).filter(Boolean);
        if (projected.length < GPR_CABLE_PATH_ALIGNMENT_CONFIG.minMatchedPoints) return null;

        const sortedDistances = projected.map(item => item.distance).sort((a, b) => a - b);
        const medianDistance = sortedDistances[Math.floor(sortedDistances.length / 2)];
        const meanDistance = sortedDistances.reduce((sum, value) => sum + value, 0) / sortedDistances.length;
        const outlierLimit = maxDistance * GPR_CABLE_PATH_ALIGNMENT_CONFIG.outlierDistanceRatio;
        const withinCount = sortedDistances.filter(value => value <= outlierLimit).length;
        const requiredWithinCount = Math.max(
            GPR_CABLE_PATH_ALIGNMENT_CONFIG.minMatchedPoints,
            Math.ceil(projected.length * 0.67)
        );

        const alongValues = projected.map(item => item.along);
        const minAlong = Math.min(...alongValues);
        const maxAlong = Math.max(...alongValues);
        const alongSpan = maxAlong - minAlong;
        if (
            medianDistance > maxDistance ||
            withinCount < requiredWithinCount ||
            alongSpan < GPR_CABLE_PATH_ALIGNMENT_CONFIG.minAlongPathSpanM
        ) {
            return null;
        }

        // 沿黄色路径加密采样，保证弯道也跟随地上路径；每个采样点的 Y 由相邻 GPR 测点埋深插值。
        const depthSamples = projected.slice().sort((a, b) => a.along - b.along);
        const sampleDistances = [minAlong, maxAlong];
        depthSamples.forEach(item => sampleDistances.push(item.along));
        cableMeasure.cumulative.forEach(value => {
            if (value > minAlong && value < maxAlong) sampleDistances.push(value);
        });

        const sampleStep = Math.max(0.03, GPR_CABLE_PATH_ALIGNMENT_CONFIG.sampleStepM);
        for (let s = minAlong + sampleStep; s < maxAlong; s += sampleStep) {
            sampleDistances.push(s);
        }

        sampleDistances.sort((a, b) => a - b);
        const uniqueDistances = sampleDistances.filter((value, index, arr) =>
            index === 0 || Math.abs(value - arr[index - 1]) > 1e-5
        );
        const alignedPoints = uniqueDistances.map(along => {
            const pt = getGprCablePointAtDistance(cableMeasure, along);
            pt.y = interpolateGprPipeYAtCableDistance(depthSamples, along);
            return pt;
        });

        if (alignedPoints.length < 2) return null;
        return {
            alignedPoints,
            projected,
            medianDistance,
            meanDistance,
            alongSpan,
            score: medianDistance + meanDistance * 0.2
        };
    }

    function selectBestGprPipeCableAlignment(pipelineEntries) {
        const surfaceCablePath = getLoadedSurfaceCablePathForGprAlignment();
        const cableMeasure = buildGprCableHorizontalMeasure(surfaceCablePath);
        const maxDistance = getGprCableAlignmentMatchDistance();

        window.gprCablePathAlignmentStats = {
            available: !!cableMeasure,
            applied: false,
            pathPointCount: surfaceCablePath.length,
            maxMatchDistanceM: maxDistance
        };

        if (!GPR_CABLE_PATH_ALIGNMENT_CONFIG.enabled || !cableMeasure) {
            updateGprCableAlignmentBadge(
                "unavailable",
                "路径校正：无黄色路径",
                "当前已加载地图中没有可用的黄色电缆矢量路径，地下管道保留 GPR 原始位置。"
            );
            return null;
        }

        const candidates = (Array.isArray(pipelineEntries) ? pipelineEntries : [])
            .map(entry => {
                const alignment = buildGprPipeCableAlignment(entry.centers, cableMeasure, maxDistance);
                return alignment ? { ...entry, alignment } : null;
            })
            .filter(Boolean);

        // 用户明确指定过材质时，以“地下电缆”为最高优先级；明确指定为 PVC/金属的管道不参与吸附。
        const explicitCables = candidates.filter(item => item.assignment?.materialType === "cable");
        const eligible = explicitCables.length
            ? explicitCables
            : candidates.filter(item => !item.assignment || !item.assignment.materialType);
        eligible.sort((a, b) => a.alignment.score - b.alignment.score);
        const best = eligible[0] || null;

        if (!best) {
            window.gprCablePathAlignmentStats.reason = "no-matching-pipeline";
            updateGprCableAlignmentBadge(
                "unmatched",
                "路径校正：未匹配",
                `已加载黄色电缆路径，但没有地下管道通过 ${maxDistance.toFixed(2)}m 对应距离检查；为避免误校正，已保留 GPR 原始路径。`
            );
            return null;
        }

        window.gprCablePathAlignmentStats = {
            ...window.gprCablePathAlignmentStats,
            applied: true,
            pipeKey: best.colorKey,
            measuredPointCount: best.centers.length,
            alignedPointCount: best.alignment.alignedPoints.length,
            medianOffsetM: best.alignment.medianDistance,
            meanOffsetM: best.alignment.meanDistance,
            alignedSpanM: best.alignment.alongSpan
        };
        updateGprCableAlignmentBadge(
            "aligned",
            "路径校正：已对齐",
            `地下电缆已吸附到地上黄色路径；校正前中位水平误差 ${best.alignment.medianDistance.toFixed(2)}m，保留 GPR 原始埋深。`
        );
        return best;
    }

    function getPlannedGprPathBottomPoints(maxDepth, groundY = 0) {
        if (!hasUsableGprScanPlanForFusion()) return null;

        const zigzag = window.currentGprScanPlan.preview.zigzagSlam;
        if (!Array.isArray(zigzag) || zigzag.length < 2) return null;

        const topY = Number.isFinite(Number(groundY)) ? Number(groundY) : 0;
        const bottomY = topY - maxDepth;

        return zigzag.map(p => {
            const v = slamPointToGprVector(p);
            return new THREE.Vector3(v.x, bottomY, v.z);
        });
    }

    // ==========================================
    // ✅ GPR 地下环境与点云地面高度对齐工具
    // 说明：GPR 剖面以前默认以 Y=0 作为地面顶面；加载点云地图后，点云地面
    // 往往并不正好等于 Y=0，所以地下长方体会“突出地面”。这里根据已冻结的
    // 当前点云/本地点云快照，估计扫描区域附近的真实地面 Y，并作为 Depth=0 基准。
    // ==========================================
    function makeMatrixFromTransformSnapshot(transform, fallbackScale = { x: 1, y: 1, z: 1 }) {
        const matrix = new THREE.Matrix4();
        const pos = new THREE.Vector3(
            Number(transform?.position?.x || 0),
            Number(transform?.position?.y || 0),
            Number(transform?.position?.z || 0)
        );
        const rot = new THREE.Euler(
            Number(transform?.rotation?.x || 0),
            Number(transform?.rotation?.y || 0),
            Number(transform?.rotation?.z || 0),
            "XYZ"
        );
        const quat = new THREE.Quaternion().setFromEuler(rot);
        const scale = new THREE.Vector3(
            Number(transform?.scale?.x ?? fallbackScale.x),
            Number(transform?.scale?.y ?? fallbackScale.y),
            Number(transform?.scale?.z ?? fallbackScale.z)
        );
        matrix.compose(pos, quat, scale);
        return matrix;
    }

    function getGprScanFootprintXZBoxForGroundY(margin = 0.75) {
        const xs = [];
        const zs = [];

        const rect = window.currentGprScanPlan?.preview?.rectangleSlam;
        if (Array.isArray(rect) && rect.length >= 4) {
            rect.slice(0, 4).forEach(p => {
                const v = slamPointToGprVector(p);
                if (Number.isFinite(v.x) && Number.isFinite(v.z)) {
                    xs.push(v.x);
                    zs.push(v.z);
                }
            });
        }

        const zigzag = window.currentGprScanPlan?.preview?.zigzagSlam;
        if (!xs.length && Array.isArray(zigzag) && zigzag.length >= 2) {
            zigzag.forEach(p => {
                const v = slamPointToGprVector(p);
                if (Number.isFinite(v.x) && Number.isFinite(v.z)) {
                    xs.push(v.x);
                    zs.push(v.z);
                }
            });
        }

        if (!xs.length || !zs.length) return null;
        return {
            minX: Math.min(...xs) - margin,
            maxX: Math.max(...xs) + margin,
            minZ: Math.min(...zs) - margin,
            maxZ: Math.max(...zs) + margin
        };
    }

    function estimateDominantGroundYFromValues(values) {
        const arr = (Array.isArray(values) ? values : [])
            .map(Number)
            .filter(v => Number.isFinite(v) && Math.abs(v) < 1000);
        if (!arr.length) return null;

        // 点数很少时用中位数；点数足够时用高度直方图众数，避免树、墙、设备等高处点把地面抬高。
        arr.sort((a, b) => a - b);
        if (arr.length < 30) {
            return arr[Math.floor(arr.length / 2)];
        }

        const binSize = 0.05; // 5cm，高于点云噪声但能区分地面与竖向结构
        const bins = new Map();
        arr.forEach(y => {
            const key = Math.round(y / binSize);
            bins.set(key, (bins.get(key) || 0) + 1);
        });

        let bestKey = null;
        let bestCount = -1;
        bins.forEach((count, key) => {
            if (count > bestCount) {
                bestCount = count;
                bestKey = key;
            }
        });

        if (bestKey === null) return arr[Math.floor(arr.length / 2)];
        const center = bestKey * binSize;
        const near = arr.filter(y => Math.abs(y - center) <= binSize * 1.5);
        const picked = near.length >= 5 ? near : arr;
        picked.sort((a, b) => a - b);
        return picked[Math.floor(picked.length / 2)];
    }

    function collectGroundYValuesFromSnapshotPositions(positions, footprintBox = null, matrix = null) {
        const ysNear = [];
        const ysAll = [];
        if (!positions || !positions.length) return { ysNear, ysAll };

        const stride = Math.max(3, Math.ceil((positions.length / 3) / 60000) * 3);
        const tmp = new THREE.Vector3();
        for (let i = 0; i < positions.length; i += stride) {
            tmp.set(Number(positions[i]), Number(positions[i + 1]), Number(positions[i + 2]));
            if (!Number.isFinite(tmp.x) || !Number.isFinite(tmp.y) || !Number.isFinite(tmp.z)) continue;
            if (matrix) tmp.applyMatrix4(matrix);
            if (Math.abs(tmp.x) >= 9000 || Math.abs(tmp.y) >= 9000 || Math.abs(tmp.z) >= 9000) continue;

            ysAll.push(tmp.y);
            if (!footprintBox || (
                tmp.x >= footprintBox.minX && tmp.x <= footprintBox.maxX &&
                tmp.z >= footprintBox.minZ && tmp.z <= footprintBox.maxZ
            )) {
                ysNear.push(tmp.y);
            }
        }
        return { ysNear, ysAll };
    }

    function estimateGprGroundYFromLoadedPointCloud() {
        if (typeof THREE === "undefined") return 0;

        const footprintBox = getGprScanFootprintXZBoxForGroundY(0.75);
        let nearValues = [];
        let allValues = [];

        // 本地点云快照：需要把 points/group 的冻结变换一起应用，得到真实显示位置。
        const localSnapshot = window.gprFrozenLocalPcdSnapshot;
        if (window.gprLoadedMapSource === "local" && localSnapshot?.positions?.length) {
            const pointMatrix = makeMatrixFromTransformSnapshot(localSnapshot.points, { x: 1, y: 1, z: 1 });
            const groupMatrix = makeMatrixFromTransformSnapshot(localSnapshot.group, { x: 1, y: 1, z: 1 });
            const matrix = new THREE.Matrix4().multiplyMatrices(groupMatrix, pointMatrix);
            const collected = collectGroundYValuesFromSnapshotPositions(localSnapshot.positions, footprintBox, matrix);
            nearValues = nearValues.concat(collected.ysNear);
            allValues = allValues.concat(collected.ysAll);
        }

        // 当前路径页点云快照：导出时已经转换成 GPR 页世界坐标，直接取 positions 即可。
        const currentSnapshot = window.latestImportedInspectionMapSnapshot;
        const currentPositions = currentSnapshot?.pointCloud?.positions;
        if (currentPositions && currentPositions.length) {
            const collected = collectGroundYValuesFromSnapshotPositions(currentPositions, footprintBox, null);
            nearValues = nearValues.concat(collected.ysNear);
            allValues = allValues.concat(collected.ysAll);
        }

        // 优先用扫描区域附近点云；附近点太少时再退回整幅点云。
        let groundY = estimateDominantGroundYFromValues(nearValues.length >= 30 ? nearValues : allValues);

        // 如果没有点云快照，则使用已知的黄色电缆路径高度作为兜底；仍没有则保持原来的 Y=0。
        if (!Number.isFinite(groundY)) {
            const cable = currentSnapshot?.cablePath;
            if (Array.isArray(cable) && cable.length) {
                const cableYs = cable.map(p => Number(p.z ?? 0.08)).filter(Number.isFinite);
                groundY = estimateDominantGroundYFromValues(cableYs);
            }
        }

        if (!Number.isFinite(groundY)) groundY = 0;
        return Number(groundY);
    }

    function reloadInspectionMapAndScanPlanAfterGprRender(usePlannedLayout) {
        if (!usePlannedLayout) return;

        try {
            // GPR 处理会重建 threeContainer 和 WebGL 场景，因此这里需要把点云地图、电缆路径、
            // 破损点重新叠加到新场景中。
            // 注意：处理完成后不再重新绘制规划矩形和 Z 字形折线，避免它们遮挡已生成的地下剖面。
            const frozenSnapshot = window.latestImportedInspectionMapSnapshot || null;
            const mapSource = window.gprLoadedMapSource || "";

            if (mapSource === "local") {
                // 用户加载的是电脑本地 PCD 文件：GPR 场景重建后必须恢复这份冻结的本地点云，
                // 不能回退调用 loadCurrentPathFaultMapToGpr()，否则会重新读取路径页实时点云，
                // 导致地图变成履带小车扫描作业后的最新地图。
                restoreFrozenLocalPcdMapToGprScene({ fitCamera: false });
            } else if (frozenSnapshot && typeof loadCurrentPathFaultMapToGpr === "function") {
                const realExporter = window.exportCurrentPathFaultMapForGpr;

                try {
                    // 优先使用用户点击“加载当前点云地图”时保存的冻结快照，
                    // 避免 GPR 处理完成后重新读取路径页的实时点云。
                    window.exportCurrentPathFaultMapForGpr = function () {
                        return frozenSnapshot;
                    };
                    loadCurrentPathFaultMapToGpr({ fitCamera: false });
                } finally {
                    window.exportCurrentPathFaultMapForGpr = realExporter;
                }
            } else if (mapSource === "current" && typeof loadCurrentPathFaultMapToGpr === "function") {
                // 兼容旧流程：如果来源明确是“当前地图”但没有冻结快照，才允许兜底重新加载当前地图。
                loadCurrentPathFaultMapToGpr({ fitCamera: false });
            }

            // ✅ 核心修改：GPR 剖面渲染完成后隐藏规划矩形区域和 Z 字形折线。
            // 这里只清除显示层，不清空 window.currentGprScanPlan，
            // 因为后续剖面按规划路径融合仍需要使用这份扫描路径数据。
            if (typeof clearGprScanPlanOverlay === "function") {
                clearGprScanPlanOverlay();
            }
        } catch (e) {
            console.warn("⚠️ GPR 剖面已按规划路径生成，但重新叠加点云地图失败：", e);
            if (typeof clearGprScanPlanOverlay === "function") {
                clearGprScanPlanOverlay();
            }
        }
    }

    // 渲染一次完整结果：填充下拉框 + 默认显示第 1 段 + 画 Z 字 3D
    async function renderGprResults(result) {
        if (!result || !result.segments || !result.segments.length) {
            throw new Error("返回数据中没有 segments");
        }
        gprResult = result;
        if (typeof result.migrationEnabled === "boolean") {
            setGprMigrationEnabled(result.migrationEnabled, false);
        }
        currentGprSegmentIndex = 0;
        manualMigAnnotationMode = false;
        forcedPipeAnnotationMode = false;
        activeForcedPipeGroupId = null;
        forcedPipeGroupCounter = 0;
        forcedPipeAnnotationOrder = 0;
        manualMigAnnotationsBySegment.clear();
        forcedPipeAnnotationsBySegment.clear();
        gprPipeColorAssignments.clear();
        selectedGprPipeObject = null;

        const folderInputForCalibration = $("folderPathInput");
        const folderPathForCalibration = folderInputForCalibration ? folderInputForCalibration.value.trim() : currentGprFolderPath;
        const calibrationLoadInfo = await loadGprAnnotationCalibration(result, folderPathForCalibration);

        updateManualMigAnnotationUI();
        updateForcedPipeAnnotationUI();

        // 如果该文件夹存在历史人工校准，默认直接打开标注显示，确保用户再次打开时看到的是校准后的结果。
        if (calibrationLoadInfo && calibrationLoadInfo.loaded) {
            setAnnotationButtonState(true);
        }

        // ✅ 数据加载成功后，显示标注按钮
        const btnAnno = $("btnToggleAnnotation");
        if (btnAnno) btnAnno.style.display = "inline-block";

        const segmentSelect = $("segmentSelect");
        if (segmentSelect) {
            segmentSelect.innerHTML = "";
            result.segments.forEach(function (seg, idx) {
                const opt = document.createElement("option");
                opt.value = String(idx);
                const vStr = gprMigrationEnabled && seg.vBest != null ? (" v*=" + seg.vBest.toFixed(4) + " m/ns") : "";
                const name = seg.fileName || ("剖面" + (idx + 1));
                opt.textContent = (idx + 1) + " - " + name + vStr;
                segmentSelect.appendChild(opt);
            });
            segmentSelect.disabled = false;
            segmentSelect.onchange = function () {
                const index = parseInt(segmentSelect.value, 10);
                if (!isNaN(index)) showSegment(index);
            };
            segmentSelect.value = "0";
        }

        // 初次展示由 renderGprResults 统一记录完成日志，避免 showSegment 重复写入。
        showSegment(0, false);
        render3DZigzagAll(result);

        const totalYoloDetections = result.segments.reduce(
            (sum, seg) => sum + (Array.isArray(seg.rawDetections) ? seg.rawDetections.length : 0),
            0
        );
        const totalLocatedApexes = result.segments.reduce(
            (sum, seg) => sum + (Array.isArray(seg.rawDetections)
                ? seg.rawDetections.filter(detection =>
                    detection.apexCol != null && detection.apexRow != null &&
                    Number.isFinite(Number(detection.apexCol)) && Number.isFinite(Number(detection.apexRow))
                ).length
                : 0),
            0
        );
        const placementSummary = window.gprUsingRtkLayoutInCurrentRender
            ? "剖面内部按DZT固定道间距拼接，RTK仅校准整张剖面的位置和方向"
            : "剖面内部按DZT固定道间距拼接";
        const sourceSummary = "三维视图已使用原始二维剖面，显示 " + totalYoloDetections +
            " 个识别框和 " + totalLocatedApexes + " 个有效波形顶点；" + placementSummary +
            (gprMigrationEnabled ? "；本次已执行偏移成像" : "；本次已跳过偏移成像");

        if (calibrationLoadInfo && calibrationLoadInfo.reason === "signature-mismatch") {
            setStatus(
                "处理完成，YOLO 共识别到 " + totalYoloDetections + " 个管道双曲线；" + sourceSummary + "。历史校准与当前剖面不一致，未加载该校准。",
                "error"
            );
        } else {
            setStatus("处理完成，YOLO 共识别到 " + totalYoloDetections + " 个管道双曲线；" + sourceSummary + "。", "success");
            addSystemMessage("地下环境已用原始剖面重建：识别 " + totalYoloDetections + " 个目标，定位 " + totalLocatedApexes + " 个有效波形顶点。", "success");
        }
    }

    // 显示指定索引的剖面（2D）
    function showSegment(index, showStatus = true) {
        if (!gprResult || !gprResult.segments) return;
        if (index < 0 || index >= gprResult.segments.length) return;

        currentGprSegmentIndex = index;
        const seg = gprResult.segments[index];

        if (seg.rawData) {
            render2DCanvas("raw2dCanvas", seg.rawData, seg.rawDetections || []);
        }
        if (gprMigrationEnabled && seg.migratedData) {
            render2DCanvas("mig2dCanvas", seg.migratedData);
        } else {
            clearGpr2DCanvasById("mig2dCanvas");
        }

        if (showStatus) {
            const detectionCount = Array.isArray(seg.rawDetections) ? seg.rawDetections.length : 0;
            const apexCount = Array.isArray(seg.rawDetections)
                ? seg.rawDetections.filter(detection => detection.apexCol != null && detection.apexRow != null).length
                : 0;
            setStatus(
                "处理完成。当前显示第 " + (index + 1) + " 段 B 扫结果，YOLO 识别到 " + detectionCount + " 个管道双曲线，定位到 " + apexCount + " 个波形顶点。",
                "success"
            );
        }
    }

    // ====================== 2D 灰度图绘制 ======================

    function computeRobustGprAbsLimit(matrices, percentile = 0.955, maxSamples = 100000) {
        const validMatrices = (matrices || []).filter(matrix =>
            Array.isArray(matrix) && matrix.length > 0 && Array.isArray(matrix[0]) && matrix[0].length > 0
        );
        if (!validMatrices.length) return 1e-9;

        const totalValues = validMatrices.reduce((sum, matrix) =>
            sum + matrix.length * matrix[0].length, 0
        );
        const sampleStep = Math.max(1, Math.floor(totalValues / maxSamples));
        const absSamples = [];
        let maxAbs = 0;
        let valueIndex = 0;

        validMatrices.forEach(matrix => {
            for (let r = 0; r < matrix.length; r++) {
                for (let c = 0; c < matrix[r].length; c++) {
                    const value = Number(matrix[r][c]);
                    if (!Number.isFinite(value)) continue;
                    const absValue = Math.abs(value);
                    if (absValue > maxAbs) maxAbs = absValue;
                    if (valueIndex % sampleStep === 0) absSamples.push(absValue);
                    valueIndex++;
                }
            }
        });

        if (!absSamples.length || maxAbs === 0) return 1e-9;
        absSamples.sort((a, b) => a - b);
        const safePercentile = Math.max(0.5, Math.min(0.999, percentile));
        const percentileIndex = Math.floor((absSamples.length - 1) * safePercentile);
        const robustLimit = absSamples[percentileIndex];

        // 极稀疏数据的分位数可能为 0，此时回退到原来的最大值比例。
        return robustLimit > 0 ? robustLimit : Math.max(maxAbs * 0.6, 1e-9);
    }

    function render2DCanvas(canvasId, matrix, detections = []) {
        const canvas = $(canvasId);
        if (!canvas || !matrix || !matrix.length || !matrix[0].length) return;

        const rows = matrix.length;
        const cols = matrix[0].length;
        const maxCanvasWidth = 500;
        const maxCanvasHeight = 500;

        // 横纵方向必须使用同一个缩放比例。旧逻辑分别 Math.ceil：512 行只因略超 500
        // 就会令 stepY 从 1 跳到 2、画布高度骤降为 256，视觉上相当于横向拉伸近 2 倍。
        const uniformScale = Math.max(
            1,
            cols / maxCanvasWidth,
            rows / maxCanvasHeight
        );
        const w = Math.max(1, Math.round(cols / uniformScale));
        const h = Math.max(1, Math.round(rows / uniformScale));

        // 使用连续的源矩阵坐标比例，并让画布首末像素准确对应矩阵首末行/列。
        // 这些比例同时供圆圈绘制和鼠标标注坐标反算使用。
        const stepX = w > 1 && cols > 1 ? (cols - 1) / (w - 1) : 1;
        const stepY = h > 1 && rows > 1 ? (rows - 1) / (h - 1) : 1;

        canvas.width = w;
        canvas.height = h;

        // 记录本次二维绘制的矩阵尺寸和降采样比例，供鼠标手动标注反算到原始矩阵坐标。
        gpr2DRenderInfo[canvasId] = {
            rows,
            cols,
            stepX,
            stepY,
            w,
            h
        };

        const ctx = canvas.getContext("2d");
        const imgData = ctx.createImageData(w, h);

        // 使用 95.5% 分位数抵消浅层抑制导致的色标变化，使中层显示对比度保持稳定。
        const cLimit = computeRobustGprAbsLimit([matrix]);
        const minLim = -cLimit;
        const maxLim = cLimit;

        let idx = 0;
        for (let y = 0; y < h; y++) {
            const r = Math.min(rows - 1, Math.round(y * stepY));
            for (let x = 0; x < w; x++) {
                const c = Math.min(cols - 1, Math.round(x * stepX));
                const v = matrix[r][c];

                // 线性映射到 [0, 1] 范围
                let norm = (v - minLim) / (maxLim - minLim);
                // 强制截断上下限 (超出范围的强波直接变纯黑/纯白)
                norm = Math.max(0, Math.min(1, norm));

                // 去掉之前的 Math.pow() 伽马校正，保持纯线性灰度
                const gray = Math.round(norm * 255);

                imgData.data[idx++] = gray;
                imgData.data[idx++] = gray;
                imgData.data[idx++] = gray;
                imgData.data[idx++] = 255;
            }
        }
        ctx.putImageData(imgData, 0, 0);

        // YOLO 直接识别偏移前 rawData，坐标已经由后端反算到原始矩阵坐标。
        if (canvasId === "raw2dCanvas" && Array.isArray(detections) && detections.length > 0) {
            ctx.save();
            ctx.lineWidth = 2;
            ctx.strokeStyle = "#ff2b2b";
            ctx.fillStyle = "#ff2b2b";
            ctx.shadowColor = "rgba(255, 0, 0, 0.8)";
            ctx.shadowBlur = 4;
            ctx.font = "12px sans-serif";
            ctx.textBaseline = "top";

            detections.forEach(detection => {
                const x1 = Math.max(0, Number(detection.x1) / stepX);
                const y1 = Math.max(0, Number(detection.y1) / stepY);
                const x2 = Math.min(w - 1, Number(detection.x2) / stepX);
                const y2 = Math.min(h - 1, Number(detection.y2) / stepY);
                if (![x1, y1, x2, y2].every(Number.isFinite) || x2 <= x1 || y2 <= y1) return;

                ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);

                const confidence = Number(detection.confidence);
                const text = "管道 " + (Number.isFinite(confidence) ? Math.round(confidence * 100) + "%" : "");
                const textWidth = ctx.measureText(text).width;
                const labelHeight = 16;
                const labelY = y1 >= labelHeight ? y1 - labelHeight : y1;
                ctx.shadowBlur = 0;
                ctx.fillRect(x1, labelY, textWidth + 6, labelHeight);
                ctx.fillStyle = "#ffffff";
                ctx.fillText(text, x1 + 3, labelY + 2);
                ctx.fillStyle = "#ff2b2b";
                ctx.shadowBlur = 4;

                // 后端在原始矩阵中拟合出的波形顶点；低质量框不返回顶点。
                if (detection.apexCol != null && detection.apexRow != null) {
                    const apexX = Number(detection.apexCol) / stepX;
                    const apexY = Number(detection.apexRow) / stepY;
                    if (Number.isFinite(apexX) && Number.isFinite(apexY)
                        && apexX >= 0 && apexX < w && apexY >= 0 && apexY < h) {
                        ctx.save();
                        ctx.shadowBlur = 0;
                        const apexColor = detection.apexMethod === "singleWing" ? "#ffd64a" : "#00ffff";
                        ctx.strokeStyle = apexColor;
                        ctx.fillStyle = apexColor;
                        ctx.lineWidth = 2;
                        ctx.beginPath();
                        ctx.arc(apexX, apexY, 4, 0, 2 * Math.PI);
                        ctx.moveTo(apexX - 8, apexY);
                        ctx.lineTo(apexX + 8, apexY);
                        ctx.moveTo(apexX, apexY - 8);
                        ctx.lineTo(apexX, apexY + 8);
                        ctx.stroke();
                        ctx.beginPath();
                        ctx.arc(apexX, apexY, 2, 0, 2 * Math.PI);
                        ctx.fill();
                        ctx.restore();
                    }
                }
            });
            ctx.restore();
        }



        // 偏移剖面只保留人工校准圆圈；旧版亮斑自动红圈已移除。
        if (canvasId === "mig2dCanvas" && showAnnotation) {
            const redBlobs = getManualMigAnnotationBlobs(currentGprSegmentIndex);
            const forcedPipeBlobs = getForcedPipeAnnotationBlobs(currentGprSegmentIndex);

            function drawCircle(blob, colorCss, shadowCss) {
                ctx.lineWidth = 1;
                ctx.strokeStyle = colorCss;
                ctx.shadowColor = shadowCss || colorCss;
                ctx.shadowBlur = 5;
                ctx.beginPath();
                const cx = blob.col / stepX;
                const cy = blob.row / stepY;
                const r = (blob.radius + CONFIG.radiusPadding) / Math.max(stepX, stepY);
                ctx.arc(cx, cy, r, 0, 2 * Math.PI);
                ctx.stroke();
            }

            // 人工校准标注保持红色；它不参与普通管线自动连接。
            redBlobs.forEach(blob => drawCircle(blob, "#ff0000", "red"));

            // 倾斜/交叉管道强制连接标注按管道组显示不同颜色。
            forcedPipeBlobs.forEach(blob => drawCircle(blob, blob.pipeColorCss || "#0099ff", blob.pipeColorCss || "#0099ff"));

            ctx.shadowBlur = 0;
        }
    }

    // ====================== Z 字型三维拼接 ======================


    // ==========================================
    // ✅ 三维重建页：地下管道点击信息弹窗
    // 说明：管道 Mesh 生成时会注册中心线点；点击管道后，取点击点到中心线最近点作为测点，
    //       由 GPR 世界坐标反推 SLAM 平面坐标，再复用 slamToGps() 显示经纬度。
    // ==========================================
    function ensureGprPipeInfoPopup() {
        let popup = $("gprPipeInfoPopup");
        if (popup) {
            ensureGprPipeColorControls(popup);
            const existingCloseBtn = popup.querySelector(".gpr-pipe-popup-close");
            if (existingCloseBtn && existingCloseBtn.dataset.boundHide !== "1") {
                existingCloseBtn.dataset.boundHide = "1";
                existingCloseBtn.onclick = hideGprPipeInfoPopup;
            }
            return popup;
        }

        const container = $("threeContainer");
        const parent = container ? (container.closest(".gpr-view-3d") || container.parentElement) : document.body;
        if (!parent) return null;

        popup = document.createElement("div");
        popup.id = "gprPipeInfoPopup";
        popup.className = "gpr-pipe-info-popup";
        popup.style.display = "none";
        popup.innerHTML = `
            <div class="gpr-pipe-popup-close">×</div>
            <div class="gpr-pipe-popup-title-row">
                <div id="gprPipePopupTitle" class="gpr-pipe-popup-title">🟡 地下管道测点信息</div>
                <button type="button" id="gprPipeColorToggleBtn" class="pipe-color-toggle-btn">▼</button>
            </div>
            <div class="gpr-pipe-popup-item"><strong>经纬度：</strong><span id="gprPipePopupGps" class="hl-text">--</span></div>
            <div class="gpr-pipe-popup-item"><strong>深度：</strong><span id="gprPipePopupDepth" class="hl-text">--</span></div>
            <div id="gprPipeColorSection" class="gpr-pipe-color-section">
                <div class="gpr-pipe-color-actions collapsed">
                    <button type="button" data-gpr-pipe-material="cable" class="pipe-color-btn pipe-color-cable"><span class="pipe-color-dot"></span>地下电缆</button>
                    <button type="button" data-gpr-pipe-material="pvc" class="pipe-color-btn pipe-color-pvc"><span class="pipe-color-dot"></span>PVC管</button>
                    <button type="button" data-gpr-pipe-material="metal" class="pipe-color-btn pipe-color-metal"><span class="pipe-color-dot"></span>金属管道</button>
                    <button type="button" data-gpr-pipe-material-reset="1" class="pipe-color-btn pipe-color-reset">恢复默认</button>
                </div>
            </div>
        `;
        ensureGprPipeColorControls(popup);
        const closeBtn = popup.querySelector(".gpr-pipe-popup-close");
        if (closeBtn) closeBtn.onclick = hideGprPipeInfoPopup;
        parent.appendChild(popup);
        return popup;
    }

    function hideGprPipeInfoPopup() {
        const popup = $("gprPipeInfoPopup");
        if (popup) popup.style.display = "none";
        gprPipePopup3DPos = null;
        selectedGprPipeObject = null;
    }

    function registerGprPipePickMesh(mesh, centerlinePoints, meta = {}) {
        if (!mesh || !Array.isArray(centerlinePoints) || centerlinePoints.length < 2) return mesh;

        const cleanPoints = centerlinePoints
            .map(p => p && p.clone ? p.clone() : new THREE.Vector3(Number(p.x), Number(p.y), Number(p.z)))
            .filter(p => Number.isFinite(p.x) && Number.isFinite(p.y) && Number.isFinite(p.z));

        if (cleanPoints.length < 2) return mesh;

        const legacyColorKey = String(meta.legacyColorKey || meta.coordColorKey || makeGprPipeColorKeyFromPoints(cleanPoints, meta));
        const pipeColorKey = String(meta.colorKey || meta.pipeColorKey || meta.stableColorKey || legacyColorKey);
        const mergedMeta = { ...(meta || {}), colorKey: pipeColorKey, legacyColorKey };

        mesh.userData = mesh.userData || {};
        mesh.userData.isGprUndergroundPipe = true;
        mesh.userData.gprCenterlinePoints = cleanPoints;
        mesh.userData.gprPipeColorKey = pipeColorKey;
        mesh.userData.gprPipeLegacyColorKey = legacyColorKey;
        mesh.userData.gprPipeMeta = mergedMeta;
        mesh.name = meta.name || mesh.name || "地下管道可点击对象";
        applyGprPipeSavedColorIfAny(mesh);

        if (!window.gprPipePickObjects) {
            window.gprPipePickObjects = [];
        }
        if (!window.gprPipePickObjects.includes(mesh)) {
            window.gprPipePickObjects.push(mesh);
        }
        return mesh;
    }

    function isObjectEffectivelyVisible(obj) {
        let cur = obj;
        while (cur) {
            if (cur.visible === false) return false;
            cur = cur.parent;
        }
        return true;
    }

    function findClosestPointOnSegment(point, a, b) {
        const ab = new THREE.Vector3().subVectors(b, a);
        const lenSq = ab.lengthSq();
        if (lenSq < 1e-12) return a.clone();
        const t = Math.max(0, Math.min(1, new THREE.Vector3().subVectors(point, a).dot(ab) / lenSq));
        return a.clone().add(ab.multiplyScalar(t));
    }

    function findClosestPointOnPolyline(point, points) {
        if (!Array.isArray(points) || points.length === 0) return point.clone();
        if (points.length === 1) return points[0].clone();

        let best = points[0].clone();
        let bestDistSq = Infinity;
        for (let i = 0; i < points.length - 1; i++) {
            const p = findClosestPointOnSegment(point, points[i], points[i + 1]);
            const d = p.distanceToSquared(point);
            if (d < bestDistSq) {
                bestDistSq = d;
                best = p;
            }
        }
        return best;
    }

    function gprWorldPointToSlam(point) {
        // GPR 三维重建页：X/Z 为水平面，Y 为高度/深度；
        // slamPointToGprVector 的逆变换为：GPR(x,y,z) -> SLAM(x,-z)。
        return {
            x: Number(point.x),
            y: -Number(point.z)
        };
    }

    function setTextSafe(id, text) {
        const el = $(id);
        if (el) el.textContent = text;
    }

    function showGprPipeInfoPopupFromHit(intersection) {
        if (!intersection) return false;

        let obj = intersection.object;
        while (obj && !obj.userData?.isGprUndergroundPipe && obj.parent) {
            obj = obj.parent;
        }
        if (!obj || !obj.userData?.isGprUndergroundPipe) return false;

        selectedGprPipeObject = obj;
        const centerline = obj.userData.gprCenterlinePoints || [];
        // 管线中心点以GPR局部世界坐标保存；原始矩阵比例模式会缩放整个GPR组的Y轴，
        // 因此点选时先应用该组矩阵，再与射线命中的真实世界坐标比较。
        const gprWorld = window.gprDataWorldGroup;
        let centerlineForPick = centerline;
        if (gprWorld && gprWorld.scale && Math.abs(Number(gprWorld.scale.y) - 1) > 1e-9) {
            gprWorld.updateMatrixWorld(true);
            centerlineForPick = centerline.map(point => point.clone().applyMatrix4(gprWorld.matrixWorld));
        }
        const closest = findClosestPointOnPolyline(intersection.point, centerlineForPick);
        const slam = gprWorldPointToSlam(closest);
        // GPR 剖面/管道现在会按点云地面高度 groundY 对齐，
        // 因此深度不能再简单使用 -Y，而要以当前地下环境顶面作为 0m 基准。
        const groundY = Number.isFinite(Number(window.gprGroundYForDepthReference))
            ? Number(window.gprGroundYForDepthReference)
            : 0;
        const depthDisplayScale = Number.isFinite(Number(window.gprDepthDisplayScale)) && Number(window.gprDepthDisplayScale) > 0
            ? Number(window.gprDepthDisplayScale)
            : 1;
        const depth = Math.max(0, (groundY - closest.y) / depthDisplayScale);

        let gpsText = "GPS未标定";
        if (typeof window.slamToGps === "function") {
            const gps = window.slamToGps(slam.x, slam.y);
            if (gps && Number.isFinite(gps.lat) && Number.isFinite(gps.lon)) {
                gpsText = `纬度 ${gps.lat.toFixed(8)}，经度 ${gps.lon.toFixed(8)}`;
            }
        }

        setTextSafe("gprPipePopupGps", gpsText);
        setTextSafe("gprPipePopupDepth", `${depth.toFixed(2)} m`);
        updateGprPipeColorControls(obj);

        const popup = ensureGprPipeInfoPopup();
        if (!popup) return false;

        // 每次重新点击管道时，默认收起下方“指定材质”操作区，只保留测点信息和当前材质。
        const actionPanel = popup.querySelector(".gpr-pipe-color-actions");
        const toggleBtn = popup.querySelector("#gprPipeColorToggleBtn");
        if (actionPanel) actionPanel.classList.add("collapsed");
        if (toggleBtn) toggleBtn.textContent = "▼";

        popup.style.display = "block";
        gprPipePopup3DPos = closest.clone();
        updateGprPipeInfoPopupPosition();
        return true;
    }

    function updateGprPipeInfoPopupPosition() {
        const popup = $("gprPipeInfoPopup");
        if (!popup || popup.style.display === "none" || !gprPipePopup3DPos || !window.gprCamera || !window.gprRenderer) {
            return;
        }

        const dom = window.gprRenderer.domElement;
        if (!dom) return;

        const vector = gprPipePopup3DPos.clone().project(window.gprCamera);
        if (vector.z < -1 || vector.z > 1) {
            popup.style.opacity = "0";
            return;
        }
        popup.style.opacity = "1";

        const domRect = dom.getBoundingClientRect();
        const parent = popup.offsetParent || popup.parentElement || document.body;
        const parentRect = parent.getBoundingClientRect();
        const screenX = dom.dataset.gprMirroredView === "true" ? -vector.x : vector.x;
        const x = domRect.left + (screenX * 0.5 + 0.5) * domRect.width - parentRect.left;
        const y = domRect.top + (-vector.y * 0.5 + 0.5) * domRect.height - parentRect.top;

        const margin = 10;
        const clampedX = Math.max(margin, Math.min(parentRect.width - margin, x));
        const clampedY = Math.max(margin, Math.min(parentRect.height - margin, y));
        popup.style.left = clampedX + "px";
        popup.style.top = clampedY + "px";
    }

    function getGprPipeIntersectionFromMouse(event, camera, dom) {
        if (!event || !camera || !dom || typeof THREE === "undefined") return null;

        const candidates = (window.gprPipePickObjects || []).filter(isObjectEffectivelyVisible);
        if (!candidates.length) return null;

        const rect = dom.getBoundingClientRect();
        if (!rect.width || !rect.height) return null;

        const mouse = new THREE.Vector2(
            getGprCanvasPointerNdcX(event, dom, rect),
            -((event.clientY - rect.top) / rect.height) * 2 + 1
        );

        const raycaster = new THREE.Raycaster();
        raycaster.params.Line = { threshold: 0.12 };
        raycaster.setFromCamera(mouse, camera);

        const hits = raycaster.intersectObjects(candidates, true);
        return hits.length ? hits[0] : null;
    }

    function resetGprPipeHoverCursor(dom) {
        if (!dom) return;
        if (dom.style.cursor === "pointer") {
            dom.style.cursor = "";
        }
        if (dom.title === "点击查看此处经纬度和深度") {
            dom.removeAttribute("title");
        }
    }

    function updateGprPipeHoverCursor(event, camera, dom, isViewDragging = false) {
        if (!dom) return false;

        // 手动指定扫描区域时需要保持十字光标，不抢占该交互状态。
        if (manualScanPickState) return false;

        if (isViewDragging) {
            resetGprPipeHoverCursor(dom);
            return false;
        }

        const hit = getGprPipeIntersectionFromMouse(event, camera, dom);
        if (hit) {
            dom.style.cursor = "pointer";
            dom.title = "点击查看此处经纬度和深度";
            return true;
        }

        resetGprPipeHoverCursor(dom);
        return false;
    }

    function handleGprPipeClick(event, camera, dom) {
        const hit = getGprPipeIntersectionFromMouse(event, camera, dom);
        if (!hit) {
            hideGprPipeInfoPopup();
            return false;
        }

        event.preventDefault();
        event.stopPropagation();
        return showGprPipeInfoPopupFromHit(hit);
    }

    // 三维拼接固定使用完成波形识别和顶点定位的原始二维剖面；
    // 空间位置以同名CSV逐道RTK为优先，规划Z字路径和后端旧几何作为回退。
    function render3DZigzagAll(result) {
        const container = $("threeContainer");
        if (!container) return;

        if (typeof THREE === "undefined") {
            setStatus("three.js 未成功加载，无法显示三维视图。", "error");
            console.error("THREE is not defined. 检查 three.min.js 是否正确引入。");
            return;
        }

        const segments = result.segments || [];
        if (!segments.length) return;

        const hasRtkLayout = segments.some(seg => {
            return hasUsableRtkSegmentGeometry(seg, seg.rawData);
        });
        // RTK是实测位置，优先级高于人工规划的Z字路径；只有没有可用RTK时才走旧逻辑。
        const usePlannedLayout = !hasRtkLayout && hasUsableGprScanPlanForFusion();
        window.gprUsingRtkLayoutInCurrentRender = hasRtkLayout;
        window.gprRtkProjectedToSlamForCurrentRender = hasRtkLayout && hasActiveGpsSlamTransform();
        const plannedLegCount = getGprPlanLegCount();
        if (usePlannedLayout && plannedLegCount !== segments.length) {
            console.warn(`⚠️ GPR剖面数量(${segments.length})与规划Z字路径线段数量(${plannedLegCount})不一致，超出部分将保持后端原始geometry。`);
        }

        // 👇 1. 新增：彻底销毁旧的 WebGL 上下文和动画循环
        if (window.gprAnimId) {
            cancelAnimationFrame(window.gprAnimId);
            window.gprAnimId = null;
        }
        if (window.gprRenderer) {
            window.gprRenderer.dispose();
            window.gprRenderer.forceContextLoss();
            window.gprRenderer.domElement = null;
            window.gprRenderer = null;
        }
        // 👆 结束新增

        // 清空旧内容
        hideGprPipeInfoPopup();
        window.gprPipePickObjects = [];
        container.innerHTML = "";
        annotation3DGroups = []; // ✅ 清空旧的标注组
        window.currentPcdMesh = null;
        window.currentPcdGroup = null;
        window.currentImportedInspectionMapGroup = null;
        window.gprDataWorldGroup = null;
        window.gprDepthDisplayScale = 1;
        // GPR 处理会重建 WebGL 场景，旧的规划线对象已经不属于当前 scene。
        // 先清空引用，避免后续隐藏规划线时误处理旧场景对象。
        window.gprScanPlanGroup = null;
        window.gprZPathLine = null;
        // 注意：不要在这里清空 gprHasInspectionMapLayerLoaded。
        // 如果本次处理前已经加载过当前点云地图，下面会在新场景中自动重新加载并融合显示。
        window.gprCamera = null;
        window.gprSetViewToBox = null;

        // 记录每个剖面的标注圆心和半径，用于后续用圆柱连接
        const segmentAnnoCenters = [];
        const allCenters = [];
        const renderedSegmentGeometries = [];
        let rtkPositionedSegmentCount = 0;
        let rtkProjectedSegmentCount = 0;
        const rtkFitRmseValues = [];
        let globalCenterId = 0;

        // 1. 使用全部剖面的 95.5% 分位数统一色标，使浅层抑制前后的中层对比度保持稳定。
        const displayMatrices = segments
            .map(seg => seg.rawData)
            .filter(Boolean);
        const cLimit = computeRobustGprAbsLimit(displayMatrices);
        let vMin = -cLimit;  // 注意这里是用 let，因为后面在 buildSegmentMesh 会作为参数传入
        let vMax = cLimit;

        // 2. three.js 基本场景（不使用任何灯光）
        let width = container.clientWidth;
        let height = container.clientHeight;
        if (!width || !height) {
            width = 800;
            height = 500;
            container.style.minWidth = "800px";
            container.style.minHeight = "500px";
        }

        const scene = new THREE.Scene();
        scene.background = new THREE.Color(0x000000); // ✅ 改为黑色背景

        // 👇 把 GPR scene 暴露到全局，让外部点云地图加载函数能够获取到它
        window.gprScene = scene;

        const camera = new THREE.PerspectiveCamera(45, width / height, 0.01, 10000);
        window.gprCamera = camera;

        const renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(width, height);
        renderer.toneMapping = THREE.NoToneMapping;
        container.appendChild(renderer.domElement);
        // 👇 新增这一行：保存渲染器引用，方便下次销毁
        window.gprRenderer = renderer;

        // 所有剖面和路径统一放在 world 组里
        const world = new THREE.Group();
        scene.add(world);
        window.gprDataWorldGroup = world;
        window.gprDepthDisplayScale = 1;

        // ✅ 新增：创建一个专门用于存放雷达二维剖面的分组，方便一键隐藏
        window.gprSlicesGroup = new THREE.Group();
        // 原始剖面、识别框和顶点始终保留；“显示标注”只控制红圈和管线连接。
        window.gprSlicesGroup.visible = true;
        world.add(window.gprSlicesGroup);

        // 3. 计算几何包围盒 + 最大深度（单位：m）
        const bounds = {
            minX: Infinity, maxX: -Infinity,
            minY: Infinity, maxY: -Infinity,
            minZ: Infinity, maxZ: -Infinity
        };
        let maxDepth = 0.0;  // 真实深度（正值，单位 m）

        // ✅ 使用点云地图估计扫描区域附近真实地面高度，作为 GPR 深度 0m 基准。
        // 这样地下长方体顶面、GPR剖面顶边、管道和深度弹窗都会跟随点云地面，
        // 避免固定 Y=0 导致地下长方体突出地面点云。
        const gprGroundY = estimateGprGroundYFromLoadedPointCloud();
        window.gprGroundYForDepthReference = gprGroundY;
        window.gprCablePathAlignmentStats = null;
        const loadedCablePointCount = getLoadedSurfaceCablePathForGprAlignment().length;
        updateGprCableAlignmentBadge(
            loadedCablePointCount >= 2 ? "pending" : "unavailable",
            loadedCablePointCount >= 2 ? "路径校正：匹配中" : "路径校正：无黄色路径",
            loadedCablePointCount >= 2
                ? `正在用已加载的 ${loadedCablePointCount} 个黄色路径点匹配地下管道。`
                : "未同时满足黄色路径和扫描路径融合条件，地下管道将保留 GPR 原始位置。"
        );
        console.log("✅ GPR地下环境地面高度基准 groundY =", Number(gprGroundY.toFixed(4)));

        // 3.1 逐段构建剖面：DZT固定道间距成直线，RTK只校准整张剖面的中心和方向。
        segments.forEach((seg, segIndex) => {
            const mat = seg.rawData;
            const rtkGeo = createRtkSegmentGeometry(seg, mat);
            const plannedGeo = (!rtkGeo && usePlannedLayout)
                ? createPlannedSegmentGeometry(seg, segIndex, mat)
                : null;
            const geo = rtkGeo || plannedGeo || seg.geometry || {};
            if (!mat || !geo) return;
            renderedSegmentGeometries[segIndex] = geo;
            if (rtkGeo) {
                rtkPositionedSegmentCount++;
                if (rtkGeo._rtkProjectedToSlam) rtkProjectedSegmentCount++;
                if (Number.isFinite(Number(rtkGeo._rtkFitRmseM))) {
                    rtkFitRmseValues.push(Number(rtkGeo._rtkFitRmseM));
                }
            }

            // 记录该段最大深度
            if (geo.zAxis && geo.zAxis.length > 0) {
                const d = geo.zAxis[geo.zAxis.length - 1]; // 单位 m
                if (d > maxDepth) maxDepth = d;
            }

            const mesh = buildSegmentMesh(mat, geo, vMin, vMax, bounds, gprGroundY);
            // ✅ 修改：将切片网格添加到我们的专属分组中
            window.gprSlicesGroup.add(mesh);
            const rawDetectionOverlay = createGprRawDetectionOverlay(seg, geo, gprGroundY);
            if (rawDetectionOverlay.children.length > 0) {
                window.gprSlicesGroup.add(rawDetectionOverlay);
            }

            // 管线连接点直接取识别结果中的 apexRow/apexCol。
            // 普通顶点不再创建旧版红色圆环；彩色强制连接点仍保留人工圆环。
            if (seg.rawData) {
                const normalBlobs = getDetectedApexPipePoints(seg).map(blob => ({
                    ...blob,
                    linkMode: "normal",
                    colorHex: 0xff0000
                }));
                const forcedBlobs = getForcedPipeAnnotationBlobs(segIndex).map(blob => ({
                    ...blob,
                    linkMode: "forced",
                    colorHex: blob.pipeColorHex || 0x0099ff,
                    colorCss: blob.pipeColorCss || "#0099ff"
                }));
                const blobs = normalBlobs.concat(forcedBlobs);
                const annoGroup = new THREE.Group();

                const rows = seg.rawData.length;
                const cols = seg.rawData[0].length;

                // 1. 获取或计算坐标轴数据
                const zAxis = (geo.zAxis && geo.zAxis.length === rows)
                    ? geo.zAxis
                    : Array.from({ length: rows }, (_, i) => i * 0.02);

                const uAxis = (geo.uAxis && geo.uAxis.length === cols)
                    ? geo.uAxis
                    : Array.from({ length: cols }, (_, i) => i * 0.02);

                // 2. 计算比例尺 (关键步骤：计算 1 个像素代表多少米)
                const widthInMeters = uAxis[cols - 1] - uAxis[0];
                const scaleX = widthInMeters / cols;

                const heightInMeters = zAxis[rows - 1] - zAxis[0];
                const scaleY = heightInMeters / rows;

                const metersPerPixel = (scaleX + scaleY) / 2 || 0.02;

                // 当前剖面的标注列表
                const centersThisSeg = [];

                blobs.forEach(blob => {
                    // --- A. 位置计算 ---
                    const rIdx = Math.min(Math.max(0, Math.round(blob.row)), rows - 1);
                    const depth = zAxis[rIdx];   // z 轴上的物理坐标 (m, down)

                    const cIdx = Math.min(Math.max(0, Math.round(blob.col)), cols - 1);
                    const horizontal = getGprTraceHorizontalPoint(geo, cIdx, cols);
                    const worldX = horizontal.x;
                    const worldZ = horizontal.z;
                    const worldY = gprGroundY - depth;     // 以点云地面 groundY 为 Depth=0，深度向下为负

                    // --- B. 半径动态计算 ---
                    const pixelRadius = blob.radius + CONFIG.radiusPadding;
                    const ringRadius = (pixelRadius * metersPerPixel) * 0.6;
                    const tubeThickness = 0.01;

                    // 普通识别顶点仅作为管线拓扑节点，不再显示自动红色圆环。
                    if (blob.linkMode === "forced") {
                        const geometry = new THREE.TorusGeometry(ringRadius, tubeThickness, 16, 32);
                        const ringColor = blob.colorHex || blob.pipeColorHex || 0x0099ff;
                        const material = new THREE.MeshBasicMaterial({
                            color: ringColor,
                            side: THREE.DoubleSide,
                            depthTest: true,
                            depthWrite: false
                        });
                        const ring = new THREE.Mesh(geometry, material);
                        ring.position.set(worldX, worldY, worldZ);
                        ring.rotation.y = -horizontal.phi;
                        annoGroup.add(ring);
                    }

                    // 记录圆心和半径 + 在坐标轴上的投影
                    // 注意：材质保存/恢复使用的是二维剖面标注身份，而不是 worldX/worldY/worldZ。
                    // 这样即使下次三维场景按点云地图 Z 字路径融合摆放，管道材质 key 仍然稳定。
                    const annotationIdentity = makeGprAnnotationIdentityFromBlob(blob, seg, segIndex);
                    const centerObj = {
                        id: globalCenterId++,
                        center: new THREE.Vector3(worldX, worldY, worldZ),
                        radius: ringRadius,
                        zCoord: depth,   // 对应 z 轴的坐标值 (m, down)
                        yCoord: worldZ,  // 对应 y 轴的坐标值 (m)
                        segIndex: segIndex,
                        segmentName: annotationIdentity.segmentName,
                        segmentToken: annotationIdentity.segmentToken,
                        blobSource: annotationIdentity.blobSource,
                        annotationStableKey: annotationIdentity.annotationStableKey,
                        annotationLocalId: annotationIdentity.annotationLocalId,
                        annotationRow: annotationIdentity.annotationRow,
                        annotationCol: annotationIdentity.annotationCol,
                        annotationRadius: annotationIdentity.annotationRadius,
                        source: blob.source || annotationIdentity.blobSource,
                        apexKey: blob.apexKey || null,
                        manualIndex: Number.isFinite(Number(blob.manualIndex)) ? Number(blob.manualIndex) : null,
                        forcedIndex: Number.isFinite(Number(blob.forcedIndex)) ? Number(blob.forcedIndex) : null,
                        rawRow: Number(blob.row),
                        rawCol: Number(blob.col),
                        rawRadius: Number(blob.radius),
                        linkMode: blob.linkMode || "normal",
                        pipeGroupId: blob.pipeGroupId || null,
                        pipeGroupNo: blob.pipeGroupNo || null,
                        pipeColorHex: blob.pipeColorHex || blob.colorHex || null,
                        pipeColorCss: blob.pipeColorCss || blob.colorCss || null,
                        pipeOrder: Number(blob.pipeOrder || 0)
                    };
                    centersThisSeg.push(centerObj);
                    allCenters[centerObj.id] = centerObj;
                });

                annoGroup.visible = showAnnotation;
                world.add(annoGroup);
                annotation3DGroups.push(annoGroup);

                // 保存当前剖面的标注中心
                segmentAnnoCenters.push(centersThisSeg);
            }
        });

        // ================= 3.2 剖面之间的圆柱连接（按管道分组、不同颜色） =================
        if (segmentAnnoCenters.length >= 3) { // ✅ 修改：至少需要 3 个剖面才可能形成三层跨越
            const edges = [];
            const potentialLinks = []; // 用于暂存两两相邻剖面之间的候选连线

            // 1) 第一阶段：找出所有相邻两层的潜在连线
            for (let i = 0; i < segmentAnnoCenters.length - 1; i++) {
                const listA = segmentAnnoCenters[i].filter(item => item.linkMode === "normal");
                const listB = segmentAnnoCenters[i + 1].filter(item => item.linkMode === "normal");
                const currentLayerLinks = [];

                listA.forEach(a => {
                    listB.forEach(b => {
                        // “同一水平面”：z 轴投影差得小
                        const zDiff = Math.abs(a.zCoord - b.zCoord);
                        // “同一垂直面”：y 轴投影差得小
                        const yDiff = Math.abs(a.yCoord - b.yCoord);

                        if (zDiff <= LINK_CONFIG.maxZDiff &&
                            yDiff <= LINK_CONFIG.maxYDiff) {
                            currentLayerLinks.push({ a: a, b: b });
                        }
                    });
                });
                potentialLinks.push(currentLayerLinks);
            }

            // 2) 第二阶段：核心验证过滤
            // 规则：一条边 (a->b) 必须有前驱 (x->a) 或 后继 (b->y) 才能说明它至少横跨了3个剖面
            const validLinks = [];
            for (let i = 0; i < potentialLinks.length; i++) {
                const currentLayerLinks = potentialLinks[i];

                currentLayerLinks.forEach(link => {
                    let hasPredecessor = false;
                    let hasSuccessor = false;

                    // 检查前驱 (在上一层找有没有边以当前起点的 id 为终点)
                    if (i > 0) {
                        hasPredecessor = potentialLinks[i - 1].some(prevLink => prevLink.b.id === link.a.id);
                    }

                    // 检查后继 (在下一层找有没有边以当前终点的 id 为起点)
                    if (i < potentialLinks.length - 1) {
                        hasSuccessor = potentialLinks[i + 1].some(nextLink => nextLink.a.id === link.b.id);
                    }

                    // 只有具备前驱或者后继，才属于>=3个剖面的连续管道
                    if (hasPredecessor || hasSuccessor) {
                        validLinks.push(link);
                    }
                });
            }

            // 3) 根据验证通过的连线记录拓扑边。
            //    这里先不直接生成一段一段的硬直圆柱，后面按连通分量整理成整条管道后再做平滑。
            validLinks.forEach(link => {
                const a = link.a;
                const b = link.b;
                edges.push({
                    aId: a.id,
                    bId: b.id,
                    a,
                    b,
                    radius: (a.radius + b.radius) / 2
                });
            });

            // 4) 用并查集根据 edges 做“管道分组”（连通分量）
            if (edges.length > 0 && allCenters.length > 0) {
                const parent = new Array(allCenters.length);
                for (let i = 0; i < allCenters.length; i++) parent[i] = i;

                function find(x) {
                    if (parent[x] !== x) parent[x] = find(parent[x]);
                    return parent[x];
                }

                function union(x, y) {
                    const fx = find(x);
                    const fy = find(y);
                    if (fx !== fy) parent[fy] = fx;
                }

                edges.forEach(e => union(e.aId, e.bId));

                // 按根节点把边分组 -> 每组是一条管道
                const pipelinesMap = new Map(); // root -> [edge]
                edges.forEach(e => {
                    const root = find(e.aId);
                    let arr = pipelinesMap.get(root);
                    if (!arr) {
                        arr = [];
                        pipelinesMap.set(root, arr);
                    }
                    arr.push(e);
                });

                // 先整理每条候选管道并完成“黄色地上电缆路径”对应关系判断。
                // 匹配必须在所有连通分量之间全局择优，不能在 forEach 中遇到第一条就吸附。
                const pipelineEntries = Array.from(pipelinesMap.entries()).map(([root, edgeItems]) => {
                    const centerMap = new Map();
                    edgeItems.forEach(e => {
                        centerMap.set(e.aId, e.a);
                        centerMap.set(e.bId, e.b);
                    });
                    const centers = Array.from(centerMap.values())
                        .sort((a, b) => {
                            if (a.segIndex !== b.segIndex) return a.segIndex - b.segIndex;
                            if (Math.abs(a.yCoord - b.yCoord) > 1e-9) return a.yCoord - b.yCoord;
                            return a.zCoord - b.zCoord;
                        });
                    const colorKey = makeGprPipeColorKeyFromCenters(centers, { source: "auto" });
                    const legacyColorKey = makeGprPipeLegacyColorKeyFromCenters(centers, { source: "auto" });
                    return {
                        root,
                        edgeItems,
                        centers,
                        colorKey,
                        legacyColorKey,
                        assignment: getGprPipeAssignmentByKey(colorKey, [legacyColorKey])
                    };
                });
                const selectedCableAlignment = selectBestGprPipeCableAlignment(pipelineEntries);

                // 给每条管道分配不同颜色，并单独建组
                const connPipelinesGroup = new THREE.Group();
                let pipelineIndex = 0;
                const pipelineCount = pipelineEntries.length;

                pipelineEntries.forEach(entry => {
                    const edgeItems = entry.edgeItems;
                    const centers = entry.centers;
                    const pipelineGroup = new THREE.Group();

                    const color = new THREE.Color();
                    if (pipelineCount === 1) {
                        color.setHex(0xffff00);
                    } else {
                        color.setHSL((pipelineIndex / pipelineCount), 0.8, 0.5);
                    }
                    pipelineIndex++;

                    // 如果同一剖面内出现多个点，说明可能是分叉/误连。
                    // 这种情况下继续使用原来的逐段连接，避免平滑曲线错误穿插。
                    const segSet = new Set(centers.map(item => item.segIndex));
                    const cableAlignment = selectedCableAlignment && selectedCableAlignment.root === entry.root
                        ? selectedCableAlignment.alignment
                        : null;
                    const renderCenterlinePoints = cableAlignment
                        ? cableAlignment.alignedPoints
                        : centers.map(item => item.center);
                    const canUseSmoothTube = !!cableAlignment || (
                        PIPE_SMOOTH_CONFIG.enabled &&
                        centers.length >= PIPE_SMOOTH_CONFIG.minPointsForSmooth &&
                        segSet.size === centers.length
                    );

                    const autoPipeColorKey = entry.colorKey;
                    const autoLegacyColorKey = entry.legacyColorKey;
                    const autoDefaultColorHex = color.getHex();
                    const autoRenderColorHex = resolveGprPipeColorHexByKey(autoPipeColorKey, autoDefaultColorHex, [autoLegacyColorKey]);

                    if (canUseSmoothTube) {
                        const avgRadius = centers.reduce((sum, item) => sum + item.radius, 0) / centers.length;
                        const smoothTube = createSmoothTubeThroughPoints(
                            renderCenterlinePoints,
                            avgRadius,
                            autoRenderColorHex
                        );
                        if (smoothTube) {
                            registerGprPipePickMesh(
                                smoothTube,
                                renderCenterlinePoints,
                                {
                                    source: "auto",
                                    name: cableAlignment ? "黄色路径校正地下电缆" : "自动识别地下管道",
                                    colorHex: autoDefaultColorHex,
                                    colorKey: autoPipeColorKey,
                                    legacyColorKey: autoLegacyColorKey,
                                    alignedToSurfaceCable: !!cableAlignment,
                                    alignmentMedianOffsetM: cableAlignment?.medianDistance ?? null,
                                    alignmentMeanOffsetM: cableAlignment?.meanDistance ?? null
                                }
                            );
                            pipelineGroup.add(smoothTube);
                        }
                    }

                    // 兜底：点数不足 3 个、存在分叉、或平滑管道创建失败时，仍用原来的相邻圆柱连接。
                    if (pipelineGroup.children.length === 0) {
                        const fallbackEdges = cableAlignment
                            ? renderCenterlinePoints.slice(0, -1).map((point, index) => ({
                                a: point,
                                b: renderCenterlinePoints[index + 1],
                                radius: centers.reduce((sum, item) => sum + item.radius, 0) / centers.length
                            }))
                            : edgeItems.map(e => ({ a: e.a.center, b: e.b.center, radius: e.radius }));
                        fallbackEdges.forEach(e => {
                            const cylinder = createHollowCylinderBetween(e.a, e.b, e.radius);
                            if (!cylinder) return;
                            if (cylinder.material && cylinder.material.color) {
                                cylinder.material.color.setHex(autoRenderColorHex);
                            }
                            registerGprPipePickMesh(
                                cylinder,
                                [e.a, e.b],
                                {
                                    source: "auto",
                                    name: cableAlignment ? "黄色路径校正地下电缆" : "自动识别地下管道",
                                    colorHex: autoDefaultColorHex,
                                    colorKey: autoPipeColorKey,
                                    legacyColorKey: autoLegacyColorKey,
                                    alignedToSurfaceCable: !!cableAlignment,
                                    alignmentMedianOffsetM: cableAlignment?.medianDistance ?? null
                                }
                            );
                            pipelineGroup.add(cylinder);
                        });
                    }

                    connPipelinesGroup.add(pipelineGroup);
                });

                connPipelinesGroup.visible = showAnnotation;
                world.add(connPipelinesGroup);
                annotation3DGroups.push(connPipelinesGroup);
            }
        }

        // ================= 3.2b 倾斜/交叉管道彩色强制连接 =================
        // 同一标注批次（同色/同 groupId）的圆圈在相邻剖面之间强制连接，不再受 maxZDiff / maxYDiff 限制。
        if (segmentAnnoCenters.length >= 2) {
            const forcedPipeMap = new Map(); // pipeGroupId -> {colorHex, bySegment: Map(segIndex -> [centerObj])}

            segmentAnnoCenters.forEach((centers, segIndex) => {
                centers.forEach(centerObj => {
                    if (centerObj.linkMode !== "forced" || !centerObj.pipeGroupId) return;

                    let pipeInfo = forcedPipeMap.get(centerObj.pipeGroupId);
                    if (!pipeInfo) {
                        pipeInfo = {
                            pipeGroupId: centerObj.pipeGroupId,
                            colorHex: centerObj.pipeColorHex || 0x0099ff,
                            bySegment: new Map()
                        };
                        forcedPipeMap.set(centerObj.pipeGroupId, pipeInfo);
                    }

                    if (!pipeInfo.bySegment.has(segIndex)) {
                        pipeInfo.bySegment.set(segIndex, []);
                    }
                    pipeInfo.bySegment.get(segIndex).push(centerObj);
                });
            });

            const forcedConnGroup = new THREE.Group();
            let forcedEdgeCount = 0;

            forcedPipeMap.forEach(pipeInfo => {
                const onePipeGroup = new THREE.Group();
                const pipeColorHex = pipeInfo.colorHex || 0x0099ff;

                // 同一颜色/同一批次可能在每个剖面标 1 个点，也可能误标多个点。
                // 这里仍按每个剖面内的添加顺序 k 进行一一配对；
                // 对每一条 k 序列，跨多个相邻剖面的点组成一段平滑管道。
                let maxPointCountInOneSegment = 0;
                for (let i = 0; i < segmentAnnoCenters.length; i++) {
                    const list = (pipeInfo.bySegment.get(i) || []).slice().sort((a, b) => a.pipeOrder - b.pipeOrder);
                    maxPointCountInOneSegment = Math.max(maxPointCountInOneSegment, list.length);
                }

                function addForcedPipeRun(run) {
                    if (!Array.isArray(run) || run.length < 2) return;

                    const avgRadius = run.reduce((sum, item) => sum + item.radius, 0) / run.length;
                    const forcedPipeColorKey = makeGprPipeColorKeyFromCenters(run, {
                        source: "forced",
                        pipeGroupId: pipeInfo.pipeGroupId || ""
                    });
                    const forcedLegacyColorKey = makeGprPipeLegacyColorKeyFromCenters(run, {
                        source: "forced",
                        pipeGroupId: pipeInfo.pipeGroupId || ""
                    });
                    const forcedRenderColorHex = resolveGprPipeColorHexByKey(forcedPipeColorKey, pipeColorHex, [forcedLegacyColorKey]);
                    let mesh = null;

                    if (PIPE_SMOOTH_CONFIG.enabled && run.length >= PIPE_SMOOTH_CONFIG.minPointsForSmooth) {
                        mesh = createSmoothTubeThroughPoints(
                            run.map(item => item.center),
                            avgRadius,
                            forcedRenderColorHex
                        );
                    }

                    // 只有两个点时本身就是直线；或者平滑管道创建失败时，使用原来的直线圆柱兜底。
                    if (!mesh && run.length === 2) {
                        mesh = createHollowCylinderBetween(run[0].center, run[1].center, avgRadius);
                        if (mesh && mesh.material && mesh.material.color) {
                            mesh.material.color.setHex(forcedRenderColorHex);
                        }
                    }

                    if (!mesh) {
                        // 兜底：极端情况下逐段连接，避免什么都不显示。
                        for (let j = 0; j < run.length - 1; j++) {
                            const a = run[j];
                            const b = run[j + 1];
                            const cylinder = createHollowCylinderBetween(a.center, b.center, (a.radius + b.radius) / 2);
                            if (!cylinder) continue;
                            if (cylinder.material && cylinder.material.color) {
                                cylinder.material.color.setHex(forcedRenderColorHex);
                            }
                            registerGprPipePickMesh(
                                cylinder,
                                [a.center, b.center],
                                { source: "forced", name: "手动倾斜管道", colorHex: pipeColorHex, colorKey: forcedPipeColorKey, legacyColorKey: forcedLegacyColorKey, pipeGroupId: pipeInfo.pipeGroupId || "" }
                            );
                            onePipeGroup.add(cylinder);
                            forcedEdgeCount++;
                        }
                        return;
                    }

                    registerGprPipePickMesh(
                        mesh,
                        run.map(item => item.center),
                        { source: "forced", name: "手动倾斜管道", colorHex: pipeColorHex, colorKey: forcedPipeColorKey, legacyColorKey: forcedLegacyColorKey, pipeGroupId: pipeInfo.pipeGroupId || "" }
                    );
                    onePipeGroup.add(mesh);
                    forcedEdgeCount += Math.max(1, run.length - 1);
                }

                for (let k = 0; k < maxPointCountInOneSegment; k++) {
                    let currentRun = [];

                    for (let i = 0; i < segmentAnnoCenters.length; i++) {
                        const list = (pipeInfo.bySegment.get(i) || []).slice().sort((a, b) => a.pipeOrder - b.pipeOrder);
                        const item = list[k];

                        if (item) {
                            currentRun.push(item);
                        } else {
                            // 中间缺失某个剖面的同序号点时，断开成两段，避免跨空段硬连。
                            addForcedPipeRun(currentRun);
                            currentRun = [];
                        }
                    }

                    addForcedPipeRun(currentRun);
                }

                if (onePipeGroup.children.length > 0) {
                    forcedConnGroup.add(onePipeGroup);
                }
            });

            if (forcedEdgeCount > 0) {
                forcedConnGroup.visible = showAnnotation;
                world.add(forcedConnGroup);
                annotation3DGroups.push(forcedConnGroup);
            }
        }

        // 没有形成自动管道连通分量时也要结束“匹配中”状态，明确告知没有执行误吸附。
        if (!window.gprCablePathAlignmentStats) {
            selectBestGprPipeCableAlignment([]);
        }

        // 3.3 路径折线（放在底部）。RTK测线分别绘制，避免在相邻测线之间制造虚假的直线连接。
        const pathGroup = new THREE.Group();
        const pathMat = new THREE.LineBasicMaterial({ color: 0xffffff });

        if (rtkPositionedSegmentCount > 0) {
            renderedSegmentGeometries.forEach(geo => {
                if (!geo?._fromRtk || !Array.isArray(geo.traceX) || !Array.isArray(geo.traceY)) return;
                const points = [];
                for (let c = 0; c < Math.min(geo.traceX.length, geo.traceY.length); c++) {
                    const x = Number(geo.traceX[c]);
                    const z = Number(geo.traceY[c]);
                    if (!Number.isFinite(x) || !Number.isFinite(z)) continue;
                    const previous = points[points.length - 1];
                    if (!previous || Math.hypot(x - previous.x, z - previous.z) > 1e-6) {
                        points.push(new THREE.Vector3(x, gprGroundY - maxDepth, z));
                    }
                }
                if (points.length >= 2) {
                    pathGroup.add(new THREE.Line(
                        new THREE.BufferGeometry().setFromPoints(points),
                        pathMat
                    ));
                }
            });
        } else {
            const plannedPathPoints = usePlannedLayout
                ? getPlannedGprPathBottomPoints(maxDepth, gprGroundY)
                : null;
            const bottomPathPoints = [];

            if (plannedPathPoints && plannedPathPoints.length >= 2) {
                bottomPathPoints.push(...plannedPathPoints);
            } else if (Array.isArray(result.pathX) && Array.isArray(result.pathY) &&
                result.pathX.length === result.pathY.length) {
                for (let i = 0; i < result.pathX.length; i++) {
                    bottomPathPoints.push(new THREE.Vector3(
                        result.pathX[i],
                        gprGroundY - maxDepth,
                        result.pathY[i]
                    ));
                }
            }

            if (bottomPathPoints.length >= 2) {
                pathGroup.add(new THREE.Line(
                    new THREE.BufferGeometry().setFromPoints(bottomPathPoints),
                    pathMat
                ));
            }
        }

        if (pathGroup.children.length > 0) {
            world.add(pathGroup);
            window.gprZPathLine = pathGroup;
            pathGroup.visible = window.isGridVisible !== false;
        }

        // RTK三维拼接保持水平位置和长度不变，只放大深度的显示尺度，使剖面网格
        // 与原始二维图一样按矩阵单元比例显示。以groundY为不动的Depth=0基准。
        const depthDisplayScale = rtkPositionedSegmentCount > 0
            ? computeGprMatrixAspectDepthScale(segments, renderedSegmentGeometries)
            : 1;
        const displayedMaxDepth = maxDepth * depthDisplayScale;
        world.scale.y = depthDisplayScale;
        world.position.y = gprGroundY * (1 - depthDisplayScale);
        world.userData.depthDisplayScale = depthDisplayScale;
        window.gprDepthDisplayScale = depthDisplayScale;
        if (Number.isFinite(bounds.minY) && Number.isFinite(bounds.maxY)) {
            bounds.minY = gprGroundY - (gprGroundY - bounds.minY) * depthDisplayScale;
            bounds.maxY = gprGroundY - (gprGroundY - bounds.maxY) * depthDisplayScale;
        }
        console.log(
            `✅ 三维剖面按原始二维矩阵比例显示：深度显示倍率 ×${depthDisplayScale.toFixed(3)}，Depth刻度仍为真实米数。`
        );

        // 3.4 画三维坐标轴 / 地下环境长方体。
        // 只有多张剖面构成平行测线时，才让横向 Distance 轴与第一张剖面平行；
        // 规划路径或后端旧几何形成的 Z 字布局继续使用原有坐标框，避免错误旋转。
        const isParallelProfileLayout = maxDepth > 0 && !usePlannedLayout &&
            areGprProfilesParallel(renderedSegmentGeometries);
        const validRenderedGeometries = renderedSegmentGeometries.filter(Boolean);
        const firstRenderedGeometry = validRenderedGeometries[0] || null;
        let depthAxisCameraDirection = null;
        let mirrorHorizontalView = false;
        if (maxDepth > 0) {
            const usedFirstProfileBox = isParallelProfileLayout
                ? addAxesAlignedToFirstProfile(
                    scene,
                    renderedSegmentGeometries,
                    maxDepth,
                    gprGroundY,
                    depthDisplayScale
                )
                : false;
            if (usedFirstProfileBox && firstRenderedGeometry?._fromRtk) {
                const cameraDirection = window.gprAxesGroup?.userData?.depthAxisCameraDirection;
                if (cameraDirection && Number.isFinite(cameraDirection.x) && Number.isFinite(cameraDirection.z)) {
                    depthAxisCameraDirection = cameraDirection;
                    mirrorHorizontalView = window.gprAxesGroup.userData.mirrorHorizontalView === true;
                }
            }
            const usedOrientedBox = !usedFirstProfileBox && usePlannedLayout &&
                addOrientedAxesFromScanPlan(scene, bounds, maxDepth, gprGroundY);
            if (!usedFirstProfileBox && !usedOrientedBox) {
                addAxes(scene, bounds, displayedMaxDepth, gprGroundY, maxDepth);
            }
        }

        // 4. 相机围绕“数据中心”绕圈观察
        const centerX = (bounds.minX + bounds.maxX) / 2;
        const centerY = (bounds.minY + bounds.maxY) / 2;
        const centerZ = (bounds.minZ + bounds.maxZ) / 2;

        const spanX = bounds.maxX - bounds.minX;
        const spanZ = bounds.maxZ - bounds.minZ;
        const spanY = bounds.maxY - bounds.minY;
        const span = Math.max(spanX, spanZ, spanY) || 1.0;

        // ✅ 三维重建页相机操作统一改成“路径查找及故障定点”页面同款 orbitTarget 逻辑。
        // 关键点：平移时只移动水平面 X/Z 上的 orbitTarget，不让中心点沿 Y 高度方向漂移；
        // 左键旋转始终围绕最新 orbitTarget 旋转，因此中键拖到哪里，就围绕哪里旋转。
        const orbitTarget = new THREE.Vector3(centerX, centerY, centerZ);
        if (camera && camera.up) camera.up.set(0, 1, 0);

        // 相机控制范围。默认跟随 GPR 剖面尺寸；当加载当前点云地图后，
        // 由 gprSetViewToBox 更新为点云/电缆/破损点整体包围盒尺寸，避免拖拽和缩放被小剖面限制。
        let controlSpan = span;

        let radius = Math.max(span * 2.0, 3.0);
        // 平行RTK布局从首张剖面所在一侧观察。需要从局部反侧观察时，
        // 仅水平镜像画面，使首张采集道序仍显示为从左到右。
        const initialCameraTheta = depthAxisCameraDirection
            ? Math.atan2(depthAxisCameraDirection.z, depthAxisCameraDirection.x)
            : Math.PI / 4;
        let theta = initialCameraTheta;
        let phi = Math.PI / 3.5;

        if (depthAxisCameraDirection) {
            console.log(
                `✅ 平行RTK剖面已从首张所在侧观察：theta=${(initialCameraTheta * 180 / Math.PI).toFixed(2)}°，水平镜像视图=${mirrorHorizontalView}。`
            );
        }

        function updateCamera() {
            // 路径查找页是 Z 轴向上；三维重建页是 Y 轴向上。
            // 这里保持同样的 orbit 控制手感，只把高度轴从 Z 适配为 Y。
            const x = orbitTarget.x + radius * Math.sin(phi) * Math.cos(theta);
            const y = orbitTarget.y + radius * Math.cos(phi);
            const z = orbitTarget.z + radius * Math.sin(phi) * Math.sin(theta);
            camera.up.set(0, 1, 0);
            camera.position.set(x, y, z);
            camera.lookAt(orbitTarget);
        }

        // ✅ 供“加载当前点云地图 / 本地地图”按钮在导入后自动拉近视角
        window.gprSetViewToBox = function (box) {
            if (!box || box.isEmpty()) return;
            const size = new THREE.Vector3();
            box.getCenter(orbitTarget);
            box.getSize(size);
            const newSpan = Math.max(size.x, size.y, size.z, 1.0);
            controlSpan = newSpan;
            radius = Math.max(newSpan * 1.8, 3.0);
            theta = initialCameraTheta;
            phi = Math.PI / 3.5;
            updateCamera();
        };

        updateCamera();

        window.gprRtkProjectedToSlamForCurrentRender = rtkProjectedSegmentCount > 0;
        const locatedApexCount = segments.reduce(
            (sum, seg) => sum + (Array.isArray(seg.rawDetections)
                ? seg.rawDetections.filter(detection => detection.apexCol != null && detection.apexRow != null).length
                : 0),
            0
        );
        const rawProfileText = `三维曲面使用原始二维剖面，自动标注采用${locatedApexCount}个有效波形顶点；`;
        if (rtkPositionedSegmentCount > 0) {
            const coordinateText = rtkProjectedSegmentCount === rtkPositionedSegmentCount
                ? "已通过GPS/SLAM标定落到当前点云地图坐标"
                : "使用统一RTK局部米制坐标显示（未与点云地图标定坐标对齐）";
            const fallbackCount = segments.length - rtkPositionedSegmentCount;
            const fallbackText = fallbackCount > 0
                ? `；另有${fallbackCount}个剖面缺少有效同名CSV，已回退旧几何`
                : "";
            const fitRmseText = rtkFitRmseValues.length
                ? `，RTK直线拟合平均横向残差${(rtkFitRmseValues.reduce((sum, value) => sum + value, 0) / rtkFitRmseValues.length).toFixed(3)}m`
                : "";
            const aspectText = Math.abs(depthDisplayScale - 1) > 1e-3
                ? `；三维剖面按原始二维矩阵比例显示（深度显示×${depthDisplayScale.toFixed(2)}，刻度仍为真实米数）`
                : "";
            setStatus(
                `✅ ${rawProfileText}GPR剖面内部按DZT固定道间距拼接，RTK仅用于整剖面位置与方向校准：${rtkPositionedSegmentCount}/${segments.length}个剖面，${coordinateText}${fitRmseText}${fallbackText}${aspectText}。`,
                fallbackCount > 0 ? "warning" : "success"
            );
        } else if (usePlannedLayout) {
            reloadInspectionMapAndScanPlanAfterGprRender(true);
            const usedLegs = Math.min(segments.length, plannedLegCount);
            const alignmentStats = window.gprCablePathAlignmentStats;
            const alignmentText = alignmentStats?.applied
                ? ` 地下电缆已按黄色路径校正（原中位水平偏差 ${alignmentStats.medianOffsetM.toFixed(2)}m），埋深保持 GPR 测量值。`
                : " 未找到可安全对应的黄色路径/地下管道，未执行路径校正。";
            setStatus(`✅ ${rawProfileText}GPR剖面已按点云地图中规划的Z字扫描路径融合显示：已匹配${usedLegs}/${segments.length}个剖面。${alignmentText}`, "success");
        }

        // ========== 交互：左键旋转，中键/右键平移，滚轮缩放 ==========

        const dom = renderer.domElement;
        dom.dataset.gprMirroredView = mirrorHorizontalView ? "true" : "false";
        if (mirrorHorizontalView) {
            dom.style.transform = "scaleX(-1)";
        }
        if (mirrorHorizontalView || Math.abs(depthDisplayScale - 1) > 1e-3) {
            const displayModeNote = document.createElement("div");
            displayModeNote.className = "gpr-display-mode-note";
            const modeParts = [];
            if (Math.abs(depthDisplayScale - 1) > 1e-3) {
                modeParts.push(`原始剖面比例 · 深度显示×${depthDisplayScale.toFixed(2)}`);
            }
            if (mirrorHorizontalView) modeParts.push("水平镜像");
            modeParts.push("RTK坐标未变");
            displayModeNote.textContent = modeParts.join(" · ");
            container.appendChild(displayModeNote);
        }
        let isRotating = false;
        let isPanning = false;
        let prevPos = { x: 0, y: 0 };
        let clickStartX = 0;
        let clickStartY = 0;
        let hasDraggedForView = false;

        dom.addEventListener("mousedown", e => {
            e.preventDefault();

            if (e.button === 0) {
                isRotating = true;
                isPanning = false;
            } else if (e.button === 1 || e.button === 2) {
                isPanning = true;
                isRotating = false;
            } else {
                return;
            }

            prevPos = { x: e.clientX, y: e.clientY };
            clickStartX = e.clientX;
            clickStartY = e.clientY;
            hasDraggedForView = false;
        });

        window.addEventListener("mouseup", () => {
            isRotating = false;
            isPanning = false;
        });

        dom.addEventListener("contextmenu", e => e.preventDefault());

        dom.addEventListener("click", e => {
            if (hasDraggedForView) return;
            handleGprPipeClick(e, camera, dom);
        });

        dom.addEventListener("mousemove", e => {
            const dx = e.clientX - prevPos.x;
            const dy = e.clientY - prevPos.y;
            const horizontalDrag = mirrorHorizontalView ? -dx : dx;
            prevPos = { x: e.clientX, y: e.clientY };

            if (Math.hypot(e.clientX - clickStartX, e.clientY - clickStartY) > 4) {
                hasDraggedForView = true;
            }

            if (isRotating) {
                // 与路径查找页保持一致：左右拖动改变水平角，上下拖动改变俯仰角。
                theta += horizontalDrag * 0.01;
                phi -= dy * 0.01;
                phi = Math.max(0.1, Math.min(Math.PI / 2 - 0.1, phi));
                updateCamera();
            } else if (isPanning) {
                // 与路径查找页保持一致：按相机当前朝向在水平面上平移 orbitTarget。
                // 三维重建页水平面为 X/Z，因此取 forward.x 和 forward.z，Y 不参与平移。
                const speed = Math.max(radius * 0.002, 0.001);

                const forward = new THREE.Vector3();
                camera.getWorldDirection(forward);

                const right = new THREE.Vector3()
                    .crossVectors(forward, camera.up)
                    .normalize();

                const forwardXZ = new THREE.Vector3(forward.x, 0, forward.z);
                if (forwardXZ.lengthSq() > 1e-8) {
                    forwardXZ.normalize();
                    orbitTarget.addScaledVector(right, -horizontalDrag * speed);
                    orbitTarget.addScaledVector(forwardXZ, dy * speed);
                } else {
                    // 极端俯视角兜底：仍然只做水平面平移，避免 orbitTarget 高度漂移。
                    const fallbackForward = new THREE.Vector3(Math.cos(theta), 0, Math.sin(theta)).normalize();
                    orbitTarget.addScaledVector(right, -horizontalDrag * speed);
                    orbitTarget.addScaledVector(fallbackForward, dy * speed);
                }

                updateCamera();
            }

            updateGprPipeHoverCursor(e, camera, dom, isRotating || isPanning);
        });

        dom.addEventListener("mouseleave", () => resetGprPipeHoverCursor(dom));

        dom.addEventListener("wheel", e => {
            e.preventDefault();
            radius *= (e.deltaY > 0 ? 1.1 : 0.9);

            // 与独立点云地图视图保持一致：按当前点云/剖面整体尺寸自适应缩放范围。
            const minRadius = 0.02;
            const maxRadius = Math.max(controlSpan * 80.0, 1000.0);
            radius = Math.max(minRadius, Math.min(maxRadius, radius));
            updateCamera();
        });

        // ========== 响应窗口大小变化 (Resize) ==========
        function onWindowResize() {
            if (!container) return;

            const newWidth = container.clientWidth;
            const newHeight = container.clientHeight;

            camera.aspect = newWidth / newHeight;
            camera.updateProjectionMatrix();

            renderer.setSize(newWidth, newHeight);
        }

        window.addEventListener('resize', onWindowResize);

        const resizeObserver = new ResizeObserver(() => {
            onWindowResize();
        });
        resizeObserver.observe(container);

        // ========== 动画循环 ==========
        function animate() {
            // 👇 1. 修改：将动画循环 ID 挂载到 window 上，方便前面步骤 1 中进行 cancelAnimationFrame 销毁
            window.gprAnimId = requestAnimationFrame(animate);

            // 👇 2. 核心拦截：如果当前用户切换到了“机器人导航”或“履带车”等其他面板，直接退出，跳过极其耗费 GPU 的场景渲染
            if (window.currentActivePanel !== 'gpr') return;

            // ✅ 动态智能显隐（视角自适应）逻辑
            // 坐标框若已与第一张剖面对齐，则先把相机位置变换到坐标框局部坐标，
            // 再判断应显示哪一个侧面；未旋转的旧坐标框仍沿用世界坐标判断。
            if (window.boxFaces && window.boxCenter && camera) {
                let camX = camera.position.x;
                let camY = camera.position.y;
                let camZ = camera.position.z;
                let centerXForFaces = window.boxCenter.x;
                let centerYForFaces = window.boxCenter.y;
                let centerZForFaces = window.boxCenter.z;

                const axesGroup = window.gprAxesGroup;
                if (axesGroup?.userData?.faceVisibilityInLocalAxes && axesGroup.userData.boxCenterLocal) {
                    axesGroup.updateMatrixWorld(true);
                    const localCamera = axesGroup.worldToLocal(camera.position.clone());
                    const localCenter = axesGroup.userData.boxCenterLocal;
                    camX = localCamera.x;
                    camY = localCamera.y;
                    camZ = localCamera.z;
                    centerXForFaces = localCenter.x;
                    centerYForFaces = localCenter.y;
                    centerZForFaces = localCenter.z;
                }

                // X轴：如果你在右边看，就隐藏右侧网格，把左侧网格当背景
                window.boxFaces.right.visible = (camX < centerXForFaces);
                window.boxFaces.left.visible = (camX >= centerXForFaces);

                // Y轴：地下场景保持“开顶”显示，顶面网格永远不渲染，只保留底面作为地板。
                window.boxFaces.top.visible = false;
                window.boxFaces.bottom.visible = (camY >= centerYForFaces);

                // Z轴：如果你在前面看，就隐藏前面网格，把后面当背景
                window.boxFaces.front.visible = (camZ < centerZForFaces);
                window.boxFaces.back.visible = (camZ >= centerZForFaces);
            }

            updateGprAxisLabelSprites(camera, renderer);
            if (mirrorHorizontalView) keepGprSceneSpritesReadableInMirror(scene);

            renderer.render(scene, camera);

            if (typeof updatePopupPosition === 'function') {
                updatePopupPosition();
            }
            if (typeof updateGprPipeInfoPopupPosition === 'function') {
                updateGprPipeInfoPopupPosition();
            }
        }
        animate();
    }

    // 根据多个圆心生成一整条平滑管道。
    // points.length >= 3 时使用 CatmullRomCurve3 平滑；points.length == 2 时退化为直线连接。
    function createSmoothTubeThroughPoints(points, radius, colorHex = 0xffff00) {
        if (!Array.isArray(points) || points.length < 2) return null;

        const cleanPoints = [];
        points.forEach(pt => {
            if (!pt) return;
            const p = pt.clone ? pt.clone() : new THREE.Vector3(Number(pt.x), Number(pt.y), Number(pt.z));
            if (!Number.isFinite(p.x) || !Number.isFinite(p.y) || !Number.isFinite(p.z)) return;

            // 去掉连续重复点，避免 TubeGeometry 在零长度段上出现异常。
            const last = cleanPoints[cleanPoints.length - 1];
            if (!last || last.distanceTo(p) > 1e-6) {
                cleanPoints.push(p);
            }
        });

        if (cleanPoints.length < 2) return null;

        const safeRadius = Math.max(Number(radius) || 0.02, 0.005);

        if (cleanPoints.length < PIPE_SMOOTH_CONFIG.minPointsForSmooth || !PIPE_SMOOTH_CONFIG.enabled) {
            const cylinder = createHollowCylinderBetween(cleanPoints[0], cleanPoints[1], safeRadius);
            if (cylinder && cylinder.material && cylinder.material.color) {
                cylinder.material.color.setHex(colorHex);
            }
            return cylinder;
        }

        const curve = new THREE.CatmullRomCurve3(
            cleanPoints,
            false,
            PIPE_SMOOTH_CONFIG.curveType || "centripetal",
            PIPE_SMOOTH_CONFIG.tension ?? 0.35
        );

        const tubularSegments = Math.max(
            24,
            Math.ceil((cleanPoints.length - 1) * (PIPE_SMOOTH_CONFIG.tubularSegmentsPerSpan || 18))
        );

        const geometry = new THREE.TubeGeometry(
            curve,
            tubularSegments,
            safeRadius,
            PIPE_SMOOTH_CONFIG.radialSegments || 12,
            false
        );

        const mesh = new THREE.Mesh(
            geometry,
            new THREE.MeshBasicMaterial({
                color: colorHex,
                side: THREE.DoubleSide
            })
        );
        mesh.userData.gprCenterlinePoints = cleanPoints.map(p => p.clone());
        return mesh;
    }

    // 在两点之间生成一个空心圆柱体（沿两点连线方向）
    // p1, p2: THREE.Vector3
    // radius: 圆柱半径
    function createHollowCylinderBetween(p1, p2, radius) {
        const dir = new THREE.Vector3().subVectors(p2, p1);
        const length = dir.length();
        if (length === 0) return null;

        const mid = new THREE.Vector3().addVectors(p1, p2).multiplyScalar(0.5);

        // CylinderGeometry 默认沿 Y 轴（高度方向）拉伸
        const geometry = new THREE.CylinderGeometry(
            radius,      // top radius
            radius,      // bottom radius
            length,      // height
            12,          // radial segments
            1,           // height segments
            true         // openEnded = true -> 空心（没有上下两个盖子）
        );

        const material = new THREE.MeshBasicMaterial({
            color: 0xffff00, // ✅ 将初始底层颜色也改为黄色
            side: THREE.DoubleSide
        });

        const mesh = new THREE.Mesh(geometry, material);
        mesh.userData.gprCenterlinePoints = [p1.clone(), p2.clone()];

        // 1. 放到中点
        mesh.position.copy(mid);

        // 2. 旋转到对准 p1 -> p2 的方向
        const axis = new THREE.Vector3(0, 1, 0); // Cylinder 默认朝向
        const dirNorm = dir.clone().normalize();
        const quaternion = new THREE.Quaternion().setFromUnitVectors(axis, dirNorm);
        mesh.quaternion.copy(quaternion);

        return mesh;
    }

    // 为单个剖面构建一个三维网格平面，并根据几何参数放到正确位置
    function buildSegmentMesh(matrix, geometry, vMin, vMax, bounds, groundY = 0) {
        const rows = matrix.length;
        const cols = matrix[0].length;

        const uAxis = (geometry.uAxis && geometry.uAxis.length === cols)
            ? geometry.uAxis
            : Array.from({ length: cols }, (_, i) => i);
        const zAxis = (geometry.zAxis && geometry.zAxis.length === rows)
            ? geometry.zAxis
            : Array.from({ length: rows }, (_, i) => i);

        const phi = geometry.phiRad || 0.0;
        const startX = geometry.startX || 0.0;
        const startY = geometry.startY || 0.0;
        const traceX = geometry.traceX;
        const traceY = geometry.traceY;
        const useTracePositions = Array.isArray(traceX) && Array.isArray(traceY) &&
            traceX.length === cols && traceY.length === cols &&
            traceX.every(v => Number.isFinite(Number(v))) &&
            traceY.every(v => Number.isFinite(Number(v)));

        const cosPhi = Math.cos(phi);
        const sinPhi = Math.sin(phi);

        const vertexCount = rows * cols;
        const positions = new Float32Array(vertexCount * 3);
        const colors = new Float32Array(vertexCount * 3);

        let idxV = 0;
        let idxC = 0;
        for (let r = 0; r < rows; r++) {
            const depth = zAxis[r];      // >=0
            const worldY = groundY - depth; // 以点云地面 groundY 为 Depth=0，深度向下为负

            for (let c = 0; c < cols; c++) {
                const u = uAxis[c];

                // RTK剖面使用整剖面刚体拟合后的均匀道位置；否则回退到直线参数几何。
                const xHoriz = useTracePositions ? Number(traceX[c]) : startX + u * cosPhi;
                const yHoriz = useTracePositions ? Number(traceY[c]) : startY + u * sinPhi;

                const worldX = xHoriz;
                const worldZ = yHoriz;

                positions[idxV++] = worldX;
                positions[idxV++] = worldY;
                positions[idxV++] = worldZ;

                // 更新整体包围盒
                if (worldX < bounds.minX) bounds.minX = worldX;
                if (worldX > bounds.maxX) bounds.maxX = worldX;
                if (worldY < bounds.minY) bounds.minY = worldY;
                if (worldY > bounds.maxY) bounds.maxY = worldY;
                if (worldZ < bounds.minZ) bounds.minZ = worldZ;
                if (worldZ > bounds.maxZ) bounds.maxZ = worldZ;

                // 顶点灰度颜色（和 2D 剖面保持一致的风格）
                const v = matrix[r][c];

                // === 替换为这部分新代码 ===
                let norm = (v - vMin) / (vMax - vMin); // [0,1]
                norm = Math.max(0, Math.min(1, norm)); // 严格截断

                // 去掉 Math.pow(norm, 1.2)，使用纯线性映射
                const gray = norm;

                colors[idxC++] = gray;
                colors[idxC++] = gray;
                colors[idxC++] = gray;
            }
        }

        // 每个网格单元两个三角形
        const cellCount = (rows - 1) * (cols - 1);
        const indices = new Uint32Array(cellCount * 6);
        let idxI = 0;
        for (let r = 0; r < rows - 1; r++) {
            for (let c = 0; c < cols - 1; c++) {
                const a = r * cols + c;
                const b = r * cols + (c + 1);
                const c2 = (r + 1) * cols + c;
                const d = (r + 1) * cols + (c + 1);

                // 三角形1: a, c2, b
                indices[idxI++] = a;
                indices[idxI++] = c2;
                indices[idxI++] = b;

                // 三角形2: b, c2, d
                indices[idxI++] = b;
                indices[idxI++] = c2;
                indices[idxI++] = d;
            }
        }

        const geom = new THREE.BufferGeometry();
        geom.setAttribute("position", new THREE.BufferAttribute(positions, 3));
        geom.setAttribute("color", new THREE.BufferAttribute(colors, 3));
        geom.setIndex(new THREE.BufferAttribute(indices, 1));

        // MeshBasicMaterial，不受光照影响，正反两面一样亮
        const material = new THREE.MeshBasicMaterial({
            vertexColors: true,
            side: THREE.DoubleSide,
            fog: false,
            toneMapped: false,
            // 将剖面自身的深度值轻微后移，避免与贴在同一平面上的识别框发生Z-fighting；
            // 不关闭深度测试，因此更靠近相机的其他剖面仍会正常遮挡这些标注。
            polygonOffset: true,
            polygonOffsetFactor: 1,
            polygonOffsetUnits: 1
        });

        const mesh = new THREE.Mesh(geom, material);
        console.log("GPR mesh material type =", mesh.material.type);
        return mesh;
    }

    // 在按规划扫描路径融合 GPR 剖面时，地下环境长方体必须跟随扫描矩形的真实方向绘制。
    // 原来的 addAxes() 使用 bounds.minX/maxX/minZ/maxZ 生成全局坐标轴对齐包围盒；
    // 当“加载当前点云地图”后规划的扫描区域存在旋转角时，长方体会看起来相对点云地图斜放。
    // 这里使用 currentGprScanPlan.preview.rectangleSlam 的四个角点直接生成“有方向”的长方体。
    function addOrientedAxesFromScanPlan(scene, bounds, maxDepth, groundY = 0) {
        if (!scene || !isFinite(maxDepth) || maxDepth <= 0) return false;
        const topY = Number.isFinite(Number(groundY)) ? Number(groundY) : 0;
        const bottomY = topY - maxDepth;

        const rectSlam = window.currentGprScanPlan?.preview?.rectangleSlam;
        if (!Array.isArray(rectSlam) || rectSlam.length < 4) return false;

        const topCorners = rectSlam.slice(0, 4).map(p => {
            const v = slamPointToGprVector(p);
            return new THREE.Vector3(v.x, topY, v.z);
        });

        const valid = topCorners.every(v => Number.isFinite(v.x) && Number.isFinite(v.z));
        if (!valid) return false;

        const bottomCorners = topCorners.map(v => new THREE.Vector3(v.x, bottomY, v.z));
        const center = new THREE.Vector3();
        topCorners.forEach(v => center.add(v));
        center.multiplyScalar(0.25);
        center.y = topY - maxDepth / 2;

        const lengthA = topCorners[0].distanceTo(topCorners[1]);
        const lengthB = topCorners[1].distanceTo(topCorners[2]);
        const xLen = Math.max(lengthA, 0.01);
        const zLen = Math.max(lengthB, 0.01);
        const yLen = maxDepth;
        const nTicks = 5;

        const axisGroup = new THREE.Group();
        axisGroup.name = "按扫描矩形方向生成的地下环境长方体";
        scene.add(axisGroup);

        window.gprAxesGroup = axisGroup;
        if (window.isGridVisible === false) {
            axisGroup.visible = false;
        }

        window.boxFaces = {
            bottom: new THREE.Group(),
            top: new THREE.Group(),
            back: new THREE.Group(),
            front: new THREE.Group(),
            left: new THREE.Group(),
            right: new THREE.Group()
        };
        Object.values(window.boxFaces).forEach(g => axisGroup.add(g));
        window.boxFaces.top.visible = false;
        window.boxCenter = center.clone();

        function axisLine(start, end, colorHex = 0xffffff) {
            const geom = new THREE.BufferGeometry().setFromPoints([start, end]);
            const mat = new THREE.LineBasicMaterial({ color: colorHex });
            return new THREE.Line(geom, mat);
        }

        function lerpPoint(a, b, t) {
            return new THREE.Vector3().lerpVectors(a, b, t);
        }

        function addLineTo(group, a, b, colorHex = 0xffffff) {
            group.add(axisLine(a, b, colorHex));
        }

        // 白色外框：顶面只保留边框，不创建顶面纹理，实现“开顶”。
        for (let i = 0; i < 4; i++) {
            const j = (i + 1) % 4;
            addLineTo(axisGroup, topCorners[i], topCorners[j], 0xffffff);
            addLineTo(axisGroup, bottomCorners[i], bottomCorners[j], 0xffffff);
            addLineTo(axisGroup, topCorners[i], bottomCorners[i], 0xffffff);
        }

        // 底面青色网格。
        for (let i = 1; i < nTicks; i++) {
            const t = i / nTicks;
            addLineTo(window.boxFaces.bottom, lerpPoint(bottomCorners[0], bottomCorners[1], t), lerpPoint(bottomCorners[3], bottomCorners[2], t), 0x00ffff);
            addLineTo(window.boxFaces.bottom, lerpPoint(bottomCorners[0], bottomCorners[3], t), lerpPoint(bottomCorners[1], bottomCorners[2], t), 0x00ffff);
        }

        // 两个长侧面黄色网格：0-1 和 3-2。
        for (let i = 1; i < nTicks; i++) {
            const t = i / nTicks;
            addLineTo(window.boxFaces.left, lerpPoint(topCorners[0], topCorners[1], t), lerpPoint(bottomCorners[0], bottomCorners[1], t), 0xffff00);
            addLineTo(window.boxFaces.right, lerpPoint(topCorners[3], topCorners[2], t), lerpPoint(bottomCorners[3], bottomCorners[2], t), 0xffff00);

            addLineTo(window.boxFaces.left, lerpPoint(topCorners[0], bottomCorners[0], t), lerpPoint(topCorners[1], bottomCorners[1], t), 0xffff00);
            addLineTo(window.boxFaces.right, lerpPoint(topCorners[3], bottomCorners[3], t), lerpPoint(topCorners[2], bottomCorners[2], t), 0xffff00);
        }

        // 两个短侧面红色网格：1-2 和 0-3。
        for (let i = 1; i < nTicks; i++) {
            const t = i / nTicks;
            addLineTo(window.boxFaces.front, lerpPoint(topCorners[1], topCorners[2], t), lerpPoint(bottomCorners[1], bottomCorners[2], t), 0xff0000);
            addLineTo(window.boxFaces.back, lerpPoint(topCorners[0], topCorners[3], t), lerpPoint(bottomCorners[0], bottomCorners[3], t), 0xff0000);

            addLineTo(window.boxFaces.front, lerpPoint(topCorners[1], bottomCorners[1], t), lerpPoint(topCorners[2], bottomCorners[2], t), 0xff0000);
            addLineTo(window.boxFaces.back, lerpPoint(topCorners[0], bottomCorners[0], t), lerpPoint(topCorners[3], bottomCorners[3], t), 0xff0000);
        }

        // 土壤纹理面：使用自定义四边形，保证面跟随扫描矩形方向，而不是全局 X/Z 方向。
        // 注意：土壤贴图面直接挂到 axisGroup，不挂到 window.boxFaces.xxx；
        // 这样保留原来的“只隐藏侧面网格线”的视角隐藏逻辑，不会把土壤贴图面一起隐藏。
        const textureLoader = new THREE.TextureLoader();
        const soilTexture = textureLoader.load('/img/soil_diffuse2.jpg');
        soilTexture.wrapS = THREE.RepeatWrapping;
        soilTexture.wrapT = THREE.RepeatWrapping;

        const soilMaterial = new THREE.MeshBasicMaterial({
            color: 0xffffff,
            map: soilTexture,
            transparent: true,
            opacity: 0.65,
            side: THREE.DoubleSide,
            depthWrite: false
        });

        function createQuadFace(points, repeatU, repeatV) {
            const geom = new THREE.BufferGeometry();
            const verts = new Float32Array([
                points[0].x, points[0].y, points[0].z,
                points[1].x, points[1].y, points[1].z,
                points[2].x, points[2].y, points[2].z,
                points[0].x, points[0].y, points[0].z,
                points[2].x, points[2].y, points[2].z,
                points[3].x, points[3].y, points[3].z
            ]);
            const uvs = new Float32Array([
                0, 0,
                repeatU, 0,
                repeatU, repeatV,
                0, 0,
                repeatU, repeatV,
                0, repeatV
            ]);
            geom.setAttribute('position', new THREE.BufferAttribute(verts, 3));
            geom.setAttribute('uv', new THREE.BufferAttribute(uvs, 2));
            geom.computeVertexNormals();
            return geom;
        }

        function addQuadFace(name, points, repeatU, repeatV) {
            const mesh = new THREE.Mesh(createQuadFace(points, repeatU, repeatV), soilMaterial);
            mesh.name = name;
            axisGroup.add(mesh);
            return mesh;
        }

        const repeatPerMeter = 0.5;
        addQuadFace('bottom', [bottomCorners[0], bottomCorners[1], bottomCorners[2], bottomCorners[3]], xLen * repeatPerMeter, zLen * repeatPerMeter);
        addQuadFace('left', [topCorners[0], topCorners[1], bottomCorners[1], bottomCorners[0]], xLen * repeatPerMeter, yLen * repeatPerMeter);
        addQuadFace('right', [topCorners[3], topCorners[2], bottomCorners[2], bottomCorners[3]], xLen * repeatPerMeter, yLen * repeatPerMeter);
        addQuadFace('front', [topCorners[1], topCorners[2], bottomCorners[2], bottomCorners[1]], zLen * repeatPerMeter, yLen * repeatPerMeter);
        addQuadFace('back', [topCorners[0], topCorners[3], bottomCorners[3], bottomCorners[0]], zLen * repeatPerMeter, yLen * repeatPerMeter);

        // ================= 3. 按单独加载 GPR 数据时的局部坐标效果绘制刻度 =================
        // 说明：有点云地图并使用规划路径融合时，长方体会按扫描矩形方向旋转。
        // 坐标轴不再显示全局 bounds 坐标值，改为和“单独加载 GPR 数据”时一致：
        // 1) Distance(m) 从 0 开始，沿长方体长边显示 0 ~ 扫描长度；
        // 2) 横向刻度从 0 开始，沿长方体短边显示 0 ~ 扫描宽度；
        // 3) Depth(m) 从顶面 0 开始向下显示 0 ~ maxDepth；
        // 4) 坐标轴基准点取“底面短边上与 Z 字形路径起点竖直对齐的点”。
        const axisNTicks = 5;
        const downDir = new THREE.Vector3(0, -1, 0);

        const zigzagSlam = window.currentGprScanPlan?.preview?.zigzagSlam;
        let zigzagStartTop = topCorners[0].clone();
        if (Array.isArray(zigzagSlam) && zigzagSlam.length > 0) {
            const startVec = slamPointToGprVector(zigzagSlam[0]);
            if (Number.isFinite(startVec.x) && Number.isFinite(startVec.z)) {
                zigzagStartTop = new THREE.Vector3(startVec.x, topY, startVec.z);
            }
        }

        let originIndex = 0;
        let bestStartDist = Infinity;
        topCorners.forEach((corner, idx) => {
            const dist = corner.distanceTo(zigzagStartTop);
            if (dist < bestStartDist) {
                bestStartDist = dist;
                originIndex = idx;
            }
        });

        const nextIndex = idx => (idx + 1) % 4;
        const prevIndex = idx => (idx + 3) % 4;
        const originFrontBottomLeft = bottomCorners[originIndex].clone();
        const originTopLeft = topCorners[originIndex].clone();
        const originNeighbors = [prevIndex(originIndex), nextIndex(originIndex)];

        // 从起点相邻的两条边里选择更长的一条作为 Distance(m) 方向。
        const n0 = originNeighbors[0];
        const n1 = originNeighbors[1];
        const d0 = bottomCorners[n0].distanceTo(originFrontBottomLeft);
        const d1 = bottomCorners[n1].distanceTo(originFrontBottomLeft);
        const xEndIndex = d0 >= d1 ? n0 : n1;
        const zEndFromOriginIndex = d0 >= d1 ? n1 : n0;

        const xEndBottom = bottomCorners[xEndIndex].clone();
        const zEndFromOriginBottom = bottomCorners[zEndFromOriginIndex].clone();
        const xDir = new THREE.Vector3().subVectors(xEndBottom, originFrontBottomLeft).normalize();
        const zDir = new THREE.Vector3().subVectors(zEndFromOriginBottom, originFrontBottomLeft).normalize();
        const xAxisLen = Math.max(originFrontBottomLeft.distanceTo(xEndBottom), 0.01);
        const zAxisLen = Math.max(originFrontBottomLeft.distanceTo(zEndFromOriginBottom), 0.01);

        const axisSpan = Math.max(xAxisLen, zAxisLen, yLen) || 1.0;
        const axisTickSize = axisSpan * 0.022;
        const axisLabelSize = axisSpan * 0.06;

        // X轴刻度 (Distance)：从与 Z 字形路径起点竖直对齐的底面点开始，按局部距离 0 ~ 扫描长度显示。
        for (let i = 0; i <= axisNTicks; i++) {
            const ratio = i / axisNTicks;
            const pos = originFrontBottomLeft.clone().add(xDir.clone().multiplyScalar(xAxisLen * ratio));
            axisGroup.add(axisLine(pos, pos.clone().add(downDir.clone().multiplyScalar(axisTickSize))));

            const val = xAxisLen * ratio;
            const label = makeTextSprite(val.toFixed(2), "#ffffff");
            label.scale.set(axisLabelSize, axisLabelSize, 1);
            label.position.copy(pos).add(downDir.clone().multiplyScalar(axisTickSize * 1.35));
            if (i === axisNTicks) {
                label.position.add(xDir.clone().multiplyScalar(-axisTickSize * 1.3));
            }
            axisGroup.add(label);
        }

        const xLabel = makeTextSprite("Distance(m)", "#ffffff");
        xLabel.scale.set(axisLabelSize * 1.5, axisLabelSize * 1.5, 1);
        xLabel.position.copy(originFrontBottomLeft)
            .add(xDir.clone().multiplyScalar(xAxisLen / 2))
            .add(downDir.clone().multiplyScalar(axisTickSize * 3.5));
        axisGroup.add(xLabel);

        // Y轴刻度 (Depth)：顶面为 0，向下到 maxDepth。
        for (let i = 0; i <= axisNTicks; i++) {
            const d = yLen * i / axisNTicks;
            const pos = originTopLeft.clone().add(downDir.clone().multiplyScalar(d));
            axisGroup.add(axisLine(pos, pos.clone().add(xDir.clone().multiplyScalar(-axisTickSize))));

            const label = makeTextSprite(d.toFixed(1), "#ffffff");
            label.scale.set(axisLabelSize, axisLabelSize, 1);
            label.position.copy(pos).add(xDir.clone().multiplyScalar(-axisTickSize * 1.35));
            axisGroup.add(label);
        }

        const yLabel = makeTextSprite("Depth(m)", "#ffffff");
        yLabel.scale.set(axisLabelSize * 1.5, axisLabelSize * 1.5, 1);
        yLabel.position.copy(originTopLeft)
            .add(xDir.clone().multiplyScalar(-axisTickSize * 4))
            .add(downDir.clone().multiplyScalar(yLen / 2));
        axisGroup.add(yLabel);

        // 横向短边刻度：保持原始视觉习惯，放在 Distance 轴末端的短边上；数值按局部宽度 0 ~ 扫描宽度显示，不再显示负值。
        const zStartBottom = xEndBottom.clone();
        const zEndBottom = xEndBottom.clone().add(zDir.clone().multiplyScalar(zAxisLen));
        for (let i = 0; i <= axisNTicks; i++) {
            const ratio = i / axisNTicks;
            const pos = zStartBottom.clone().add(zDir.clone().multiplyScalar(zAxisLen * ratio));
            axisGroup.add(axisLine(pos, pos.clone().add(downDir.clone().multiplyScalar(axisTickSize))));

            const val = zAxisLen * ratio;
            const label = makeTextSprite(val.toFixed(2), "#ffffff");
            label.scale.set(axisLabelSize, axisLabelSize, 1);
            label.position.copy(pos)
                .add(xDir.clone().multiplyScalar(axisTickSize * 0.65))
                .add(downDir.clone().multiplyScalar(axisTickSize * 1.25));
            if (i === 0) {
                label.position
                    .add(xDir.clone().multiplyScalar(axisTickSize * 0.4))
                    .add(zDir.clone().multiplyScalar(axisTickSize * 0.5));
            }
            axisGroup.add(label);
        }

        console.log('✅ 已按单独加载GPR数据的局部坐标效果生成地下环境坐标轴：Distance/横向刻度从0开始，Depth从顶面0开始。');
        return true;
    }

    function getGprProfileHorizontalDirection(geometry) {
        if (!geometry) return null;

        const traceX = geometry.traceX;
        const traceY = geometry.traceY;
        let dx = 0;
        let dz = 0;
        if (
            Array.isArray(traceX) && Array.isArray(traceY) &&
            traceX.length >= 2 && traceY.length === traceX.length
        ) {
            dx = Number(traceX[traceX.length - 1]) - Number(traceX[0]);
            dz = Number(traceY[traceY.length - 1]) - Number(traceY[0]);
        }

        if (!Number.isFinite(dx) || !Number.isFinite(dz) || Math.hypot(dx, dz) < 1e-8) {
            const phi = Number(geometry.phiRad);
            if (!Number.isFinite(phi)) return null;
            dx = Math.cos(phi);
            dz = Math.sin(phi);
        }

        const length = Math.hypot(dx, dz);
        if (!(length > 1e-8)) return null;
        return { x: dx / length, z: dz / length };
    }

    function getGprProfileHorizontalCenter(geometry) {
        if (!geometry) return null;

        const traceX = geometry.traceX;
        const traceY = geometry.traceY;
        if (
            Array.isArray(traceX) && Array.isArray(traceY) &&
            traceX.length >= 2 && traceY.length === traceX.length
        ) {
            const firstX = Number(traceX[0]);
            const firstZ = Number(traceY[0]);
            const lastX = Number(traceX[traceX.length - 1]);
            const lastZ = Number(traceY[traceY.length - 1]);
            if ([firstX, firstZ, lastX, lastZ].every(Number.isFinite)) {
                return { x: (firstX + lastX) / 2, z: (firstZ + lastZ) / 2 };
            }
        }

        const startX = Number(geometry.startX);
        const startZ = Number(geometry.startY);
        const length = Number(geometry.length);
        const direction = getGprProfileHorizontalDirection(geometry);
        if (![startX, startZ, length].every(Number.isFinite) || !direction) return null;
        return {
            x: startX + direction.x * length / 2,
            z: startZ + direction.z * length / 2
        };
    }

    /**
     * 判断当前是否为“多张平行剖面”布局。
     * 方向比较忽略采集正反向，但只要出现明显转角（Z字折线）就不视为平行布局。
     */
    function areGprProfilesParallel(geometries, toleranceDeg = 8) {
        const validGeometries = (Array.isArray(geometries) ? geometries : []).filter(Boolean);
        if (validGeometries.length < 2) return false;

        // 上位机规划路径明确属于Z字扫描，不参与第一剖面坐标轴对齐。
        if (validGeometries.some(geometry => geometry._fromPlannedScanPath)) return false;

        const directions = validGeometries.map(getGprProfileHorizontalDirection);
        if (directions.some(direction => !direction)) return false;

        const reference = directions[0];
        const minimumParallelDot = Math.cos(Math.max(0, toleranceDeg) * Math.PI / 180);
        const allParallel = directions.slice(1).every(direction => {
            const absoluteDot = Math.abs(reference.x * direction.x + reference.z * direction.z);
            return absoluteDot >= minimumParallelDot;
        });

        if (!allParallel) {
            console.log("ℹ️ 当前剖面包含明显转角，判定为Z字/非平行拼接，坐标轴不跟随第一张剖面旋转。");
        }
        return allParallel;
    }

    /**
     * 以第一张剖面的道方向作为局部横轴，将全部剖面投影到该坐标系后生成包围盒。
     * 调用方必须先确认是多张平行剖面；本函数只调整坐标框朝向，不改动任何剖面或RTK位置。
     */
    function addAxesAlignedToFirstProfile(scene, geometries, maxDepth, groundY = 0, depthDisplayScale = 1) {
        const validGeometries = (Array.isArray(geometries) ? geometries : []).filter(Boolean);
        if (!scene || validGeometries.length < 2 || !(maxDepth > 0)) return false;

        const reference = validGeometries[0];
        const referenceDirection = getGprProfileHorizontalDirection(reference);
        if (!referenceDirection) return false;
        // 局部 +X 始终沿首张剖面的 RTK 道序，不移动或反转任何雷达数据。
        const xDirX = referenceDirection.x;
        const xDirZ = referenceDirection.z;
        const firstCenter = reference._fromRtk ? getGprProfileHorizontalCenter(reference) : null;
        const lastCenter = reference._fromRtk
            ? getGprProfileHorizontalCenter(validGeometries[validGeometries.length - 1])
            : null;
        const firstToLastAcross = firstCenter && lastCenter
            ? (lastCenter.x - firstCenter.x) * -xDirZ + (lastCenter.z - firstCenter.z) * xDirX
            : 0;
        // 若后续测线位于默认 +Z 侧，改从 -Z 侧观察，确保首张在前。
        // 此时屏幕横向需要镜像，才能同时让采集方向从左到右；镜像仅作用于视图。
        const mirrorHorizontalView = firstToLastAcross > 0;
        const zDirX = mirrorHorizontalView ? xDirZ : -xDirZ;
        const zDirZ = mirrorHorizontalView ? -xDirX : xDirX;
        const phi = Math.atan2(xDirZ, xDirX);

        const horizontalPoints = [];
        validGeometries.forEach(geometry => {
            const traceX = geometry.traceX;
            const traceY = geometry.traceY;
            if (
                Array.isArray(traceX) && Array.isArray(traceY) &&
                traceX.length >= 2 && traceY.length === traceX.length
            ) {
                for (let i = 0; i < traceX.length; i++) {
                    const x = Number(traceX[i]);
                    const z = Number(traceY[i]);
                    if (Number.isFinite(x) && Number.isFinite(z)) horizontalPoints.push({ x, z });
                }
                return;
            }

            const startX = Number(geometry.startX);
            const startZ = Number(geometry.startY);
            const length = Number(geometry.length);
            const geometryPhi = Number(geometry.phiRad);
            if (![startX, startZ, length, geometryPhi].every(Number.isFinite)) return;
            horizontalPoints.push({ x: startX, z: startZ });
            horizontalPoints.push({
                x: startX + length * Math.cos(geometryPhi),
                z: startZ + length * Math.sin(geometryPhi)
            });
        });
        if (horizontalPoints.length < 2) return false;

        let minAlong = Infinity;
        let maxAlong = -Infinity;
        let minAcross = Infinity;
        let maxAcross = -Infinity;
        horizontalPoints.forEach(point => {
            const along = point.x * xDirX + point.z * xDirZ;
            const across = point.x * zDirX + point.z * zDirZ;
            minAlong = Math.min(minAlong, along);
            maxAlong = Math.max(maxAlong, along);
            minAcross = Math.min(minAcross, across);
            maxAcross = Math.max(maxAcross, across);
        });
        if (![minAlong, maxAlong, minAcross, maxAcross].every(Number.isFinite)) return false;

        const horizontalSpan = Math.max(maxAlong - minAlong, maxAcross - minAcross, 0.01);
        const margin = Math.max(0.02, horizontalSpan * 0.015);
        const safeDepthDisplayScale = Number.isFinite(Number(depthDisplayScale)) && Number(depthDisplayScale) > 0
            ? Number(depthDisplayScale)
            : 1;
        const displayedMaxDepth = maxDepth * safeDepthDisplayScale;
        const localBounds = {
            minX: minAlong - margin,
            maxX: maxAlong + margin,
            minY: groundY - displayedMaxDepth,
            maxY: groundY,
            minZ: minAcross - margin,
            maxZ: maxAcross + margin
        };

        addAxes(scene, localBounds, displayedMaxDepth, groundY, maxDepth);
        const axisGroup = window.gprAxesGroup;
        if (!axisGroup || !window.boxCenter) return false;

        const localCenter = window.boxCenter.clone();
        axisGroup.rotation.y = -phi;
        // 坐标框自身允许镜像横向法线，使 Depth 刻度仍在靠近相机的一侧；
        // 所有 GPR 网格及 RTK 世界坐标不属于此组，保持原样。
        axisGroup.scale.z = mirrorHorizontalView ? -1 : 1;
        axisGroup.name = "横轴与第一张GPR剖面平行的地下环境坐标框";
        axisGroup.userData.faceVisibilityInLocalAxes = true;
        axisGroup.userData.boxCenterLocal = localCenter;
        axisGroup.userData.referenceProfilePhiRad = phi;
        axisGroup.userData.depthAxisCameraDirection = { x: zDirX, z: zDirZ };
        axisGroup.userData.mirrorHorizontalView = mirrorHorizontalView;
        axisGroup.updateMatrixWorld(true);
        window.boxCenter = localCenter.clone().applyMatrix4(axisGroup.matrixWorld);
        window.gprAxesReferencePhiRad = phi;

        console.log(`✅ 三维横轴正向已与第一张剖面的RTK采集方向一致：方位角 ${(phi * 180 / Math.PI).toFixed(2)}°`);
        return true;
    }

    // 在场景中添加一个三维包围盒和彩色网格（适配深色主题）
    function addAxes(scene, bounds, maxDepth, groundY = 0, depthLabelMax = maxDepth) {
        if (!isFinite(maxDepth) || maxDepth <= 0) return;
        const topY = Number.isFinite(Number(groundY)) ? Number(groundY) : 0;
        const bottomY = topY - maxDepth;
        const physicalDepthMax = Number.isFinite(Number(depthLabelMax)) && Number(depthLabelMax) > 0
            ? Number(depthLabelMax)
            : maxDepth;

        const axisGroup = new THREE.Group();
        scene.add(axisGroup);

        // 👇========= 🌟 新增代码：将坐标系组暴露给全局，并记住用户的显隐状态 =========👇
        window.gprAxesGroup = axisGroup;
        if (window.isGridVisible === false) {
            axisGroup.visible = false;
        }
        // 👆========================================================================👆

        const xLen = Math.max(bounds.maxX - bounds.minX, 0.01);
        const zLen = Math.max(bounds.maxZ - bounds.minZ, 0.01);
        const yLen = maxDepth; // 深度，向下为正

        const span = Math.max(xLen, zLen, yLen) || 1.0;
        // ✅ 优化：将指向刻度数值的短线缩短一些，避免视觉上过长、过于突兀
        const tickSize = span * 0.022;
        const labelSize = span * 0.06;
        const nTicks = 5;

        // 辅助画线函数
        function axisLine(start, end, colorHex = 0xffffff) {
            const geom = new THREE.BufferGeometry().setFromPoints([start, end]);
            const mat = new THREE.LineBasicMaterial({ color: colorHex });
            return new THREE.Line(geom, mat);
        }

        // ================= 1. 画出外围的整体长方体线框 (白色) =================
        const cx = (bounds.maxX + bounds.minX) / 2;
        const cy = topY - maxDepth / 2;
        const cz = (bounds.maxZ + bounds.minZ) / 2;
        const boxGeom = new THREE.BoxGeometry(xLen, yLen, zLen);
        const edges = new THREE.EdgesGeometry(boxGeom);
        const boxMat = new THREE.LineBasicMaterial({ color: 0xffffff });
        const boxMesh = new THREE.LineSegments(edges, boxMat);
        boxMesh.position.set(cx, cy, cz);
        axisGroup.add(boxMesh);

        // ================= 2. 绘制 5 个面彩色网格（顶面开口） =================

        // 创建全局对象保存各面的网格组，方便外部调用
        window.boxFaces = {
            bottom: new THREE.Group(),
            top: new THREE.Group(),
            back: new THREE.Group(),
            front: new THREE.Group(),
            left: new THREE.Group(),
            right: new THREE.Group()
        };

        // 将它们统一添加到主坐标轴组中
        Object.values(window.boxFaces).forEach(g => axisGroup.add(g));
        window.boxFaces.top.visible = false;

        // 记录长方体的几何中心，用于后续判断相机视角
        window.boxCenter = new THREE.Vector3(
            (bounds.minX + bounds.maxX) / 2,
            topY - maxDepth / 2,
            (bounds.minZ + bounds.maxZ) / 2
        );

        // 底面 (y = -maxDepth) & 顶面 (y = 0) 网格 (青色 Cyan)
        for (let i = 1; i < nTicks; i++) {
            let tX = (xLen / nTicks) * i;
            window.boxFaces.bottom.add(axisLine(new THREE.Vector3(bounds.minX + tX, bottomY, bounds.minZ), new THREE.Vector3(bounds.minX + tX, bottomY, bounds.maxZ), 0x00ffff));
        }
        for (let i = 1; i < nTicks; i++) {
            let tZ = (zLen / nTicks) * i;
            window.boxFaces.bottom.add(axisLine(new THREE.Vector3(bounds.minX, bottomY, bounds.minZ + tZ), new THREE.Vector3(bounds.maxX, bottomY, bounds.minZ + tZ), 0x00ffff));
        }

        // 后面 (z = bounds.minZ) & 前面 (z = bounds.maxZ) 网格 (红色 Red)
        for (let i = 1; i < nTicks; i++) {
            let tY = (yLen / nTicks) * i;
            window.boxFaces.back.add(axisLine(new THREE.Vector3(bounds.minX, topY - tY, bounds.minZ), new THREE.Vector3(bounds.maxX, topY - tY, bounds.minZ), 0xff0000));
            window.boxFaces.front.add(axisLine(new THREE.Vector3(bounds.minX, topY - tY, bounds.maxZ), new THREE.Vector3(bounds.maxX, topY - tY, bounds.maxZ), 0xff0000));
        }
        for (let i = 1; i < nTicks; i++) {
            let tX = (xLen / nTicks) * i;
            window.boxFaces.back.add(axisLine(new THREE.Vector3(bounds.minX + tX, topY, bounds.minZ), new THREE.Vector3(bounds.minX + tX, bottomY, bounds.minZ), 0xff0000));
            window.boxFaces.front.add(axisLine(new THREE.Vector3(bounds.minX + tX, topY, bounds.maxZ), new THREE.Vector3(bounds.minX + tX, bottomY, bounds.maxZ), 0xff0000));
        }

        // 左侧面 (x = bounds.minX) & 右侧面 (x = bounds.maxX) 网格 (黄色 Yellow)
        for (let i = 1; i < nTicks; i++) {
            let tY = (yLen / nTicks) * i;
            window.boxFaces.left.add(axisLine(new THREE.Vector3(bounds.minX, topY - tY, bounds.minZ), new THREE.Vector3(bounds.minX, topY - tY, bounds.maxZ), 0xffff00));
            window.boxFaces.right.add(axisLine(new THREE.Vector3(bounds.maxX, topY - tY, bounds.minZ), new THREE.Vector3(bounds.maxX, topY - tY, bounds.maxZ), 0xffff00));
        }
        for (let i = 1; i < nTicks; i++) {
            let tZ = (zLen / nTicks) * i;
            window.boxFaces.left.add(axisLine(new THREE.Vector3(bounds.minX, topY, bounds.minZ + tZ), new THREE.Vector3(bounds.minX, bottomY, bounds.minZ + tZ), 0xffff00));
            window.boxFaces.right.add(axisLine(new THREE.Vector3(bounds.maxX, topY, bounds.minZ + tZ), new THREE.Vector3(bounds.maxX, bottomY, bounds.minZ + tZ), 0xffff00));
        }
        // ================= 升级：为 6 个面添加真实的土壤纹理贴图 (无光照渲染版) =================
        const textureLoader = new THREE.TextureLoader();
        const soilTexture = textureLoader.load('/img/soil_diffuse2.jpg');
        soilTexture.wrapS = THREE.RepeatWrapping;
        soilTexture.wrapT = THREE.RepeatWrapping;

        const repeatPerMeter = 0.5;

        // 1. ✅ 核心修改：改用 MeshBasicMaterial，它不需要灯光也能完美显示贴图
        const soilMaterial = new THREE.MeshBasicMaterial({
            color: 0xffffff,         // 纯白底色，确保图片原本的颜色不被改变
            map: soilTexture,        // ✅ 贴上土壤图片
            transparent: true,       // 允许透明
            opacity: 0.65,           // 透明度，让内部红色数据透出来 (可按需微调)
            side: THREE.DoubleSide,  // 双面渲染
            depthWrite: false        // 防止遮挡管线
        });

        // 2. 辅助函数：直接修改几何体的 UV 映射比例
        function scaleUV(geometry, scaleU, scaleV) {
            const uv = geometry.attributes.uv;
            for (let i = 0; i < uv.count; i++) {
                uv.setXY(i, uv.getX(i) * scaleU, uv.getY(i) * scaleV);
            }
        }

        // 3. 定义几何体，并分别缩放 UV 
        const planeX = new THREE.PlaneGeometry(zLen, yLen);
        scaleUV(planeX, zLen * repeatPerMeter, yLen * repeatPerMeter);

        const planeY = new THREE.PlaneGeometry(xLen, zLen);
        scaleUV(planeY, xLen * repeatPerMeter, zLen * repeatPerMeter);

        const planeZ = new THREE.PlaneGeometry(xLen, yLen);
        scaleUV(planeZ, xLen * repeatPerMeter, yLen * repeatPerMeter);

        // 4. 创建长方体的 5 个面（不渲染顶面）
        // 顶面不创建纹理面，保持地下场景“开顶”显示，避免遮挡内部管道与剖面。
        const bottomFace = new THREE.Mesh(planeY, soilMaterial);
        bottomFace.position.set(cx, bottomY, cz);
        bottomFace.rotation.x = Math.PI / 2;
        axisGroup.add(bottomFace);

        const leftFace = new THREE.Mesh(planeX, soilMaterial);
        leftFace.position.set(bounds.minX, cy, cz);
        leftFace.rotation.y = -Math.PI / 2;
        axisGroup.add(leftFace);

        const rightFace = new THREE.Mesh(planeX, soilMaterial);
        rightFace.position.set(bounds.maxX, cy, cz);
        rightFace.rotation.y = Math.PI / 2;
        axisGroup.add(rightFace);

        const frontFace = new THREE.Mesh(planeZ, soilMaterial);
        frontFace.position.set(cx, cy, bounds.maxZ);
        axisGroup.add(frontFace);

        const backFace = new THREE.Mesh(planeZ, soilMaterial);
        backFace.position.set(cx, cy, bounds.minZ);
        backFace.rotation.y = Math.PI;
        axisGroup.add(backFace);
        // =====================================================================

        // ================= 3. 添加坐标轴刻度与文字 (白色) =================
        // 选择前端左下角为原点基准
        const originFrontBottomLeft = new THREE.Vector3(bounds.minX, bottomY, bounds.maxZ);
        const originTopLeft = new THREE.Vector3(bounds.minX, topY, bounds.maxZ);

        // X轴刻度 (Distance)
        for (let i = 0; i <= nTicks; i++) {
            const t = (xLen / nTicks) * i;
            const pos = originFrontBottomLeft.clone().add(new THREE.Vector3(t, 0, 0));
            axisGroup.add(axisLine(pos, pos.clone().add(new THREE.Vector3(0, -tickSize, 0))));
            const val = bounds.minX + t;
            const label = makeTextSprite(val.toFixed(2), "#ffffff"); // 传递白色
            label.scale.set(labelSize, labelSize, 1);
            label.position.copy(pos).add(new THREE.Vector3(0, -tickSize * 1.35, 0));
            // ✅ 修复：右下角 X/Z 两轴交汇处最后一个 X 轴刻度与 Z 轴刻度重叠，
            // 将 X 轴最后一个刻度数字稍微向左挪一点。
            if (i === nTicks) {
                label.position.add(new THREE.Vector3(-tickSize * 1.3, 0, 0));
            }
            axisGroup.add(label);
        }
        const xLabel = makeTextSprite("Distance(m)", "#ffffff");
        xLabel.scale.set(labelSize * 1.5, labelSize * 1.5, 1);
        xLabel.position.copy(originFrontBottomLeft).add(new THREE.Vector3(xLen / 2, -tickSize * 3.5, 0));
        axisGroup.add(xLabel);

        // Y轴刻度 (Time/Depth)
        for (let i = 0; i <= nTicks; i++) {
            const d = (yLen / nTicks) * i;
            const pos = originTopLeft.clone().add(new THREE.Vector3(0, -d, 0));
            axisGroup.add(axisLine(pos, pos.clone().add(new THREE.Vector3(-tickSize, 0, 0))));
            const depthValue = (physicalDepthMax / nTicks) * i;
            const label = makeTextSprite(depthValue.toFixed(1), "#ffffff");
            label.scale.set(labelSize, labelSize, 1);
            label.position.copy(pos).add(new THREE.Vector3(-tickSize * 1.35, 0, 0));
            axisGroup.add(label);
        }
        const yLabel = makeTextSprite("Depth(m)", "#ffffff");
        yLabel.scale.set(labelSize * 1.5, labelSize * 1.5, 1);
        // 为了显示类似 Time(ns) 的效果可以旋转标签，这里直接放左边
        yLabel.position.copy(originTopLeft).add(new THREE.Vector3(-tickSize * 4, -yLen / 2, 0));
        axisGroup.add(yLabel);

        // Z轴横向刻度
        for (let i = 0; i <= nTicks; i++) {
            const t = (zLen / nTicks) * i;
            const pos = originFrontBottomLeft.clone().add(new THREE.Vector3(xLen, 0, -t));
            axisGroup.add(axisLine(pos, pos.clone().add(new THREE.Vector3(0, -tickSize, 0))));
            const val = bounds.maxZ - t;
            const label = makeTextSprite(val.toFixed(2), "#ffffff");
            label.scale.set(labelSize, labelSize, 1);
            // ✅ 优化：Z 轴刻度数字整体向外侧轻微错开；
            // 在与 X 轴交汇的第一个刻度处再额外右移/前移，避免两个数字重叠。
            label.position.copy(pos).add(new THREE.Vector3(tickSize * 0.65, -tickSize * 1.25, 0));
            if (i === 0) {
                label.position.add(new THREE.Vector3(tickSize * 0.4, 0, tickSize * 0.5));
            }
            axisGroup.add(label);
        }
    }

    // 用 Canvas 生成一个动态宽度的文字 Sprite
    // variant = "tick"：刻度数字，弱背景、轻描边；variant = "axis"：轴标题，小型胶囊标签。
    function makeTextSprite(text, fontColor = "#ffffff") {
        const canvas = document.createElement("canvas");
        const ctx = canvas.getContext("2d");

        // 1. 设置字体并测量文字真实宽度
        const fontSize = 60;
        ctx.font = "bold " + fontSize + "px Arial";
        const textWidth = ctx.measureText(text).width;

        // 2. 动态设置 Canvas 宽高（宽度留出 40px 边距防裁剪，高度固定保持原有缩放基准）
        canvas.width = Math.max(256, textWidth + 40);
        canvas.height = 256;

        // 3. Canvas 尺寸改变后需重置 ctx 属性
        ctx.font = "bold " + fontSize + "px Arial";
        ctx.fillStyle = "rgba(255,255,255,0)"; // 透明背景
        ctx.fillRect(0, 0, canvas.width, canvas.height);

        ctx.fillStyle = fontColor;
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(text, canvas.width / 2, canvas.height / 2);

        const texture = new THREE.CanvasTexture(canvas);
        texture.needsUpdate = true;

        const material = new THREE.SpriteMaterial({
            map: texture,
            transparent: true
        });

        const sprite = new THREE.Sprite(material);

        // 4. ✅ 核心：将长宽比存起来，防止长文本被强行压缩成正方形
        sprite.userData.aspectRatio = canvas.width / canvas.height;

        return sprite;
    }

    // WebGL 画布水平镜像后，补偿 Sprite 贴图自身的镜像，保持刻度和地图文字可读。
    // 地图/标注 Sprite 可能在场景建成后加入，因此每帧只处理新出现的贴图。
    function keepGprSceneSpritesReadableInMirror(scene) {
        scene.traverse(object => {
            if (!object.isSprite || !object.material?.map) return;
            const texture = object.material.map;
            texture.userData ||= {};
            if (texture.userData.gprMirrorTextCorrected) return;
            texture.repeat.x = -texture.repeat.x;
            texture.offset.x = 1 - texture.offset.x;
            texture.userData.gprMirrorTextCorrected = true;
            texture.needsUpdate = true;
        });
    }

    // 根据相机距离动态调整坐标轴文字的世界尺寸，使它在屏幕上保持稳定像素大小。
    function updateGprAxisLabelSprites(camera, renderer) {
        if (!window.gprAxesGroup || !camera || !renderer || typeof THREE === "undefined") return;
        const dom = renderer.domElement;
        const viewportHeight = Math.max(dom.clientHeight || dom.height || 1, 1);
        const fovRad = (camera.fov || 45) * Math.PI / 180;

        window.gprAxesGroup.traverse(obj => {
            if (!obj || !obj.userData || !obj.userData.isGprAxisLabel) return;
            const pixelHeight = Number(obj.userData.pixelHeight || 22);
            const distance = camera.position.distanceTo(obj.position);
            let worldHeight = 2 * distance * Math.tan(fovRad / 2) * (pixelHeight / viewportHeight);
            const minH = Number(obj.userData.minWorldHeight || 0.04);
            const maxH = Number(obj.userData.maxWorldHeight || 0.4);
            worldHeight = Math.max(minH, Math.min(maxH, worldHeight));
            const aspect = Number(obj.userData.aspectRatio || 1);
            obj.scale.set(worldHeight * aspect, worldHeight, 1);
        });
    }

    // ==========================================
    // 5. ✅ 机械臂控制函数 (已适配 Java Spring Boot)
    // ==========================================

    // 🔴 全局状态：记录当前6个关节的角度 (初始化为0度)
    // 因为 Java 接口每次都需要发送所有关节的绝对位置
    var currentAngles = [0, 0, 0, 0, 0, 0];

    // 辅助函数：限制角度范围 (防止撞机)
    // ✅ 修改：定义 6 个关节的独立运动范围 (请根据图片修正具体数值)
    const JOINT_LIMITS = [
        { min: -154, max: 154 }, // J1 基座
        { min: 0, max: 195 },   // J2 大臂
        { min: -175, max: 0 }, // J3 小臂
        { min: -102, max: 102 }, // J4 腕旋
        { min: -75, max: 75 },   // J5 腕折
        { min: -170, max: 170 }  // J6 末端
    ];
    // ✅ 修改：传入 index 以获取对应关节的限位
    function clampAngle(index, angle) {
        const limit = JOINT_LIMITS[index];
        return Math.max(limit.min, Math.min(limit.max, angle));
    }

    // 🔴 记录当前的末端位姿状态 (需确保定义在全局)
    var currentEndPose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    var isEndPoseSynced = false;

    // ✅ 新增：末端控制的定时器与锁
    let cartesianJogTimer = null;

    // 5.1 增量式末端控制 (核心修改：立即更新本地状态，支持高频累加)
    window.controlArmStep = function (dx, dy, dz, drx = 0, dry = 0, drz = 0) {
        if (!isEndPoseSynced) {
            console.warn("⚠️ 正在同步机械臂真实坐标，请稍后再点击！");
            return;
        }

        // 计算目标坐标
        const targetX = currentEndPose[0] + dx;
        const targetY = currentEndPose[1] + dy;
        const targetZ = currentEndPose[2] + dz;
        const targetRX = currentEndPose[3] + drx;
        const targetRY = currentEndPose[4] + dry;
        const targetRZ = currentEndPose[5] + drz;

        // ✅ 核心：立即更新本地全局状态！
        // 这样在定时器下一次循环到来时，就会在这个新坐标的基础上继续做加法，而不会等后端的慢反馈
        currentEndPose[0] = targetX;
        currentEndPose[1] = targetY;
        currentEndPose[2] = targetZ;
        currentEndPose[3] = targetRX;
        currentEndPose[4] = targetRY;
        currentEndPose[5] = targetRZ;

        const url = `${ARM_BACKEND_URL}/arm/move_end?x=${targetX}&y=${targetY}&z=${targetZ}&rx=${targetRX}&ry=${targetRY}&rz=${targetRZ}`;

        fetch(url, { method: "GET" })
            .catch(err => console.error("⚠️ 末端控制请求失败:", err));
    };

    // ✅ 废弃定时器，仅标记状态
    let isCartesianJogging = false;

    // ✅ 新增：长按开始末端连续运动
    // 参数 axis: 0(X), 1(Y), 2(Z), 3(RX), 4(RY), 5(RZ)
    // 参数 direction: 1 (正向) 或 -1 (反向)
    window.startCartesianJog = function (axis, direction, event) {
        if (event) {
            event.preventDefault();
            event.stopPropagation();
        }

        if (isCartesianJogging) return;
        isCartesianJogging = true;

        console.log(`🚀 [前端] 开始末端连续运动: 轴 ${axis} 方向 ${direction}`);
        const url = `${ARM_BACKEND_URL}/arm/jog_end?axis=${axis}&direction=${direction}`;
        fetch(url).catch(err => console.error("Start 失败:", err));
    };

    // ✅ 新增：松开按钮停止运动 (复用已有的 /arm/stop)
    window.stopCartesianJog = function () {
        if (!isCartesianJogging) return;
        isCartesianJogging = false;

        console.log("🛑 [前端] 松开按钮，末端长按运动停止");
        const url = `${ARM_BACKEND_URL}/arm/stop`;
        fetch(url).catch(err => console.error("Stop 失败:", err));
    };

    // 5.2 关节控制 (点击按钮步进)
    // index: 0-5 (对应 J1-J6), direction: 1 (正向) 或 -1 (反向)
    window.controlJoint = function (index, direction) {
        // 1. 定义步长 (每次点击转动多少度)
        const step = 5;

        // 2. 更新本地记录的角度状态
        let newAngle = currentAngles[index] + (direction > 0 ? step : -step);

        // 限制角度范围
        // ✅ 修改：传入 index 参数
        newAngle = clampAngle(index, newAngle);

        // 更新全局状态
        currentAngles[index] = newAngle;

        const jointName = "J" + (index + 1);
        console.log(`🚀 准备发送指令: ${jointName} -> ${newAngle}° (当前全状态: ${currentAngles})`);

        // 3. 拼接 URL 参数
        // 目标格式: /arm/move?j1=0&j2=0&...&speed=50
        // 使用 map 和 join 快速生成查询字符串
        const queryParams = currentAngles.map((angle, i) => `j${i + 1}=${angle}`).join("&");
        const url = `${ARM_BACKEND_URL}/arm/move?${queryParams}&speed=50`;

        // 4. 发送 GET 请求
        fetch(url, {
            method: "GET" // 对应 Java Controller 的 @GetMapping
        })
            .then(res => res.text()) // Java 返回的是纯字符串 (String)
            .then(data => {
                // 只要后端返回的内容包含 "success" 或 "成功" 即视为正常
                // ✅ 加上对 "OK" 的判断
                if (data.includes("success") || data.includes("成功") || data.includes("指令发送成功") || data.includes("OK")) {
                    console.log(`✅ ${jointName} 移动成功:`, data);
                } else {
                    console.error(`❌ 移动可能失败: ${data}`);
                }
            })
            .catch(err => {
                console.error("⚠️ Java 后端连接失败 (请检查 SpringBoot 是否运行在 8080 端口)", err);
            })
            .finally(() => {
                // ✅ 关键新增：无论成功失败，请求结束了，解除忙碌状态
                isArmBusy = false;
            });
    };
    // ==========================================
    // 5.3 ✅ 向前按点、向后按点、归零 控制逻辑
    // ==========================================

    // 向前按点：[0, 120, -130, 0, 20, 0]
    window.moveArmToFrontPress = function () {
        console.log("🎯 开始执行【向前按点】...");
        const targetPose = [0, 120, -130, 0, 20, 0];
        const speed = 50;

        // 更新全局状态 (很重要，防止下次单关节控制时位置错乱)
        if (typeof currentAngles !== 'undefined') {
            currentAngles = [...targetPose];
        }

        const queryPose = targetPose.map((a, i) => `j${i + 1}=${a}`).join("&");
        const url = `${ARM_BACKEND_URL}/arm/move?${queryPose}&speed=${speed}`;

        fetch(url)
            .then(res => res.text())
            .then(msg => console.log("向前按点反馈:", msg))
            .catch(err => console.error("向前按点指令发送失败:", err));
    };

    // 向后按点：[0, 35, -170, 0, -45, 0]
    window.moveArmToBackPress = function () {
        console.log("🎯 开始执行【向后按点】...");
        const targetPose = [0, 35, -170, 0, -45, 0];
        const speed = 50;

        // 更新全局状态
        if (typeof currentAngles !== 'undefined') {
            currentAngles = [...targetPose];
        }

        const queryPose = targetPose.map((a, i) => `j${i + 1}=${a}`).join("&");
        const url = `${ARM_BACKEND_URL}/arm/move?${queryPose}&speed=${speed}`;

        fetch(url)
            .then(res => res.text())
            .then(msg => console.log("向后按点反馈:", msg))
            .catch(err => console.error("向后按点指令发送失败:", err));
    };

    // 归零：[0, 0, 0, 0, 0, 0]
    window.moveArmToZero = function () {
        console.log("🔄 开始执行【归零】...");
        const targetPose = [0, 0, 0, 0, 0, 0];
        const speed = 50;

        // 更新全局状态
        if (typeof currentAngles !== 'undefined') {
            currentAngles = [...targetPose];
        }

        const queryPose = targetPose.map((a, i) => `j${i + 1}=${a}`).join("&");
        const url = `${ARM_BACKEND_URL}/arm/move?${queryPose}&speed=${speed}`;

        fetch(url)
            .then(res => res.text())
            .then(msg => console.log("归零反馈:", msg))
            .catch(err => console.error("归零指令发送失败:", err));
    };
    // ==========================================
    // 5.4 ✅ 破损点右侧拍照位姿序列 (丝滑过渡版 + 自动抓拍)
    // ==========================================
    window.captureFaultSequence = function () {
        // 🌟 新增：拦截校验，确保当前有破损点可以绑定
        if (!window.lastFaultMarker || !window.lastFaultMarker.userData.faultId) {
            alert("⚠️ 当前没有检测到破损点，无法进行右侧抓拍绑定！");
            return;
        }
        const faultId = window.lastFaultMarker.userData.faultId;

        console.log("📸 开始机械臂破损点【右侧】拍照序列...");

        // 1. 定义动作 A：右侧拍照点 J1 为负 90 度
        const targetPose1 = [-90, 150, -80, 0, 60, 0];
        const speed = 50;

        if (typeof currentAngles !== 'undefined') { currentAngles = [...targetPose1]; }

        const queryPose1 = targetPose1.map((a, i) => `j${i + 1}=${a}`).join("&");
        const urlPose1 = `${ARM_BACKEND_URL}/arm/move?${queryPose1}&speed=${speed}`;

        console.log("🚀 动作1: 前往右侧拍照姿态", targetPose1);
        fetch(urlPose1).then(res => res.text()).then(msg => console.log("右侧拍照姿态到达反馈:", msg));

        // 🌟 新增：在 1.5 秒时抓拍（此时机械臂已到达指定位置并稳定）
        setTimeout(() => {
            console.log("📸 正在提取右侧正上方画面...");
            fetch(ARM_CAMERA_BASE + "/api/capture_arm")
                .then(res => res.json())
                .then(data => {
                    if (data.status === "success") {
                        if (!window.faultImagesMap.has(faultId)) {
                            // 🌟 补全四个方向
                            window.faultImagesMap.set(faultId, { left: "", right: "", front: "", back: "" });
                        }
                        // 绑定右侧图片
                        bindCapturedImageToFault(faultId, "right", data.image)
                            .catch(err => {
                                console.error("右侧图片上传或绑定失败:", err);
                                alert("右侧图片上传失败：" + err.message);
                            });
                    }
                })
                .catch(err => console.error("右侧抓拍失败:", err));
        }, 1500);

        // 2. 必须结结实实等 2000 毫秒（2秒），让机械臂停稳拍照
        setTimeout(() => {
            console.log("🔄 动作2: 2秒停留结束，抬起至右侧过渡姿态");
            const targetPose2 = [-90, 90, -80, 0, 60, 0];
            if (typeof currentAngles !== 'undefined') { currentAngles = [...targetPose2]; }
            const queryPose2 = targetPose2.map((a, i) => `j${i + 1}=${a}`).join("&");
            const urlPose2 = `${ARM_BACKEND_URL}/arm/move?${queryPose2}&speed=${speed}`;
            fetch(urlPose2).then(res => res.text()).then(msg => console.log("发送右侧过渡姿态指令"));

            setTimeout(() => {
                console.log("🔄 动作3: 顺滑切入归零轨迹");
                const target0 = [0, 0, 0, 0, 0, 0];
                if (typeof currentAngles !== 'undefined') { currentAngles = [...target0]; }
                const query0 = target0.map((a, i) => `j${i + 1}=${a}`).join("&");
                const url0 = `${ARM_BACKEND_URL}/arm/move?${query0}&speed=${speed}`;
                fetch(url0).then(res => res.text()).then(msg => console.log("归零指令已发送"));
            }, 800);
        }, 2000);
    };


    // ==========================================
    // 5.5 ✅ 破损点左侧拍照位姿序列 (丝滑过渡版 + 自动抓拍)
    // ==========================================
    window.captureFaultLeftSequence = function () {
        // 🌟 新增：拦截校验，确保当前有破损点可以绑定
        if (!window.lastFaultMarker || !window.lastFaultMarker.userData.faultId) {
            alert("⚠️ 当前没有检测到破损点，无法进行左侧抓拍绑定！");
            return;
        }
        const faultId = window.lastFaultMarker.userData.faultId;

        console.log("📸 开始机械臂破损点【左侧】拍照序列...");

        const targetPose1 = [90, 150, -80, 0, 60, 0];
        const speed = 50;

        if (typeof currentAngles !== 'undefined') { currentAngles = [...targetPose1]; }

        const queryPose1 = targetPose1.map((a, i) => `j${i + 1}=${a}`).join("&");
        const urlPose1 = `${ARM_BACKEND_URL}/arm/move?${queryPose1}&speed=${speed}`;

        console.log("🚀 动作1: 前往左侧拍照姿态", targetPose1);
        fetch(urlPose1).then(res => res.text()).then(msg => console.log("左侧拍照到达反馈:", msg));

        // 🌟 新增：在 1.5 秒时抓拍
        setTimeout(() => {
            console.log("📸 正在提取左侧正上方画面...");
            fetch(ARM_CAMERA_BASE + "/api/capture_arm")
                .then(res => res.json())
                .then(data => {
                    if (data.status === "success") {
                        if (!window.faultImagesMap.has(faultId)) {
                            // 🌟 补全四个方向
                            window.faultImagesMap.set(faultId, { left: "", right: "", front: "", back: "" });
                        }
                        // 绑定左侧图片
                        bindCapturedImageToFault(faultId, "left", data.image)
                            .catch(err => {
                                console.error("左侧图片上传或绑定失败:", err);
                                alert("左侧图片上传失败：" + err.message);
                            });
                    }
                })
                .catch(err => console.error("左侧抓拍失败:", err));
        }, 1500);

        setTimeout(() => {
            console.log("🔄 动作2: 拍照完成，抬起至左侧过渡姿态");
            const targetPose2 = [90, 90, -80, 0, 60, 0];
            if (typeof currentAngles !== 'undefined') { currentAngles = [...targetPose2]; }
            const queryPose2 = targetPose2.map((a, i) => `j${i + 1}=${a}`).join("&");
            const urlPose2 = `${ARM_BACKEND_URL}/arm/move?${queryPose2}&speed=${speed}`;
            fetch(urlPose2).then(res => res.text()).then(msg => console.log("发送左侧过渡姿态指令"));

            setTimeout(() => {
                console.log("🔄 动作3: 顺滑切入归零轨迹");
                const target0 = [0, 0, 0, 0, 0, 0];
                if (typeof currentAngles !== 'undefined') { currentAngles = [...target0]; }
                const query0 = target0.map((a, i) => `j${i + 1}=${a}`).join("&");
                const url0 = `${ARM_BACKEND_URL}/arm/move?${query0}&speed=${speed}`;
                fetch(url0).then(res => res.text()).then(msg => console.log("归零指令已发送"));
            }, 800);
        }, 2000);
    };
    // ==========================================
    // 5.6 ✅ 破损点前方拍照序列 (直接截取机器狗前摄)
    // ==========================================
    window.captureFaultFrontSequence = function () {
        captureFaultRobotCamera("front", "前方");
    };

    // ==========================================
    // 5.7 ✅ 破损点后方拍照序列 (直接截取机器狗后摄)
    // ==========================================
    window.captureFaultBackSequence = function () {
        captureFaultRobotCamera("back", "后方");
    };

    // ==========================================
    // ✅ [新增] 夹爪控制与状态轮询逻辑
    // ==========================================
    let isGripperJogging = false;
    let gripperTarget = 0;

    window.updateGripperLabel = function (val) {
        const el = document.getElementById('disp_gripper');
        if (el) el.textContent = parseFloat(val).toFixed(1) + ' mm';
    };

    window.setGripperValue = function (val) {
        gripperTarget = parseFloat(val);
        window.updateGripperLabel(gripperTarget);
        const url = `${ARM_BACKEND_URL}/arm/gripper?value=${gripperTarget}`;
        fetch(url).catch(err => console.error("夹爪数值设定失败:", err));
    };

    window.startGripperJog = function (dir) {
        if (isGripperJogging) return;
        isGripperJogging = true;
        const url = `${ARM_BACKEND_URL}/arm/gripper_jog?dir=${dir}`;
        fetch(url).catch(err => { isGripperJogging = false; console.error("夹爪点动失败:", err); });
    };

    window.stopGripperJog = function () {
        if (!isGripperJogging) return;
        isGripperJogging = false;
        const url = `${ARM_BACKEND_URL}/arm/gripper_stop`;
        fetch(url).catch(err => console.error("夹爪停止失败:", err));
    };
    // 全局变量，记录状态
    let isStopping = false; // 是否正在停止中
    // ==========================================
    // 5.4 ✅ 终极版：状态锁控制 (State Machine)
    // ==========================================

    // 🔴 核心变量：记录机械臂是否正在运动
    // 只有当它是 true 时，我们才允许发 stop
    let isJogging = false;

    window.startArmJog = function (index, direction, event) {
        // 1. 阻止浏览器默认行为 (防止拖拽按钮、选中文字)
        if (event) {
            event.preventDefault();
            event.stopPropagation();
        }

        // 2. 如果已经在运动中，防止重复发送 Start (屏蔽微小抖动)
        if (isJogging) return;

        // 3. 立即上锁
        isJogging = true;

        console.log(`🚀 [前端] 开始运动: J${index + 1} 方向 ${direction}`);

        // 4. 发送 Start 请求
        const url = `${ARM_BACKEND_URL}/arm/jog?index=${index}&direction=${direction}`;
        fetch(url).catch(err => {
            console.error("Start 失败:", err);
            // 如果发送失败，必须把锁打开，否则用户没法再触发了
            isJogging = false;
        });
    };

    window.stopArmJog = function () {
        // 1. 🔴 核心判断：如果当前根本没在运动，直接无视！
        // 这行代码能过滤掉 99% 的重复 Stop 指令 (比如同时触发了 mouseup 和 mouseleave)
        if (!isJogging) return;

        // 2. 立即标记为“已停止”
        // 这样后续的第2、第3个重复事件进来时，会在第一步就被拦住
        isJogging = false;

        console.log("🛑 [前端] 触发停止");

        // 3. 发送 Stop 请求
        const url = `${ARM_BACKEND_URL}/arm/stop`;
        fetch(url).catch(err => console.error("Stop 失败:", err));
    };

    // 封装一个安全的发送函数，防止请求堆积
    function safeControlJoint(index, direction) {
        // 如果上一次请求还在路上，这次就先跳过，防止阻塞串口
        if (isArmBusy) {
            console.log("⏳ 机械臂忙，跳过此帧...");
            return;
        }

        isArmBusy = true; // 标记忙碌

        // 调用你原本的 controlJoint 逻辑
        // 注意：这里我们需要改造一下原本的 controlJoint，让它返回 Promise
        // 或者我们直接在这里手动调用 fetch，为了复用，建议我们修改 controlJoint

        // 这里我们直接调用原函数，但在原函数里处理 isArmBusy 的复位
        window.controlJoint(index, direction);
    }
    // ==========================================
    // 6. 信号波形显示 & [新] AUC 算法移植 (保留声信号)
    // ==========================================
    (function initSignalChart() {
        const canvas = document.getElementById("signalCanvas");
        const statusDiv = document.getElementById("signalStatus");
        if (!canvas) return;

        const ctx = canvas.getContext("2d");
        const Y_MAX = 4096; // ADC 12-bit
        const EM_BASELINE = 2048;             // 电磁信号 ADC 中心基线
        const EM_VISUAL_THRESHOLD_LIMIT = 1000; // 阈值虚线最大视觉位移：超过 1000 后虚线固定，改为压缩纵轴

        // --- 1. 数据存储 (保留双通道) ---
        let channel1Data = []; // 磁信号 (用于绘图)
        let channel2Data = []; // 声信号 (用于绘图 - ✅ 保持不变)

        // 历史数值缓存 (UI显示用 - ✅ 保持不变)
        let magHistory = [0, 0, 0];
        let sndHistory = [0, 0, 0];

        // --- 2. [新增] AUC 算法专用变量 (仅针对磁信号) ---
        const BUFFER_SIZE = 20000;
        const ringBuffer = new Float32Array(BUFFER_SIZE);
        const sndRingBuffer = new Float32Array(BUFFER_SIZE); // 声信号同步环形缓存
        let writePtr = 0;       // 写入指针
        let totalSamples = 0;   // 绝对计数器
        let algoState = 'IDLE'; // IDLE -> WAITING_DATA
        let triggerIndex = 0;
        let lastTriggerTime = -9999;

        // 3. 自动调整画布大小 (✅ 保持不变)
        function resizeCanvas() {
            const parent = canvas.parentElement;
            if (parent && parent.clientWidth > 0) {
                canvas.width = parent.clientWidth;
                canvas.height = parent.clientHeight;
                requestAnimationFrame(drawSignal);
            }
        }
        new ResizeObserver(resizeCanvas).observe(canvas.parentElement);


        // ============================================================
        // 🚀 [核心逻辑替换] AUC 算法处理流
        // ============================================================

        // 处理新进来的磁信号数据 (Feed Data)
        function processSignals(magChunk, sndChunk) {
            for (let i = 0; i < magChunk.length; i++) {
                const rawMag = magChunk[i];
                const rawSnd = sndChunk[i];
                const adjustedMag = rawMag - 2048; // 去直流

                // 双通道同步写入环形缓冲区
                ringBuffer[writePtr] = adjustedMag;
                sndRingBuffer[writePtr] = rawSnd - 2048; // 声信号也去直流，防止 FFT 出现巨大的 0Hz 峰值

                writePtr = (writePtr + 1) % BUFFER_SIZE;
                totalSamples++;

                // 原有的 AUC 实时触发检测
                if (algoState === 'IDLE') {
                    if (Math.abs(adjustedMag) > AUC_CONFIG.PULSE_THRESHOLD) {
                        if (totalSamples - lastTriggerTime > AUC_CONFIG.DISTANCE_MIN) {
                            algoState = 'WAITING_DATA';
                            triggerIndex = totalSamples;
                            lastTriggerTime = totalSamples;
                        }
                    }
                }
            }

            if (algoState === 'WAITING_DATA') {
                if (totalSamples >= triggerIndex + AUC_CONFIG.WINDOW_AFTER) {
                    performAUCAnalysis();
                    algoState = 'IDLE';
                }
            }
        }

        // 执行 AUC 积分分析 (已简化为纯脉冲阈值判定)
        function performAUCAnalysis() {
            // 提取窗口数据: [Trigger - BEFORE, Trigger + AFTER] (保留此段，为了下方提取峰值使用)
            const startAbs = triggerIndex - AUC_CONFIG.WINDOW_BEFORE;
            const len = AUC_CONFIG.WINDOW_BEFORE + AUC_CONFIG.WINDOW_AFTER;
            const segment = new Float32Array(len);

            // 从环形缓冲区复制 (处理回绕)
            for (let i = 0; i < len; i++) {
                let idx = (startAbs + i) % BUFFER_SIZE;
                if (idx < 0) idx += BUFFER_SIZE;
                segment[i] = ringBuffer[idx];
            }

            // ==========================================
            // 🗑️ 删除了计算均值、方差、Z-score和AUC能量积分的代码
            // ✅ 改为直接判定：因为能进入此函数，说明在 processSignals 已突破脉冲阈值
            // ==========================================
            const isDischarge = true;

            // 4. 结果输出
            if (isDischarge) {
                console.log(`🔴 [故障判定] 突破脉冲阈值，触发破损点记录...`);
                window.g_isFaultDetected = true;
                window.g_faultTriggeredInBS = true;

                // ======== ⚡ 保留原有的：触发声波 FFT 处理 ========
                const WINDOW_POINTS = 10000;
                const HALF_WINDOW = 5000; // 0.5秒
                const N_FFT = 16384;

                const sndFftInput = new Float32Array(N_FFT);
                const startIdx = triggerIndex - HALF_WINDOW;

                for (let i = 0; i < WINDOW_POINTS; i++) {
                    let idx = (startIdx + i) % BUFFER_SIZE;
                    if (idx < 0) idx += BUFFER_SIZE;
                    const multiplier = 0.5 * (1 - Math.cos((2 * Math.PI * i) / (WINDOW_POINTS - 1)));
                    sndFftInput[i] = sndRingBuffer[idx] * multiplier;
                }

                const magSpectrum = computeMagnitudeFFT(sndFftInput);
                window.latestAcousticFFT = {
                    spectrum: magSpectrum,
                    sampleRate: AUC_CONFIG.SAMPLING_RATE,
                    fftSize: N_FFT
                };
                // ============================================

                const peakVal = Math.abs(segment[AUC_CONFIG.WINDOW_BEFORE]) + 2048;
                window.g_realtimeMag = peakVal;


                setTimeout(() => { window.g_isFaultDetected = false; }, 500);

                if (window.fftTimeoutId) {
                    clearTimeout(window.fftTimeoutId);
                }
                window.fftTimeoutId = setTimeout(() => {
                    window.latestAcousticFFT = null;
                    console.log("⏱️ FFT 画面展示完毕，已自动切回实时声信号波形");
                }, 2000);
            }
        }


        // ============================================================
        // 🎨 绘图逻辑 (保留原有双通道绘制)
        // ============================================================
        function drawSignal() {
            // 🚀 核心拦截：如果你没在看“故障定点(pathFault)”面板，直接跳过波形清屏和绘制！
            if (window.currentActivePanel !== 'pathFault') {
                requestAnimationFrame(drawSignal);
                return;
            }

            const w = canvas.width;
            const h = canvas.height;
            if (h === 0 || w === 0) return;

            const halfH = h / 2;

            // 清屏
            ctx.fillStyle = "#000000";
            ctx.fillRect(0, 0, w, h);

            // 中线
            ctx.strokeStyle = "#444";
            ctx.lineWidth = 1;
            ctx.beginPath(); ctx.moveTo(0, halfH); ctx.lineTo(w, halfH); ctx.stroke();

            // ✅ 绘制 Ch1 (电磁信号)
            if (channel1Data && channel1Data.length > 0) {
                // 当电磁阈值 > 1000 时，仅压缩电磁信号纵轴；声信号保持原有显示方式。
                drawChannel(channel1Data, 0, halfH, "#00ccff", {
                    compressByPulseThreshold: true
                });

                drawPulseThresholdLines(0, halfH);
            }

            // ✅ 绘制 Ch2 (声信号 - 保持不变)
            if (window.latestAcousticFFT) {
                // 发生过故障：显示 1s 窗口截取的特定频段 FFT
                drawFFTChannel(window.latestAcousticFFT, halfH, halfH, "#ff3333");
            } else if (channel2Data && channel2Data.length > 0) {
                // 尚未发生故障：退化为原样显示实时时域波形作为占位
                drawChannel(channel2Data, halfH, halfH, "#ff3333");
            }

            // 更新 UI 数值
            updateUIValues();

            requestAnimationFrame(drawSignal);
        }

        function drawFFTChannel(fftObj, topY, height, color) {
            ctx.strokeStyle = color;
            ctx.lineWidth = 1.5;
            ctx.beginPath();

            const w = canvas.width;
            const { spectrum, sampleRate, fftSize } = fftObj;

            // FFT 频率分辨率 Δf = Fs / N
            const df = sampleRate / fftSize;

            // 将设定的 Hz 映射到对应的索引 Bin
            const minBin = Math.floor(FFT_CONFIG.FREQ_MIN / df);
            const maxBin = Math.ceil(FFT_CONFIG.FREQ_MAX / df);

            // 寻找当前截断频段内的最大幅值，用于动态自适应 Y 轴缩放
            let maxMag = 0;
            for (let i = minBin; i <= maxBin && i < spectrum.length; i++) {
                if (spectrum[i] > maxMag) maxMag = spectrum[i];
            }
            if (maxMag === 0) maxMag = 1;

            // 绘制频谱
            let drewFirst = false;
            for (let i = minBin; i <= maxBin && i < spectrum.length; i++) {
                // X 坐标：按频段区间归一化
                const x = ((i - minBin) / (maxBin - minBin)) * w;

                // Y 坐标：幅值自适应归一化，留 10% 顶部边距
                const norm = (spectrum[i] / maxMag) * 0.9;
                const y = topY + height - (norm * height);

                if (!drewFirst) {
                    ctx.moveTo(x, y);
                    drewFirst = true;
                } else {
                    ctx.lineTo(x, y);
                }
            }
            ctx.stroke();

            // 绘制频段标签标识
            ctx.fillStyle = color;
            ctx.font = "12px monospace";
            ctx.fillText(`声波FFT: ${FFT_CONFIG.FREQ_MIN} - ${FFT_CONFIG.FREQ_MAX} Hz`, 5, topY + 15);
        }

        // 获取电磁信号显示压缩比例：
        // 阈值 <=1000：比例为 1，保持原始显示；
        // 阈值 >1000：把真实电磁偏移按 1000 / 当前阈值压缩，让阈值虚线固定在 1000 的位置。
        function getElectromagneticVisualScale() {
            const threshold = Math.max(0, Number(AUC_CONFIG.PULSE_THRESHOLD) || 0);
            if (threshold <= EM_VISUAL_THRESHOLD_LIMIT) {
                return 1.0;
            }
            return EM_VISUAL_THRESHOLD_LIMIT / threshold;
        }

        function getDisplayedElectromagneticValue(rawValue) {
            const scale = getElectromagneticVisualScale();
            let val = Number(rawValue);

            if (!Number.isFinite(val)) {
                val = EM_BASELINE;
            }

            // 围绕 2048 基线压缩，而不是简单整体除以比例，避免基线漂移。
            val = EM_BASELINE + (val - EM_BASELINE) * scale;

            if (val > Y_MAX) val = Y_MAX;
            if (val < 0) val = 0;
            return val;
        }

        function valueToY(value, topY, height) {
            let val = Number(value);
            if (!Number.isFinite(val)) val = EM_BASELINE;
            if (val > Y_MAX) val = Y_MAX;
            if (val < 0) val = 0;

            const normalized = val / Y_MAX;
            return topY + (height - (normalized * height));
        }

        function drawPulseThresholdLines(topY, height) {
            try {
                ctx.save();
                ctx.beginPath();
                ctx.setLineDash([5, 5]);
                ctx.lineWidth = 1.5;
                ctx.strokeStyle = "rgba(255, 165, 0, 0.9)";

                const threshold = Math.max(0, Number(AUC_CONFIG.PULSE_THRESHOLD) || 0);
                const lineThreshold = Math.min(threshold, EM_VISUAL_THRESHOLD_LIMIT);

                // 阈值 <=1000：虚线跟随真实阈值；
                // 阈值 >1000：虚线固定在 ±1000 的视觉位置。
                const yTop = Math.round(valueToY(EM_BASELINE + lineThreshold, topY, height));
                const yBottom = Math.round(valueToY(EM_BASELINE - lineThreshold, topY, height));

                ctx.moveTo(0, yTop);
                ctx.lineTo(canvas.width, yTop);
                ctx.moveTo(0, yBottom);
                ctx.lineTo(canvas.width, yBottom);
                ctx.stroke();

                ctx.setLineDash([]);
                ctx.fillStyle = "#FFA500";
                ctx.font = "bold 12px Consolas, monospace";
                ctx.textAlign = "right";

                // 文字仍显示真实阈值，便于现场确认当前算法判定阈值。
                ctx.fillText(`+${threshold}`, canvas.width - 10, yTop - 6);
                ctx.fillText(`-${threshold}`, canvas.width - 10, yBottom + 14);

                if (threshold > EM_VISUAL_THRESHOLD_LIMIT) {
                    const scale = getElectromagneticVisualScale();
                    ctx.textAlign = "left";
                    ctx.fillStyle = "rgba(255, 210, 90, 0.95)";
                    ctx.font = "bold 11px Microsoft YaHei, Consolas, monospace";
                    ctx.fillText(`电磁纵轴压缩 ×${scale.toFixed(2)}`, 8, topY + 34);
                }

                ctx.restore();
            } catch (e) {
                console.error("阈值刻度绘制失败:", e);
            }
        }

        // 通用单通道绘图函数
        function drawChannel(data, topY, height, color, options = {}) {
            ctx.strokeStyle = color;
            ctx.lineWidth = 1.5;
            ctx.beginPath();

            const len = data.length;
            const w = canvas.width;

            for (let i = 0; i < len; i++) {
                const x = (i / len) * w;
                let val = data[i];

                if (options.compressByPulseThreshold) {
                    val = getDisplayedElectromagneticValue(val);
                } else {
                    if (val > Y_MAX) val = Y_MAX;
                    if (val < 0) val = 0;
                }

                const y = valueToY(val, topY, height);

                if (i === 0) ctx.moveTo(x, y);
                else ctx.lineTo(x, y);
            }
            ctx.stroke();

            ctx.fillStyle = color;
            ctx.font = "12px monospace";
            const label = topY === 0 ? `电磁信号` : `声信号`;
            ctx.fillText(label, 5, topY + 15);
        }

        // 更新左侧滚动的数值 (UI)
        function updateUIValues() {

            // 2. ✅ 声信号数值 (保持不变)
            if (channel2Data.length > 0) {
                const currSnd = Math.max(...channel2Data);
                sndHistory.push(Math.floor(currSnd));
                if (sndHistory.length > 3) sndHistory.shift();

                const elSnd = document.getElementById('sndSignalVal');
                // 假设你的 HTML 里有 sndSignalVal 这个 ID，如果没有可忽略
                if (elSnd) {
                    elSnd.textContent = sndHistory.join('   ');
                }
            }
        }

        // --- 4. 声磁 WebSocket：心跳保活 + Wi-Fi 断开后自动重连 ---
        const wsUrl = "ws://10.21.31.104:8001/ws/signal";
        const SIGNAL_RECONNECT_DELAY_MS = 2000;
        const SIGNAL_HEARTBEAT_INTERVAL_MS = 5000;
        let signalWs = null;
        let signalReconnectTimer = null;
        let signalHeartbeatTimer = null;

        function updateSignalConnectionStatus(text, color) {
            if (!statusDiv) return;
            statusDiv.textContent = text;
            statusDiv.style.color = color;
        }

        function clearSignalHeartbeat() {
            if (signalHeartbeatTimer !== null) {
                window.clearInterval(signalHeartbeatTimer);
                signalHeartbeatTimer = null;
            }
        }

        function scheduleSignalReconnect() {
            if (signalReconnectTimer !== null) return;

            updateSignalConnectionStatus("● RECONNECTING", "#ffb000");
            console.warn("⚠️ 声磁 WebSocket 已断开，2 秒后自动重连...");
            signalReconnectTimer = window.setTimeout(() => {
                signalReconnectTimer = null;
                connectSignalWebSocket();
            }, SIGNAL_RECONNECT_DELAY_MS);
        }

        function connectSignalWebSocket() {
            if (signalWs && (
                signalWs.readyState === WebSocket.OPEN ||
                signalWs.readyState === WebSocket.CONNECTING
            )) {
                return;
            }

            console.log("正在尝试连接声磁 WebSocket:", wsUrl);
            const socket = new WebSocket(wsUrl);
            signalWs = socket;

            socket.onopen = () => {
                if (signalWs !== socket) return;

                if (signalReconnectTimer !== null) {
                    window.clearTimeout(signalReconnectTimer);
                    signalReconnectTimer = null;
                }

                clearSignalHeartbeat();
                signalHeartbeatTimer = window.setInterval(() => {
                    if (signalWs === socket && socket.readyState === WebSocket.OPEN) {
                        socket.send("ping");
                    }
                }, SIGNAL_HEARTBEAT_INTERVAL_MS);

                console.log("📡 AUC Signal Engine Connected");
                updateSignalConnectionStatus("● AUC LIVE", "#0f0");
                resizeCanvas();
            };

            socket.onmessage = (event) => {
                if (signalWs !== socket) return;

                try {
                    const msg = JSON.parse(event.data);
                    if (msg.type === "mixed_signal" || msg.type === "signal") {
                        // 1. 处理磁信号 (Ch1) -> 送入 AUC 算法 & 绘图缓存
                        if (msg.mag_data && msg.snd_data) {
                            channel1Data = msg.mag_data;
                            channel2Data = msg.snd_data;
                            processSignals(msg.mag_data, msg.snd_data);
                        } else if (msg.ch1 && msg.ch2) {
                            channel1Data = msg.ch1;
                            channel2Data = msg.ch2;
                            processSignals(msg.ch1, msg.ch2);
                        }
                    }
                } catch (e) {
                    console.error("Data Parse Error", e);
                }
            };

            socket.onerror = (err) => {
                if (signalWs !== socket) return;

                console.warn("声磁 WebSocket 连接异常:", err);
                updateSignalConnectionStatus("● ERROR", "#f00");
                socket.close();
            };

            socket.onclose = () => {
                if (signalWs !== socket) return;

                signalWs = null;
                clearSignalHeartbeat();
                updateSignalConnectionStatus("● OFFLINE", "#f00");
                scheduleSignalReconnect();
            };
        }

        connectSignalWebSocket();
    })();

    // ==========================================
    // I. 自动巡线按钮逻辑
    // ==========================================
    const btnAutoTrack = document.getElementById("btnAutoTrack");

    // 定义状态：是否正在巡线
    let isAutoTracking = false;

    if (btnAutoTrack) {
        // 使用 onclick 避免重复绑定
        btnAutoTrack.onclick = function () {
            // 1. 切换状态变量
            isAutoTracking = !isAutoTracking;

            // 2. 发送请求给后端 Spring Boot
            // 注意：这里需要你先创建 PipelineController (见第3步)
            const url = `/api/pipeline/toggle?enable=${isAutoTracking}`;

            console.log(`🚀 发送自动巡线指令: ${isAutoTracking}`);

            fetch(url, { method: "POST" })
                .then(res => res.text())
                .then(msg => {
                    console.log("后端反馈:", msg);

                    // 3. 根据状态更新按钮 UI
                    if (isAutoTracking) {
                        // === 开启状态 ===
                        // 变红/变橙，文字改为“停止”
                        this.innerHTML = '<span class="btn-icon">🛑</span> 停止巡线';
                        this.classList.add("active"); // 激活样式
                        this.classList.add("pulsing"); // (可选) 添加呼吸灯动画效果

                        setStatus("自动巡线模式：运行中...", "success");
                        const lbl = document.getElementById("currentModeLabel");
                        if (lbl) {
                            lbl.textContent = "⚡ 自动巡线中";
                            lbl.style.color = "#ffaa00"; // 橙色高亮
                        }
                    } else {
                        // === 关闭状态 ===
                        // 恢复默认，文字改为“启动”
                        this.innerHTML = '<span class="btn-icon">📡</span> 启动自动巡线';
                        this.classList.remove("active");
                        this.classList.remove("pulsing");

                        setStatus("自动巡线已停止", "warning");
                        // ✅ 新增：恢复显示
                        const lbl = document.getElementById("currentModeLabel");
                        if (lbl) {
                            lbl.textContent = "运动 (常规)";
                            lbl.style.color = "#00ff00";
                        }
                    }
                })
                .catch(err => {
                    console.error("请求失败:", err);
                    setStatus("连接后端失败，无法切换模式", "error");
                    // 回滚状态
                    isAutoTracking = !isAutoTracking;
                });
        };
    }
    // ==========================================
    //  新增：管线仪箭头显示控制逻辑
    // ==========================================
    /**
     * 更新管线仪界面显示
     * @param {number} signal - 0=正中, 1=需向右走, 2=需向左走
     */
    function updatePipelineDisplay(signal) {
        var arrowLeft = document.getElementById('arrowLeft');
        var arrowRight = document.getElementById('arrowRight');

        // 防止页面还没加载完找不到元素报错
        if (!arrowLeft || !arrowRight) return;

        // 1. 先重置所有状态 (变暗)
        arrowLeft.className = 'arrow-icon';
        arrowRight.className = 'arrow-icon';

        // 2. 根据信号点亮对应箭头
        if (signal === 1) {
            // [信号 1] 机器狗需要向右走 -> 显示右箭头 (红色)
            arrowRight.classList.add('active-right');

        } else if (signal === 2) {
            // [信号 2] 机器狗需要向左走 -> 显示左箭头 (红色)
            arrowLeft.classList.add('active-left');

        } else if (signal === 0) {
            // [信号 0] 正上方 -> 左右都显示 (绿色)
            arrowLeft.classList.add('active-center');
            arrowRight.classList.add('active-center');
        }
    }
    // ==========================================
    // ✅ 全局急停逻辑 (Binding STOP Buttons)
    // ==========================================

    // 1. 定义急停函数
    function emergencyStop() {
        console.warn("🚨 触发急停 (Emergency Stop)！");

        // --- A. 停止机器狗运动 (物理急停) ---
        // 直接复用已有的 WebSocket 发送函数，发送全 0 指令
        // 参数：v=0, w=0, y=0, mode=0.0 (保持当前模式或强制空闲), showLog=true
        if (typeof sendMoveCommand === 'function') {
            sendMoveCommand(0.0, 0.0, 0.0, 0.0, 0.0, true);

            // 双重保险：50ms 后再发一次，防止丢包
            setTimeout(() => {
                sendMoveCommand(0.0, 0.0, 0.0, 0.0, 0.0, false);
            }, 50);
        } else {
            console.error("❌ 无法调用 sendMoveCommand，请检查代码作用域");
        }

        // --- B. 停止“寻找破损点”模式 (逻辑急停) ---
        if (typeof window.stopFaultFindingMode === 'function') {
            const stoppedFaultFinding = window.stopFaultFindingMode("emergency");
            if (stoppedFaultFinding) {
                setStatus("急停：已停止寻找破损点模式", "warning");
            }
        }

        // --- C. 重置“自动巡线”状态 (逻辑急停) ---
        const btnAutoTrack = document.getElementById('btnAutoTrack');

        if (btnAutoTrack) {
            // 检查当前是否处于“自动巡线运行中”
            // 判断依据：按钮上有 'active' 类，或者文字显示“停止”
            const isTracking = btnAutoTrack.classList.contains('active') ||
                btnAutoTrack.innerText.includes("停止");

            if (isTracking) {
                console.log("🔄 检测到自动巡线正在运行，强制复位...");

                // 模拟点击按钮
                // 这会触发你上面写的 btnAutoTrack.onclick 逻辑：
                // 1. 发送 /api/pipeline/toggle?enable=false 给后端
                // 2. 将按钮 UI 恢复为绿色“启动”状态
                btnAutoTrack.click();
            }
        }
    }

    // 2. 绑定按钮事件 (页面加载完成后执行)
    document.addEventListener('DOMContentLoaded', function () {

        // ==========================================
        // ✅ 新增：速度调节弹窗控制逻辑
        // ==========================================
        const btnSpeedSettings = document.getElementById("btnSpeedSettings");
        const speedSettingsPopup = document.getElementById("speedSettingsPopup");
        const btnCloseSpeedPopup = document.getElementById("btnCloseSpeedPopup");

        if (btnSpeedSettings && speedSettingsPopup) {
            // 点击按钮切换弹窗显隐
            btnSpeedSettings.addEventListener("click", function (e) {
                e.stopPropagation(); // 阻止事件冒泡
                speedSettingsPopup.classList.toggle("hidden");

                // 🌟 修复：因为 input range 之前处于 hidden 状态（宽度为0），
                // 浮动 tooltip 可能会算错位置，这里在显示时强制重新计算一次位置
                if (!speedSettingsPopup.classList.contains("hidden")) {
                    setTimeout(() => {
                        ['manualSpeedX', 'manualSpeedY', 'autoSpeedX', 'autoSpeedY'].forEach(id => {
                            const input = document.getElementById(id);
                            if (input) {
                                let tipId = '';
                                if (id === 'manualSpeedX') tipId = 'valManX';
                                if (id === 'manualSpeedY') tipId = 'valManY';
                                if (id === 'autoSpeedX') tipId = 'valAutoX';
                                if (id === 'autoSpeedY') tipId = 'valAutoY';
                                window.updateFloatingThumb(input, tipId);
                            }
                        });
                    }, 10);
                }
            });

            // 点击右上角 X 关闭
            if (btnCloseSpeedPopup) {
                btnCloseSpeedPopup.addEventListener("click", function () {
                    speedSettingsPopup.classList.add("hidden");
                });
            }

            // 点击屏幕其他区域自动关闭弹窗
            document.addEventListener("click", function (e) {
                if (!speedSettingsPopup.classList.contains("hidden") &&
                    !speedSettingsPopup.contains(e.target) &&
                    e.target !== btnSpeedSettings) {
                    speedSettingsPopup.classList.add("hidden");
                }
            });

            // 阻止点击弹窗内部时触发全局关闭
            speedSettingsPopup.addEventListener("click", function (e) {
                e.stopPropagation();
            });
        }
        // 获取两个 STOP 按钮
        const btnSideStop = document.getElementById('btnSideStop'); // 侧边栏 STOP
        const btnStopMove = document.getElementById('btnStopMove'); // 控制盘 STOP

        // 绑定点击事件
        // 注意：这两个按钮在上面的 motionBtnIds 里已经绑定了 mousedown/mouseup 用于停止手动运动
        // 这里绑定 click 是为了处理“自动巡线”状态的复位
        if (btnSideStop) {
            btnSideStop.addEventListener('click', function (e) {
                e.preventDefault();
                emergencyStop();
            });
        }

        if (btnStopMove) {
            btnStopMove.addEventListener('click', function (e) {
                e.preventDefault();
                emergencyStop();
            });
        }

        // 绑定空格键作为急停快捷键
        document.addEventListener('keydown', function (event) {
            if (event.code === 'Space') {
                // 如果当前焦点不在输入框里，才触发急停
                if (document.activeElement.tagName !== 'INPUT') {
                    console.log("⌨️ 检测到空格键，触发急停");
                    event.preventDefault(); // 防止页面滚动
                    emergencyStop();
                }
            }
        });

        // ==========================================
        // ✅ 键盘控制绑定 (WASD + QE)
        // ==========================================
        const keyMap = {
            'KeyW': 'btnSideMoveForward',
            'KeyS': 'btnSideMoveBackward',
            'KeyA': 'btnSideTurnLeft',
            'KeyD': 'btnSideTurnRight',
            'KeyQ': 'btnSideStrafeLeft',
            'KeyE': 'btnSideStrafeRight'
        };

        const pressedKeys = new Set();
        let keyboardTimer = null;

        // ✅ 新增：处理键盘组合运动逻辑
        function updateKeyboardMotion() {
            // 1. 获取速度设定
            const elSpeedX = document.getElementById('manualSpeedX');
            const elSpeedY = document.getElementById('manualSpeedY');
            const manualX = elSpeedX ? (parseFloat(elSpeedX.value) || 0.5) : 0.5;
            const manualY = elSpeedY ? (parseFloat(elSpeedY.value) || 0.3) : 0.3;
            const manualW = 1.0;

            let v = 0.0, w = 0.0, y = 0.0;

            // 2. 根据按键叠加速度
            if (pressedKeys.has('KeyW')) v += manualX;
            if (pressedKeys.has('KeyS')) v -= manualX;
            if (pressedKeys.has('KeyA')) w += manualW;
            if (pressedKeys.has('KeyD')) w -= manualW;
            if (pressedKeys.has('KeyQ')) y += manualY;
            if (pressedKeys.has('KeyE')) y -= manualY;

            // 3. 发送指令 (仅当有按键按下时在此处发送，松开逻辑在 keyup 处理)
            if (pressedKeys.size > 0) {
                sendMoveCommand(v, w, y, 0.0, 0.0, false);
            }
        }

        document.addEventListener('keydown', function (event) {
            // 忽略输入框内的按键
            if (document.activeElement.tagName === 'INPUT' || document.activeElement.tagName === 'TEXTAREA') return;

            const btnId = keyMap[event.code];
            if (btnId) {
                if (!pressedKeys.has(event.code)) {
                    pressedKeys.add(event.code);
                    const btn = document.getElementById(btnId);
                    if (btn) btn.classList.add('active');

                    // ✅ 核心修改：不再调用单任务的 handleMotion，而是启动组合定时器
                    if (!keyboardTimer) {
                        updateKeyboardMotion(); // 立即响应第一帧
                        keyboardTimer = setInterval(updateKeyboardMotion, 150);
                    } else {
                        updateKeyboardMotion(); // 状态变化，立即刷新
                    }
                }
            }
        });

        document.addEventListener('keyup', function (event) {
            const btnId = keyMap[event.code];
            if (btnId) {
                if (pressedKeys.has(event.code)) {
                    pressedKeys.delete(event.code);
                    const btn = document.getElementById(btnId);
                    if (btn) btn.classList.remove('active');

                    // ✅ 核心修改：检查剩余按键
                    if (pressedKeys.size === 0) {
                        // 全部松开，停止定时器并发送停车
                        if (keyboardTimer) {
                            clearInterval(keyboardTimer);
                            keyboardTimer = null;
                        }
                        sendMoveCommand(0.0, 0.0, 0.0, 0.0, 0.0, true);
                    } else {
                        // 还有其他键，更新运动状态
                        updateKeyboardMotion();
                    }
                }
            }
        });
    });
    // ==========================================
    // II. SLAM 3D 可视化 (A*避障规划 + 纯追踪平滑曲线 + 误差计算 + 强度热力图 + 卡尔曼滤波 + 电缆跟踪)
    // ==========================================
    (function initSlamVis() {
        const container = document.getElementById("pathMapArea");
        if (!container) { setTimeout(initSlamVis, 500); return; }

        container.style.overflow = "hidden";
        container.style.background = "#111";
        container.style.position = "relative"; // 确保子元素绝对定位相对于此容器

        // scanValidComponents / mapCursor / mapStoredComponents 的单位均为 Float32 分量数，
        // 不是点数。统一单位可避免规划器把历史点云长度再次乘 3。
        let scanValidComponents = 0;
        let mapCursor = 0;
        let mapStoredComponents = 0;
        let lastKeyframePose = { x: -999, y: -999 };
        let shouldNotifyPointCloudReceivedAfterStart = false;

        // ✅ 新增：用于连续破损区域追踪的状态变量
        window.lastFaultMarker = null;    // 🌟 修复：将其挂载到 window 变为全局变量
        let lastFaultCablePos = null;  // 存储上一次有效破损点的坐标 (Vector3)
        let maxFaultIdx = -1;          // 🌟 新增：记录当前探测到的最远电缆点索引

        // ==========================================
        // 🌟🌟🌟 [新增] 二分法精准定位核心状态与工具函数
        // ==========================================
        let bsState = 'INACTIVE'; // 状态: INACTIVE, INIT_WAIT, MOVING, EVAL_WAIT, DONE
        let bsGoodDist = 0;       // 有信号的距离 (P_good)
        let bsBadDist = 0;        // 无信号的距离 (P_bad)
        let bsTargetDist = 0;     // 当前机器狗需要前往的目标距离
        let bsWaitEndTime = 0;    // 5秒等待结束的时间戳
        const BS_MAX_ITERATIONS = 3; // 进入二分法后最多评估 3 次
        let bsIterationCount = 0;    // 已完成的二分法评估次数
        let lastFaultRobotDist = -1; // 上一次正常测到信号时，机器狗在偏移路径上的行驶距离
        let offsetPathDists = [];    // 缓存偏移路径上每个点到起点的累加距离
        let bsFootprints = [];       // ✅ 新增：用于记录机器狗前进时的真实历史脚印

        let bsLastPoseStaleWarningTime = 0;
        const BS_PATH_CONTROL = Object.freeze({
            lookaheadM: 0.45,
            localProjectionWindowM: 2.0,
            arriveDistM: 0.10,
            arriveCrossTrackM: 0.20,
            minSpeed: 0.06,
            maxForwardSpeed: 0.22,
            maxReverseSpeed: 0.16,
            distanceKp: 0.85,
            yawDeadbandRad: 5 * Math.PI / 180,
            yawKp: 1.05,
            maxYawCommand: 0.35,
            alignEnterRad: 24 * Math.PI / 180,
            alignExitRad: 10 * Math.PI / 180,
            crossTrackDeadbandM: 0.025,
            crossTrackKp: 0.85,
            maxStrafeCommand: 0.12,
            poseTimeoutMs: 400
        });
        let bsLastProjectionS = null;
        let bsPathAligning = false;

        // 1. 新增状态变量
        let isManualDrawingCable = false;

        // 2. 绑定按钮点击事件
        const btnManualDraw = document.getElementById('btnManualDrawCable');
        if (btnManualDraw) {
            btnManualDraw.onclick = function () {
                isManualDrawingCable = !isManualDrawingCable;
                if (isManualDrawingCable) {
                    this.innerText = "⏹ 结束绘制";
                    this.classList.add('drawing-active');
                    setStatus("手动绘制模式：当前路径记录为电缆", "success");
                } else {
                    this.innerText = "📍 手动绘制电缆";
                    this.classList.remove('drawing-active');
                    setStatus("手动绘制结束", "warning");
                }
            };
        }

        const btnStartPointCloudLidar = document.getElementById('btnStartPointCloudLidar');
        const btnStopPointCloudLidar = document.getElementById('btnStopPointCloudLidar');

        function setLidarControlButtonsDisabled(disabled) {
            if (btnStartPointCloudLidar) btnStartPointCloudLidar.disabled = disabled;
            if (btnStopPointCloudLidar) btnStopPointCloudLidar.disabled = disabled;
        }

        if (btnStartPointCloudLidar) {
            btnStartPointCloudLidar.addEventListener('click', async function () {
                const oldText = btnStartPointCloudLidar.innerText;
                setLidarControlButtonsDisabled(true);
                btnStartPointCloudLidar.innerText = "⏳ 启动中...";
                setStatus("正在向点云激光雷达发送启动指令...", "warning");

                try {
                    const res = await fetch("/api/lidar/start", { method: "POST" });
                    const data = await res.json().catch(() => ({}));

                    if (!res.ok || data.success === false) {
                        throw new Error(data.message || ("HTTP " + res.status));
                    }

                    btnStartPointCloudLidar.innerText = "✅ 雷达已启动";
                    shouldNotifyPointCloudReceivedAfterStart = true;
                    setStatus("点云激光雷达已成功启动", "success");

                    setTimeout(() => {
                        btnStartPointCloudLidar.innerText = oldText || "📡 启动点云雷达";
                    }, 2000);
                } catch (err) {
                    shouldNotifyPointCloudReceivedAfterStart = false;
                    console.error("点云激光雷达启动失败", err);
                    btnStartPointCloudLidar.innerText = "❌ 启动失败";
                    setStatus("点云激光雷达启动失败", "error");
                    alert("点云激光雷达启动失败：" + (err.message || err));

                    setTimeout(() => {
                        btnStartPointCloudLidar.innerText = oldText || "📡 启动点云雷达";
                    }, 2000);
                } finally {
                    setLidarControlButtonsDisabled(false);
                }
            });
        }

        if (btnStopPointCloudLidar) {
            btnStopPointCloudLidar.addEventListener('click', async function () {
                const oldText = btnStopPointCloudLidar.innerText;
                setLidarControlButtonsDisabled(true);
                btnStopPointCloudLidar.innerText = "⏳ 关闭中...";
                setStatus("正在向点云激光雷达发送关闭指令...", "warning");

                try {
                    const res = await fetch("/api/lidar/stop", { method: "POST" });
                    const data = await res.json().catch(() => ({}));

                    if (!res.ok || data.success === false) {
                        throw new Error(data.message || ("HTTP " + res.status));
                    }

                    shouldNotifyPointCloudReceivedAfterStart = false;
                    btnStopPointCloudLidar.innerText = "✅ 雷达已关闭";
                    setStatus("点云激光雷达已关闭，点云扫描已结束", "success");

                    setTimeout(() => {
                        btnStopPointCloudLidar.innerText = oldText || "⏹ 关闭点云雷达";
                    }, 2000);
                } catch (err) {
                    console.error("点云激光雷达关闭失败", err);
                    btnStopPointCloudLidar.innerText = "❌ 关闭失败";
                    setStatus("点云激光雷达关闭失败", "error");
                    alert("点云激光雷达关闭失败：" + (err.message || err));

                    setTimeout(() => {
                        btnStopPointCloudLidar.innerText = oldText || "⏹ 关闭点云雷达";
                    }, 2000);
                } finally {
                    setLidarControlButtonsDisabled(false);
                }
            });
        }

        function clampNumber(value, min, max) {
            const n = Number(value);
            if (!Number.isFinite(n)) return min;
            return Math.max(min, Math.min(max, n));
        }

        // 1. 计算整条路径的距离积分
        function computePathDists(path) {
            offsetPathDists = [0];
            let d = 0;
            for (let i = 1; i < path.length; i++) {
                d += Math.hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
                offsetPathDists.push(d);
            }
        }

        function projectPoseToPath(pose, path, options = {}) {
            if (!pose || !Array.isArray(path) || path.length < 2) return null;
            if (offsetPathDists.length !== path.length) computePathDists(path);

            const robotX = Number(pose.x);
            const robotY = Number(pose.y);
            if (!Number.isFinite(robotX) || !Number.isFinite(robotY)) return null;

            const nearS = Number(options.nearS);
            const windowM = Number(options.windowM);
            const hasLocalWindow = Number.isFinite(nearS) && Number.isFinite(windowM) && windowM > 0;
            const minS = hasLocalWindow ? nearS - windowM : -Infinity;
            const maxS = hasLocalWindow ? nearS + windowM : Infinity;

            let best = null;

            for (let pass = 0; pass < 2; pass++) {
                const useLocalWindow = pass === 0 && hasLocalWindow;

                for (let i = 0; i < path.length - 1; i++) {
                    const segStartS = offsetPathDists[i];
                    const segEndS = offsetPathDists[i + 1];
                    if (useLocalWindow && (segEndS < minS || segStartS > maxS)) continue;

                    const p1 = path[i];
                    const p2 = path[i + 1];
                    const x1 = Number(p1.x);
                    const y1 = Number(p1.y);
                    const x2 = Number(p2.x);
                    const y2 = Number(p2.y);
                    if (!Number.isFinite(x1) || !Number.isFinite(y1) ||
                        !Number.isFinite(x2) || !Number.isFinite(y2)) {
                        continue;
                    }

                    const dx = x2 - x1;
                    const dy = y2 - y1;
                    const len2 = dx * dx + dy * dy;
                    if (len2 < 1e-8) continue;

                    const segLen = Math.sqrt(len2);
                    const t = clampNumber(
                        ((robotX - x1) * dx + (robotY - y1) * dy) / len2,
                        0,
                        1
                    );
                    const projX = x1 + t * dx;
                    const projY = y1 + t * dy;
                    const errX = robotX - projX;
                    const errY = robotY - projY;
                    const distSq = errX * errX + errY * errY;
                    const s = segStartS + t * segLen;
                    const localPenalty = hasLocalWindow ? Math.abs(s - nearS) * 0.0001 : 0;
                    const score = distSq + localPenalty;

                    if (!best || score < best.score) {
                        const tx = dx / segLen;
                        const ty = dy / segLen;
                        best = {
                            x: projX,
                            y: projY,
                            s,
                            yaw: Math.atan2(dy, dx),
                            crossTrackM: tx * errY - ty * errX,
                            segmentIndex: i,
                            t,
                            distM: Math.sqrt(distSq),
                            score
                        };
                    }
                }

                if (best) return best;
            }

            return null;
        }

        function getCurrentOffsetPathProjection() {
            if (!window.offsetCablePath || window.offsetCablePath.length < 2) return null;
            const projection = projectPoseToPath(currentRobotPose, window.offsetCablePath, {
                nearS: bsLastProjectionS,
                windowM: BS_PATH_CONTROL.localProjectionWindowM
            });
            if (projection) bsLastProjectionS = projection.s;
            return projection;
        }

        // 2. 计算当前坐标在路径上所处的“里程数” (距离起点的距离)
        function getDistOnPath(pose, path) {
            const projection = projectPoseToPath(pose, path);
            return projection ? projection.s : 0;
        }

        // 3. 根据给定的“里程数”，反推该点的 (x, y) 坐标
        function getPoseAtDist(targetDist, path) {
            if (!Array.isArray(path) || path.length === 0) return { x: 0, y: 0, yaw: 0 };
            if (path.length === 1) return { x: path[0].x, y: path[0].y, yaw: 0 };
            if (offsetPathDists.length !== path.length) computePathDists(path);

            const yawAtSegment = (idx) => {
                const safeIdx = Math.max(0, Math.min(path.length - 2, idx));
                return Math.atan2(
                    path[safeIdx + 1].y - path[safeIdx].y,
                    path[safeIdx + 1].x - path[safeIdx].x
                );
            };

            if (targetDist <= 0) return { x: path[0].x, y: path[0].y, yaw: yawAtSegment(0) };
            if (targetDist >= offsetPathDists[offsetPathDists.length - 1]) {
                return { x: path[path.length - 1].x, y: path[path.length - 1].y, yaw: yawAtSegment(path.length - 2) };
            }
            for (let i = 0; i < offsetPathDists.length - 1; i++) {
                if (targetDist >= offsetPathDists[i] && targetDist <= offsetPathDists[i + 1]) {
                    const segLen = offsetPathDists[i + 1] - offsetPathDists[i];
                    let t = 0;
                    if (segLen > 0) t = (targetDist - offsetPathDists[i]) / segLen;
                    return {
                        x: path[i].x + t * (path[i + 1].x - path[i].x),
                        y: path[i].y + t * (path[i + 1].y - path[i].y),
                        yaw: yawAtSegment(i)
                    };
                }
            }
            return { x: path[path.length - 1].x, y: path[path.length - 1].y, yaw: yawAtSegment(path.length - 2) };
        }

        function resetBinaryMotionController() {
            window.bsInitialMoveDir = undefined;
            bsPathAligning = false;
        }

        function initializeBinaryMotionController(moveDirection) {
            resetBinaryMotionController();
            window.bsInitialMoveDir = moveDirection;
        }

        function isBinaryPoseFresh(now = Date.now()) {
            return Number.isFinite(Number(currentRobotPose.x)) &&
                Number.isFinite(Number(currentRobotPose.y)) &&
                Number.isFinite(Number(currentRobotPose.yaw)) &&
                Number.isFinite(Number(currentRobotPose.updatedAt)) &&
                now - Number(currentRobotPose.updatedAt) <= BS_PATH_CONTROL.poseTimeoutMs;
        }

        function computeBinaryPathFollowCommand(targetDist, projection) {
            const path = window.offsetCablePath;
            if (!projection || !Array.isArray(path) || path.length < 2 || offsetPathDists.length !== path.length) {
                return null;
            }

            const totalDist = offsetPathDists[offsetPathDists.length - 1] || 0;
            const targetS = clampNumber(targetDist, 0, totalDist);
            const currentS = projection.s;
            const distError = targetS - currentS;
            const absDistError = Math.abs(distError);

            let moveDir = Math.sign(distError);
            if (moveDir === 0 && typeof window.bsInitialMoveDir === 'number') {
                moveDir = window.bsInitialMoveDir;
            }
            if (moveDir === 0) moveDir = 1;

            const lookahead = Math.min(
                BS_PATH_CONTROL.lookaheadM,
                Math.max(absDistError, BS_PATH_CONTROL.arriveDistM)
            );
            const referenceS = clampNumber(currentS + moveDir * lookahead, 0, totalDist);
            const referencePose = getPoseAtDist(referenceS, path);
            const pathYaw = Number.isFinite(referencePose.yaw) ? referencePose.yaw : projection.yaw;
            const robotYaw = Number(currentRobotPose.yaw);
            if (!Number.isFinite(pathYaw) || !Number.isFinite(robotYaw)) return null;

            const yawError = normalizeAngleRad(pathYaw - robotYaw);
            const absYawError = Math.abs(yawError);

            if (absYawError >= BS_PATH_CONTROL.alignEnterRad) {
                bsPathAligning = true;
            } else if (absYawError <= BS_PATH_CONTROL.alignExitRad) {
                bsPathAligning = false;
            }

            let w = 0.0;
            if (absYawError > BS_PATH_CONTROL.yawDeadbandRad) {
                w = clampNumber(
                    BS_PATH_CONTROL.yawKp * yawError,
                    -BS_PATH_CONTROL.maxYawCommand,
                    BS_PATH_CONTROL.maxYawCommand
                );
            }

            let v = 0.0;
            if (!bsPathAligning && absDistError > BS_PATH_CONTROL.arriveDistM) {
                const maxSpeed = moveDir > 0
                    ? BS_PATH_CONTROL.maxForwardSpeed
                    : BS_PATH_CONTROL.maxReverseSpeed;
                const baseSpeed = clampNumber(
                    BS_PATH_CONTROL.distanceKp * absDistError,
                    BS_PATH_CONTROL.minSpeed,
                    maxSpeed
                );
                const yawScale = clampNumber(
                    1.0 - 0.65 * absYawError / BS_PATH_CONTROL.alignEnterRad,
                    0.35,
                    1.0
                );
                const lateralScale = clampNumber(
                    1.0 - 0.55 * Math.abs(projection.crossTrackM) / 0.45,
                    0.45,
                    1.0
                );
                v = moveDir * baseSpeed * yawScale * lateralScale;
            }

            let y = 0.0;
            const crossTrack = Math.abs(projection.crossTrackM) > BS_PATH_CONTROL.crossTrackDeadbandM
                ? projection.crossTrackM
                : 0.0;
            if (!bsPathAligning && crossTrack !== 0.0) {
                const pathLeftX = -Math.sin(pathYaw);
                const pathLeftY = Math.cos(pathYaw);
                const bodyLeftX = -Math.sin(robotYaw);
                const bodyLeftY = Math.cos(robotYaw);
                const correctionX = -BS_PATH_CONTROL.crossTrackKp * crossTrack * pathLeftX;
                const correctionY = -BS_PATH_CONTROL.crossTrackKp * crossTrack * pathLeftY;
                y = clampNumber(
                    correctionX * bodyLeftX + correctionY * bodyLeftY,
                    -BS_PATH_CONTROL.maxStrafeCommand,
                    BS_PATH_CONTROL.maxStrafeCommand
                );
            }

            return {
                v,
                w,
                y,
                targetS,
                currentS,
                distError,
                crossTrackM: projection.crossTrackM,
                yawError
            };
        }

        // ==========================================
        // ✅ 寻找破损点：以机器狗当前朝向决定电缆搜索方向
        // 说明：savedCablePath 的绘制顺序可能是 A->B，但现场可以从 B 点朝 A 点开始寻找。
        // 这里在点击“寻找破损点”瞬间，根据机器狗朝向与最近电缆切向的夹角，自动选择正向或反向搜索。
        // ==========================================
        function getClosestPathIndex2D(path, pose) {
            if (!Array.isArray(path) || path.length === 0) return 0;

            let closestIdx = 0;
            let minD = Infinity;

            for (let i = 0; i < path.length; i++) {
                const d = Math.hypot(path[i].x - pose.x, path[i].y - pose.y);
                if (d < minD) {
                    minD = d;
                    closestIdx = i;
                }
            }

            return closestIdx;
        }

        function getPathTangent2D(path, idx) {
            if (!Array.isArray(path) || path.length < 2) {
                return { x: 1, y: 0 };
            }

            const safeIdx = Math.max(0, Math.min(path.length - 1, idx));
            let prevIdx = safeIdx;
            let nextIdx = safeIdx;

            // 向前、向后各找一个有效点，避免相邻重复点导致方向向量为 0。
            while (prevIdx > 0) {
                const d = Math.hypot(path[safeIdx].x - path[prevIdx - 1].x, path[safeIdx].y - path[prevIdx - 1].y);
                prevIdx--;
                if (d > 0.03) break;
            }
            while (nextIdx < path.length - 1) {
                const d = Math.hypot(path[nextIdx + 1].x - path[safeIdx].x, path[nextIdx + 1].y - path[safeIdx].y);
                nextIdx++;
                if (d > 0.03) break;
            }

            let dx = path[nextIdx].x - path[prevIdx].x;
            let dy = path[nextIdx].y - path[prevIdx].y;
            let len = Math.hypot(dx, dy);

            if (len < 1e-6) {
                if (safeIdx < path.length - 1) {
                    dx = path[safeIdx + 1].x - path[safeIdx].x;
                    dy = path[safeIdx + 1].y - path[safeIdx].y;
                } else {
                    dx = path[safeIdx].x - path[safeIdx - 1].x;
                    dy = path[safeIdx].y - path[safeIdx - 1].y;
                }
                len = Math.hypot(dx, dy);
            }

            if (len < 1e-6) return { x: 1, y: 0 };
            return { x: dx / len, y: dy / len };
        }

        function buildFaultSearchPathByRobotHeading(rawPath, robotPose) {
            const cleanPath = (rawPath || [])
                .map(p => ({ x: Number(p.x), y: Number(p.y) }))
                .filter(p => Number.isFinite(p.x) && Number.isFinite(p.y));

            if (cleanPath.length < 2) {
                throw new Error("未检测到有效的电缆路径！请先进行自动巡线生成路径。");
            }

            const closestIdxInDrawPath = getClosestPathIndex2D(cleanPath, robotPose);
            const tangent = getPathTangent2D(cleanPath, closestIdxInDrawPath);
            const robotYaw = Number.isFinite(Number(robotPose.yaw)) ? Number(robotPose.yaw) : 0;
            const robotForward = {
                x: Math.cos(robotYaw),
                y: Math.sin(robotYaw)
            };

            const headingDot = tangent.x * robotForward.x + tangent.y * robotForward.y;
            const shouldReverse = headingDot < 0;
            const pathPoints = shouldReverse ? cleanPath.slice().reverse() : cleanPath.slice();
            const closestIdxInSearchPath = shouldReverse
                ? pathPoints.length - 1 - closestIdxInDrawPath
                : closestIdxInDrawPath;

            console.log(
                `🧭 [寻找破损点] 当前朝向判定：${shouldReverse ? "反向搜索(B→A)" : "正向搜索(A→B)"}`,
                `headingDot=${headingDot.toFixed(3)}, closestIdx=${closestIdxInSearchPath}`
            );

            return {
                pathPoints,
                closestIdx: closestIdxInSearchPath,
                isReversed: shouldReverse,
                headingDot
            };
        }

        // ==========================================
        // 0. 卡尔曼滤波与矩阵运算工具 (移植自 Python)
        // ==========================================
        const KalmanSmoother = {
            // --- 基础矩阵运算 ---
            zeros: (r, c) => new Float64Array(r * c),
            eye: (n) => {
                const m = new Float64Array(n * n);
                for (let i = 0; i < n; i++) m[i * n + i] = 1.0;
                return m;
            },
            matmul: (A, rowsA, colsA, B, rowsB, colsB) => {
                if (colsA !== rowsB) throw new Error("Matrix dimension mismatch");
                const C = new Float64Array(rowsA * colsB);
                for (let i = 0; i < rowsA; i++) {
                    for (let j = 0; j < colsB; j++) {
                        let sum = 0.0;
                        for (let k = 0; k < colsA; k++) {
                            sum += A[i * colsA + k] * B[k * colsB + j];
                        }
                        C[i * colsB + j] = sum;
                    }
                }
                return C;
            },
            add: (A, B) => {
                const C = new Float64Array(A.length);
                for (let i = 0; i < A.length; i++) C[i] = A[i] + B[i];
                return C;
            },
            sub: (A, B) => {
                const C = new Float64Array(A.length);
                for (let i = 0; i < A.length; i++) C[i] = A[i] - B[i];
                return C;
            },
            transpose: (A, rows, cols) => {
                const T = new Float64Array(rows * cols);
                for (let i = 0; i < rows; i++) {
                    for (let j = 0; j < cols; j++) {
                        T[j * rows + i] = A[i * cols + j];
                    }
                }
                return T;
            },
            inv2x2: (M) => {
                const det = M[0] * M[3] - M[1] * M[2];
                if (Math.abs(det) < 1e-10) return M;
                const invDet = 1.0 / det;
                return new Float64Array([
                    M[3] * invDet, -M[1] * invDet,
                    -M[2] * invDet, M[0] * invDet
                ]);
            },
            inv4x4: (m) => {
                const inv = new Float64Array(16);
                const a = m;
                const a00 = a[0], a01 = a[1], a02 = a[2], a03 = a[3];
                const a10 = a[4], a11 = a[5], a12 = a[6], a13 = a[7];
                const a20 = a[8], a21 = a[9], a22 = a[10], a23 = a[11];
                const a30 = a[12], a31 = a[13], a32 = a[14], a33 = a[15];
                const b00 = a00 * a11 - a01 * a10;
                const b01 = a00 * a12 - a02 * a10;
                const b02 = a00 * a13 - a03 * a10;
                const b03 = a01 * a12 - a02 * a11;
                const b04 = a01 * a13 - a03 * a11;
                const b05 = a02 * a13 - a03 * a12;
                const b06 = a20 * a31 - a21 * a30;
                const b07 = a20 * a32 - a22 * a30;
                const b08 = a20 * a33 - a23 * a30;
                const b09 = a21 * a32 - a22 * a31;
                const b10 = a21 * a33 - a23 * a31;
                const b11 = a22 * a33 - a23 * a32;
                let det = b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;
                if (Math.abs(det) < 1e-10) return m;
                det = 1.0 / det;
                inv[0] = (a11 * b11 - a12 * b10 + a13 * b09) * det;
                inv[1] = (a02 * b10 - a01 * b11 - a03 * b09) * det;
                inv[2] = (a31 * b05 - a32 * b04 + a33 * b03) * det;
                inv[3] = (a22 * b04 - a21 * b05 - a23 * b03) * det;
                inv[4] = (a12 * b08 - a10 * b11 - a13 * b07) * det;
                inv[5] = (a00 * b11 - a02 * b08 + a03 * b07) * det;
                inv[6] = (a32 * b02 - a30 * b05 - a33 * b01) * det;
                inv[7] = (a20 * b05 - a22 * b02 + a23 * b01) * det;
                inv[8] = (a10 * b10 - a11 * b08 + a13 * b06) * det;
                inv[9] = (a01 * b08 - a00 * b10 - a03 * b06) * det;
                inv[10] = (a30 * b04 - a31 * b02 + a33 * b00) * det;
                inv[11] = (a21 * b02 - a20 * b04 - a23 * b00) * det;
                inv[12] = (a11 * b07 - a10 * b09 - a12 * b06) * det;
                inv[13] = (a00 * b09 - a01 * b07 + a02 * b06) * det;
                inv[14] = (a31 * b01 - a30 * b03 - a32 * b00) * det;
                inv[15] = (a20 * b03 - a21 * b01 + a22 * b00) * det;
                return inv;
            },

            // --- 卡尔曼平滑主逻辑 ---
            smooth: function (points) {
                const n = points.length;
                if (n < 3) return points;

                const x_data = points.map(p => p.x);
                const y_data = points.map(p => p.y);

                const dt = 1.0;
                const process_noise = 0.05;
                const measurement_noise = 0.3;

                // 初始化矩阵
                const F = new Float64Array([1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1]);
                const FT = this.transpose(F, 4, 4);
                const H = new Float64Array([1, 0, 0, 0, 0, 1, 0, 0]);
                const HT = this.transpose(H, 2, 4);

                const Q = this.eye(4);
                for (let i = 0; i < 16; i++) Q[i] *= process_noise;
                const R = this.eye(2);
                for (let i = 0; i < 4; i++) R[i] *= measurement_noise;

                let x_est = new Float64Array([x_data[0], y_data[0], 0, 0]);
                let P = this.eye(4);

                const x_filtered = new Array(n);
                const P_filtered = new Array(n);
                const P_predicted = new Array(n);

                // 前向滤波
                for (let i = 0; i < n; i++) {
                    const x_pred = this.matmul(F, 4, 4, x_est, 4, 1);
                    const P_pred_temp = this.matmul(F, 4, 4, P, 4, 4);
                    const P_pred = this.add(this.matmul(P_pred_temp, 4, 4, FT, 4, 4), Q);

                    if (i === 0) {
                        x_est = new Float64Array([x_data[0], y_data[0], 0, 0]);
                        P = this.eye(4);
                        x_filtered[i] = x_est;
                        P_filtered[i] = P;
                        P_predicted[i] = P;
                        continue;
                    }
                    P_predicted[i] = P_pred;

                    const z = new Float64Array([x_data[i], y_data[i]]);
                    const H_x_pred = this.matmul(H, 2, 4, x_pred, 4, 1);
                    const y_res = this.sub(z, H_x_pred);
                    const H_P_pred = this.matmul(H, 2, 4, P_pred, 4, 4);
                    const S_temp = this.matmul(H_P_pred, 2, 4, HT, 4, 2);
                    const S = this.add(S_temp, R);
                    const S_inv = this.inv2x2(S);
                    const P_pred_HT = this.matmul(P_pred, 4, 4, HT, 4, 2);
                    const K = this.matmul(P_pred_HT, 4, 2, S_inv, 2, 2);
                    const K_y = this.matmul(K, 4, 2, y_res, 2, 1);
                    x_est = this.add(x_pred, K_y);
                    const K_H = this.matmul(K, 4, 2, H, 2, 4);
                    const I = this.eye(4);
                    const I_minus_KH = this.sub(I, K_H);
                    P = this.matmul(I_minus_KH, 4, 4, P_pred, 4, 4);

                    x_filtered[i] = x_est;
                    P_filtered[i] = P;
                }

                // 后向平滑 (RTS)
                const x_smoothed = new Array(n);
                x_smoothed[n - 1] = x_filtered[n - 1];

                for (let i = n - 2; i >= 0; i--) {
                    const x_filt = x_filtered[i];
                    const P_filt = P_filtered[i];
                    const P_pred_next = P_predicted[i + 1];
                    const x_pred_next = this.matmul(F, 4, 4, x_filt, 4, 1);
                    const inv_P_pred = this.inv4x4(P_pred_next);
                    const P_filt_FT = this.matmul(P_filt, 4, 4, FT, 4, 4);
                    const C = this.matmul(P_filt_FT, 4, 4, inv_P_pred, 4, 4);
                    const diff = this.sub(x_smoothed[i + 1], x_pred_next);
                    const correction = this.matmul(C, 4, 4, diff, 4, 1);
                    x_smoothed[i] = this.add(x_filt, correction);
                }

                const result = [];
                for (let i = 0; i < n; i++) {
                    result.push(new THREE.Vector3(x_smoothed[i][0], x_smoothed[i][1], 0.08));
                }
                return result;
            }
        };

        // --- 1. 初始化场景 ---
        const width = container.clientWidth || 400;
        const height = container.clientHeight || 300;
        const scene = new THREE.Scene();
        scene.fog = new THREE.FogExp2(0x111111, 0.02);

        const camera = new THREE.PerspectiveCamera(60, width / height, 0.1, 1000);
        camera.up.set(0, 0, 1);

        const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
        renderer.setSize(width, height);

        const oldCanvas = container.querySelector('canvas');
        if (oldCanvas) container.removeChild(oldCanvas);
        container.appendChild(renderer.domElement);

        // --- 2. 创建对象 ---

        // A. 机器狗
        const dogGroup = new THREE.Group();

        // 保留红色箭头，方便你调试机器狗的真实朝向是否正确
        const arrowHelper = new THREE.ArrowHelper(
            new THREE.Vector3(1, 0, 0),
            new THREE.Vector3(0, 0, 0),
            1.0,
            0x00ff00
        );
        const arrowZOffset = -0.9;
        arrowHelper.position.set(0, 0, arrowZOffset);
        dogGroup.add(arrowHelper);
        scene.add(dogGroup);

        // ==========================================
        // 💡 新增：为 3D 模型添加环境光与平行光
        // GLB 模型使用 PBR 材质，没有光照会显示为纯黑
        // ==========================================
        const ambientLight = new THREE.AmbientLight(0xffffff, 1.2);
        scene.add(ambientLight);

        const directionalLight = new THREE.DirectionalLight(0xffffff, 1.5);
        directionalLight.position.set(10, -10, 20); // 根据你的 Z 轴向上的坐标系调整光源位置
        scene.add(directionalLight);

        // ==========================================
        // 🐕 新增：异步加载 M20 Pro 的 GLB 模型
        // ==========================================
        if (typeof THREE.GLTFLoader !== 'undefined') {
            const loader = new THREE.GLTFLoader();

            loader.load('/models/M20_3D.glb', function (gltf) {
                const m20Model = gltf.scene;

                // 1. 缩放调整：GLB 内部单位可能和你的场景 (m) 不一致，按需调整
                m20Model.scale.set(1, 1, 1);

                // 2. 姿态调整：如果发现狗侧躺着或倒着，调整欧拉角
                // 提示：你场景的 Z 轴是朝上的，很多 3D 软件导出的 Y 轴朝上
                m20Model.rotation.x = Math.PI / 2; // 如果站立方向不对，尝试解开这行注释
                m20Model.rotation.y = 0;
                m20Model.rotation.z = 0; // 朝向修正（与红色 arrowHelper 保持一致）

                // 3. 🚩【核心修改】位置偏移调整：把狗拎起来
                // 参数分别是：x(前后), y(左右), z(上下)
                // 这里的 0.6 是假设狗站高 60cm，你可以根据实际视觉效果微调这个数字
                m20Model.position.set(0, 0, -0.55);

                // 遍历模型并开启阴影/调整材质（可选）
                m20Model.traverse((child) => {
                    if (child.isMesh) {
                        // child.material.wireframe = true; // 测试用：如果模型太黑可以看线框
                    }
                });

                dogGroup.add(m20Model);
                console.log("✅ 绝影 M20 Pro 3D 模型加载成功！");

            }, undefined, function (error) {
                console.error("❌ 模型加载失败，降级显示方块:", error);
                // 失败兜底：显示原来的绿框
                const bodyMesh = new THREE.Mesh(
                    new THREE.BoxGeometry(0.9, 0.50, 0.2),
                    new THREE.MeshBasicMaterial({ color: 0x00ff00, wireframe: true })
                );
                dogGroup.add(bodyMesh);
            });
        } else {
            console.warn("⚠️ 未引入 GLTFLoader.js，使用默认方块代替");
            const bodyMesh = new THREE.Mesh(
                new THREE.BoxGeometry(0.9, 0.50, 0.2),
                new THREE.MeshBasicMaterial({ color: 0x00ff00, wireframe: true })
            );
            dogGroup.add(bodyMesh);
        }

        // 临时联调参数：当前观察到 SLAM 场景整体低于 grid 约 0.8m，
        // 先只对点云/机器狗/轨迹做统一高度补偿，不动 grid。
        const SLAM_Z_OFFSET = 0.8;

        // ✅ 地面点云覆盖路径时的自适应抬高配置
        // 不采用“路径上方只要有点就抬高”的判断，因为行人经过也会在路径上方产生临时点云。
        // 这里按 XY 网格累计高度直方图，只接受跨多个相邻网格、重复出现且占主导的低位高度层作为地面。
        const PATH_GROUND_LIFT_CONFIG = {
            cellSizeM: 0.20,
            queryRadiusM: 0.36,
            heightBinM: 0.03,
            maxBinsPerCell: 7,
            minGroundVotes: 12,
            minSupportingCells: 2,
            maxGroundAbovePathM: 0.12,
            maxGroundBelowPathM: 0.45,
            pathRadiusM: 0.03,
            separationEpsilonM: 0.003,
            gridSampleStride: 3,
            maxGridCells: 120000
        };
        const pathGroundHeightGrid = new Map();

        function getPathGroundCellKey(cellX, cellY) {
            return cellX + ":" + cellY;
        }

        function recordPointForPathGroundDetection(x, y, worldZ) {
            if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(worldZ)) return;
            if (Math.abs(x) >= 9000 || Math.abs(y) >= 9000 || Math.abs(worldZ) >= 100) return;

            const cellSize = PATH_GROUND_LIFT_CONFIG.cellSizeM;
            const cellX = Math.floor(x / cellSize);
            const cellY = Math.floor(y / cellSize);
            const key = getPathGroundCellKey(cellX, cellY);
            let cell = pathGroundHeightGrid.get(key);

            if (!cell) {
                // 限制长期运行时的索引大小；被移除的旧区域在再次扫描到时会自动重建。
                if (pathGroundHeightGrid.size >= PATH_GROUND_LIFT_CONFIG.maxGridCells) {
                    const removeCount = Math.ceil(PATH_GROUND_LIFT_CONFIG.maxGridCells * 0.08);
                    const iterator = pathGroundHeightGrid.keys();
                    for (let i = 0; i < removeCount; i++) {
                        const old = iterator.next();
                        if (old.done) break;
                        pathGroundHeightGrid.delete(old.value);
                    }
                }
                cell = { bins: new Map() };
                pathGroundHeightGrid.set(key, cell);
            }

            const heightBin = Math.round(worldZ / PATH_GROUND_LIFT_CONFIG.heightBinM);
            cell.bins.set(heightBin, Math.min(65535, (cell.bins.get(heightBin) || 0) + 1));

            // 人体/车辆等竖向目标会产生许多零散高度层。每格只保留投票最稳定的少数高度层，
            // 防止这些临时高位点长期占用内存或参与地面判断。
            if (cell.bins.size > PATH_GROUND_LIFT_CONFIG.maxBinsPerCell) {
                let weakestBin = null;
                let weakestVotes = Infinity;
                cell.bins.forEach((votes, bin) => {
                    if (votes < weakestVotes) {
                        weakestVotes = votes;
                        weakestBin = bin;
                    }
                });
                if (weakestBin !== null) cell.bins.delete(weakestBin);
            }
        }

        function estimateStableGroundHeightNearPath(x, y, pathWorldZ) {
            const cfg = PATH_GROUND_LIFT_CONFIG;
            const centerCellX = Math.floor(x / cfg.cellSizeM);
            const centerCellY = Math.floor(y / cfg.cellSizeM);
            const cellRadius = Math.ceil(cfg.queryRadiusM / cfg.cellSizeM);
            const aggregated = new Map(); // heightBin -> {votes, cells}

            for (let dx = -cellRadius; dx <= cellRadius; dx++) {
                for (let dy = -cellRadius; dy <= cellRadius; dy++) {
                    const cellCenterDx = dx * cfg.cellSizeM;
                    const cellCenterDy = dy * cfg.cellSizeM;
                    if (Math.hypot(cellCenterDx, cellCenterDy) > cfg.queryRadiusM + cfg.cellSizeM * 0.75) continue;

                    const cell = pathGroundHeightGrid.get(getPathGroundCellKey(centerCellX + dx, centerCellY + dy));
                    if (!cell) continue;

                    cell.bins.forEach((votes, bin) => {
                        const height = bin * cfg.heightBinM;
                        if (height < pathWorldZ - cfg.maxGroundBelowPathM ||
                            height > pathWorldZ + cfg.maxGroundAbovePathM) {
                            return;
                        }
                        let item = aggregated.get(bin);
                        if (!item) {
                            item = { votes: 0, cells: 0 };
                            aggregated.set(bin, item);
                        }
                        item.votes += votes;
                        item.cells += 1;
                    });
                }
            }

            if (!aggregated.size) return null;

            // 高度相邻的三个 bin 合并评分，以适应地面小坡度和点云测高噪声。
            const scored = [];
            aggregated.forEach((item, bin) => {
                let votes = 0;
                let cells = 0;
                for (let offset = -1; offset <= 1; offset++) {
                    const neighbor = aggregated.get(bin + offset);
                    if (!neighbor) continue;
                    votes += neighbor.votes;
                    cells += neighbor.cells;
                }
                scored.push({ bin, votes, cells });
            });
            scored.sort((a, b) => b.votes - a.votes || a.bin - b.bin);
            const strongest = scored[0];
            if (!strongest || strongest.votes < cfg.minGroundVotes || strongest.cells < cfg.minSupportingCells) {
                return null;
            }

            // 若较低高度层拥有足够多的稳定投票，优先认定它为地面。
            // 行人通常形成更高的点层，因此即使短时点数较多，也不会替代下方连续地面层。
            const stableCandidates = scored.filter(item =>
                item.votes >= cfg.minGroundVotes &&
                item.cells >= cfg.minSupportingCells
            );
            const ground = stableCandidates.reduce((lowest, item) =>
                !lowest || item.bin < lowest.bin ? item : lowest
                , null);
            if (!ground) return null;

            let weightedHeight = 0;
            let totalVotes = 0;
            for (let offset = -1; offset <= 1; offset++) {
                const bin = ground.bin + offset;
                const item = aggregated.get(bin);
                if (!item) continue;
                weightedHeight += bin * cfg.heightBinM * item.votes;
                totalVotes += item.votes;
            }
            const centerZ = totalVotes > 0 ? weightedHeight / totalVotes : ground.bin * cfg.heightBinM;

            // 地面顶边使用“地面主高度层及其紧邻上层”中仍具有稳定空间支持的最高 bin。
            // 更高的人体点层与地面层不连续，不会进入这个顶边计算。
            let topBin = ground.bin;
            for (let offset = 0; offset <= 1; offset++) {
                const bin = ground.bin + offset;
                const item = aggregated.get(bin);
                if (item && item.votes >= cfg.minGroundVotes && item.cells >= cfg.minSupportingCells) {
                    topBin = Math.max(topBin, bin);
                }
            }

            return {
                centerZ,
                // 加半个量化步长，表示该高度 bin 的实际最高边界，而不是固定抬高距离。
                topZ: (topBin + 0.5) * cfg.heightBinM
            };
        }

        function getRenderedPointCloudRadiusM() {
            const scanPointSize = Number(scanMesh?.material?.size);
            const mapPointSize = Number(mapMesh?.material?.size);
            const pointDiameter = Math.max(
                Number.isFinite(scanPointSize) ? scanPointSize : 0,
                Number.isFinite(mapPointSize) ? mapPointSize : 0
            );
            return Math.max(0, pointDiameter * 0.5);
        }

        function liftPathDisplayPointsAboveGround(points, parentWorldZ = 0) {
            if (!Array.isArray(points) || !points.length) return [];
            const cfg = PATH_GROUND_LIFT_CONFIG;
            const lifted = points.map(source => {
                return source.clone ? source.clone() : new THREE.Vector3(
                    Number(source.x), Number(source.y), Number(source.z || 0)
                );
            });
            const pointCloudRadiusM = getRenderedPointCloudRadiusM();
            let wholePathLiftM = 0;

            // 先逐点检查覆盖情况，但暂不改变任何点的高度。
            // 整条路径最终统一采用其中最大的必要抬高量，保持原有三维形状不变。
            lifted.forEach(point => {
                const baseWorldZ = Number(point.z) + Number(parentWorldZ || 0);
                const groundSurface = estimateStableGroundHeightNearPath(point.x, point.y, baseWorldZ);
                if (!groundSurface || !Number.isFinite(groundSurface.topZ)) return;

                // 只有地面点的球形显示范围确实触及管线路径时才抬高。
                // 高于该窄地面带的行人点云不会进入 groundSurface，因此不会触发本条件。
                const groundCloudTopZ = groundSurface.topZ + pointCloudRadiusM;
                const pathBottomZ = baseWorldZ - cfg.pathRadiusM;
                if (groundCloudTopZ < pathBottomZ) return;

                // 目标中心高度按当前地面点云的真实顶边动态计算：
                // 地面点顶边 + 管线路径半径 + 极小分离量，刚好脱离覆盖即可。
                const targetWorldZ = groundCloudTopZ + cfg.pathRadiusM + cfg.separationEpsilonM;
                const liftM = targetWorldZ - baseWorldZ;
                if (liftM > wholePathLiftM) wholePathLiftM = liftM;
            });

            if (wholePathLiftM > 0) {
                lifted.forEach(point => {
                    point.z += wholePathLiftM;
                });
            }

            // 用高度签名判断地面索引更新后是否真的需要重建 TubeGeometry。
            // 行人高位点被过滤后签名不会改变，因此不会导致路径重绘或抬高。
            let heightHash = lifted.length >>> 0;
            lifted.forEach(point => {
                heightHash = (((heightHash * 31) >>> 0) + (Math.round(point.z * 1000) >>> 0)) >>> 0;
            });
            lifted.groundLiftedPointCount = wholePathLiftM > 0 ? lifted.length : 0;
            lifted.groundWholePathLiftM = wholePathLiftM;
            lifted.groundLiftSignature = `${lifted.length}:${heightHash}`;
            return lifted;
        }

        // B1. 实时雷达层
        const scanGeom = new THREE.BufferGeometry();
        // 👇 修改后：扩大为 50000 个点，尺寸调为 0.05
        const scanPositions = new Float32Array(50000 * 3);
        const scanColors = new Float32Array(50000 * 3);
        scanGeom.setAttribute('position', new THREE.BufferAttribute(scanPositions, 3));
        scanGeom.setAttribute('color', new THREE.BufferAttribute(scanColors, 3));

        const scanMesh = new THREE.Points(scanGeom, new THREE.PointsMaterial({
            vertexColors: true,
            size: 0.03, // 稍微调大一点，更容易看清
            sizeAttenuation: true
        }));
        scanMesh.frustumCulled = false;
        scanMesh.position.z = SLAM_Z_OFFSET;
        scene.add(scanMesh);

        // B2. 历史地图层
        const mapMaxPoints = 3000000;
        const mapMaxComponents = mapMaxPoints * 3;
        const mapGeom = new THREE.BufferGeometry();
        const mapPositions = new Float32Array(mapMaxPoints * 3);
        const mapColors = new Float32Array(mapMaxPoints * 3);

        mapGeom.setAttribute('position', new THREE.BufferAttribute(mapPositions, 3));
        mapGeom.setAttribute('color', new THREE.BufferAttribute(mapColors, 3));

        for (let i = 0; i < mapMaxPoints * 3; i++) mapPositions[i] = 99999;

        let pendingMapUpdateRanges = [];

        function queueMapBufferUpdateRange(offset, count) {
            if (!Number.isFinite(offset) || !Number.isFinite(count) || count <= 0) return;
            pendingMapUpdateRanges.push({
                offset: Math.max(0, Math.floor(offset)),
                count: Math.min(mapMaxComponents, Math.floor(count))
            });
        }

        function markAttributeUpdateRanges(attribute, ranges) {
            if (!attribute || !ranges.length) return;

            if (typeof attribute.clearUpdateRanges === "function") {
                attribute.clearUpdateRanges();
            }

            if (typeof attribute.addUpdateRange === "function") {
                ranges.forEach(range => {
                    if (range.count > 0) attribute.addUpdateRange(range.offset, range.count);
                });
                attribute.needsUpdate = true;
                return;
            }

            let start = Infinity;
            let end = -Infinity;
            ranges.forEach(range => {
                if (range.count <= 0) return;
                start = Math.min(start, range.offset);
                end = Math.max(end, range.offset + range.count);
            });

            if (!Number.isFinite(start) || end <= start) return;
            if (attribute.updateRange) {
                attribute.updateRange.offset = start;
                attribute.updateRange.count = end - start;
            }
            attribute.needsUpdate = true;
        }

        function flushPendingMapBufferUpdates() {
            if (!pendingMapUpdateRanges.length) return;

            const ranges = pendingMapUpdateRanges;
            pendingMapUpdateRanges = [];

            markAttributeUpdateRanges(mapMesh.geometry.attributes.position, ranges);
            markAttributeUpdateRanges(mapMesh.geometry.attributes.color, ranges);
        }

        function markFullAttributeUpdate(attribute) {
            if (!attribute) return;
            if (typeof attribute.clearUpdateRanges === "function") {
                attribute.clearUpdateRanges();
            }
            if (attribute.updateRange) {
                attribute.updateRange.offset = 0;
                attribute.updateRange.count = -1;
            }
            attribute.needsUpdate = true;
        }

        function appendScanToHistoryMap(scanPos, scanCol, componentCount) {
            if (!scanPos || !scanCol || componentCount <= 0) return;

            let remaining = Math.min(componentCount, mapMaxComponents);
            let sourceOffset = 0;

            while (remaining > 0) {
                if (mapCursor >= mapMaxComponents) mapCursor = 0;

                const chunkCount = Math.min(remaining, mapMaxComponents - mapCursor);
                mapPositions.set(scanPos.subarray(sourceOffset, sourceOffset + chunkCount), mapCursor);
                mapColors.set(scanCol.subarray(sourceOffset, sourceOffset + chunkCount), mapCursor);
                queueMapBufferUpdateRange(mapCursor, chunkCount);

                mapCursor += chunkCount;
                mapStoredComponents = Math.min(mapMaxComponents, mapStoredComponents + chunkCount);
                sourceOffset += chunkCount;
                remaining -= chunkCount;

                if (mapCursor >= mapMaxComponents) mapCursor = 0;
            }
        }

        const mapMesh = new THREE.Points(mapGeom, new THREE.PointsMaterial({
            vertexColors: true,
            size: 0.03,
            sizeAttenuation: true
        }));
        mapMesh.frustumCulled = false;
        mapMesh.position.z = SLAM_Z_OFFSET;
        scene.add(mapMesh);



        // C. 机器狗轨迹
        const trailGroup = new THREE.Group();
        trailGroup.position.z = SLAM_Z_OFFSET;
        scene.add(trailGroup);
        const faultMarkerGroup = new THREE.Group();
        scene.add(faultMarkerGroup);

        let lastFaultPos = new THREE.Vector3(-999, -999, -999);

        // ✅ 修改 1: 机器狗路径改回原来的青色 (0x00ccff)
        const trailMat = new THREE.MeshBasicMaterial({ color: 0x00ccff });

        const trailPoints = [];
        let lastTrailPos = new THREE.Vector3(-999, -999, 0);
        let currentTrailMesh = null;

        // D. 地下电缆
        const cableGroup = new THREE.Group(); scene.add(cableGroup);
        const redCableGroup = new THREE.Group(); scene.add(redCableGroup);
        // ✅ 修改 2: 电缆路径改为黄色
        const cableMat = new THREE.MeshBasicMaterial({ color: 0xffff00 });
        const redCableMat = new THREE.MeshBasicMaterial({ color: 0xff0000 });
        const cablePoints = [];
        let segmentPoints = [];
        let lastCablePos = new THREE.Vector3(-999, -999, 0);
        let lastSignal = -1;
        let currentCableMesh = null;
        let lastDepthUpdateIdx = 0; // ✅ 新增：用于记录上一次绑定深度的游标位置
        const PIPELINE_FEATURE_SIDE_OFFSET_M = 0.16; // 特征点允许保留较明显的左右定位
        const PIPELINE_TRACK_SIDE_OFFSET_M = 0.07;   // 源版本连续补点只作轻微修正，避免把 1/2 信号画成锯齿线
        let lastPathGroundLiftRefreshMs = 0;

        function rememberPathGroundLiftState(mesh, visiblePoints) {
            if (!mesh || !visiblePoints) return mesh;
            mesh.userData = mesh.userData || {};
            mesh.userData.groundLiftSignature = visiblePoints.groundLiftSignature || "";
            mesh.userData.groundLiftedPointCount = visiblePoints.groundLiftedPointCount || 0;
            mesh.userData.groundWholePathLiftM = Number(visiblePoints.groundWholePathLiftM || 0);
            return mesh;
        }

        function refreshRenderedPathsForGroundLift(force = false) {
            const now = performance.now();
            if (!force && now - lastPathGroundLiftRefreshMs < 750) return;
            lastPathGroundLiftRefreshMs = now;

            if (currentTrailMesh && trailPoints.length >= 2) {
                const visibleTrailPoints = liftPathDisplayPointsAboveGround(trailPoints, trailGroup.position.z);
                if (currentTrailMesh.userData?.groundLiftSignature !== visibleTrailPoints.groundLiftSignature) {
                    const curve = new THREE.CatmullRomCurve3(visibleTrailPoints);
                    const newGeometry = new THREE.TubeGeometry(curve, trailPoints.length * 3, 0.03, 6, false);
                    currentTrailMesh.geometry.dispose();
                    currentTrailMesh.geometry = newGeometry;
                    rememberPathGroundLiftState(currentTrailMesh, visibleTrailPoints);
                }
            }

            const cableDisplayPoints = Array.isArray(window.savedCablePath) ? window.savedCablePath : [];
            if (currentCableMesh && cableDisplayPoints.length >= 2) {
                const visibleCablePoints = liftPathDisplayPointsAboveGround(cableDisplayPoints, cableGroup.position.z);
                if (currentCableMesh.userData?.groundLiftSignature !== visibleCablePoints.groundLiftSignature) {
                    const curve = new THREE.CatmullRomCurve3(visibleCablePoints, false, "centripetal", 0.35);
                    const newGeometry = new THREE.TubeGeometry(
                        curve,
                        Math.max(cableDisplayPoints.length * 8, 16),
                        0.03,
                        8,
                        false
                    );
                    currentCableMesh.geometry.dispose();
                    currentCableMesh.geometry = newGeometry;
                    rememberPathGroundLiftState(currentCableMesh, visibleCablePoints);
                }
            }
        }

        function isValidCablePos(pos) {
            return pos && Number.isFinite(pos.x) && Number.isFinite(pos.y) && pos.x !== -999;
        }

        function normalizeAngleRad(rad) {
            while (rad > Math.PI) rad -= Math.PI * 2;
            while (rad < -Math.PI) rad += Math.PI * 2;
            return rad;
        }

        function blendYawRad(a, b, bWeight) {
            const aWeight = 1 - bWeight;
            return Math.atan2(
                Math.sin(a) * aWeight + Math.sin(b) * bWeight,
                Math.cos(a) * aWeight + Math.cos(b) * bWeight
            );
        }

        function smoothCableDisplayPoints(points, sourcePoints = points) {
            if (!points || points.length < 5) return points;
            const smoothed = points.map((p) => p.clone());

            for (let pass = 0; pass < 2; pass++) {
                for (let i = 1; i < smoothed.length - 1; i++) {
                    const prev = smoothed[i - 1];
                    const curr = smoothed[i];
                    const next = smoothed[i + 1];
                    const source = sourcePoints[i]?.userData?.source;
                    const keepCurrent = source === "feature" ? 0.72 : 0.58;
                    const neighbor = (1 - keepCurrent) * 0.5;
                    curr.x = prev.x * neighbor + curr.x * keepCurrent + next.x * neighbor;
                    curr.y = prev.y * neighbor + curr.y * keepCurrent + next.y * neighbor;
                }
            }

            return smoothed;
        }

        function getRecentCableYawRad() {
            if (cablePoints.length >= 2) {
                const p2 = cablePoints[cablePoints.length - 1];
                for (let i = cablePoints.length - 2; i >= 0; i--) {
                    const p1 = cablePoints[i];
                    const dx = p2.x - p1.x;
                    const dy = p2.y - p1.y;
                    if (Math.hypot(dx, dy) > 0.08) {
                        return Math.atan2(dy, dx);
                    }
                }
            }

            const pred = window.g_pipelinePrediction;
            if (pred && Number.isFinite(Number(pred.yaw))) {
                return Number(pred.yaw) * Math.PI / 180;
            }

            return window.currentRobotPose && Number.isFinite(window.currentRobotPose.yaw)
                ? window.currentRobotPose.yaw
                : 0;
        }

        function estimateCablePointFromRobotSignal(px, py, sig, prediction, mode) {
            const robotYaw = window.currentRobotPose && Number.isFinite(window.currentRobotPose.yaw)
                ? window.currentRobotPose.yaw
                : 0;
            const headingX = Math.cos(robotYaw);
            const headingY = Math.sin(robotYaw);
            const rightX = headingY;
            const rightY = -headingX;

            const sideBase = mode === "feature" ? PIPELINE_FEATURE_SIDE_OFFSET_M : PIPELINE_TRACK_SIDE_OFFSET_M;
            let sideOffset = 0;
            if (sig === 1) sideOffset = sideBase;       // 电缆在右侧，向右找线
            else if (sig === 2) sideOffset = -sideBase; // 电缆在左侧，向左找线

            const measured = new THREE.Vector3(
                px + rightX * sideOffset,
                py + rightY * sideOffset,
                0.08
            );

            if (mode === "feature" || !isValidCablePos(lastCablePos)) {
                return measured;
            }

            let predictedYaw = getRecentCableYawRad();
            if (prediction && Number.isFinite(Number(prediction.yaw))) {
                predictedYaw = blendYawRad(predictedYaw, Number(prediction.yaw) * Math.PI / 180, 0.45);
            }

            const dirX = Math.cos(predictedYaw);
            const dirY = Math.sin(predictedYaw);
            const dx = measured.x - lastCablePos.x;
            const dy = measured.y - lastCablePos.y;
            const along = dx * dirX + dy * dirY;
            const stepLimit = mode === "dense" ? 0.10 : 0.24;
            const step = Math.max(0.05, Math.min(Math.abs(along), stepLimit));
            const sign = along >= 0 ? 1 : -1;
            const predicted = new THREE.Vector3(
                lastCablePos.x + dirX * step * sign,
                lastCablePos.y + dirY * step * sign,
                0.08
            );

            const measuredWeight = mode === "dense" ? 0.22 : 0.12;
            return new THREE.Vector3(
                predicted.x * (1 - measuredWeight) + measured.x * measuredWeight,
                predicted.y * (1 - measuredWeight) + measured.y * measuredWeight,
                0.08
            );
        }

        // ==========================================
        // ✅ 向三维重建页导出当前路径页三维地图快照
        // 导出内容：历史点云 mapMesh + 黄色电缆路径 window.savedCablePath + 红色破损点 faultMarkerGroup
        // 坐标转换在加载到 GPR 页面时执行。
        // ==========================================
        window.exportCurrentPathFaultMapForGpr = function () {
            const posAttr = mapMesh.geometry.attributes.position;
            const colAttr = mapMesh.geometry.attributes.color;
            const srcPos = posAttr.array;
            const srcCol = colAttr.array;

            mapMesh.updateMatrixWorld(true);
            const worldMatrix = mapMesh.matrixWorld;
            const temp = new THREE.Vector3();

            let validCount = 0;
            for (let i = 0; i < srcPos.length; i += 3) {
                const x = srcPos[i];
                const y = srcPos[i + 1];
                const z = srcPos[i + 2];
                if (
                    Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(z) &&
                    Math.abs(x) < 9000 && Math.abs(y) < 9000 && Math.abs(z) < 9000
                ) {
                    validCount++;
                }
            }

            const positions = new Float32Array(validCount * 3);
            const colors = new Float32Array(validCount * 3);

            let j = 0;
            for (let i = 0; i < srcPos.length; i += 3) {
                const x = srcPos[i];
                const y = srcPos[i + 1];
                const z = srcPos[i + 2];
                if (
                    !Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z) ||
                    Math.abs(x) >= 9000 || Math.abs(y) >= 9000 || Math.abs(z) >= 9000
                ) {
                    continue;
                }

                // 路径页世界坐标：X/Y 为水平面，Z 为高度；GPR页：X/Z 为水平面，Y 为高度。
                // 这里必须使用 -Y 作为 GPR 页 Z 轴，避免交换 Y/Z 后造成地图镜像。
                temp.set(x, y, z).applyMatrix4(worldMatrix);
                positions[j] = temp.x;
                positions[j + 1] = temp.z;
                positions[j + 2] = -temp.y;

                colors[j] = srcCol[i] ?? 1.0;
                colors[j + 1] = srcCol[i + 1] ?? 1.0;
                colors[j + 2] = srcCol[i + 2] ?? 1.0;
                j += 3;
            }

            const cablePath = Array.isArray(window.savedCablePath)
                ? window.savedCablePath.map(p => ({
                    x: Number(p.x),
                    y: Number(p.y),
                    z: Number(p.z ?? 0.08),
                    depth: p.userData ? (p.userData.depth ?? null) : (p.depth ?? null)
                }))
                : [];

            const faults = [];
            const faultWorldPos = new THREE.Vector3();
            faultMarkerGroup.children.forEach(group => {
                group.updateMatrixWorld(true);
                faultWorldPos.setFromMatrixPosition(group.matrixWorld);
                faults.push({
                    faultId: group.userData?.faultId || "",
                    x: Number(faultWorldPos.x),
                    y: Number(faultWorldPos.y),
                    z: Number(faultWorldPos.z ?? 0.08)
                });
            });

            return {
                pointCloud: {
                    positions,
                    colors,
                    count: validCount
                },
                cablePath,
                faults
            };
        };

        // ==========================================
        // ✅ 正式 B/S：巡检结果保存与历史回显
        // 只保存路径点、破损点坐标、图片 URL，不保存点云
        // ==========================================
        function roundInspectionCoordinate(value, digits = 10) {
            const n = Number(value);
            return Number.isFinite(n) ? Number(n.toFixed(digits)) : null;
        }

        function getStoredInspectionGps(point) {
            if (!point) return null;
            const lat = roundInspectionCoordinate(point.lat ?? point.latitude ?? point.userData?.lat, 10);
            const lon = roundInspectionCoordinate(point.lon ?? point.lng ?? point.longitude ?? point.userData?.lon, 10);
            return lat !== null && lon !== null ? { lat, lon } : null;
        }

        function getGpsFromSlamPoint(x, y, sourcePoint = null) {
            const sx = Number(x);
            const sy = Number(y);

            if (Number.isFinite(sx) && Number.isFinite(sy) && typeof window.slamToGps === "function") {
                const gps = window.slamToGps(sx, sy);
                const lat = roundInspectionCoordinate(gps?.lat, 10);
                const lon = roundInspectionCoordinate(gps?.lon, 10);
                if (lat !== null && lon !== null) return { lat, lon };
            }

            return getStoredInspectionGps(sourcePoint);
        }

        function getCurrentSlamFromInspectionGps(point) {
            const gps = getStoredInspectionGps(point);
            if (gps && typeof window.gpsToSlam === "function") {
                const slam = window.gpsToSlam(gps.lat, gps.lon);
                const x = Number(slam?.x);
                const y = Number(slam?.y);
                if (Number.isFinite(x) && Number.isFinite(y)) {
                    return {
                        x,
                        y,
                        projectedFromGps: true
                    };
                }
            }

            return {
                x: Number(point?.x),
                y: Number(point?.y),
                projectedFromGps: false
            };
        }

        function hasInspectionGps(point) {
            return !!getStoredInspectionGps(point);
        }

        function plainCablePoint(p) {
            const x = Number(p.x);
            const y = Number(p.y);
            const gps = getGpsFromSlamPoint(x, y, p);

            return {
                x,
                y,
                z: Number(p.z ?? 0.08),
                lat: gps ? gps.lat : null,
                lon: gps ? gps.lon : null,
                depth: p.userData ? (p.userData.depth ?? null) : (p.depth ?? null)
            };
        }

        function vectorCablePoint(p) {
            const projected = getCurrentSlamFromInspectionGps(p);
            const v = new THREE.Vector3(
                projected.x,
                projected.y,
                Number(p.z ?? 0.08)
            );
            const gps = getStoredInspectionGps(p);

            v.userData = {
                depth: p.depth ?? null,
                lat: gps ? gps.lat : null,
                lon: gps ? gps.lon : null,
                originalX: Number(p.x),
                originalY: Number(p.y),
                projectedFromGps: projected.projectedFromGps
            };

            return v;
        }

        function disposeObject3D(obj) {
            obj.traverse(child => {
                if (child.geometry) child.geometry.dispose();

                if (child.material) {
                    if (child.material.map) child.material.map.dispose();
                    child.material.dispose();
                }
            });
        }

        function clearThreeGroup(group) {
            group.children.slice().forEach(child => {
                group.remove(child);
                disposeObject3D(child);
            });
        }

        function drawCablePathFromPoints(points) {
            if (!points || points.length < 2) return;

            if (currentCableMesh) {
                cableGroup.remove(currentCableMesh);
                disposeObject3D(currentCableMesh);
                currentCableMesh = null;
            }

            const visiblePoints = liftPathDisplayPointsAboveGround(points, cableGroup.position.z);
            const curve = new THREE.CatmullRomCurve3(visiblePoints);
            const tube = new THREE.TubeGeometry(
                curve,
                Math.max(points.length * 8, 16),
                0.03,
                8,
                false
            );

            currentCableMesh = new THREE.Mesh(tube, cableMat);
            rememberPathGroundLiftState(currentCableMesh, visiblePoints);
            cableGroup.add(currentCableMesh);
        }

        function createFaultMarkerFromHistory(fault) {
            const faultId = fault.faultId || fault.id || ("fault_" + Date.now());
            const projected = getCurrentSlamFromInspectionGps(fault);
            const gps = getStoredInspectionGps(fault);

            const faultGroup = new THREE.Group();

            faultGroup.position.set(
                projected.x,
                projected.y,
                Number(fault.z ?? 0.08)
            );

            faultGroup.userData.faultId = faultId;
            faultGroup.userData.lat = gps ? gps.lat : null;
            faultGroup.userData.lon = gps ? gps.lon : null;
            faultGroup.userData.originalX = Number(fault.x);
            faultGroup.userData.originalY = Number(fault.y);
            faultGroup.userData.projectedFromGps = projected.projectedFromGps;

            const sphereGeo = new THREE.SphereGeometry(0.12, 16, 16);
            const sphereMat = new THREE.MeshBasicMaterial({ color: 0xff0000 });
            const faultMesh = new THREE.Mesh(sphereGeo, sphereMat);
            faultGroup.add(faultMesh);

            const canvas = document.createElement("canvas");
            canvas.width = 128;
            canvas.height = 128;

            const ctx = canvas.getContext("2d");
            ctx.clearRect(0, 0, 128, 128);
            ctx.font = "bold 90px Arial";
            ctx.textAlign = "center";
            ctx.textBaseline = "middle";
            ctx.fillStyle = "#ffdd00";
            ctx.shadowColor = "#ff0000";
            ctx.shadowBlur = 15;
            ctx.fillText("⚡", 64, 64);

            const texture = new THREE.CanvasTexture(canvas);
            const spriteMat = new THREE.SpriteMaterial({
                map: texture,
                transparent: true,
                depthTest: false
            });

            const lightningSprite = new THREE.Sprite(spriteMat);
            lightningSprite.scale.set(0.6, 0.6, 1);
            lightningSprite.position.set(0, 0, 0.3);
            faultGroup.add(lightningSprite);

            faultMarkerGroup.add(faultGroup);

            return faultGroup;
        }

        async function normalizeImagesToUrls(faultId, imgs) {
            const safeImgs = imgs || {};
            const result = {
                left: safeImgs.left || "",
                right: safeImgs.right || "",
                front: safeImgs.front || "",
                back: safeImgs.back || ""
            };

            for (const direction of ["left", "right", "front", "back"]) {
                const val = result[direction];

                if (val && val.startsWith("data:")) {
                    result[direction] = await uploadFaultImageBase64(faultId, direction, val);
                    setFaultImageUrl(faultId, direction, result[direction]);
                }
            }

            return result;
        }

        async function collectCurrentInspectionPayload() {
            const cablePath = Array.isArray(window.savedCablePath)
                ? window.savedCablePath.map(plainCablePoint)
                : [];

            if (cablePath.length >= 2 && cablePath.some(point => !hasInspectionGps(point))) {
                throw new Error("GPS calibration is required before saving inspection-result cable coordinates.");
            }

            if (cablePath.length < 2) {
                throw new Error("当前没有有效的黄色电缆路径，无法保存巡检结果。");
            }

            const faults = [];

            for (const group of faultMarkerGroup.children) {
                const faultId = group.userData?.faultId;
                if (!faultId) continue;

                const imgs = window.faultImagesMap && window.faultImagesMap.has(faultId)
                    ? window.faultImagesMap.get(faultId)
                    : { left: "", right: "", front: "", back: "" };

                const imageUrls = await normalizeImagesToUrls(faultId, imgs);

                const faultX = Number(group.position.x);
                const faultY = Number(group.position.y);
                const faultZ = Number(group.position.z ?? 0.08);
                const faultGps = getGpsFromSlamPoint(faultX, faultY, group.userData);

                if (!faultGps) {
                    throw new Error("GPS calibration is required before saving inspection-result fault coordinates.");
                }

                faults.push({
                    faultId: faultId,
                    x: faultX,
                    y: faultY,
                    z: faultZ,
                    lat: faultGps.lat,
                    lon: faultGps.lon,
                    images: {
                        left: imageUrls.left || "",
                        right: imageUrls.right || "",
                        front: imageUrls.front || "",
                        back: imageUrls.back || ""
                    }
                });
            }

            const now = new Date();

            return {
                name: "巡检结果 " + now.toLocaleString(),
                savedAt: now.toISOString(),
                cablePath: cablePath,
                faults: faults
            };
        }

        async function saveCurrentInspectionResultToServer() {
            const btn = document.getElementById("btnSaveInspectionResult");
            const oldText = btn ? btn.textContent : "";

            try {
                if (btn) {
                    btn.disabled = true;
                    btn.textContent = "⏳ 保存中...";
                }

                const payload = await collectCurrentInspectionPayload();

                const res = await fetch(INSPECTION_API.saveResult, {
                    method: "POST",
                    headers: {
                        "Content-Type": "application/json"
                    },
                    body: JSON.stringify(payload)
                });

                if (!res.ok) {
                    throw new Error("保存巡检结果失败：" + res.status);
                }

                await res.json().catch(() => null);

                alert("✅ 巡检结果已保存到机器狗主机 MySQL 中，仅保留最近五次。");
                addSystemMessage("巡检结果已成功保存。", "success");

                if (window.renderInspectionHistoryList) {
                    window.renderInspectionHistoryList();
                }

            } catch (err) {
                console.error("保存巡检结果失败：", err);
                alert("❌ " + err.message);
            } finally {
                if (btn) {
                    btn.disabled = false;
                    btn.textContent = oldText || "💾 保存巡检结果";
                }
            }
        }

        async function loadInspectionResultFromServer(id) {
            try {
                const res = await fetch(INSPECTION_API.detail(id));

                if (!res.ok) {
                    throw new Error("读取巡检历史失败：" + res.status);
                }

                const record = await res.json();

                if (window.switchMainPanel) {
                    window.switchMainPanel("pathFault");
                }

                clearThreeGroup(cableGroup);
                clearThreeGroup(faultMarkerGroup);
                clearThreeGroup(redCableGroup);

                currentCableMesh = null;

                const points = (record.cablePath || []).map(vectorCablePoint);

                window.savedCablePath = points;

                drawCablePathFromPoints(points);

                if (!window.faultImagesMap) {
                    window.faultImagesMap = new Map();
                }

                window.faultImagesMap.clear();

                (record.faults || []).forEach(fault => {
                    const group = createFaultMarkerFromHistory(fault);
                    const faultId = group.userData.faultId;

                    window.faultImagesMap.set(faultId, {
                        left: fault.images?.left || "",
                        right: fault.images?.right || "",
                        front: fault.images?.front || "",
                        back: fault.images?.back || ""
                    });
                });

                window.lastFaultMarker =
                    faultMarkerGroup.children[faultMarkerGroup.children.length - 1] || null;

                // ✅ 关键修复：从历史巡检结果恢复破损点后，同步恢复可发送给履带小车的破损点坐标
                if (window.lastFaultMarker) {
                    lastFaultCablePos = window.lastFaultMarker.position.clone();
                    window.lastFaultCablePos = lastFaultCablePos.clone();

                    console.log("✅ 已从历史巡检结果恢复破损点发送坐标：", window.lastFaultCablePos);
                } else {
                    lastFaultCablePos = null;
                    window.lastFaultCablePos = null;
                }

                setStatus("✅ 已加载历史巡检结果：" + (record.name || ""), "success");
                addSystemMessage(`已加载历史巡检结果：${record.name || "未命名记录"}。`, "success");

            } catch (err) {
                console.error("加载历史巡检结果失败：", err);
                alert("❌ " + err.message);
            }
        }

        window.renderInspectionHistoryList = async function () {
            const tbody = document.getElementById("inspectResultTableBody");
            if (!tbody) return;

            tbody.innerHTML = `
                <tr>
                    <td colspan="4" class="inspect-result-empty">正在读取巡检历史...</td>
                </tr>
            `;

            try {
                const res = await fetch(INSPECTION_API.listResults);

                if (!res.ok) {
                    throw new Error("读取巡检历史失败：" + res.status);
                }

                const history = await res.json();

                if (!Array.isArray(history) || history.length === 0) {
                    tbody.innerHTML = `
                        <tr>
                            <td colspan="4" class="inspect-result-empty">暂无保存记录</td>
                        </tr>
                    `;
                    return;
                }

                tbody.innerHTML = "";

                history.slice(0, 5).forEach((item, index) => {
                    const timeText = item.savedAt
                        ? new Date(item.savedAt).toLocaleString()
                        : "--";

                    const faultCount = item.faultCount ?? 0;
                    const pathCount = item.pathPointCount ?? 0;
                    const pathLength = Number(item.pathLengthMeters);
                    const pathLengthText = Number.isFinite(pathLength)
                        ? "约 " + pathLength.toFixed(pathLength >= 10 ? 1 : 2) + " m"
                        : "--";

                    const tr = document.createElement("tr");
                    tr.className = "inspect-result-row";
                    tr.tabIndex = 0;
                    tr.title = "点击加载该巡检结果";

                    const nameTd = document.createElement("td");
                    nameTd.className = "inspect-result-name";
                    nameTd.textContent = item.name || ("巡检结果 " + (index + 1));

                    const timeTd = document.createElement("td");
                    timeTd.textContent = timeText;

                    const pathTd = document.createElement("td");
                    pathTd.textContent = pathLengthText;
                    pathTd.title = "由保存的电缆路径坐标估算；原始路径点数：" + pathCount;

                    const faultTd = document.createElement("td");
                    faultTd.textContent = String(faultCount);

                    tr.appendChild(nameTd);
                    tr.appendChild(timeTd);
                    tr.appendChild(pathTd);
                    tr.appendChild(faultTd);

                    tr.onclick = () => loadInspectionResultFromServer(item.id);
                    tr.onkeydown = (e) => {
                        if (e.key === "Enter" || e.key === " ") {
                            e.preventDefault();
                            loadInspectionResultFromServer(item.id);
                        }
                    };

                    tbody.appendChild(tr);
                });

            } catch (err) {
                console.error("读取巡检历史失败：", err);
                tbody.innerHTML = `
                    <tr>
                        <td colspan="4" class="inspect-result-empty error">读取失败，请检查后端接口</td>
                    </tr>
                `;
            }
        };

        const btnSaveInspectionResult = document.getElementById("btnSaveInspectionResult");
        if (btnSaveInspectionResult) {
            btnSaveInspectionResult.onclick = saveCurrentInspectionResultToServer;
        }

        const btnRefreshInspectionResults = document.getElementById("btnRefreshInspectionResults");
        if (btnRefreshInspectionResults) {
            btnRefreshInspectionResults.onclick = () => window.renderInspectionHistoryList();
        }

        // E. 网格与交互
        const gridHelper = new THREE.GridHelper(50, 50, 0x888888, 0x444444);
        gridHelper.rotation.x = Math.PI / 2;
        scene.add(gridHelper);

        const planeGeo = new THREE.PlaneGeometry(200, 200);
        const planeMat = new THREE.MeshBasicMaterial({ visible: false });
        const clickPlane = new THREE.Mesh(planeGeo, planeMat);
        scene.add(clickPlane);

        // F. 导航标记
        const navGroup = new THREE.Group();
        scene.add(navGroup);

        const flagObj = new THREE.Group();
        const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 1, 8), new THREE.MeshBasicMaterial({ color: 0xffffff }));
        pole.rotation.x = Math.PI / 2; pole.position.z = 0.5;
        const flag = new THREE.Mesh(new THREE.ShapeGeometry(new THREE.Shape().moveTo(0, 0).lineTo(0, 0.4).lineTo(0.6, 0.2).lineTo(0, 0)), new THREE.MeshBasicMaterial({ color: 0xff0000, side: THREE.DoubleSide }));
        flag.rotation.x = Math.PI / 2; flag.position.set(0, 0, 0.6);
        flagObj.add(pole); flagObj.add(flag);
        flagObj.visible = false;
        navGroup.add(flagObj);

        const navLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineDashedMaterial({ color: 0x00ff00, dashSize: 0.2, gapSize: 0.1 }));
        navGroup.add(navLine);

        // --- 3. 交互控制 ---
        let radius = 8;
        let theta = Math.PI * 1.25;
        let phi = Math.PI / 3.5;
        const orbitTarget = new THREE.Vector3(0, 0, 0);

        let isPicking = false;
        let navGoal = null;
        let isNavigating = false;
        let navInterval = null;
        let currentRobotPose = { x: 0, y: 0, yaw: 0, updatedAt: 0 };
        window.currentRobotPose = currentRobotPose; // 🌟 增加这一行：挂载到全局

        // 核心路径变量
        let navPath = [];
        let navOriginalPath = [];
        let activeNavTrail = [];

        // ✅ 新增：电缆跟踪与寻找破损点专用状态变量
        let isCableTrackingMode = false; // 是否处于“跟踪电缆”模式
        let isFaultFindingMode = false;  // 是否处于“寻找破损点”模式 (新增)
        let cableTrackingState = 'IDLE'; // IDLE, TO_START, FOLLOWING

        const elNavPanel = document.getElementById('navInfoPanel');
        const elNavDist = document.getElementById('navDistVal');
        const elNavError = document.getElementById('navErrorVal');

        function updateCamera() {
            const x = orbitTarget.x + radius * Math.sin(phi) * Math.cos(theta);
            const y = orbitTarget.y + radius * Math.sin(phi) * Math.sin(theta);
            const z = orbitTarget.z + radius * Math.cos(phi);
            camera.position.set(x, y, z);
            camera.lookAt(orbitTarget);
            camera.up.set(0, 0, 1);
        }
        updateCamera();

        function updateDistanceUI() {
            if (navPath.length > 0 && currentRobotPose) {
                const goal = navPath[navPath.length - 1];
                const dx = goal.x - currentRobotPose.x;
                const dy = goal.y - currentRobotPose.y;
                const dist = Math.sqrt(dx * dx + dy * dy);

                if (elNavDist) elNavDist.textContent = dist.toFixed(2);
                if (elNavPanel) elNavPanel.style.display = 'block';
            } else {
                if (elNavPanel) elNavPanel.style.display = 'none';
            }
        }

        // --- ✅ A* 寻路算法 ---
        const GRID_SIZE = 0.2;
        const NAV_OBSTACLE_CONFIG = Object.freeze({
            // 用户指定：只将离局部地面 0.2m～1.2m 的点云视为障碍物。
            minHeightAboveGroundM: 0.2,
            maxHeightAboveGroundM: 1.2,
            localGroundQueryRadiusM: 0.8,
            minGroundVotes: 8,
            minGroundCells: 2,
            // 当前 SLAM_Z_OFFSET 的目的就是把地面对齐到世界坐标 Z=0；
            // 局部地面暂时无法估计时，用它作为安全兜底，而不是退回原始 Z 判断。
            fallbackGroundWorldZ: 0.0,
            // 使用明确的硬膨胀距离，不再按“机体外接圆半径 + 0.15m”计算。
            obstacleInflationRadiusM: 0.30,
            goalSearchRadiusM: 1.6,
            runtimeCheckIntervalMs: 500,
            runtimeLookaheadM: 2.0
        });
        const NAV_INFLATION_RADIUS_M = NAV_OBSTACLE_CONFIG.obstacleInflationRadiusM;

        let latestNavObstacles = new Set();
        let latestRuntimeNavObstacles = new Set();
        let latestNavObstacleStats = null;
        let lastRuntimeObstacleCheckAt = 0;

        function worldToGrid(x, y) {
            return { x: Math.round(x / GRID_SIZE), y: Math.round(y / GRID_SIZE) };
        }
        function gridToWorld(gx, gy) {
            return { x: gx * GRID_SIZE, y: gy * GRID_SIZE };
        }
        function gridKey(gx, gy) {
            return `${gx},${gy}`;
        }

        /**
         * 从已有的地面高度直方图中估计当前位置的局部地面。
         * 查询半径大于常见纸箱半宽，因此纸箱正下方被遮挡时仍能使用四周地面。
         */
        function estimateNavigationGroundHeight(x, y) {
            const groundCfg = PATH_GROUND_LIFT_CONFIG;
            const centerCellX = Math.floor(x / groundCfg.cellSizeM);
            const centerCellY = Math.floor(y / groundCfg.cellSizeM);
            const cellRadius = Math.ceil(
                NAV_OBSTACLE_CONFIG.localGroundQueryRadiusM / groundCfg.cellSizeM
            );
            const aggregated = new Map();

            for (let dx = -cellRadius; dx <= cellRadius; dx++) {
                for (let dy = -cellRadius; dy <= cellRadius; dy++) {
                    const distanceM = Math.hypot(dx, dy) * groundCfg.cellSizeM;
                    if (distanceM > NAV_OBSTACLE_CONFIG.localGroundQueryRadiusM + groundCfg.cellSizeM * 0.5) {
                        continue;
                    }

                    const cell = pathGroundHeightGrid.get(
                        getPathGroundCellKey(centerCellX + dx, centerCellY + dy)
                    );
                    if (!cell) continue;

                    cell.bins.forEach((votes, bin) => {
                        let item = aggregated.get(bin);
                        if (!item) {
                            item = { votes: 0, cells: 0 };
                            aggregated.set(bin, item);
                        }
                        item.votes += votes;
                        item.cells += 1;
                    });
                }
            }

            if (!aggregated.size) return null;

            const candidates = [];
            aggregated.forEach((item, bin) => {
                let votes = 0;
                let cells = 0;
                for (let offset = -1; offset <= 1; offset++) {
                    const neighbor = aggregated.get(bin + offset);
                    if (!neighbor) continue;
                    votes += neighbor.votes;
                    cells += neighbor.cells;
                }
                if (
                    votes >= NAV_OBSTACLE_CONFIG.minGroundVotes &&
                    cells >= NAV_OBSTACLE_CONFIG.minGroundCells
                ) {
                    candidates.push({ bin, votes, cells });
                }
            });
            if (!candidates.length) return null;

            // 地面应当是具有连续空间支持的最低稳定高度层；纸箱顶面虽然点数多，
            // 但其下方/周围更低的稳定地面层会优先被选中。
            candidates.sort((a, b) => a.bin - b.bin || b.votes - a.votes);
            const ground = candidates[0];
            let weightedHeight = 0;
            let totalVotes = 0;
            for (let offset = -1; offset <= 1; offset++) {
                const bin = ground.bin + offset;
                const item = aggregated.get(bin);
                if (!item) continue;
                weightedHeight += bin * groundCfg.heightBinM * item.votes;
                totalVotes += item.votes;
            }
            return totalVotes > 0
                ? weightedHeight / totalVotes
                : ground.bin * groundCfg.heightBinM;
        }

        function forEachValidCloudPoint(positionArray, componentCount, zOffset, visitor) {
            if (!positionArray || componentCount <= 0) return;
            const safeCount = Math.min(
                positionArray.length,
                Math.max(0, Math.floor(componentCount / 3) * 3)
            );
            for (let i = 0; i < safeCount; i += 3) {
                const x = Number(positionArray[i]);
                const y = Number(positionArray[i + 1]);
                const localZ = Number(positionArray[i + 2]);
                if (
                    !Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(localZ) ||
                    Math.abs(x) >= 9000 || Math.abs(y) >= 9000 || Math.abs(localZ) >= 9000
                ) {
                    continue;
                }
                visitor(x, y, localZ + Number(zOffset || 0));
            }
        }

        function inflateObstacleCells(baseObstacleCells) {
            const inflated = new Set();
            const cellRadius = Math.ceil(NAV_INFLATION_RADIUS_M / GRID_SIZE);

            baseObstacleCells.forEach(key => {
                const [gx, gy] = key.split(',').map(Number);
                for (let dx = -cellRadius; dx <= cellRadius; dx++) {
                    for (let dy = -cellRadius; dy <= cellRadius; dy++) {
                        if (Math.hypot(dx, dy) * GRID_SIZE > NAV_INFLATION_RADIUS_M) {
                            continue;
                        }
                        inflated.add(gridKey(gx + dx, gy + dy));
                    }
                }
            });
            return inflated;
        }

        function buildNavigationObstacleGrid(start, options = {}) {
            const includeHistory = options.includeHistory !== false;
            const logStats = options.logStats !== false;
            const baseObstacleCells = new Set();
            const groundCache = new Map();
            let validPointCount = 0;
            let obstaclePointCount = 0;
            let fallbackGroundCellCount = 0;

            const addPoint = (worldX, worldY, worldZ) => {
                validPointCount++;
                const g = worldToGrid(worldX, worldY);
                const key = gridKey(g.x, g.y);
                let groundZ;
                if (groundCache.has(key)) {
                    groundZ = groundCache.get(key);
                } else {
                    groundZ = estimateNavigationGroundHeight(worldX, worldY);
                    if (!Number.isFinite(groundZ)) {
                        groundZ = NAV_OBSTACLE_CONFIG.fallbackGroundWorldZ;
                        fallbackGroundCellCount++;
                    }
                    groundCache.set(key, groundZ);
                }

                const heightAboveGround = worldZ - groundZ;
                if (
                    heightAboveGround >= NAV_OBSTACLE_CONFIG.minHeightAboveGroundM &&
                    heightAboveGround <= NAV_OBSTACLE_CONFIG.maxHeightAboveGroundM
                ) {
                    baseObstacleCells.add(key);
                    obstaclePointCount++;
                }
            };

            const scanArr = scanMesh.geometry.attributes.position.array;
            forEachValidCloudPoint(
                scanArr,
                scanValidComponents,
                scanMesh.position.z,
                addPoint
            );

            if (includeHistory) {
                const mapArr = mapMesh.geometry.attributes.position.array;
                forEachValidCloudPoint(
                    mapArr,
                    mapStoredComponents,
                    mapMesh.position.z,
                    addPoint
                );
            }

            const obstacles = inflateObstacleCells(baseObstacleCells);
            const gridStart = worldToGrid(start.x, start.y);
            // 只释放机器狗中心所在的起始格，避免自身点云令 A* 无法起步；
            // 不清空周围膨胀区，近距离真实障碍仍会阻止规划。
            obstacles.delete(gridKey(gridStart.x, gridStart.y));

            const stats = {
                heightBandM: [
                    NAV_OBSTACLE_CONFIG.minHeightAboveGroundM,
                    NAV_OBSTACLE_CONFIG.maxHeightAboveGroundM
                ],
                validPointCount,
                obstaclePointCount,
                baseObstacleCellCount: baseObstacleCells.size,
                inflatedObstacleCellCount: obstacles.size,
                fallbackGroundCellCount,
                inflationRadiusM: Number(NAV_INFLATION_RADIUS_M.toFixed(3)),
                includeHistory
            };
            latestNavObstacleStats = stats;
            window.latestNavObstacleStats = { ...stats };
            if (includeHistory) {
                // 完整地图障碍快照供纯追踪选取前瞻点时做线段碰撞检查。
                // 仅实时帧构建的临时栅格不能覆盖掉这份完整快照。
                latestNavObstacles = obstacles;
            }

            if (logStats) {
                console.info("🧱 导航障碍栅格已更新:", stats);
                if (obstaclePointCount === 0) {
                    console.warn("⚠️ 当前未提取到离地 0.2m～1.2m 的障碍点，请确认点云和局部地面数据有效。");
                }
            }
            return { obstacles, stats };
        }

        function isGridLineCollisionFree(start, end, obstacles) {
            if (!start || !end) return false;
            const distance = Math.hypot(end.x - start.x, end.y - start.y);
            const stepM = GRID_SIZE * 0.25;
            const steps = Math.max(1, Math.ceil(distance / stepM));
            for (let i = 0; i <= steps; i++) {
                const t = i / steps;
                const x = start.x + (end.x - start.x) * t;
                const y = start.y + (end.y - start.y) * t;
                const g = worldToGrid(x, y);
                if (obstacles.has(gridKey(g.x, g.y))) return false;
            }
            return true;
        }

        function isControlSegmentCollisionFree(start, end) {
            return isGridLineCollisionFree(start, end, latestNavObstacles) &&
                isGridLineCollisionFree(start, end, latestRuntimeNavObstacles);
        }

        function isPathPrefixCollisionFree(start, path, obstacles, maxDistanceM = Infinity) {
            if (!start || !Array.isArray(path) || path.length === 0) return false;
            let previous = { x: Number(start.x), y: Number(start.y) };
            let checkedDistance = 0;

            for (const point of path) {
                const segmentLength = Math.hypot(point.x - previous.x, point.y - previous.y);
                if (!isGridLineCollisionFree(previous, point, obstacles)) return false;
                checkedDistance += segmentLength;
                if (checkedDistance >= maxDistanceM) break;
                previous = point;
            }
            return true;
        }

        function planPathAStar(start, end, options = {}) {
            console.log("🔍 A* 规划中...");
            const gridStart = worldToGrid(start.x, start.y);
            let gridEnd = worldToGrid(end.x, end.y);
            const { obstacles, stats } = buildNavigationObstacleGrid(start, {
                includeHistory: options.includeHistory !== false,
                logStats: options.logStats !== false
            });
            if (stats.validPointCount === 0) {
                alert("❌ 当前没有可用于规划的有效点云，请先启动点云雷达并等待地图显示。");
                return null;
            }

            const endKey = gridKey(gridEnd.x, gridEnd.y);
            if (obstacles.has(endKey)) {
                console.warn("目标点不可达，搜索附近...");
                let found = false, minD = Infinity, bestG = null;
                const searchRadiusCells = Math.ceil(
                    NAV_OBSTACLE_CONFIG.goalSearchRadiusM / GRID_SIZE
                );
                for (let dx = -searchRadiusCells; dx <= searchRadiusCells; dx++) {
                    for (let dy = -searchRadiusCells; dy <= searchRadiusCells; dy++) {
                        const nx = gridEnd.x + dx, ny = gridEnd.y + dy;
                        if (!obstacles.has(gridKey(nx, ny))) {
                            const d = dx * dx + dy * dy;
                            if (d < minD) {
                                minD = d;
                                bestG = { x: nx, y: ny };
                                found = true;
                            }
                        }
                    }
                }
                if (found) gridEnd = bestG;
                else { alert("❌ 目标区域完全堵塞！"); return null; }
            }

            const openSet = [];
            const closedSet = new Set();
            const cameFrom = {};
            const gScore = {};
            const fScore = {};

            const startKey = gridKey(gridStart.x, gridStart.y);
            openSet.push(gridStart);
            gScore[startKey] = 0;
            fScore[startKey] = Math.hypot(gridStart.x - gridEnd.x, gridStart.y - gridEnd.y);

            let iterations = 0;
            while (openSet.length > 0) {
                if (iterations++ > 5000) { console.warn("A* 超时"); break; }

                openSet.sort((a, b) =>
                    (fScore[gridKey(a.x, a.y)] ?? Infinity) -
                    (fScore[gridKey(b.x, b.y)] ?? Infinity)
                );
                const current = openSet.shift();
                const currentKey = gridKey(current.x, current.y);

                if (current.x === gridEnd.x && current.y === gridEnd.y) {
                    const path = [];
                    let currKey = currentKey;
                    while (currKey) {
                        const coords = currKey.split(',').map(Number);
                        path.unshift(gridToWorld(coords[0], coords[1]));
                        if (currKey === startKey) break;
                        currKey = cameFrom[currKey];
                    }
                    return simplifyPath(path, obstacles);
                }

                closedSet.add(currentKey);
                const neighbors = [
                    { x: 0, y: 1 }, { x: 0, y: -1 }, { x: 1, y: 0 }, { x: -1, y: 0 },
                    { x: 1, y: 1 }, { x: 1, y: -1 }, { x: -1, y: 1 }, { x: -1, y: -1 }
                ];

                for (const n of neighbors) {
                    const neighbor = { x: current.x + n.x, y: current.y + n.y };
                    const nKey = gridKey(neighbor.x, neighbor.y);
                    if (closedSet.has(nKey) || obstacles.has(nKey)) continue;

                    // 对角移动时两个相邻正交格都必须可通行，禁止从障碍拐角缝隙斜穿。
                    if (n.x !== 0 && n.y !== 0) {
                        if (
                            obstacles.has(gridKey(current.x + n.x, current.y)) ||
                            obstacles.has(gridKey(current.x, current.y + n.y))
                        ) {
                            continue;
                        }
                    }

                    const cost = (n.x === 0 || n.y === 0) ? 1.0 : Math.SQRT2;
                    const tentativeG = gScore[currentKey] + cost;

                    if (tentativeG < (gScore[nKey] ?? Infinity)) {
                        cameFrom[nKey] = currentKey;
                        gScore[nKey] = tentativeG;
                        fScore[nKey] = tentativeG + Math.hypot(neighbor.x - gridEnd.x, neighbor.y - gridEnd.y);
                        if (!openSet.some(o => o.x === neighbor.x && o.y === neighbor.y)) {
                            openSet.push(neighbor);
                        }
                    }
                }
            }
            alert("❌ 无法找到路径");
            return null;
        }

        function simplifyPath(path, obstacles) {
            if (path.length < 3) return path;
            const simplified = [path[0]];
            let anchorIndex = 0;

            // 只允许删除“锚点到候选点之间完全无碰撞”的中间点。
            // 这可以避免旧版按 0.4m 抽点后，连线重新切入纸箱或墙角。
            while (anchorIndex < path.length - 1) {
                let nextIndex = anchorIndex + 1;
                for (let candidateIndex = path.length - 1; candidateIndex > anchorIndex + 1; candidateIndex--) {
                    if (isGridLineCollisionFree(path[anchorIndex], path[candidateIndex], obstacles)) {
                        nextIndex = candidateIndex;
                        break;
                    }
                }
                simplified.push(path[nextIndex]);
                anchorIndex = nextIndex;
            }
            return simplified;
        }

        function calculateAvgError() {
            if (navOriginalPath.length < 2 || activeNavTrail.length === 0) {
                if (elNavError) elNavError.textContent = "--";
                return;
            }
            function distToSegmentSquared(p, v, w) {
                const l2 = (w.x - v.x) ** 2 + (w.y - v.y) ** 2;
                if (l2 === 0) return (p.x - v.x) ** 2 + (p.y - v.y) ** 2;
                let t = ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l2;
                t = Math.max(0, Math.min(1, t));
                return (p.x - (v.x + t * (w.x - v.x))) ** 2 + (p.y - (v.y + t * (w.y - v.y))) ** 2;
            }
            function distToPolyline(p, polyline) {
                let minDst2 = Infinity;
                for (let i = 0; i < polyline.length - 1; i++) {
                    const d2 = distToSegmentSquared(p, polyline[i], polyline[i + 1]);
                    if (d2 < minDst2) minDst2 = d2;
                }
                return Math.sqrt(minDst2);
            }

            const sampleCount = 50;
            let totalDistance = 0;
            let count = 0;
            const step = Math.max(1, Math.floor(activeNavTrail.length / sampleCount));

            for (let i = 0; i < activeNavTrail.length; i += step) {
                if (count >= sampleCount) break;
                totalDistance += distToPolyline(activeNavTrail[i], navOriginalPath);
                count++;
            }
            const avgError = count > 0 ? (totalDistance / count) : 0;
            if (elNavError) {
                elNavError.textContent = avgError.toFixed(3);
                elNavError.style.color = "#ffaa00";
            }
        }

        let isRotating = false, isPanning = false;
        let popup3DPos = null; // 用于记录点击的 3D 世界坐标
        let prevPos = { x: 0, y: 0 };
        const raycaster = new THREE.Raycaster();
        const mouse = new THREE.Vector2();

        // ==========================================
        // 新增：鼠标悬浮检测（电缆 Hover 变小手）
        // ==========================================
        const hoverMouse = new THREE.Vector2();
        renderer.domElement.addEventListener('mousemove', (e) => {
            if (isPicking) return; // 如果正在“选点导航”模式，不改变指针

            // 1. 获取鼠标在画布上的归一化坐标
            const rect = renderer.domElement.getBoundingClientRect();
            hoverMouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
            hoverMouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;

            // 2. 射线检测是否碰到了电缆
            if (currentCableMesh) {
                raycaster.setFromCamera(hoverMouse, camera);
                const intersects = raycaster.intersectObject(currentCableMesh);

                // 如果鼠标正好在电缆上，变成小手；否则恢复默认
                if (intersects.length > 0) {
                    renderer.domElement.style.cursor = 'pointer';
                } else {
                    renderer.domElement.style.cursor = 'default';
                }
            }
        });
        // 1. 选点与电缆信息查询
        renderer.domElement.addEventListener('click', (e) => {
            const rect = renderer.domElement.getBoundingClientRect();
            mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
            mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;

            raycaster.setFromCamera(mouse, camera);

            // --- A. 如果当前处于 [📍选点] 导航模式 ---
            if (isPicking) {
                const intersects = raycaster.intersectObject(clickPlane);
                if (intersects.length > 0) {
                    const p = intersects[0].point;
                    const target = { x: p.x, y: p.y };
                    const path = planPathAStar(currentRobotPose, target);

                    if (path && path.length > 0) {
                        navPath = path;
                        navGoal = path[path.length - 1];
                        flagObj.position.set(navGoal.x, navGoal.y, 0);
                        flagObj.visible = true;

                        isPicking = false;
                        const btn = document.getElementById('btnSetNavPoint');
                        if (btn) btn.classList.remove('active');
                        renderer.domElement.style.cursor = 'default';

                        if (elNavError) elNavError.textContent = "--";
                        updateNavLine();
                        updateDistanceUI();
                    }
                }
                return;
            }

            // --- B. 如果是非选点模式，严格检测是否点中了黄色电缆 ---
            if (currentCableMesh) {
                // --- B. 如果是非选点模式，检测点击电缆或破损点 ---
                let hitPoint = null;
                let clickedFaultId = null;

                // 1. 优先检测是否点中了破损点 (遍历 faultMarkerGroup)
                const intersectsFault = raycaster.intersectObject(faultMarkerGroup, true);
                if (intersectsFault.length > 0) {
                    hitPoint = intersectsFault[0].point;
                    // 向上寻找绑定的 faultId
                    let obj = intersectsFault[0].object;
                    while (obj && !obj.userData.faultId && obj.parent) {
                        obj = obj.parent;
                    }
                    clickedFaultId = obj.userData?.faultId;
                }
                // 2. 否则检测是否点中了黄色电缆
                else if (currentCableMesh) {
                    const intersectsCable = raycaster.intersectObject(currentCableMesh);
                    if (intersectsCable.length > 0) {
                        hitPoint = intersectsCable[0].point;
                    }
                }

                // 只要点中了有效物体（电缆或破损点）
                if (hitPoint) {
                    popup3DPos = hitPoint.clone();

                    let minDistanceToSegment = Infinity;
                    let bestAccumulatedDist = 0;
                    let currentAccumulatedDist = 0;
                    let hitDepth = null; // ✅ 新增：用于暂存点击位置的深度

                    if (window.savedCablePath && window.savedCablePath.length > 1) {
                        for (let i = 0; i < window.savedCablePath.length - 1; i++) {
                            const p1 = window.savedCablePath[i];
                            const p2 = window.savedCablePath[i + 1];
                            const dx = p2.x - p1.x, dy = p2.y - p1.y, dz = p2.z - p1.z;
                            const segLenSq = dx * dx + dy * dy + dz * dz;
                            const segLen = Math.sqrt(segLenSq);

                            let t = 0;
                            if (segLenSq > 0) {
                                const dot = ((hitPoint.x - p1.x) * dx + (hitPoint.y - p1.y) * dy + (hitPoint.z - p1.z) * dz);
                                t = Math.max(0, Math.min(1, dot / segLenSq));
                            }
                            const projX = p1.x + t * dx, projY = p1.y + t * dy, projZ = p1.z + t * dz;
                            const distToProj = Math.hypot(hitPoint.x - projX, hitPoint.y - projY, hitPoint.z - projZ);

                            if (distToProj < minDistanceToSegment) {
                                minDistanceToSegment = distToProj;
                                bestAccumulatedDist = currentAccumulatedDist + (t * segLen);
                                // ✅ 获取该线段起点的深度属性
                                hitDepth = p1.userData ? p1.userData.depth : null;
                            }
                            currentAccumulatedDist += segLen;
                        }
                    }

                    let gpsText = "未就绪 (等待标定)";
                    const calculatedGps = window.slamToGps(hitPoint.x, hitPoint.y);
                    // ✅ 修改：将弹窗的经纬度显示精度提升至小数点后 8 位
                    if (calculatedGps) gpsText = `${calculatedGps.lat.toFixed(8)}, ${calculatedGps.lon.toFixed(8)}`;
                    else if (window.currentGps) gpsText = "标定中...请移动机器狗>1米";

                    document.getElementById('popupDist').textContent = bestAccumulatedDist.toFixed(2);

                    // ✅ 修改：将弹窗深度更新为绑定的真实数值
                    // 假设 Python 发送的深度单位是 cm，如果是 m 请去掉 / 100
                    if (hitDepth != null) {
                        document.getElementById('popupDepth').textContent = (hitDepth / 100).toFixed(2) + " m";
                    } else {
                        document.getElementById('popupDepth').textContent = "未检测到";
                    }

                    document.getElementById('popupGps').textContent = gpsText;

                    const elPopup = document.getElementById('cableInfoPopup');
                    if (elPopup) elPopup.style.display = 'block';


                    // ✅ 核心：更新右上角固定的实景抓拍预留区
                    if (clickedFaultId && window.faultImagesMap && window.faultImagesMap.has(clickedFaultId)) {
                        const imgs = window.faultImagesMap.get(clickedFaultId);
                        showFaultImagesInModal(imgs);
                    }
                    return;
                }
            }

            // --- C. 如果点击了空白处，隐藏弹窗 ---
            const elPopup = document.getElementById('cableInfoPopup');
            if (elPopup && e.target.tagName !== 'BUTTON') {
                elPopup.style.display = 'none';
            }
        });

        function updateNavLine() {
            if (navPath.length > 0 && flagObj.visible) {
                const pts = [];
                pts.push(new THREE.Vector3(currentRobotPose.x, currentRobotPose.y, 0.2));
                navPath.forEach(p => pts.push(new THREE.Vector3(p.x, p.y, 0.2)));
                navLine.geometry.setFromPoints(pts);
                navLine.computeLineDistances();
                navLine.visible = true;
            } else {
                navLine.visible = false;
            }
        }

        // 2. 视角
        renderer.domElement.addEventListener('mousedown', e => {
            if (isPicking) return;
            e.preventDefault();
            if (e.button === 0) isRotating = true;
            else if (e.button === 1) isPanning = true;
            prevPos = { x: e.offsetX, y: e.offsetY };
        });

        renderer.domElement.addEventListener('mousemove', e => {
            if (isPicking) return;
            const dx = e.offsetX - prevPos.x;
            const dy = e.offsetY - prevPos.y;
            prevPos = { x: e.offsetX, y: e.offsetY };

            if (isRotating) {
                theta -= dx * 0.01;
                phi -= dy * 0.01;
                phi = Math.max(0.1, Math.min(Math.PI / 2 - 0.1, phi));
                updateCamera();
            } else if (isPanning) {
                const speed = radius * 0.002;
                const forward = new THREE.Vector3();
                camera.getWorldDirection(forward);
                const right = new THREE.Vector3().crossVectors(forward, camera.up).normalize();
                const forwardXY = new THREE.Vector3(forward.x, forward.y, 0).normalize();
                orbitTarget.addScaledVector(right, -dx * speed);
                orbitTarget.addScaledVector(forwardXY, dy * speed);
                updateCamera();
            }
        });

        window.addEventListener('mouseup', () => { isRotating = false; isPanning = false; });
        renderer.domElement.addEventListener('wheel', e => {
            e.preventDefault();
            radius *= (e.deltaY > 0 ? 1.1 : 0.9);
            radius = Math.max(1, Math.min(100, radius));
            updateCamera();
        });

        // 3. 按钮
        const btnSet = document.getElementById('btnSetNavPoint');
        const btnStart = document.getElementById('btnStartNav');
        const btnClear = document.getElementById('btnClearNav');

        function setNavigationButtonState(navigating) {
            if (!btnStart) return;
            btnStart.innerText = navigating ? "■ 停止" : "▶ 导航";
            btnStart.title = navigating ? "立即停止并退出自主导航" : "开始前往目标点";
            btnStart.classList.toggle('success', !navigating);
            btnStart.classList.toggle('stop-active', navigating);
            btnStart.setAttribute('aria-pressed', navigating ? 'true' : 'false');
        }
        setNavigationButtonState(false);

        if (btnSet) {
            btnSet.onclick = function () {
                isPicking = !isPicking;
                if (isPicking) {
                    this.classList.add('active');
                    renderer.domElement.style.cursor = 'crosshair';
                } else {
                    this.classList.remove('active');
                    renderer.domElement.style.cursor = 'default';
                }
            };
        }

        if (btnClear) {
            btnClear.onclick = function () {
                stopNav(false);
                navPath = [];
                navGoal = null;
                flagObj.visible = false;
                navLine.visible = false;
                updateDistanceUI();
                if (elNavError) elNavError.textContent = "--";
            };
        }

        if (btnStart) {
            btnStart.onclick = function () {
                if (isNavigating) {
                    console.log("🛑 用户点击导航按钮停止自主导航");
                    stopNav(false);
                    setStatus("自主导航已停止，机器狗已退出自主导航模式。", "warning");
                    return;
                }
                if (navPath.length === 0) {
                    alert("请先点击 [📍选点] 生成避障路径！");
                    return;
                }
                startNav();
            };
        }

        const stopBtnIds = ['btnStopMove', 'btnSideStop'];
        stopBtnIds.forEach(id => {
            const btn = document.getElementById(id);
            if (btn) {
                btn.addEventListener('click', function () {
                    if (isNavigating) {
                        console.log("🛑 检测到手动停止");
                        stopNav(false);
                    }
                });
            }
        });

        function startNav() {
            console.log("🚀 开始导航...");

            // 选点后到点击“开始”之间，纸箱可能出现或点云可能继续更新。
            // 常规导航必须基于启动瞬间的最新点云重新规划一次。
            if (!isCableTrackingMode && !isFaultFindingMode && navGoal) {
                const refreshedPath = planPathAStar(currentRobotPose, navGoal, {
                    includeHistory: true,
                    logStats: true
                });
                if (!refreshedPath || refreshedPath.length === 0) {
                    setStatus("启动导航失败：最新点云中没有安全可行路径。", "error");
                    return;
                }
                navPath = refreshedPath;
                navGoal = refreshedPath[refreshedPath.length - 1];
                flagObj.position.set(navGoal.x, navGoal.y, 0);
                updateNavLine();
                updateDistanceUI();
            }

            isNavigating = true;
            lastRuntimeObstacleCheckAt = 0;
            latestRuntimeNavObstacles = new Set();
            if (!isCableTrackingMode && !isFaultFindingMode) {
                setNavigationButtonState(true);
            }

            // 只有当不是电缆跟踪且不是寻找破损模式时，才记录 A* 原始路径
            if (!isCableTrackingMode && !isFaultFindingMode) {
                navOriginalPath = [...navPath];
                navOriginalPath.unshift({ x: currentRobotPose.x, y: currentRobotPose.y });
                activeNavTrail = [];
                if (elNavError) elNavError.textContent = "运行中...";
            }

            if (typeof sendMoveCommand === 'function') sendMoveCommand(0, 0, 0, 0.0, 3.0, true);
            if (navInterval) clearInterval(navInterval);
            navInterval = setInterval(controlLoop, 100);
        }

        function stopNav(isSuccess) {
            const wasNavigating = isNavigating || navInterval !== null;
            isNavigating = false;
            if (navInterval) { clearInterval(navInterval); navInterval = null; }
            if (typeof sendMoveCommand === 'function') sendMoveCommand(0, 0, 0, 0.0, 0.0, true);
            // 手动停止时补发一帧零速度，降低单帧 WebSocket/UDP 丢包导致的继续运动风险。
            if (wasNavigating && typeof sendMoveCommand === 'function') {
                window.setTimeout(() => {
                    if (!isNavigating) {
                        sendMoveCommand(0, 0, 0, 0.0, 0.0, false);
                    }
                }, 50);
            }
            setNavigationButtonState(false);
            console.log("🛑 导航停止");

            // ✅✅✅ 修正：电缆跟踪与寻找破损模式停止逻辑
            if (isCableTrackingMode || isFaultFindingMode) {
                const wasFaultFindingMode = isFaultFindingMode;
                if (cableTrackingState === 'TO_START' && isSuccess) {
                    console.log("✅ 已到达起始切入点，开始沿线运行...");
                    cableTrackingState = 'FOLLOWING';

                    if (isCableTrackingMode && window.savedCablePath) {
                        navPath = window.savedCablePath.map(p => ({ x: p.x, y: p.y }));
                    } else if (isFaultFindingMode && window.offsetCablePath) {
                        navPath = window.offsetCablePath; // 使用计算好的偏移路径
                    }
                    startNav();
                } else {
                    console.log("⏹️ 自动跟踪/寻找任务结束或停止");

                    isCableTrackingMode = false;
                    isFaultFindingMode = false;
                    cableTrackingState = 'IDLE';
                    if (wasFaultFindingMode) resetFaultFindingRuntimeState();

                    // 恢复两个按钮 UI
                    const btn1 = document.getElementById("btnTrackCableOverlay");
                    if (btn1) {
                        btn1.innerText = "⚡ 跟踪电缆";
                        btn1.style.color = "#00ccff";
                        btn1.style.border = "1px solid #00ccff";
                    }
                    const btn2 = document.getElementById("btnFindFaultOverlay");
                    if (btn2) {
                        btn2.innerHTML = '<span class="btn-icon">🔍</span> 寻找破损点';
                    }
                }
            } else {
                // 普通 A* 导航停止
                if (isSuccess) calculateAvgError();
            }
        }

        function resetFaultFindingRuntimeState() {
            bsState = 'INACTIVE';
            bsGoodDist = 0;
            bsBadDist = 0;
            bsTargetDist = 0;
            bsWaitEndTime = 0;
            bsIterationCount = 0;
            lastFaultRobotDist = -1;
            offsetPathDists = [];
            maxFaultIdx = -1;
            bsFootprints = [];
            bsLastProjectionS = null;
            bsPathAligning = false;
            resetBinaryMotionController();
            window.g_faultTriggeredInBS = false;
        }

        window.stopFaultFindingMode = function (reason = "manual") {
            if (!isFaultFindingMode) return false;
            console.log(reason === "emergency"
                ? "🛑 急停停止寻找破损点模式"
                : "🛑 停止寻找破损点模式");
            stopNav(false);
            return true;
        };

        // ✅ 核心控制：纯追踪 (Pure Pursuit) + 二分法拦截
        function controlLoop() {
            if (navPath.length === 0) { stopNav(true); return; }

            // 常规选点导航每 500ms 用最新一帧点云检查前方 2m。
            // 一旦原路径被新障碍占用，先发停车指令，再基于历史+实时点云重规划。
            if (!isCableTrackingMode && !isFaultFindingMode && navGoal) {
                const now = Date.now();
                if (now - lastRuntimeObstacleCheckAt >= NAV_OBSTACLE_CONFIG.runtimeCheckIntervalMs) {
                    lastRuntimeObstacleCheckAt = now;
                    const liveGrid = buildNavigationObstacleGrid(currentRobotPose, {
                        includeHistory: false,
                        logStats: false
                    });
                    latestRuntimeNavObstacles = liveGrid.obstacles;
                    const prefixIsSafe = isPathPrefixCollisionFree(
                        currentRobotPose,
                        navPath,
                        liveGrid.obstacles,
                        NAV_OBSTACLE_CONFIG.runtimeLookaheadM
                    );

                    if (!prefixIsSafe) {
                        console.warn("🛑 最新点云占用了前方路径，停车并重新规划。");
                        if (typeof sendMoveCommand === 'function') {
                            sendMoveCommand(0, 0, 0, 0.0, 0.0, false);
                        }

                        const replannedPath = planPathAStar(currentRobotPose, navGoal, {
                            includeHistory: true,
                            logStats: true
                        });
                        if (!replannedPath || replannedPath.length === 0) {
                            stopNav(false);
                            setStatus("前方出现障碍且无法重规划，机器狗已停车。", "error");
                            return;
                        }

                        navPath = replannedPath;
                        navGoal = replannedPath[replannedPath.length - 1];
                        flagObj.position.set(navGoal.x, navGoal.y, 0);
                        updateNavLine();
                        updateDistanceUI();
                        setStatus("检测到新障碍，已停车并完成绕障重规划。", "warning");
                        return;
                    }
                }
            }

            // =========================================================
            // 🌟🌟🌟 [修改] 步进等待 (Step-and-Wait) + 二分法定位接管
            // =========================================================
            if (isFaultFindingMode && window.offsetCablePath && window.offsetCablePath.length > 0) {
                if (offsetPathDists.length === 0) computePathDists(window.offsetCablePath);
                const currentProjection = getCurrentOffsetPathProjection();
                const currentDist = currentProjection
                    ? currentProjection.s
                    : getDistOnPath(currentRobotPose, window.offsetCablePath);

                // 确保机器狗已经并入偏移路径，而不是还在去起点的路上
                if (cableTrackingState === 'FOLLOWING') {

                    // 1. 初始化步进阶段
                    if (bsState === 'INACTIVE') {
                        bsState = 'STEP_FORWARD';
                        bsTargetDist = currentDist + 0.8; // 设定 0.8m 后的目标距离
                        bsGoodDist = currentDist;         // 默认当前起点为有信号区域
                        console.log(`🚀 [破损点步进] 开始步进，目标距离: ${bsTargetDist.toFixed(2)}m`);
                    }

                    // 2. 步进前行阶段 (STEP_FORWARD)
                    else if (bsState === 'STEP_FORWARD') {

                        // ✅ [新增] 运动中检测到信号：动态推迟目标点 (无缝顺延，不停车)
                        if (window.g_faultTriggeredInBS) {
                            console.log(`⚡ [破损点步进] 运动中 (${currentDist.toFixed(2)}m处) 检测到信号！重新以此点推算 0.8m 目标...`);
                            bsGoodDist = currentDist;         // 刷新有信号边界为当前位置
                            bsTargetDist = currentDist + 0.8; // 目标直接顺延，从当前点再往后推 0.8m
                            window.g_faultTriggeredInBS = false; // 消费掉这个信号
                            // 这里我们不 return，让代码继续往下走，这样机器狗就不会有任何停顿，直接平滑继续走
                        }

                        if (currentDist >= bsTargetDist) {
                            // 到达最新节点，立即停车并开始等待倒计时
                            bsState = 'STEP_WAIT';
                            bsWaitEndTime = Date.now() + getFaultSearchWaitMs();
                            window.g_faultTriggeredInBS = false; // 清空这 0.8m 移动中可能产生的历史信号干扰
                            sendMoveCommand(0, 0, 0, 0.0, 0.0, true);
                            console.log(`⏱️ [破损点步进] 到达 ${currentDist.toFixed(2)}m，停车等待 ${getFaultSearchWaitLabel()} 检测信号...`);
                            return; // 拦截结束，不执行下面的纯追踪
                        }
                        // 还没走到 0.8m，这里不 return，放行去执行下方的常规纯追踪 (前进)
                    }

                    // 3. 停车等待阶段 (STEP_WAIT)
                    else if (bsState === 'STEP_WAIT') {
                        sendMoveCommand(0, 0, 0, 0.0, 0.0, false); // 发送心跳包，保持原地停车

                        // A. 等待期间检测到信号：立即中断等待，继续前行
                        if (window.g_faultTriggeredInBS) {
                            console.log(`✅ [破损点步进] 收到破损点信号！立即继续前进 0.8m...`);
                            bsGoodDist = currentDist;         // 更新有信号边界
                            bsTargetDist = currentDist + 0.8; // 目标再推 0.8m
                            bsState = 'STEP_FORWARD';
                            window.g_faultTriggeredInBS = false;
                            return;
                        }

                        // B. 等待时间满，未检测到信号：触发二分法
                        if (Date.now() >= bsWaitEndTime) {
                            console.log(`🛑 [二分定位触发] ${getFaultSearchWaitLabel()}无信号！进入二分法寻找 (搜索区间: ${bsGoodDist.toFixed(2)}m ~ ${currentDist.toFixed(2)}m)`);
                            bsBadDist = currentDist; // 更新无信号边界
                            bsState = 'MOVING';      // 直接跳转到二分法移动阶段
                            bsIterationCount = 0;
                            bsTargetDist = (bsGoodDist + bsBadDist) / 2.0; // 目标锁定为中点 (比如倒退 1m)
                            window.g_faultTriggeredInBS = false;
                            return;
                        }

                        // C. 还在等待时间内，继续等待
                        return;
                    }

                    // 4. 二分法接管阶段 (复用原来的逻辑)
                    else if (bsState === 'MOVING' || bsState === 'EVAL_WAIT' || bsState === 'DONE' || bsState === 'INIT_WAIT') {
                        handleBinarySearchControl(currentDist, currentProjection);
                        return;
                    }
                }
            }

            // =========================================================
            // ✅ [新增] 剔除身后的历史轨迹点，彻底解决机器狗 180 度掉头问题
            // =========================================================
            // 1. 寻找机器狗在当前剩余路径上“物理距离最近”的点
            let closestIdx = 0;
            let minD = Infinity;
            for (let i = 0; i < navPath.length; i++) {
                const d = Math.hypot(navPath[i].x - currentRobotPose.x, navPath[i].y - currentRobotPose.y);
                if (d < minD) {
                    minD = d;
                    closestIdx = i;
                }
            }

            // 2. 将最近点之前的所有“已路过”点全部从数组中销毁
            // 这样 navPath[0] 永远是机器狗身旁或正前方的点，绝不回头
            if (closestIdx > 0) {
                navPath.splice(0, closestIdx);
            }
            // =========================================================

            // 3. 在清理后的、只包含前方的路径中，正常寻找前瞻目标点
            const LOOKAHEAD_DIST = 0.8;
            let targetIdx = 0;
            let target = navPath[0];
            let hasCollisionFreeTarget = false;

            for (let i = 0; i < navPath.length; i++) {
                const p = navPath[i];
                const d = Math.hypot(p.x - currentRobotPose.x, p.y - currentRobotPose.y);

                // 普通选点导航的前瞻点必须能从机器狗当前位置直线安全到达。
                // 遇到障碍拐角时保留并瞄准拐角前的安全点，禁止纯追踪跨点切弯。
                if (
                    !isCableTrackingMode && !isFaultFindingMode &&
                    !isControlSegmentCollisionFree(currentRobotPose, p)
                ) {
                    break;
                }
                hasCollisionFreeTarget = true;
                target = p;
                targetIdx = i;

                if (d > LOOKAHEAD_DIST) {
                    break;
                }
            }

            if (!hasCollisionFreeTarget) {
                if (typeof sendMoveCommand === 'function') {
                    sendMoveCommand(0, 0, 0, 0.0, 0.0, false);
                }
                console.warn("🛑 当前没有可直达的安全前瞻点，等待下一次障碍复检与重规划。");
                return;
            }

            // 4. 剔除目标点之前的过渡点，使得路径持续向前推进
            if (targetIdx > 0) {
                navPath.splice(0, targetIdx);
            }

            const dx = target.x - currentRobotPose.x;
            const dy = target.y - currentRobotPose.y;
            const dist = Math.sqrt(dx * dx + dy * dy);

            if (navPath.length === 1 && dist < 0.2) {
                stopNav(true);
                return;
            }

            const targetYaw = Math.atan2(dy, dx);
            let diff = targetYaw - currentRobotPose.yaw;
            while (diff > Math.PI) diff -= 2 * Math.PI;
            while (diff < -Math.PI) diff += 2 * Math.PI;

            let v = 0, w = 0;
            if (Math.abs(diff) > 0.8) {
                // 角度偏差大于 0.8 弧度时，降速到 0.1 m/s
                v = 0.1;
                w = diff > 0 ? 0.6 : -0.6;
            } else {
                // 动态调速：偏角越小，速度越接近 0.4 m/s
                v = 0.4 * (1.0 - Math.abs(diff) / 1.5);
                // 最小保底速度 0.15 m/s
                v = Math.max(0.15, v);
                w = diff * 1.5;
            }

            if (typeof sendMoveCommand === 'function') {
                sendMoveCommand(v, w, 0.0, 0.0, 0.0, false);
            }
        }
        function handleBinarySearchControl(currentDist, currentProjection = null) {
            const now = Date.now();

            // 阶段 1: 触发后的首次 5秒 等待
            if (bsState === 'INIT_WAIT') {
                if (now < bsWaitEndTime) return; // 还在等

                // 5秒后仍无信号，开始倒车寻找中间点
                bsTargetDist = (bsGoodDist + bsBadDist) / 2.0; // 首次目标就是倒退一半 (1m)
                bsState = 'MOVING';
                console.log(`🔙 [二分定位] 开始倒退至 ${bsTargetDist.toFixed(2)}m 处...`);
                return;
            }

            // 阶段 2: 到达某个测试点后，按页面参数等待并评估是否有信号
            if (bsState === 'EVAL_WAIT') {
                // ✅ 修复 1：补充停车心跳包，防止单次指令丢失或物理惯性导致的持续滑行（倒退过头）
                if (typeof sendMoveCommand === 'function') {
                    sendMoveCommand(0, 0, 0, 0.0, 0.0, false);
                }

                const signalDetectedNow = window.g_faultTriggeredInBS;
                const timeIsUp = (now >= bsWaitEndTime);

                if (!signalDetectedNow && !timeIsUp) {
                    return;
                }

                // --- 评估结算阶段：收到信号了，或者时间终于到了 ---
                if (signalDetectedNow) {
                    console.log(`✅ [二分定位] ⚡ 提前在 ${currentDist.toFixed(2)}m 处检测到信号！立刻中断等待，更新有信号边界。`);
                    bsGoodDist = currentDist; // 更新有信号边界
                } else {
                    console.log(`❌ [二分定位] ⏳ 等待 ${getFaultSearchWaitLabel()} 满，在 ${currentDist.toFixed(2)}m 处无信号！更新无信号边界。`);
                    bsBadDist = currentDist;  // 更新无信号边界
                }

                bsIterationCount += 1;
                console.log(`🎯 [二分定位] 已完成第 ${bsIterationCount}/${BS_MAX_ITERATIONS} 次二分评估，当前区间: ${Math.min(bsGoodDist, bsBadDist).toFixed(2)}m ~ ${Math.max(bsGoodDist, bsBadDist).toFixed(2)}m`);

                if (bsIterationCount >= BS_MAX_ITERATIONS) {
                    console.log(`🎯 [二分定位] 已完成 ${BS_MAX_ITERATIONS} 次二分法寻找，最终参考破损点位于: ${bsGoodDist.toFixed(2)}m`);
                    bsState = 'DONE';
                    setStatus(`🎯 破损点定位完成：已完成 ${BS_MAX_ITERATIONS} 次二分法寻找`, "success");
                    addSystemMessage("破损点定位已完成。", "success");
                    stopNav(true);
                    return;
                }

                // 计算新的目标点并进入移动状态
                bsTargetDist = (bsGoodDist + bsBadDist) / 2.0;
                bsState = 'MOVING';

                // 清理上一轮的信号标志，准备前往下一个点
                window.g_faultTriggeredInBS = false;
                const action = (bsTargetDist > currentDist) ? "向前移动" : "倒退移动";
                console.log(`🤖 [二分定位] 准备${action}至: ${bsTargetDist.toFixed(2)}m (搜索区间: ${Math.min(bsGoodDist, bsBadDist).toFixed(2)} ~ ${Math.max(bsGoodDist, bsBadDist).toFixed(2)})`);
                return;
            }

            // 阶段 3: 向目标点移动 (支持倒车防偏移)
            if (bsState === 'MOVING') {
                const projection = currentProjection || getCurrentOffsetPathProjection();
                if (!projection) {
                    sendMoveCommand(0, 0, 0, 0.0, 0.0, false);
                    console.warn("⚠️ [二分定位] 无法将当前位置投影到偏移路径，已暂停移动。");
                    return;
                }

                const activeCurrentDist = projection.s;
                const distError = bsTargetDist - activeCurrentDist;

                // 每一轮二分移动开始时锁定移动方向，并初始化对应的航向控制器。
                if (typeof window.bsInitialMoveDir === 'undefined') {
                    initializeBinaryMotionController(Math.sign(distError));
                }
                const currentDir = Math.sign(distError);
                const isOvershot = (window.bsInitialMoveDir !== 0 && currentDir !== 0 && currentDir !== window.bsInitialMoveDir);
                const lateralError = Math.abs(projection.crossTrackM);

                // 里程到位且横向误差收敛后，才进入原地等待检测；避免“到里程但不在轨迹上”。
                if ((Math.abs(distError) < BS_PATH_CONTROL.arriveDistM && lateralError <= BS_PATH_CONTROL.arriveCrossTrackM) ||
                    (isOvershot && lateralError <= BS_PATH_CONTROL.arriveCrossTrackM)) {
                    sendMoveCommand(0, 0, 0, 0.0, 0.0, true); // 停车
                    bsState = 'EVAL_WAIT';
                    bsWaitEndTime = Date.now() + getFaultSearchWaitMs();

                    resetBinaryMotionController();
                    // ✅ 修复 2：到达目标点时，强制清空移动过程中可能积攒的“脏信号”
                    window.g_faultTriggeredInBS = false;

                    console.log(`⏱️ [二分定位] 到达(或穿透)目标点，原地等待 ${getFaultSearchWaitLabel()} 检测信号...`);
                    return;
                }

                const moveNow = Date.now();
                if (!isBinaryPoseFresh(moveNow)) {
                    sendMoveCommand(0, 0, 0, 0.0, 0.0, false);
                    if (moveNow - bsLastPoseStaleWarningTime > 1000) {
                        console.warn("⚠️ [二分定位] SLAM 位姿反馈超时，已暂停移动等待最新航向。");
                        bsLastPoseStaleWarningTime = moveNow;
                    }
                    return;
                }

                const command = computeBinaryPathFollowCommand(bsTargetDist, projection);
                if (!command) {
                    sendMoveCommand(0, 0, 0, 0.0, 0.0, false);
                    console.warn("⚠️ [二分定位] 路径闭环控制计算失败，已暂停移动。");
                    return;
                }

                if (typeof sendMoveCommand === 'function') {
                    sendMoveCommand(command.v, command.w, command.y, 0.0, 0.0, false);
                }
            }
        }

        // 5. 旧清除按钮 (修改后：只清除蓝色机器狗轨迹)
        const btnClearDog = document.getElementById("btnClearDogPath");
        if (btnClearDog) {
            btnClearDog.onclick = function () {
                console.log("🧹 清除机器狗轨迹...");

                // 1. 清除轨迹数据点
                trailPoints.length = 0;
                lastTrailPos.set(-999, -999, 0);

                // 2. 清除轨迹 Mesh
                if (currentTrailMesh) {
                    trailGroup.remove(currentTrailMesh);
                    if (currentTrailMesh.geometry) currentTrailMesh.geometry.dispose();
                    if (currentTrailMesh.material) currentTrailMesh.material.dispose();
                    currentTrailMesh = null;
                }

                // ✅ 已删除 faultMarkerGroup 的清除逻辑，现在由 btnClearFaults 专用按钮负责
            };
        }
        // ✅ 新增: 存储破损点图片的全局字典
        window.faultImagesMap = new Map();


        // ✅ 新增: 清除点云数据按钮逻辑
        const btnClearCloud = document.getElementById("btnClearCloud");
        if (btnClearCloud) {
            btnClearCloud.onclick = function () {
                console.log("🧹 清除所有点云数据...");
                const scanAttr = scanMesh.geometry.attributes.position;
                const scanArr = scanAttr.array;
                for (let i = 0; i < scanArr.length; i++) scanArr[i] = 99999;
                scanAttr.needsUpdate = true;

                const mapAttr = mapMesh.geometry.attributes.position;
                const mapArr = mapAttr.array;
                for (let i = 0; i < mapArr.length; i++) mapArr[i] = 99999;
                markFullAttributeUpdate(mapAttr);
                markFullAttributeUpdate(mapMesh.geometry.attributes.color);

                mapCursor = 0;
                mapStoredComponents = 0;
                scanValidComponents = 0;
                pendingMapUpdateRanges = [];
                pathGroundHeightGrid.clear();
                refreshRenderedPathsForGroundLift(true);
                lastKeyframePose = { x: -999, y: -999 };
            };
        }

        // ====== 👇 修复：保存当前前端可见点云为 PCD 文件 👇 ======
        // 原问题：旧代码请求 /api/slam/save_pcd，但后端是另开 WebSocket 缓存点云；
        // 如果后端缓存为空，就会下载到只有 PCD 头部的小文件。
        // 这里改为直接导出当前页面 mapMesh 中已经渲染出来的历史点云，所见即所得。
        function clampColorToByte(v) {
            const n = Number(v);
            if (!Number.isFinite(n)) return 255;
            return Math.max(0, Math.min(255, Math.round(n * 255)));
        }

        function isValidPointCoord(x, y, z) {
            return Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(z)
                && Math.abs(x) < 9000 && Math.abs(y) < 9000 && Math.abs(z) < 9000;
        }

        function roundNumber(value, digits = 8) {
            const n = Number(value);
            if (!Number.isFinite(n)) return null;
            return Number(n.toFixed(digits));
        }

        function clonePlainGps(gps) {
            if (!gps) return null;
            const lat = Number(gps.lat);
            const lon = Number(gps.lon);
            if (!Number.isFinite(lat) || !Number.isFinite(lon)) return null;
            return {
                lat: roundNumber(lat, 10),
                lon: roundNumber(lon, 10)
            };
        }

        function clonePlainSlam(pt) {
            if (!pt) return null;
            const x = Number(pt.x);
            const y = Number(pt.y);
            const z = Number(pt.z ?? 0);
            if (!Number.isFinite(x) || !Number.isFinite(y)) return null;
            return {
                x: roundNumber(x, 5),
                y: roundNumber(y, 5),
                z: roundNumber(z, 5)
            };
        }

        function computeCurrentVisibleMapBounds() {
            if (!mapMesh || !mapMesh.geometry || !mapMesh.geometry.attributes.position) {
                return null;
            }

            const pos = mapMesh.geometry.attributes.position.array;
            let minX = Infinity, minY = Infinity, minZ = Infinity;
            let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
            let count = 0;

            for (let i = 0; i < pos.length; i += 3) {
                const x = Number(pos[i]);
                const y = Number(pos[i + 1]);
                const z = Number(pos[i + 2]);
                if (!isValidPointCoord(x, y, z)) continue;

                minX = Math.min(minX, x);
                minY = Math.min(minY, y);
                minZ = Math.min(minZ, z);
                maxX = Math.max(maxX, x);
                maxY = Math.max(maxY, y);
                maxZ = Math.max(maxZ, z);
                count++;
            }

            if (count <= 0) return null;

            const center = {
                x: (minX + maxX) / 2,
                y: (minY + maxY) / 2,
                z: (minZ + maxZ) / 2
            };

            const slamCorners = [
                { name: "minX_minY", x: minX, y: minY, z: minZ },
                { name: "maxX_minY", x: maxX, y: minY, z: minZ },
                { name: "maxX_maxY", x: maxX, y: maxY, z: minZ },
                { name: "minX_maxY", x: minX, y: maxY, z: minZ },
                { name: "center", ...center }
            ];

            const gpsCorners = (typeof window.slamToGps === "function" && window.isGpsCalibrated)
                ? slamCorners.map(corner => {
                    const gps = window.slamToGps(corner.x, corner.y);
                    return {
                        name: corner.name,
                        slam: clonePlainSlam(corner),
                        gps: clonePlainGps(gps)
                    };
                })
                : [];

            return {
                pointCount: count,
                slam: {
                    min: clonePlainSlam({ x: minX, y: minY, z: minZ }),
                    max: clonePlainSlam({ x: maxX, y: maxY, z: maxZ }),
                    center: clonePlainSlam(center)
                },
                gpsSamples: gpsCorners
            };
        }

        function buildCurrentVisibleMapGeoMetadata(pcdFileName, pointCount) {
            const hasCalibration = !!(
                window.geoTransform ||
                (window.isGpsCalibrated &&
                    window.gpsAnchor &&
                    window.slamAnchor &&
                    Number.isFinite(Number(window.headingOffset)))
            );

            const bounds = computeCurrentVisibleMapBounds();
            const cablePath = Array.isArray(window.savedCablePath)
                ? window.savedCablePath.map(p => ({
                    x: roundNumber(p.x, 5),
                    y: roundNumber(p.y, 5),
                    z: roundNumber(p.z ?? 0.08, 5),
                    depth: p.userData ? (p.userData.depth ?? null) : (p.depth ?? null)
                })).filter(p => Number.isFinite(p.x) && Number.isFinite(p.y) && Number.isFinite(p.z))
                : [];

            return {
                version: 3,
                type: "slam_pcd_geo_metadata",
                createdAt: new Date().toISOString(),
                pcdFileName: pcdFileName,
                pointCount: pointCount,
                coordinateSystem: {
                    pcd: "SLAM局部坐标，字段为 x y z rgb，单位m，Z轴向上",
                    gprViewTransform: "三维重建页显示时使用 (x, y, z) -> (x, z, -y) 变换，保持与路径页方向一致",
                    gpsTransform: "优先通过 geoTransform(scale + rotation + translation) 将 SLAM 平面坐标换算为经纬度；兼容旧版 gpsAnchor + slamAnchor + headingOffset"
                },
                calibration: {
                    isGpsCalibrated: hasCalibration,
                    gpsAnchor: clonePlainGps(window.gpsAnchor),
                    slamAnchor: clonePlainSlam(window.slamAnchor),
                    headingOffset: Number.isFinite(Number(window.headingOffset)) ? Number(window.headingOffset) : 0,
                    currentGps: clonePlainGps(window.currentGps),
                    geoTransform: window.geoTransform || null,
                    method: window.geoTransform ? "trajectory_similarity_2d_ransac" : "legacy_heading_offset",
                    gpsDecimals: window.geoTransform?.gpsDecimals ?? 6,
                    rmsErrorM: window.geoTransform?.rmsErrorM ?? null,
                    maxErrorM: window.geoTransform?.maxErrorM ?? null,
                    scale: window.geoTransform?.scale ?? null,
                    sampleCount: window.geoTransform?.rawSampleCount || window.geoTransform?.sampleCount || null,
                    inlierCount: window.geoTransform?.inlierCount || null,
                    outlierCount: window.geoTransform?.outlierCount || null,
                    totalTravelM: window.geoTransform?.totalTravelM ?? null,
                    spatialSpanM: window.geoTransform?.spatialSpanM ?? null,
                    calibratedAt: window.geoTransform?.calibratedAt || null
                },
                mapBounds: bounds,
                // PCD 本体只能保存点云。将黄色电缆矢量路径放进同名 geo.json，
                // 使本地点云再次进入三维重建页后仍可显示路径并校正地下 GPR 管道。
                inspectionOverlays: {
                    cablePath
                },
                usage: {
                    loadTip: "读取点云时同时选择本 .geo.json 文件，可恢复经纬度标定和黄色电缆矢量路径；旧版文件仍恢复 window.gpsAnchor/window.slamAnchor/window.headingOffset。",
                    crawlerTip: "恢复标定后，三维重建页指定扫描区域会调用 slamToGps() 生成履带小车经纬度航点。"
                }
            };
        }

        function applyLoadedMapGeoMetadata(meta, sourceName = "") {
            if (!meta || typeof meta !== "object") return false;

            const calibration = meta.calibration || meta.gpsCalibration || meta;
            const loadedTransform = normalizeLoadedGeoTransform(calibration.geoTransform || meta.geoTransform || calibration.transform);
            const gpsAnchor = clonePlainGps(calibration.gpsAnchor) || (loadedTransform ? { lat: loadedTransform.originLat, lon: loadedTransform.originLon } : null);
            const slamAnchor = clonePlainSlam(calibration.slamAnchor) || (loadedTransform ? { x: 0, y: 0, z: 0 } : null);
            const headingOffset = Number(calibration.headingOffset ?? calibration.headingOffsetRad ?? loadedTransform?.theta ?? 0);

            if (!loadedTransform && (!gpsAnchor || !slamAnchor || !Number.isFinite(headingOffset))) {
                console.warn("⚠️ 点云经纬度元数据缺少 geoTransform 或 gpsAnchor / slamAnchor / headingOffset，已跳过恢复：", meta);
                return false;
            }

            window.geoTransform = loadedTransform;
            window.gpsAnchor = gpsAnchor;
            window.slamAnchor = slamAnchor;
            window.headingOffset = normalizeAngleRad(headingOffset);
            window.isGpsCalibrated = true;
            if (calibration.currentGps) {
                const currentGps = clonePlainGps(calibration.currentGps);
                if (currentGps) window.currentGps = currentGps;
            }

            window.loadedPointCloudGeoMetadata = meta;
            window.loadedPointCloudGeoSource = sourceName || meta.pcdFileName || "loaded-map";

            console.log("✅ 已恢复点云地图经纬度标定：", {
                source: window.loadedPointCloudGeoSource,
                gpsAnchor: window.gpsAnchor,
                slamAnchor: window.slamAnchor,
                headingOffset: window.headingOffset,
                geoTransform: window.geoTransform
            });

            return true;
        }

        function inferGeoJsonPathFromPcdPath(pcdPath) {
            const path = String(pcdPath || "").trim();
            if (!path) return "";
            if (/\.pcd$/i.test(path)) {
                return path.replace(/\.pcd$/i, ".geo.json");
            }
            return path + ".geo.json";
        }

        async function tryLoadServerPcdGeoMetadata(pcdPath) {
            const geoPath = inferGeoJsonPathFromPcdPath(pcdPath);
            if (!geoPath) return null;

            try {
                // 复用已有 /api/gpr/pcd?filePath=... 文件读取接口读取同名 geo.json。
                // 如果后端限制只能读取 .pcd，这里会静默失败；本地点云按钮仍支持手动选择 geo.json。
                const url = "/api/gpr/pcd?filePath=" + encodeURIComponent(geoPath);
                const res = await fetch(url, { cache: "no-store" });
                if (!res.ok) return null;

                const text = await res.text();
                const meta = JSON.parse(text);
                return meta;
            } catch (err) {
                console.warn("未读取到同名点云经纬度元数据：", geoPath, err);
                return null;
            }
        }

        function readFileAsText(file) {
            return new Promise((resolve, reject) => {
                const reader = new FileReader();
                reader.onload = e => resolve(String(e.target.result || ""));
                reader.onerror = () => reject(new Error("读取文件失败：" + file.name));
                reader.readAsText(file);
            });
        }

        function readFileAsArrayBuffer(file) {
            return new Promise((resolve, reject) => {
                const reader = new FileReader();
                reader.onload = e => resolve(e.target.result);
                reader.onerror = () => reject(new Error("读取文件失败：" + file.name));
                reader.readAsArrayBuffer(file);
            });
        }

        function buildCurrentVisibleMapPcdText() {
            if (!mapMesh || !mapMesh.geometry || !mapMesh.geometry.attributes.position) {
                throw new Error("点云视图尚未初始化，无法保存。");
            }

            const pos = mapMesh.geometry.attributes.position.array;
            const colAttr = mapMesh.geometry.attributes.color;
            const col = colAttr ? colAttr.array : null;

            const lines = [];
            let pointCount = 0;

            for (let i = 0; i < pos.length; i += 3) {
                const x = Number(pos[i]);
                const y = Number(pos[i + 1]);
                const z = Number(pos[i + 2]);

                // mapPositions 初始化时用 99999 作为空点占位，这里必须过滤掉。
                if (!isValidPointCoord(x, y, z)) continue;

                let r = 255, g = 255, b = 255;
                if (col && i + 2 < col.length) {
                    r = clampColorToByte(col[i]);
                    g = clampColorToByte(col[i + 1]);
                    b = clampColorToByte(col[i + 2]);
                }

                const rgb = ((r & 255) << 16) | ((g & 255) << 8) | (b & 255);
                lines.push(`${x.toFixed(5)} ${y.toFixed(5)} ${z.toFixed(5)} ${rgb}`);
                pointCount++;
            }

            if (pointCount <= 0) {
                throw new Error("当前页面没有有效点云数据，请先确认路径查找页面已经实时显示点云后再保存。");
            }

            const header = [
                "# .PCD v0.7 - Point Cloud Data file format",
                "VERSION 0.7",
                "FIELDS x y z rgb",
                "SIZE 4 4 4 4",
                "TYPE F F F U",
                "COUNT 1 1 1 1",
                `WIDTH ${pointCount}`,
                "HEIGHT 1",
                "VIEWPOINT 0 0 0 1 0 0 0",
                `POINTS ${pointCount}`,
                "DATA ascii"
            ].join("\n");

            return {
                pointCount,
                text: header + "\n" + lines.join("\n") + "\n"
            };
        }

        function downloadTextFile(text, filename, mimeType = "application/octet-stream") {
            const blob = new Blob([text], { type: mimeType });
            const url = window.URL.createObjectURL(blob);
            const a = document.createElement("a");
            a.style.display = "none";
            a.href = url;
            a.download = filename;
            document.body.appendChild(a);
            a.click();

            setTimeout(() => {
                window.URL.revokeObjectURL(url);
                document.body.removeChild(a);
            }, 0);
        }

        const btnSavePcd = document.getElementById("btnSavePcd");
        if (btnSavePcd) {
            btnSavePcd.onclick = function () {
                const originalText = this.innerText;
                try {
                    this.innerText = "⏳ 正在生成...";
                    this.disabled = true;

                    const result = buildCurrentVisibleMapPcdText();
                    const dateStr = new Date().toISOString().replace(/T/, "_").replace(/:/g, "").split(".")[0];
                    const baseName = `slam_map_${dateStr}`;
                    const pcdFileName = `${baseName}.pcd`;
                    const geoFileName = `${baseName}.geo.json`;

                    const geoMeta = buildCurrentVisibleMapGeoMetadata(pcdFileName, result.pointCount);
                    downloadTextFile(result.text, pcdFileName, "application/octet-stream");
                    downloadTextFile(JSON.stringify(geoMeta, null, 2), geoFileName, "application/json");

                    const savedCableCount = geoMeta.inspectionOverlays?.cablePath?.length || 0;
                    if (geoMeta.calibration && geoMeta.calibration.isGpsCalibrated) {
                        const cableTip = savedCableCount >= 2 ? `，黄色电缆路径 ${savedCableCount} 点` : "";
                        setStatus(`✅ 点云和经纬度元数据已保存：${result.pointCount} 个点${cableTip}，${pcdFileName} + ${geoFileName}`, "success");
                        addSystemMessage(`点云和经纬度元数据已保存：${pcdFileName} + ${geoFileName}。`, "success");
                    } else {
                        alert("⚠️ 当前 GPS 尚未标定，已保存 PCD 和 geo.json，但 geo.json 中没有可用经纬度标定。后续读取该地图时不能直接生成履带小车 GPS 航点。 ");
                        setStatus(`⚠️ PCD 已保存：${result.pointCount} 个点，但 GPS 未标定，geo.json 不可用于巡检航点换算。`, "error");
                    }

                    console.log("✅ PCD 与经纬度元数据已由前端可见点云导出：", {
                        pointCount: result.pointCount,
                        pcdFileName,
                        geoFileName,
                        geoMeta
                    });
                } catch (err) {
                    console.error("保存 PCD 报错:", err);
                    alert("保存失败：" + (err.message || err));
                    setStatus("❌ PCD 保存失败：" + (err.message || err), "error");
                } finally {
                    this.innerText = originalText;
                    this.disabled = false;
                }
            };
        }
        // ====== 👆 修复逻辑结束 👆 ======
        // ✅ [新增] 清除破损点按钮逻辑
        const btnClearFaults = document.getElementById("btnClearFaults");
        if (btnClearFaults) {
            btnClearFaults.onclick = function () {
                console.log("🧹 清除所有破损点标记...");
                if (window.faultImagesMap) window.faultImagesMap.clear();

                // 🌟 修复：清除所有破损点时，直接强制清空 4 张图片
                const imgLeft = document.getElementById('dispCaptureLeft');
                const imgRight = document.getElementById('dispCaptureRight');
                const imgFront = document.getElementById('dispCaptureFront');
                const imgBack = document.getElementById('dispCaptureBack');
                if (imgLeft) imgLeft.removeAttribute('src');
                if (imgRight) imgRight.removeAttribute('src');
                if (imgFront) imgFront.removeAttribute('src');
                if (imgBack) imgBack.removeAttribute('src');

                maxFaultIdx = -1;
                bsState = 'INACTIVE';
                lastFaultRobotDist = -1;

                // 🌟 修复：统一使用 window.lastFaultMarker
                window.lastFaultMarker = null;

                // 1. 重置连续追踪的状态变量 (防止红线乱连)
                if (typeof lastFaultMarker !== 'undefined') lastFaultMarker = null;
                if (typeof lastFaultCablePos !== 'undefined') lastFaultCablePos = null;
                window.lastFaultCablePos = null; // 👈 新增这一行：同步清空全局的破损点坐标
                maxFaultIdx = -1;  // 🌟 新增：重置最远探测进度

                // 2. 清除红色球体组 (faultMarkerGroup)
                if (faultMarkerGroup) {
                    while (faultMarkerGroup.children.length > 0) {
                        const obj = faultMarkerGroup.children[0];
                        faultMarkerGroup.remove(obj);
                        // 递归释放内存 (兼容 Group 和 单个 Mesh)
                        obj.traverse(child => {
                            if (child.geometry) child.geometry.dispose();
                            if (child.material) {
                                if (child.material.map) child.material.map.dispose();
                                child.material.dispose();
                            }
                        });
                    }
                }

                // 3. 清除红色电缆路径组 (redCableGroup)
                // 注意：redCableGroup 是在 initSlamVis 开头定义的
                if (redCableGroup) {
                    while (redCableGroup.children.length > 0) {
                        const mesh = redCableGroup.children[0];
                        redCableGroup.remove(mesh);
                        // 释放内存
                        if (mesh.geometry) mesh.geometry.dispose();
                        if (mesh.material) mesh.material.dispose();
                    }
                }
            };
        }



        const btnClearCable = document.getElementById("btnClearCablePath");
        if (btnClearCable) {
            btnClearCable.onclick = function () {
                cablePoints.length = 0;
                segmentPoints.length = 0;
                lastDepthUpdateIdx = 0; // ✅ 新增：重置深度绑定游标
                lastCablePos.set(-999, -999, 0);

                if (currentCableMesh) {
                    cableGroup.remove(currentCableMesh);
                    currentCableMesh.geometry.dispose();
                    currentCableMesh = null;
                }
                while (redCableGroup.children.length > 0) {
                    const mesh = redCableGroup.children[0];
                    redCableGroup.remove(mesh);
                    if (mesh.geometry) mesh.geometry.dispose();
                }
                // 追加这行：同时隐藏信息弹窗
                document.getElementById('cableInfoPopup').style.display = 'none';
            };
        }

        // =========================================================
        // ✅ [新增] KML 航线文件读取与三维黄色电缆路径绘制
        // =========================================================
        const btnLoadKml = document.getElementById("btnLoadKml");
        const kmlFileInput = document.getElementById("kmlFileInput");

        if (btnLoadKml && kmlFileInput) {
            // 点击视觉按钮触发隐藏的文件域选择
            btnLoadKml.onclick = function () {
                // 拦截校验：确保系统已经标定（有RTK固定解或执行了 Shift+C 虚拟标定），否则无法换算物理坐标
                if (!window.isGpsCalibrated) {
                    alert("⚠️ 航线读取失败：当前 SLAM 坐标系尚未与 GPS 地理坐标完成配准标定！\n室内测试请先通过 [Shift + C] 注入虚拟标定参数。");
                    return;
                }
                kmlFileInput.click();
            };

            // 文件选择后触发读取解析
            kmlFileInput.onchange = function (e) {
                const file = e.target.files[0];
                if (!file) return;

                setStatus("正在解析 KML 航线文件...", "warning");
                const reader = new FileReader();

                reader.onload = function (evt) {
                    try {
                        const kmlText = evt.target.result;

                        // 1. 使用浏览器原生 DOMParser 解析 XML 结构的 KML 文件
                        const parser = new DOMParser();
                        const xmlDoc = parser.parseFromString(kmlText, "text/xml");
                        const coordsTags = xmlDoc.getElementsByTagName("coordinates");

                        if (!coordsTags.length) {
                            throw new Error("未在 KML 文件中找到符合规范的 <coordinates> 数据节点！");
                        }

                        // 2. 清空当前地图上的旧电缆数据及显示网格
                        cablePoints.length = 0;
                        segmentPoints.length = 0;
                        lastDepthUpdateIdx = 0;
                        if (currentCableMesh) {
                            cableGroup.remove(currentCableMesh);
                            currentCableMesh.geometry.dispose();
                            currentCableMesh = null;
                        }

                        // 3. 逐个提取坐标文本并转换为 SLAM 3D Vector3 点
                        for (let i = 0; i < coordsTags.length; i++) {
                            const txt = coordsTags[i].textContent.trim();
                            // 兼容空格、换行或制表符等不规则分隔符
                            const parts = txt.split(/[\s\n\r]+/);

                            parts.forEach(part => {
                                if (!part) return;
                                const latLonAlt = part.split(',');
                                if (latLonAlt.length >= 2) {
                                    const lon = parseFloat(latLonAlt[0]);
                                    const lat = parseFloat(latLonAlt[1]);

                                    if (!isNaN(lon) && !isNaN(lat)) {
                                        // 转换为 SLAM 坐标下的 {x, y}
                                        const slamPt = window.gpsToSlam(lat, lon);
                                        if (slamPt) {
                                            // 创建 3D 点对象，深度(Z轴高度)默认固定在 0.08m 面层
                                            const cablePt = new THREE.Vector3(slamPt.x, slamPt.y, 0.08);
                                            cablePt.userData = { depth: null }; // 预留深度属性空间
                                            cablePoints.push(cablePt);
                                        }
                                    }
                                }
                            });
                        }

                        if (cablePoints.length < 2) {
                            throw new Error("有效航线轨迹测点少于 2 个，无法生成电缆实体线。");
                        }

                        // 4. 核心：调用卡尔曼滤波平滑算法，计算渲染曲线
                        let displayPoints = smoothCableDisplayPoints(KalmanSmoother.smooth(cablePoints), cablePoints);

                        // 将深度属性同步传导至平滑点
                        for (let i = 0; i < displayPoints.length; i++) {
                            displayPoints[i].userData = {
                                depth: cablePoints[i].userData.depth,
                                source: cablePoints[i].userData.source,
                                signal: cablePoints[i].userData.signal
                            };
                        }

                        // 5. 挂载到全局变量，激活随后的柔性并入跟踪算法 (Flexible Merge)
                        window.savedCablePath = displayPoints;
                        window.syncCablePathPrediction?.(true);

                        // 6. 构造 CatmullRom 连续曲线并在 3D Scene 中生成黄色粗线条管线
                        const visibleCablePoints = liftPathDisplayPointsAboveGround(displayPoints, cableGroup.position.z);
                        const curve = new THREE.CatmullRomCurve3(visibleCablePoints, false, 'centripetal', 0.35);
                        const tube = new THREE.TubeGeometry(curve, displayPoints.length * 8, 0.03, 8, false);
                        currentCableMesh = new THREE.Mesh(tube, cableMat); // 复用自带的黄色材质 cableMat
                        rememberPathGroundLiftState(currentCableMesh, visibleCablePoints);
                        cableGroup.add(currentCableMesh);

                        // 修改最后一个测点为历史相对参考点
                        const lastPt = cablePoints[cablePoints.length - 1];
                        lastCablePos.set(lastPt.x, lastPt.y, 0.08);

                        setStatus(`✅ KML航线加载成功！共绘制 ${cablePoints.length} 个基准电缆路径点。`, "success");
                        addSystemMessage("KML航线加载成功。", "success");
                        console.log(`📂 [KML 加载] 成功导入航线线段，管道模型已成功在世界坐标重绘。`);

                    } catch (err) {
                        console.error("KML 航线解析报错:", err);
                        alert("❌ 航线文件加载失败:\n" + err.message);
                        setStatus("KML 加载失败", "error");
                    } finally {
                        // 清空文件域的值，保证下一次选中同一个文件时依然能触发 onchange
                        kmlFileInput.value = "";
                    }
                };

                reader.readAsText(file);
            };
        }

        // ✅✅✅ 新增：注入“跟踪电缆”悬浮按钮逻辑
        (function injectCableTrackButton() {
            // 创建按钮
            const btnTrack = document.createElement('button');
            btnTrack.innerText = "⚡ 跟踪电缆";
            btnTrack.id = "btnTrackCableOverlay";
            // 设置样式：悬浮在 pathMapArea 右上角
            btnTrack.style.position = "absolute";
            // ✅✅✅ 修改 2: 将按钮位置下移到 200px，位于“清除”按钮下方
            btnTrack.style.top = "200px";
            btnTrack.style.right = "10px";
            btnTrack.style.zIndex = "999";
            btnTrack.style.padding = "8px 12px";
            btnTrack.style.backgroundColor = "rgba(0, 0, 0, 0.6)";
            btnTrack.style.border = "1px solid #00ccff";
            btnTrack.style.color = "#00ccff";
            btnTrack.style.borderRadius = "4px";
            btnTrack.style.cursor = "pointer";
            btnTrack.style.fontSize = "14px";
            btnTrack.style.fontWeight = "bold";
            btnTrack.style.transition = "all 0.3s";

            // 鼠标悬停效果
            btnTrack.onmouseover = () => { btnTrack.style.backgroundColor = "rgba(0, 204, 255, 0.2)"; };
            btnTrack.onmouseout = () => { btnTrack.style.backgroundColor = "rgba(0, 0, 0, 0.6)"; };

            container.appendChild(btnTrack);

            // 点击事件逻辑
            btnTrack.onclick = function () {
                if (isFaultFindingMode) {
                    alert("⚠️ 请先停止寻找破损点模式！");
                    return;
                }
                // 如果已经在跟踪（或去起点的路上），点击则停止
                if (isNavigating && isCableTrackingMode) {
                    console.log("🛑 用户点击停止跟踪");
                    stopNav(false);
                    // 按钮重置逻辑已在 stopNav 中统一处理
                    return;
                }

                // --- 开始跟踪逻辑 ---
                if (!window.savedCablePath || window.savedCablePath.length < 2) {
                    alert("⚠️ 未检测到有效的电缆路径！请先进行自动巡线生成路径。");
                    return;
                }

                isCableTrackingMode = true;
                btnTrack.innerText = "⏹ 停止跟踪";
                btnTrack.style.color = "#ff4444";
                btnTrack.style.border = "1px solid #ff4444";

                // 转换数据格式方便计算
                const pathPoints = window.savedCablePath.map(p => ({ x: p.x, y: p.y }));

                // ========================================================
                // 🚀 核心逻辑修改：基于向量方向的柔性并入
                // ========================================================

                // 1. 获取电缆起点和大致走向 (取前5个点的方向)
                const p0 = pathPoints[0];
                const pNext = pathPoints[Math.min(5, pathPoints.length - 1)];

                // 2. 计算向量
                // 电缆方向向量 (Start -> Next)
                const cableDirX = pNext.x - p0.x;
                const cableDirY = pNext.y - p0.y;

                // 机器狗相对于起点的向量 (Start -> Robot)
                const robotVecX = currentRobotPose.x - p0.x;
                const robotVecY = currentRobotPose.y - p0.y;

                // 3. 计算点积 (Dot Product)
                // 如果点积为负，说明机器狗在起点的“后方”（逆着电缆方向）
                const dotProduct = robotVecX * cableDirX + robotVecY * cableDirY;
                const distToStart = Math.hypot(robotVecX, robotVecY);

                // 4. 判定条件：只有在“后方”且“距离起点有一定距离”时，才判定为需要由 A* 引导回起点
                // -0.5 是一个容差，避免在起点正侧面垂直时误判
                const isBehindStart = (dotProduct < -0.1) && (distToStart > 1.0);

                if (isBehindStart) {
                    console.log(`🚀 位于电缆起点后方 (Dot=${dotProduct.toFixed(2)}), 启用 A* 导航至起点...`);
                    cableTrackingState = 'TO_START';
                    // 使用 A* 规划到起点
                    const path = planPathAStar(currentRobotPose, p0);
                    if (path) {
                        navPath = path;
                        startNav();
                    } else {
                        // 如果 A* 失败，回退到直接跟踪
                        alert("⚠️ 无法规划路径到起点，尝试直接跟踪...");
                        cableTrackingState = 'FOLLOWING';
                        navPath = pathPoints;
                        startNav();
                    }
                } else {
                    // === 柔性并入逻辑 ===
                    console.log("✅ 位于电缆前方或侧方，启用柔性并入 (Flexible Merge)...");
                    cableTrackingState = 'FOLLOWING';

                    // 1. 找到离机器狗当前位置最近的路径点索引
                    let minD = Infinity;
                    let closestIdx = 0;
                    for (let i = 0; i < pathPoints.length; i++) {
                        const d = Math.hypot(pathPoints[i].x - currentRobotPose.x, pathPoints[i].y - currentRobotPose.y);
                        if (d < minD) {
                            minD = d;
                            closestIdx = i;
                        }
                    }

                    // 2. 截取路径：只保留最近点之后的路径
                    // 这样纯追踪算法 (Pure Pursuit) 会自动计算出一条从当前位置切入到最近点前方的平滑曲线
                    if (closestIdx >= pathPoints.length - 1) {
                        alert("⚠️ 已经到达电缆终点附近！");
                        stopNav(false);
                        return;
                    }

                    console.log(`📍 锁定最近点索引: ${closestIdx}, 距离: ${minD.toFixed(2)}m`);
                    navPath = pathPoints.slice(closestIdx);
                    startNav();
                }
            };
            // ========================================================
            // ✅ 绑定底部“寻找破损点”按钮逻辑
            // ========================================================
            const btnFault = document.getElementById("btnFindFaultOverlay");
            if (btnFault) {
                btnFault.onclick = function () {
                    // 如果已经在寻找（或去起点的路上），点击则停止
                    if (isNavigating && isFaultFindingMode) {
                        console.log("🛑 用户点击停止寻找破损点");
                        stopNav(false);
                        return;
                    }

                    if (isCableTrackingMode) {
                        alert("⚠️ 请先停止跟踪电缆！");
                        return;
                    }

                    if (!window.savedCablePath || window.savedCablePath.length < 2) {
                        alert("⚠️ 未检测到有效的电缆路径！请先进行自动巡线生成路径。");
                        return;
                    }

                    bsState = 'INACTIVE';
                    bsIterationCount = 0;
                    lastFaultRobotDist = -1;
                    offsetPathDists = [];
                    maxFaultIdx = -1;
                    bsFootprints = []; // ✅ 新增：每次点击寻找破损点时，清空旧脚印
                    bsLastProjectionS = null;
                    bsPathAligning = false;
                    resetBinaryMotionController();
                    isFaultFindingMode = true;
                    btnFault.innerHTML = '<span class="btn-icon">⏹</span> 停止寻找';

                    // === 核心算法：以机器狗当前朝向决定寻找破损点的搜索方向 ===
                    // 说明：savedCablePath 的绘制顺序可能是 A->B，但现场可以从 B 点朝 A 点开始。
                    // 因此点击“寻找破损点”的瞬间，不再固定沿绘制方向，而是把机器狗车头朝向的那一侧作为电缆前方。
                    let searchInfo;
                    try {
                        searchInfo = buildFaultSearchPathByRobotHeading(window.savedCablePath, currentRobotPose);
                    } catch (err) {
                        alert("⚠️ " + err.message);
                        stopNav(false);
                        return;
                    }

                    const pathPoints = searchInfo.pathPoints;
                    const n = pathPoints.length;
                    const normals = [];

                    // 记录本次寻找方向，后面的红色破损点“单向推进锁”会用它来判断进度。
                    window.faultSearchReverse = searchInfo.isReversed;
                    window.faultSearchHeadingDot = searchInfo.headingDot;
                    window.faultSearchStartClosestIdx = searchInfo.closestIdx;

                    // 如果机器狗已经位于当前朝向对应的搜索终点附近，说明前方已经没有可寻找路径。
                    if (searchInfo.closestIdx >= n - 1) {
                        alert("⚠️ 当前机器狗朝向对应的电缆前方已经没有可搜索路径，请调整机器狗朝向后再开始寻找破损点。");
                        stopNav(false);
                        return;
                    }

                    // 1. 求出“已按机器狗朝向重排后的路径”上每个点的左法向量。
                    //    如果判定为反向搜索，pathPoints 已经被反转，所以后续所有距离积分和二分定位都会按 B->A 推进。
                    for (let i = 0; i < n; i++) {
                        const tangent = getPathTangent2D(pathPoints, i);
                        normals.push({ nx: -tangent.y, ny: tangent.x });
                    }

                    // 2. 判断机器狗位于当前搜索方向的左侧还是右侧。
                    //    注意这里不再使用电缆绘制起点 p0，而是使用离机器狗最近的电缆点，避免 B 点反向启动时误判。
                    const nearestCableIdx = Math.max(0, Math.min(n - 1, searchInfo.closestIdx));
                    const nearestCablePoint = pathPoints[nearestCableIdx];
                    const tangentAtRobot = getPathTangent2D(pathPoints, nearestCableIdx);
                    const rx = currentRobotPose.x - nearestCablePoint.x;
                    const ry = currentRobotPose.y - nearestCablePoint.y;
                    const cross = tangentAtRobot.x * ry - tangentAtRobot.y * rx;

                    let offsetDist = 0.5;
                    if (cross < 0) {
                        offsetDist = -0.5;
                        console.log(`👉 [寻找破损点] 机器狗位于当前搜索方向右侧，生成右侧 ${Math.abs(offsetDist).toFixed(2)}m 平行路径`);
                    } else {
                        console.log(`👈 [寻找破损点] 机器狗位于当前搜索方向左侧，生成左侧 ${Math.abs(offsetDist).toFixed(2)}m 平行路径`);
                    }

                    // 3. 生成平行偏移的新路径。
                    let offsetPath = [];
                    for (let i = 0; i < n; i++) {
                        offsetPath.push({
                            x: pathPoints[i].x + normals[i].nx * offsetDist,
                            y: pathPoints[i].y + normals[i].ny * offsetDist
                        });
                    }

                    // 4. 路径平滑处理，消除法向量计算放大的微小毛刺，让机器狗运动更丝滑。
                    const SMOOTH_ITERATIONS = 8;
                    for (let iter = 0; iter < SMOOTH_ITERATIONS; iter++) {
                        let tempPath = [offsetPath[0]];
                        for (let i = 1; i < offsetPath.length - 1; i++) {
                            tempPath.push({
                                x: (offsetPath[i - 1].x + offsetPath[i].x + offsetPath[i + 1].x) / 3.0,
                                y: (offsetPath[i - 1].y + offsetPath[i].y + offsetPath[i + 1].y) / 3.0
                            });
                        }
                        tempPath.push(offsetPath[offsetPath.length - 1]);
                        offsetPath = tempPath;
                    }

                    window.offsetCablePath = offsetPath; // 存入全局供二分法与步进寻找使用
                    offsetPathDists = [];
                    bsLastProjectionS = null;
                    bsPathAligning = false;

                    // === 柔性并入逻辑：从离机器狗最近的位置开始，沿“车头朝向对应的前方”搜索 ===
                    console.log("✅ 启用按车头方向的柔性平行并入 (Heading-Based Flexible Parallel Merge)...");
                    cableTrackingState = 'FOLLOWING';

                    let minD = Infinity;
                    let closestIdx = 0;
                    for (let i = 0; i < offsetPath.length; i++) {
                        const d = Math.hypot(offsetPath[i].x - currentRobotPose.x, offsetPath[i].y - currentRobotPose.y);
                        if (d < minD) {
                            minD = d;
                            closestIdx = i;
                        }
                    }

                    if (closestIdx >= offsetPath.length - 1) {
                        alert("⚠️ 当前机器狗朝向对应的电缆前方已经没有可搜索路径，请调整机器狗朝向后再开始寻找破损点。");
                        stopNav(false);
                        return;
                    }

                    console.log(
                        `📍 [寻找破损点] 并入索引: ${closestIdx}, 距离偏移路径: ${minD.toFixed(2)}m, ` +
                        `搜索方向: ${searchInfo.isReversed ? "B→A/反向" : "A→B/正向"}`
                    );

                    navPath = offsetPath.slice(closestIdx);
                    startNav();
                };
            }
        })();

        // --- 6. WebSocket ---
        const ROBOT_IP = "10.21.31.104";
        const poseWsUrl = "ws://" + ROBOT_IP + ":9090/pose";
        const cloudWsUrl = "ws://" + ROBOT_IP + ":9090/cloud";
        let hasLoggedPoseSuccess = false;
        let hasLoggedCloudSuccess = false;
        let isFirstFrame = true;
        let poseWs = null;
        let cloudWs = null;
        let poseReconnectTimer = null;
        let cloudReconnectTimer = null;
        let isPointCloudScanPaused = false;
        const btnTogglePointCloudScan = document.getElementById('btnTogglePointCloudScan');

        // 👇 1. 新增：记录点云渲染时间戳，用于节流
        let lastPointCloudRenderTime = 0;
        const POINT_CLOUD_RECEIVED_NOTICE = "已成功接收到点云数据。";
        const POINT_CLOUD_CACHE_NOTICE = "当前点云数据传输量较大，为避免发生卡顿，已自动清理缓存";
        const CLOUD_BACKPRESSURE_CONFIG = {
            parseWarnMs: 120,
            processWarnMs: 180,
            totalWarnMs: 260,
            mainThreadLagWarnMs: 700,
            payloadWarnBytes: 5 * 1024 * 1024,
            bufferedAmountWarnBytes: 8 * 1024 * 1024,
            pendingRangeWarnCount: 18,
            keepLatestPendingRanges: 4,
            dropFramesAfterCleanup: 2,
            cleanupCooldownMs: 10000
        };
        let lastCloudAutoCleanupAt = 0;
        let pendingCloudFramesToDrop = 0;

        function getPayloadByteLength(payload) {
            if (typeof payload === "string") return payload.length * 2;
            if (payload && Number.isFinite(payload.byteLength)) return payload.byteLength;
            return 0;
        }

        function compactPendingMapUpdateRanges() {
            if (pendingMapUpdateRanges.length <= CLOUD_BACKPRESSURE_CONFIG.keepLatestPendingRanges) return;
            pendingMapUpdateRanges = pendingMapUpdateRanges.slice(-CLOUD_BACKPRESSURE_CONFIG.keepLatestPendingRanges);
        }

        function autoCleanupPointCloudRuntimeCache(reason) {
            const now = performance.now();
            if (now - lastCloudAutoCleanupAt < CLOUD_BACKPRESSURE_CONFIG.cleanupCooldownMs) return false;

            lastCloudAutoCleanupAt = now;
            compactPendingMapUpdateRanges();
            pathGroundHeightGrid.clear();
            pendingCloudFramesToDrop = Math.max(
                pendingCloudFramesToDrop,
                CLOUD_BACKPRESSURE_CONFIG.dropFramesAfterCleanup
            );

            addSystemMessage(POINT_CLOUD_CACHE_NOTICE, "warning");
            setStatus(POINT_CLOUD_CACHE_NOTICE, "warning");
            console.warn("点云处理检测到积压，已清理浏览器端运行缓存:", reason);
            return true;
        }

        function monitorCloudProcessingBackpressure(stats) {
            if (!stats) return;

            const reasons = [];
            if (stats.parseMs > CLOUD_BACKPRESSURE_CONFIG.parseWarnMs) {
                reasons.push(`parse=${stats.parseMs.toFixed(1)}ms`);
            }
            if (stats.processMs > CLOUD_BACKPRESSURE_CONFIG.processWarnMs) {
                reasons.push(`process=${stats.processMs.toFixed(1)}ms`);
            }
            if (stats.totalMs > CLOUD_BACKPRESSURE_CONFIG.totalWarnMs) {
                reasons.push(`total=${stats.totalMs.toFixed(1)}ms`);
            }
            if (stats.bufferedAmount > CLOUD_BACKPRESSURE_CONFIG.bufferedAmountWarnBytes) {
                reasons.push(`buffered=${Math.round(stats.bufferedAmount / 1024 / 1024)}MB`);
            }
            if (stats.payloadBytes > CLOUD_BACKPRESSURE_CONFIG.payloadWarnBytes) {
                reasons.push(`payload=${Math.round(stats.payloadBytes / 1024 / 1024)}MB`);
            }
            if (stats.pendingRangeCount > CLOUD_BACKPRESSURE_CONFIG.pendingRangeWarnCount) {
                reasons.push(`pendingRanges=${stats.pendingRangeCount}`);
            }

            if (reasons.length) {
                autoCleanupPointCloudRuntimeCache(reasons.join(", "));
            }
        }

        let lastCloudLagProbeAt = performance.now();
        window.setInterval(() => {
            const now = performance.now();
            const lagMs = now - lastCloudLagProbeAt - 1000;
            lastCloudLagProbeAt = now;
            if (
                !document.hidden &&
                !isPointCloudScanPaused &&
                hasLoggedCloudSuccess &&
                cloudWs &&
                cloudWs.readyState === WebSocket.OPEN &&
                lagMs > CLOUD_BACKPRESSURE_CONFIG.mainThreadLagWarnMs
            ) {
                autoCleanupPointCloudRuntimeCache(`mainThreadLag=${lagMs.toFixed(1)}ms`);
            }
        }, 1000);

        function updatePointCloudScanButton() {
            if (!btnTogglePointCloudScan) return;

            if (isPointCloudScanPaused) {
                btnTogglePointCloudScan.innerText = "开始点云扫描";
                btnTogglePointCloudScan.title = "恢复接收并绘制点云数据";
            } else {
                btnTogglePointCloudScan.innerText = "暂停点云扫描";
                btnTogglePointCloudScan.title = "暂停接收并绘制点云数据，当前点云地图保持定格";
            }
        }

        function pausePointCloudScan() {
            isPointCloudScanPaused = true;
            hasLoggedCloudSuccess = false;

            if (cloudReconnectTimer !== null) {
                window.clearTimeout(cloudReconnectTimer);
                cloudReconnectTimer = null;
            }

            const socket = cloudWs;
            cloudWs = null;
            if (socket && (
                socket.readyState === WebSocket.OPEN ||
                socket.readyState === WebSocket.CONNECTING
            )) {
                socket.close();
            }

            updatePointCloudScanButton();
            setStatus("点云扫描已暂停，当前点云地图保持定格。", "warning");
            addSystemMessage("点云扫描已暂停，当前点云地图保持定格。", "warning");
        }

        function resumePointCloudScan() {
            isPointCloudScanPaused = false;
            updatePointCloudScanButton();
            connectCloudWebSocket();
            setStatus("点云扫描已开始，正在接收并绘制点云数据。", "success");
        }

        if (btnTogglePointCloudScan) {
            updatePointCloudScanButton();
            btnTogglePointCloudScan.addEventListener('click', function () {
                if (isPointCloudScanPaused) {
                    resumePointCloudScan();
                } else {
                    pausePointCloudScan();
                }
            });
        }

        function schedulePoseReconnect() {
            if (poseReconnectTimer !== null) return;

            console.warn("⚠️ 位姿 WebSocket 已断开，2 秒后自动重连...");
            poseReconnectTimer = window.setTimeout(() => {
                poseReconnectTimer = null;
                connectPoseWebSocket();
            }, 2000);
        }

        function scheduleCloudReconnect() {
            if (isPointCloudScanPaused) return;
            if (cloudReconnectTimer !== null) return;

            console.warn("⚠️ 点云 WebSocket 已断开，2 秒后自动重连...");
            cloudReconnectTimer = window.setTimeout(() => {
                cloudReconnectTimer = null;
                connectCloudWebSocket();
            }, 2000);
        }

        function connectPoseWebSocket() {
            if (poseWs && (
                poseWs.readyState === WebSocket.OPEN ||
                poseWs.readyState === WebSocket.CONNECTING
            )) {
                return;
            }

            console.log("正在尝试连接位姿 WebSocket:", poseWsUrl);
            const socket = new WebSocket(poseWsUrl);
            poseWs = socket;

            socket.onopen = () => {
                if (poseWs !== socket) return;

                if (poseReconnectTimer !== null) {
                    window.clearTimeout(poseReconnectTimer);
                    poseReconnectTimer = null;
                }
                console.log("✅ 位姿 WebSocket 已连接:", poseWsUrl);
            };

            socket.onmessage = (event) => {
                if (poseWs !== socket) return;

                try {
                    const data = JSON.parse(event.data);
                    if (data.type === 'pose_data') {
                        if (!hasLoggedPoseSuccess) {
                            console.log("%c🚀 位姿数据传输成功！", "color: #00e5ff; font-weight: bold;");
                            hasLoggedPoseSuccess = true;
                        }
                        updateVis(data, { processPose: true, processScan: false });
                    }
                } catch (e) {
                    console.error("位姿 WebSocket 消息解析失败:", e);
                }
            };

            socket.onerror = (event) => {
                if (poseWs !== socket) return;

                console.warn("位姿 WebSocket 连接异常:", event);
                socket.close();
            };

            socket.onclose = () => {
                if (poseWs !== socket) return;

                poseWs = null;
                hasLoggedPoseSuccess = false;
                schedulePoseReconnect();
            };
        }

        function connectCloudWebSocket() {
            if (isPointCloudScanPaused) return;

            if (cloudWs && (
                cloudWs.readyState === WebSocket.OPEN ||
                cloudWs.readyState === WebSocket.CONNECTING
            )) {
                return;
            }

            console.log("正在尝试连接点云 WebSocket:", cloudWsUrl);
            const socket = new WebSocket(cloudWsUrl);
            cloudWs = socket;

            socket.onopen = () => {
                if (cloudWs !== socket) return;
                if (isPointCloudScanPaused) {
                    socket.close();
                    return;
                }

                if (cloudReconnectTimer !== null) {
                    window.clearTimeout(cloudReconnectTimer);
                    cloudReconnectTimer = null;
                }
                console.log("✅ 点云 WebSocket 已连接:", cloudWsUrl);
            };

            socket.onmessage = (event) => {
                if (cloudWs !== socket) return;
                if (isPointCloudScanPaused) return;

                try {
                    const messageStartAt = performance.now();
                    const bufferedAmount = Number(socket.bufferedAmount) || 0;

                    if (pendingCloudFramesToDrop > 0) {
                        pendingCloudFramesToDrop -= 1;
                        return;
                    }

                    const parseStartAt = performance.now();
                    const data = JSON.parse(event.data);
                    const parseMs = performance.now() - parseStartAt;
                    if (data.type === 'cloud_data' || data.type === 'slam_data') {
                        if (!hasLoggedCloudSuccess) {
                            console.log("%c🌐 点云数据传输成功！", "color: #00e5ff; font-weight: bold;");
                            hasLoggedCloudSuccess = true;
                        }
                        const processStartAt = performance.now();
                        const visResult = updateVis(data, { processPose: false, processScan: true });
                        const processMs = performance.now() - processStartAt;
                        if (shouldNotifyPointCloudReceivedAfterStart && visResult && visResult.pointCloudDrawn) {
                            shouldNotifyPointCloudReceivedAfterStart = false;
                            addSystemMessage(POINT_CLOUD_RECEIVED_NOTICE, "success");
                        }
                        monitorCloudProcessingBackpressure({
                            parseMs,
                            processMs,
                            totalMs: performance.now() - messageStartAt,
                            bufferedAmount,
                            payloadBytes: getPayloadByteLength(event.data),
                            pendingRangeCount: pendingMapUpdateRanges.length
                        });
                    }
                } catch (e) {
                    console.error("点云 WebSocket 消息解析失败:", e);
                }
            };

            socket.onerror = (event) => {
                if (cloudWs !== socket) return;

                console.warn("点云 WebSocket 连接异常:", event);
                socket.close();
            };

            socket.onclose = () => {
                if (cloudWs !== socket) return;

                cloudWs = null;
                hasLoggedCloudSuccess = false;
                if (isPointCloudScanPaused) return;
                scheduleCloudReconnect();
            };
        }

        connectPoseWebSocket();
        connectCloudWebSocket();

        function updateVis(data, options = {}) {
            const processPose = options.processPose !== false;
            const processScan = options.processScan !== false;
            const pose = data.robot;
            const result = { pointCloudDrawn: false };

            if (processPose) {
                // ==========================================
                // 🌟🌟🌟 新增：拦截解析 Python 传来的 RTK 数据
                // ==========================================
                if (data.rtk && data.rtk.lat && data.rtk.lon) {
                    // 1. 字符串清洗：去掉 "°N"、"°E" 等后缀，提取纯浮点数
                    const rawLat = parseGpsNumber(data.rtk.lat);
                    const rawLon = parseGpsNumber(data.rtk.lon);

                    // 2. 获取固定解状态
                    const status = data.rtk.status;
                    // 严格要求只有 "固定解" 才能用于标定基准 (如果不准可以放宽限制)
                    const isGoodSignal = (status === "固定解" || status === "4" || status === 4);

                    // 更新全局 UI 状态
                    window.currentGps = { lat: rawLat, lon: rawLon, status: status };

                    // 👇==========================================👇
                    // 🌟 新增：实时更新左下角的 GPS 显示面板
                    const gpsTextEl = document.getElementById('realtimeGpsText');
                    if (gpsTextEl) {
                        // 如果是固定解，状态文字标绿；如果是不稳定解，状态文字标橙
                        const statusClass = isGoodSignal ? 'status-fixed' : 'status-float';
                        // 左下角实时经纬度只显示 6 位小数；内部 GPS 数据仍保留原始精度，不影响标定和航点计算。
                        gpsTextEl.innerHTML = `${rawLat.toFixed(6)}°, ${rawLon.toFixed(6)}° [<span class="${statusClass}">${status}</span>]`;
                    }
                    // 👆==========================================👆

                    // 3. ✅ 多点轨迹相似变换自动标定
                    // 不要求机器狗直线运动；适配 6 位经纬度，通过多点轨迹整体拟合 scale / rotation / translation。
                    if (isGoodSignal && !window.isGpsCalibrated && !window.geoTransform && pose && pose.x !== -999) {
                        addGeoCalibrationSample(rawLat, rawLon, pose, status);
                    }
                }
                // ==========================================

                if (pose) {
                    const scaleFactor = 1.0;
                    const px = pose.x * scaleFactor;
                    const py = pose.y * scaleFactor;
                    // ✅ 新增逻辑：提取真实的 Z 轴高度，减去 0.3m 的补偿值
                    const pz = (pose.z !== undefined ? pose.z : 0.05) * scaleFactor;

                    const isRTK = data.is_rtk === true;
                    if (typeof lastRTKStatus === 'undefined') {
                        window.lastRTKStatus = null;
                    }

                    if (isRTK !== window.lastRTKStatus) {
                        if (isRTK) {
                            console.log('RTK Status: ACTIVE 🟢');
                        } else {
                            console.log('RTK Status: OFF (ODOM MODE) ⚠️');
                        }
                        window.lastRTKStatus = isRTK;
                    }

                    const receivedYaw = Number(pose.yaw);
                    currentRobotPose = {
                        x: px,
                        y: py,
                        yaw: Number.isFinite(receivedYaw) ? receivedYaw : currentRobotPose.yaw,
                        updatedAt: Date.now()
                    };
                    window.currentRobotPose = currentRobotPose; // 🌟 增加这一行：每次机器狗移动时，实时同步给全局变量

                    updateNavLine();
                    updateDistanceUI();

                    if (isNavigating) {
                        const len = activeNavTrail.length;
                        if (len === 0) activeNavTrail.push({ x: px, y: py });
                        else {
                            const lastP = activeNavTrail[len - 1];
                            if (Math.hypot(px - lastP.x, py - lastP.y) > 0.02) activeNavTrail.push({ x: px, y: py });
                        }
                    }

                    dogGroup.position.set(px, py, pz + SLAM_Z_OFFSET);
                    // ✅ 新增提取 pitch 角度，如果没有则默认为 0
                    const pitch = pose.pitch !== undefined ? pose.pitch : 0.0;

                    // ✅ 核心修改：同时设置 pitch(绕Y轴) 和 yaw(绕Z轴)
                    // 'ZYX' 是旋转顺序，必须指定，否则在同时有偏航和俯仰时模型会发生万向节死锁导致倾斜
                    dogGroup.rotation.set(0, 0, pose.yaw, 'ZYX');

                    // ==========================================
                    // ✅ [新增] 真实脚印记录与动态裁剪 (卷尺算法)
                    // ==========================================
                    if (isFaultFindingMode) {
                        const currentPos2D = { x: px, y: py };
                        if (bsFootprints.length === 0) {
                            bsFootprints.push(currentPos2D);
                        } else {
                            // 1. 寻找当前物理位置在脚印数组中的最近点
                            let closestIdx = 0;
                            let minD = Infinity;
                            for (let i = Math.max(0, bsFootprints.length - 100); i < bsFootprints.length; i++) {
                                const d = Math.hypot(px - bsFootprints[i].x, py - bsFootprints[i].y);
                                if (d < minD) { minD = d; closestIdx = i; }
                            }

                            // 2. 如果倒车退回去了（索引差距大于3，约15cm），剪掉多余的“未来”轨迹
                            if (closestIdx < bsFootprints.length - 3) {
                                bsFootprints.length = closestIdx + 1;
                            }

                            // 3. 只有离开最后一个点超过 5cm，才记录新脚印
                            const lastP = bsFootprints[bsFootprints.length - 1];
                            if (Math.hypot(px - lastP.x, py - lastP.y) > 0.05) {
                                bsFootprints.push(currentPos2D);
                            }
                        }
                    }

                    // 恢复 grid 与交互平面跟随机器狗平移的逻辑
                    if (typeof gridHelper !== 'undefined') {
                        // 让网格的中心在 XY 平面上实时跟随机器狗
                        gridHelper.position.set(px, py, 0);
                        // 确保不可见的点击平面也同步跟随，否则“选点导航”等点击交互会发生严重偏移
                        if (clickPlane) clickPlane.position.set(px, py, 0);
                    }

                    if (isFirstFrame) {
                        orbitTarget.x = px; orbitTarget.y = py; orbitTarget.z = 0;
                        updateCamera();
                        isFirstFrame = false;
                    }

                    const trailZOffset = -0.9;
                    const currentPos = new THREE.Vector3(px, py, pz + trailZOffset);
                    const isFault = window.g_isFaultDetected === true;

                    // =========================================================
                    // 🚀 [新] 连续破损区域追踪逻辑
                    // =========================================================

                    // 定义全局变量 (放在 updateVis 函数外部，或者 initSlamVis 闭包顶部)
                    // 如果这些变量还没定义，请将被替换代码块上方的这两行声明移动到 initSlamVis 函数最开头：
                    // let lastFaultMarker = null;  // 记录上一个“临时红点”
                    // let lastFaultCablePos = null; // 记录上一个破损点的坐标（用于画线）

                    if (isFault) {
                        // 1. 检查是否存在电缆路径
                        if (window.savedCablePath && window.savedCablePath.length > 0) {

                            // 2. 寻找机器狗在电缆上的高精度垂直投影点
                            let minDistance = Infinity;
                            let closestPoint = null;
                            let closestIdx = -1;

                            // 遍历所有相邻点组成的线段
                            for (let i = 0; i < window.savedCablePath.length - 1; i++) {
                                const p1 = window.savedCablePath[i];
                                const p2 = window.savedCablePath[i + 1];

                                // 计算线段的向量方向
                                const dx = p2.x - p1.x;
                                const dy = p2.y - p1.y;
                                const dz = p2.z - p1.z;
                                const segLenSq = dx * dx + dy * dy + dz * dz;

                                let t = 0;
                                if (segLenSq > 0) {
                                    // 计算机器狗坐标在当前线段上的投影比例 (0 ~ 1)
                                    const dot = ((currentPos.x - p1.x) * dx + (currentPos.y - p1.y) * dy + (currentPos.z - p1.z) * dz);
                                    t = Math.max(0, Math.min(1, dot / segLenSq));
                                }

                                // 计算出精确的 3D 垂足坐标
                                const projX = p1.x + t * dx;
                                const projY = p1.y + t * dy;
                                const projZ = p1.z + t * dz;
                                const projPt = new THREE.Vector3(projX, projY, projZ);

                                // 计算机器狗真实位置到这个垂足的距离
                                const dist = currentPos.distanceTo(projPt);

                                if (dist < minDistance) {
                                    minDistance = dist;
                                    closestPoint = projPt; // 使用计算出的垂足，而不是死板的原始顶点
                                    closestIdx = i;        // 记录索引，用于“单向推进锁”防倒退
                                }
                            }

                            if (closestPoint) {
                                // 1. 判定当前是否处于“二分法精确回退/寻找”阶段
                                const isBinaryRefining = (bsState === 'EVAL_WAIT' || bsState === 'MOVING');

                                // 2. 将原始 savedCablePath 的索引换算为“本次搜索方向”上的进度索引。
                                //    正向搜索时原索引越大表示越靠前；反向搜索时原索引越小才表示越靠前。
                                const progressIdx = window.faultSearchReverse === true
                                    ? Math.max(0, (window.savedCablePath.length - 2) - closestIdx)
                                    : closestIdx;

                                // 3. 如果是向前开荒，或者正在二分法精确定位，都允许画红点
                                if (maxFaultIdx === -1 || progressIdx > maxFaultIdx || isBinaryRefining) {

                                    if (bsState === 'INACTIVE' || bsState === 'STEP_FORWARD' || bsState === 'STEP_WAIT') {
                                        if (window.offsetCablePath && offsetPathDists.length === 0) computePathDists(window.offsetCablePath);
                                        if (window.offsetCablePath) lastFaultRobotDist = getDistOnPath(currentPos, window.offsetCablePath);
                                    }

                                    // 距离去重，防止原地微小抖动频繁重绘
                                    if (!lastFaultCablePos || closestPoint.distanceTo(lastFaultCablePos) > 0.01) {

                                        // --- A. 删除上一个临时点 ---
                                        if (window.lastFaultMarker) { // 🌟 改为 window.
                                            faultMarkerGroup.remove(window.lastFaultMarker); // 🌟 改为 window.
                                            window.lastFaultMarker.traverse(child => { // 🌟 改为 window.
                                                if (child.geometry) child.geometry.dispose();
                                                if (child.material) {
                                                    if (child.material.map) child.material.map.dispose();
                                                    child.material.dispose();
                                                }
                                            });
                                            window.lastFaultMarker = null; // 🌟 改为 window.
                                        }

                                        // --- B. 创建新的破损点组合 (红球 + 闪电标志) ---
                                        const faultGroup = new THREE.Group();
                                        faultGroup.position.copy(closestPoint);
                                        // ✅ 新增：给每个破损点分配一个唯一的 ID，用于绑定抓拍图片
                                        faultGroup.userData.faultId = 'fault_' + Date.now();

                                        const sphereGeo = new THREE.SphereGeometry(0.12, 16, 16);
                                        const sphereMat = new THREE.MeshBasicMaterial({ color: 0xff0000 });
                                        const faultMesh = new THREE.Mesh(sphereGeo, sphereMat);
                                        faultGroup.add(faultMesh);

                                        const canvas = document.createElement("canvas");
                                        canvas.width = 128; canvas.height = 128;
                                        const ctx = canvas.getContext("2d");
                                        ctx.clearRect(0, 0, 128, 128);
                                        ctx.font = "bold 90px Arial";
                                        ctx.textAlign = "center"; ctx.textBaseline = "middle";
                                        ctx.fillStyle = "#ffdd00"; ctx.shadowColor = "#ff0000"; ctx.shadowBlur = 15;
                                        ctx.fillText("⚡", 64, 64);

                                        const texture = new THREE.CanvasTexture(canvas);
                                        const spriteMat = new THREE.SpriteMaterial({ map: texture, transparent: true, depthTest: false });
                                        const lightningSprite = new THREE.Sprite(spriteMat);
                                        lightningSprite.scale.set(0.6, 0.6, 1);
                                        lightningSprite.position.set(0, 0, 0.3);
                                        faultGroup.add(lightningSprite);

                                        faultMarkerGroup.add(faultGroup);
                                        window.lastFaultMarker = faultGroup; // 🌟 改为 window.

                                        // --- C. 绘制红色电缆段 ---
                                        if (lastFaultCablePos) {
                                            const segmentPath = [lastFaultCablePos, closestPoint];
                                            const curve = new THREE.CatmullRomCurve3(segmentPath);
                                            const tubeGeo = new THREE.TubeGeometry(curve, 1, 0.04, 8, false);
                                            const tubeMat = new THREE.MeshBasicMaterial({ color: 0xff0000 });
                                            const redSegment = new THREE.Mesh(tubeGeo, tubeMat);
                                            redCableGroup.add(redSegment);
                                        }

                                        // --- D. 更新状态 ---
                                        lastFaultCablePos = closestPoint.clone();
                                        window.lastFaultCablePos = lastFaultCablePos; // 👈 新增这一行：将坐标暴露给全局 window

                                        // 🌟 核心修改：如果是二分定位阶段，千万不要更新 maxFaultIdx，防止把向后的索引锁死
                                        // 正向/反向搜索都统一使用 progressIdx，避免 B 点朝 A 点搜索时被原始绘制索引拦截。
                                        if (!isBinaryRefining) {
                                            maxFaultIdx = progressIdx;
                                        }

                                        if (bsState === 'INACTIVE' || bsState === 'STEP_FORWARD' || bsState === 'STEP_WAIT') {
                                            console.log(`🔴 [追踪] 发现更前端的破损点，标志向前推进至:`, closestPoint);
                                        } else {
                                            console.log(`🎯 [二分定位] 边界扩大，标志刷新至最前端精确位置:`, closestPoint);
                                        }
                                    }
                                }
                            }
                        }
                    } else {
                        // (可选) 如果你需要“断续”逻辑：
                        // 当信号消失一段时间后，重置 lastFaultCablePos，
                        // 这样下一次再检测到信号时，会视为一个新的独立破损区域的起点，而不会把两个很远的区域连起来。
                        //
                        // if (signalLostTimer > 2000ms) {
                        //     lastFaultCablePos = null;
                        //     lastFaultMarker = null; // 此时保留最后一个红点作为该区域终点
                        // }
                    }

                    // ✅✅✅ 修改：如果处于电缆跟踪模式，禁止绘制机器狗的蓝色轨迹
                    // 只有在【非手动绘制电缆】且【非自动跟踪】模式下，才绘制蓝色路径
                    if (!isCableTrackingMode && !isManualDrawingCable) {
                        if (currentPos.distanceTo(lastTrailPos) > 0.05) {
                            trailPoints.push(currentPos);
                            lastTrailPos = currentPos;
                            if (trailPoints.length > 3000) trailPoints.shift();

                            if (trailPoints.length > 1) {
                                if (currentTrailMesh) {
                                    trailGroup.remove(currentTrailMesh);
                                    currentTrailMesh.geometry.dispose();
                                    currentTrailMesh = null;
                                }
                                const visibleTrailPoints = liftPathDisplayPointsAboveGround(trailPoints, trailGroup.position.z);
                                const curve = new THREE.CatmullRomCurve3(visibleTrailPoints);
                                const tube = new THREE.TubeGeometry(curve, trailPoints.length * 3, 0.03, 6, false);
                                currentTrailMesh = new THREE.Mesh(tube, trailMat);
                                rememberPathGroundLiftState(currentTrailMesh, visibleTrailPoints);
                                trailGroup.add(currentTrailMesh);
                            }
                        }
                    }

                    // =========================================================
                    // ✅✅✅ [核心修改] 特征点提取 + 卡尔曼滤波平滑
                    // =========================================================
                    // 增加判断条件：如果是手动绘制模式，则绕过信号特征检测，直接绘制
                    const sig = (typeof window.g_pipelineSignal !== 'undefined') ? window.g_pipelineSignal : -1;
                    const pipelinePrediction = window.g_pipelinePrediction || {};
                    let isFeaturePoint = false;

                    // 规则 1: 捕捉“正上方”时刻 (Signal = 0)
                    if (sig === 0) {
                        isFeaturePoint = true;
                    }
                    // 规则 2: 捕捉“跨越”时刻 (1 <-> 2)
                    else if ((lastSignal === 1 && sig === 2) || (lastSignal === 2 && sig === 1)) {
                        isFeaturePoint = true;
                        console.log("⚡ 捕捉到跨越点 (1 <-> 2)");
                    }
                    // 规则 3: 回归到“正上方” (Any -> 0)
                    else if (lastSignal !== 0 && sig === 0) {
                        isFeaturePoint = true;
                    }

                    // 补点时不能直接记录机器狗中心点，否则小弧度直线段会被画成机器狗轨迹。
                    // 先用管线仪信号估计电缆相对机器狗的位置，再沿已知/预测电缆方向向前延伸。
                    const isCornerPredictionActive = !!(
                        pipelinePrediction.cornerStrafeMode ||
                        pipelinePrediction.cornerArea ||
                        Number(pipelinePrediction.pathTurnDeg || 0) >= 24
                    );
                    const hasValidPipelineSignal = sig === 0 || sig === 1 || sig === 2;
                    const shouldDensifyCablePoint = isAutoTracking && isCornerPredictionActive && hasValidPipelineSignal;
                    const canGapFillCablePoint = isAutoTracking && hasValidPipelineSignal && !isFeaturePoint;

                    // =========================================================
                    // ✅✅✅ [修改] 电缆路径绘制逻辑 (兼容自动特征点与纯手动记录)
                    // =========================================================

                    // 触发条件 1：处于自动巡线模式，且检测到了管线仪特征点
                    const isAutoRecording = (isFeaturePoint || shouldDensifyCablePoint || canGapFillCablePoint) && isAutoTracking;

                    // 触发条件 2：用户开启了纯手动绘制模式
                    const isManualRecording = typeof isManualDrawingCable !== 'undefined' && isManualDrawingCable;

                    // 只要满足上述任意一种记录条件，且当前没有在执行跟踪或寻找任务
                    if ((isAutoRecording || isManualRecording) && !isCableTrackingMode && !isFaultFindingMode) {
                        const autoRecordMode = isFeaturePoint ? "feature" : (shouldDensifyCablePoint ? "dense" : "gap");
                        const candidateCablePt = isManualRecording && !isAutoRecording
                            ? new THREE.Vector3(px, py, 0.08)
                            : estimateCablePointFromRobotSignal(px, py, sig, pipelinePrediction, autoRecordMode);
                        const dist = candidateCablePt.distanceTo(lastCablePos);
                        const minCablePointDist = isFeaturePoint ? 0.1 : (shouldDensifyCablePoint ? 0.05 : 0.35);
                        if (dist > minCablePointDist) {
                            const cablePt = candidateCablePt;

                            // 🛡️ 严格保留你原本的初始化方式，确保完美兼容你的“区间回溯绑定”逻辑
                            cablePt.userData = {
                                depth: null,
                                source: isManualRecording && !isAutoRecording ? "manual" : autoRecordMode,
                                signal: sig
                            };

                            cablePoints.push(cablePt);
                            lastCablePos = cablePt;

                            if (cablePoints.length > 2000) {
                                cablePoints.shift();
                                if (lastDepthUpdateIdx > 0) lastDepthUpdateIdx--;
                            }

                            if (cablePoints.length > 3) {
                                if (currentCableMesh) {
                                    cableGroup.remove(currentCableMesh);
                                    currentCableMesh.geometry.dispose();
                                    currentCableMesh = null;
                                }

                                // 调用卡尔曼平滑 (你的原有代码完全不需要动)
                                let displayPoints = smoothCableDisplayPoints(KalmanSmoother.smooth(cablePoints));

                                // 将原始点的深度属性同步给平滑后的显示点
                                for (let i = 0; i < displayPoints.length; i++) {
                                    displayPoints[i].userData = {
                                        depth: cablePoints[i].userData.depth,
                                        source: cablePoints[i].userData.source,
                                        signal: cablePoints[i].userData.signal
                                    };
                                }

                                window.savedCablePath = displayPoints;
                                window.syncCablePathPrediction?.();

                                // 绘制平滑曲线
                                const visibleCablePoints = liftPathDisplayPointsAboveGround(displayPoints, cableGroup.position.z);
                                const curve = new THREE.CatmullRomCurve3(visibleCablePoints, false, 'centripetal', 0.35);
                                const tube = new THREE.TubeGeometry(curve, displayPoints.length * 8, 0.03, 8, false);
                                currentCableMesh = new THREE.Mesh(tube, cableMat);
                                rememberPathGroundLiftState(currentCableMesh, visibleCablePoints);
                                cableGroup.add(currentCableMesh);
                            }
                        }
                    }
                    lastSignal = sig;

                    // ==========================================
                    // 🌟 深度区间回溯绑定 (完全保留原有代码，不动)
                    // ==========================================
                    // ... (下方是你原本的 depth 绑定代码，直接原样保留即可)

                    // ==========================================
                    // 🌟 核心修改：接收到突变深度时，执行“区间回溯绑定”
                    // ==========================================
                    if (window.g_newDepthReceived !== null && window.g_newDepthReceived !== undefined) {
                        let incomingDepth = window.g_newDepthReceived;
                        window.g_newDepthReceived = null; // 消费掉该次深度数据，防止下一帧重复绑定

                        if (cablePoints.length > 0) {
                            // 1. 回溯绑定原始路径点 (从上一次游标位置，一直绑到当前最新位置)
                            for (let i = lastDepthUpdateIdx; i < cablePoints.length; i++) {
                                cablePoints[i].userData.depth = incomingDepth;
                            }
                            console.log(`📍 深度段落回溯绑定完成！区间: [${lastDepthUpdateIdx} ~ ${cablePoints.length - 1}], 深度值: ${incomingDepth}cm`);

                            // 2. 同步更新已经卡尔曼平滑后的显示路径 (window.savedCablePath)
                            // 以防用户恰好点中了平滑后的曲线段
                            if (window.savedCablePath && window.savedCablePath.length === cablePoints.length) {
                                for (let i = lastDepthUpdateIdx; i < window.savedCablePath.length; i++) {
                                    window.savedCablePath[i].userData.depth = incomingDepth;
                                }
                            }

                            // 3. 更新游标！下一次上位机再发突变深度时，就会从这个位置开始往后绑
                            lastDepthUpdateIdx = cablePoints.length;
                        }
                    }
                }
            }

            if (!processScan) return result;
            const scan = data.scan;
            if (scan && scan.length > 0) {
                const scanAttr = scanMesh.geometry.attributes.position;
                const colAttr = scanMesh.geometry.attributes.color;

                const scanPos = scanAttr.array;
                const scanCol = colAttr.array;

                let c = 0;
                let d = 0;
                let scaleFactor = 1.0;

                const isFlat = (scan.length > 0 && typeof scan[0] === 'number');

                if (isFlat) {
                    // 🚀 核心修改 1：将步长从 4 改为 6，因为现在每个点包含 [x, y, z, r, g, b] 6个数据
                    for (let i = 0; i < scan.length; i += 6) {
                        if (c >= 50000 * 3) break;

                        const pointX = scan[i] * scaleFactor;
                        const pointY = scan[i + 1] * scaleFactor;
                        scanPos[c++] = pointX;
                        scanPos[c++] = pointY;
                        let valZ = (i + 2 < scan.length) ? scan[i + 2] : 0.1;
                        const pointZ = valZ * scaleFactor;
                        scanPos[c++] = pointZ;

                        // 只抽样建立地面高度索引，不减少实际渲染的点云数量。
                        // 索引记录的是加入 SLAM_Z_OFFSET 后的世界高度，便于与路径世界坐标直接比较。
                        const pointIndex = Math.floor(i / 6);
                        if (pointIndex % PATH_GROUND_LIFT_CONFIG.gridSampleStride === 0) {
                            recordPointForPathGroundDetection(pointX, pointY, pointZ + SLAM_Z_OFFSET);
                        }

                        // 🚀 核心修改 2：去除 HSL 赋色，直接读取 RGB 真实颜色
                        // 注意：Three.js 要求的颜色范围是 0.0 ~ 1.0，所以需要除以 255
                        let r = (i + 3 < scan.length) ? scan[i + 3] / 255.0 : 1.0;
                        let g = (i + 4 < scan.length) ? scan[i + 4] / 255.0 : 1.0;
                        let b = (i + 5 < scan.length) ? scan[i + 5] / 255.0 : 1.0;

                        scanCol[d++] = r;
                        scanCol[d++] = g;
                        scanCol[d++] = b;
                    }
                }

                // 即使本帧因渲染节流未推入 GPU，也要立即更新规划器可读取的有效长度。
                // 否则点数减少时，上一帧尾部残留点会继续被误判为当前障碍。
                scanValidComponents = c;

                // 👇 2. 新增：计算距离上次推入显存过了多少毫秒 (>60ms 即最高约 16 帧/秒)
                const now = performance.now();
                const shouldRender3D = (now - lastPointCloudRenderTime > 60);

                if (shouldRender3D) {
                    for (let i = c; i < 50000 * 3; i++) scanPos[i] = 99999;
                    scanMesh.geometry.attributes.position.needsUpdate = true;
                    scanMesh.geometry.attributes.color.needsUpdate = true;
                    result.pointCloudDrawn = c > 0;
                    refreshRenderedPathsForGroundLift();
                }

                const scanPose = data.robot || currentRobotPose;
                if (scanPose && Number.isFinite(Number(scanPose.x)) && Number.isFinite(Number(scanPose.y))) {
                    const poseX = Number(scanPose.x) * scaleFactor;
                    const poseY = Number(scanPose.y) * scaleFactor;
                    const dist = Math.sqrt(Math.pow(poseX - lastKeyframePose.x, 2) + Math.pow(poseY - lastKeyframePose.y, 2));

                    appendScanToHistoryMap(scanPos, scanCol, c);

                    // 👇 3. 只有当判定需要刷新时，才将历史地图数据推入 GPU
                    if (shouldRender3D) {
                        flushPendingMapBufferUpdates();
                        lastPointCloudRenderTime = now; // 刷新时间戳
                    }

                    lastKeyframePose = { x: poseX, y: poseY };
                }
            }
            return result;
        }
        // 将 3D 点击位置实时投影到 2D 屏幕，使弹出框跟随地图旋转缩放
        function updatePopupPosition() {
            const elPopup = document.getElementById('cableInfoPopup');
            if (!popup3DPos || !elPopup || elPopup.style.display === 'none') return;

            // 克隆坐标并将其投影到相机的标准设备坐标(NDC)
            const vector = popup3DPos.clone();
            vector.project(camera);

            // 如果模型在相机背后，隐藏浮窗
            if (vector.z > 1.0) {
                elPopup.style.display = 'none';
                return;
            }

            const rect = renderer.domElement.getBoundingClientRect();
            // 转换回 CSS 的屏幕像素坐标
            const x = (vector.x * 0.5 + 0.5) * rect.width;
            const y = (-(vector.y * 0.5) + 0.5) * rect.height;

            elPopup.style.left = `${x}px`;
            elPopup.style.top = `${y}px`;
        }

        // 修改：将原来的 animate 函数展开，并加入 updatePopupPosition()
        function animate() {
            requestAnimationFrame(animate);

            // 🚀 核心拦截：如果当前用户没在看“路径查找”面板，也没在看“常规导航”面板，直接跳过 3D 渲染计算！
            if (window.currentActivePanel !== 'pathFault' && window.currentActivePanel !== 'robot') {
                return;
            }

            renderer.render(scene, camera);
            updatePopupPosition(); // ✅ 每一帧实时刷新弹窗位置，使其跟随地图拖拽
        }
        animate();

        new ResizeObserver(() => {
            const w = container.clientWidth, h = container.clientHeight;
            if (w && h) { renderer.setSize(w, h); camera.aspect = w / h; camera.updateProjectionMatrix(); }
        }).observe(container);
    })();
    // ==========================================
    // 新增：履带小车悬浮虚拟摇杆控制逻辑 (GPU加速 + rAF丝滑版)
    // ==========================================
    (function initCarJoystick() {
        const base = document.getElementById('carJoystickBase');
        const stick = document.getElementById('carJoystickStick');
        const status = document.getElementById('carJoystickStatus');
        if (!base || !stick) return;

        const maxRadius = 50;
        let isDragging = false;

        // 目标物理偏移量 (由手指/鼠标决定)
        let targetDx = 0;
        let targetDy = 0;

        // 发送给后端的数值 (-100 ~ 100)
        let currentX = 0;
        let currentY = 0;

        let sendTimer = null;
        let renderFrameId = null;

        // 1. 核心渲染循环：贴合屏幕刷新率 (60Hz / 120Hz)
        function renderLoop() {
            if (!isDragging) return; // 停止拖动时结束循环

            // 仅在屏幕准备好刷新时，才更新 DOM (极其丝滑的关键)
            stick.style.transform = `translate(${targetDx}px, ${targetDy}px)`;

            // 计算逻辑数值并更新文本
            currentX = Math.round((targetDx / maxRadius) * 100);
            currentY = Math.round((-targetDy / maxRadius) * 100);
            status.textContent = `X:${currentX}  Y:${currentY}`;
            status.style.color = "#00e5ff";

            // 请求下一帧
            renderFrameId = requestAnimationFrame(renderLoop);
        }

        // 2. 仅仅计算坐标，不操作 DOM
        function calculateTarget(clientX, clientY) {
            const rect = base.getBoundingClientRect();
            const centerX = rect.left + rect.width / 2;
            const centerY = rect.top + rect.height / 2;

            let dx = clientX - centerX;
            let dy = clientY - centerY;
            const distance = Math.hypot(dx, dy);

            if (distance > maxRadius) {
                dx = (dx / distance) * maxRadius;
                dy = (dy / distance) * maxRadius;
            }

            targetDx = dx;
            targetDy = dy;
        }

        // 3. 归零急刹
        function resetStick() {
            isDragging = false;
            targetDx = 0;
            targetDy = 0;
            currentX = 0;
            currentY = 0;

            if (renderFrameId) cancelAnimationFrame(renderFrameId);

            stick.classList.add('returning');
            stick.style.transform = `translate(0px, 0px)`;
            status.textContent = "停止";
            status.style.color = "#00ff00";

            fetch(`/api/car/move?x=0&y=0`).catch(() => { });
        }

        // 4. 心跳发送 (10Hz)
        function startSending() {
            if (sendTimer) return;
            sendTimer = setInterval(() => {
                fetch(`/api/car/move?x=${currentX}&y=${currentY}`)
                    .catch(err => console.error("⚠️ 履带车网络异常:", err));
            }, 100);
        }

        function stopSending() {
            if (sendTimer) {
                clearInterval(sendTimer);
                sendTimer = null;
            }
        }

        // --- 现代 Pointer Events：统一兼容鼠标和触屏，并开启指针捕获 ---
        base.addEventListener('pointerdown', (e) => {
            isDragging = true;
            // 锁定指针：即使鼠标拖出 base 区域甚至拖出浏览器，依然能被捕获
            base.setPointerCapture(e.pointerId);

            stick.classList.remove('returning');
            calculateTarget(e.clientX, e.clientY);

            // 启动渲染循环和发送线程
            renderLoop();
            startSending();
        });

        base.addEventListener('pointermove', (e) => {
            if (!isDragging) return;
            // 不再这里更新 DOM，只更新目标值
            calculateTarget(e.clientX, e.clientY);
        });

        function handlePointerUp(e) {
            if (isDragging) {
                base.releasePointerCapture(e.pointerId); // 释放指针
                stopSending();
                resetStick();
            }
        }

        base.addEventListener('pointerup', handlePointerUp);
        base.addEventListener('pointercancel', handlePointerUp);
        // ==========================================
        // 修改：点击后先弹出扫描范围参数设置，再由上位机规划矩形范围与 Z 字形路径，最后发送完整航点给小车
        // ==========================================
        const btnSendFault = document.getElementById('btnSendFaultToCar');
        if (btnSendFault) {
            btnSendFault.addEventListener('click', async function () {
                const originText = btnSendFault.innerText;

                try {
                    // 1. 先弹出参数设置页面。
                    //    这样即使当前上位机还没有破损点/电缆路径/GPS 标定数据，也可以预览弹窗显示效果。
                    const params = await openScanParamModal(DEFAULT_SCAN_PARAMS);
                    if (!params) return;

                    // 2. 点击“确定”后，再进行有效数据校验。
                    let faultSlamPos = window.lastFaultCablePos || null;

                    // ✅ 兜底1：如果 lastFaultCablePos 没有同步，但 lastFaultMarker 存在，就从红色破损点模型中恢复
                    if (!faultSlamPos && window.lastFaultMarker && window.lastFaultMarker.position) {
                        faultSlamPos = window.lastFaultMarker.position.clone();
                        window.lastFaultCablePos = faultSlamPos.clone();
                    }

                    // ✅ 兜底2：如果三维重建页已经加载了当前地图，也可从快照中的 faults 恢复
                    if (!faultSlamPos && window.latestImportedInspectionMapSnapshot?.faults?.length > 0) {
                        const f = window.latestImportedInspectionMapSnapshot.faults[
                            window.latestImportedInspectionMapSnapshot.faults.length - 1
                        ];

                        faultSlamPos = new THREE.Vector3(
                            Number(f.x),
                            Number(f.y),
                            Number(f.z ?? 0.08)
                        );

                        window.lastFaultCablePos = faultSlamPos.clone();
                    }

                    if (!faultSlamPos) {
                        alert("⚠️ 当前地图中没有可发送的破损点坐标！");
                        return;
                    }
                    if (!window.isGpsCalibrated || !window.gpsAnchor) {
                        alert("⚠️ GPS 尚未标定完成，无法计算真实经纬度！\n(可按 Shift+C 触发室内虚拟标定)");
                        return;
                    }
                    if (!window.savedCablePath || window.savedCablePath.length < 2) {
                        alert("⚠️ 尚未形成有效电缆路径，无法确定扫描矩形方向！");
                        return;
                    }

                    btnSendFault.innerText = "⏳ 规划中...";
                    btnSendFault.disabled = true;

                    // 3. 上位机基于破损点、局部电缆切向和参数生成扫描路径
                    const plan = buildUpperComputerScanPlan(faultSlamPos, params, {
                        source: "fault",
                        taskPrefix: "scan"
                    });

                    // 4. 在三维重建页点云地图中绘制矩形范围和 Z 字形路径虚线。
                    //    此时只预览，不直接发送，先进入微调模式。
                    renderScanPlanOnGpr(plan);
                    btnSendFault.innerText = "🛠️ 等待微调...";

                    const finalPlan = await enterScanPlanAdjustmentMode(plan, { source: "fault" });
                    if (!finalPlan) {
                        btnSendFault.innerText = "已取消";
                        return;
                    }

                    btnSendFault.innerText = "⏳ 发送路径...";

                    // 5. 用户确认微调后，才发送完整航点给履带小车；小车不再自行规划扫描路径。
                    await sendScanPlanToCrawler(finalPlan);

                    console.log("📍 上位机规划扫描路径已微调并下发：", finalPlan);
                    btnSendFault.innerText = `✅ 已发送${finalPlan.waypoints.length}个航点`;

                    const extendTip = finalPlan.params.autoExtended
                        ? `，末段自动延长 ${finalPlan.params.extensionLength}m，实际L=${finalPlan.params.length}m`
                        : "";
                    setStatus(`✅ 已确认微调并下发履带小车扫描路径：设置L=${finalPlan.params.requestedLength}m${extendTip}，W=${finalPlan.params.width}m，夹角=${finalPlan.params.angleDeg}°，航点${finalPlan.waypoints.length}个。`, "success");

                } catch (err) {
                    console.error("⚠️ 扫描路径规划或发送失败:", err);
                    alert("❌ 扫描路径规划或发送失败：" + err.message);
                    btnSendFault.innerText = "❌ 发送失败";
                    setStatus("扫描路径规划或发送失败：" + err.message, "error");
                } finally {
                    setTimeout(() => {
                        btnSendFault.innerText = originText;
                        btnSendFault.disabled = false;
                    }, 2000);
                }
            });
        }

        const btnManualScanArea = document.getElementById('btnManualScanArea');
        if (btnManualScanArea) {
            btnManualScanArea.addEventListener('click', function () {
                startManualScanAreaPlanning(btnManualScanArea);
            });
        }
    })();
    // ==========================================
    // 新增：机器狗全向移动与转向双摇杆控制 (完美复刻履带小车丝滑版 - 支持多面板)
    // ==========================================
    (function initDogJoysticks() {
        const transBase = document.getElementById('dogTransBase');
        const transStick = document.getElementById('dogTransStick');
        const transStatus = document.getElementById('dogTransStatus');

        const elSpeedX = document.getElementById('manualSpeedX');
        const elSpeedY = document.getElementById('manualSpeedY');

        // 收集所有的转向摇杆（主面板 + 破损点面板）
        const rotBases = [document.getElementById('dogRotBase'), document.getElementById('dogRotBase2')];
        const rotSticks = [document.getElementById('dogRotStick'), document.getElementById('dogRotStick2')];
        const rotStatuses = [document.getElementById('dogRotStatus'), document.getElementById('dogRotStatus2')];

        if (!transBase) return;

        const maxTransRadius = 43;

        let isTransDragging = false;
        let isRotDragging = false;

        // 当前激活的转向摇杆元素引用
        let activeRotStick = null;
        let activeRotStatus = null;
        let activeMaxRotRadius = 58; // 动态计算半径

        // 纯视觉目标偏移量
        let t_visualDx = 0, t_visualDy = 0;
        let r_visualDx = 0;

        let targetV = 0, targetY = 0, targetW = 0;
        let sendTimer = null;
        let renderFrameId = null;

        function renderLoop() {
            if (!isTransDragging && !isRotDragging) {
                renderFrameId = null;
                return;
            }

            if (isTransDragging) {
                transStick.style.transform = `translate(${t_visualDx}px, ${t_visualDy}px)`;
                if (transStatus) {
                    transStatus.textContent = `前进: ${targetV.toFixed(2)} | 横移: ${targetY.toFixed(2)}`;
                    transStatus.style.color = "#00e5ff";
                }
            }
            if (isRotDragging && activeRotStick) {
                activeRotStick.style.transform = `translate(${r_visualDx}px, 0px)`;
                if (activeRotStatus) {
                    activeRotStatus.textContent = `转向速度: ${targetW.toFixed(2)}`;
                    activeRotStatus.style.color = "#00e5ff";
                }
            }

            renderFrameId = requestAnimationFrame(renderLoop);
        }

        // ================= 平移摇杆逻辑 (维持原样) =================
        function calculateTransTarget(clientX, clientY) {
            const rect = transBase.getBoundingClientRect();
            const centerX = rect.left + rect.width / 2;
            const centerY = rect.top + rect.height / 2;

            let dx = clientX - centerX;
            let dy = clientY - centerY;
            const distance = Math.hypot(dx, dy);

            if (distance > maxTransRadius) {
                dx = (dx / distance) * maxTransRadius;
                dy = (dy / distance) * maxTransRadius;
            }
            t_visualDx = dx;
            t_visualDy = dy;

            let logicDx = dx;
            let logicDy = dy;

            if (distance > 10) {
                if (Math.abs(logicDy) > 15 && Math.abs(logicDx) < 25) { logicDx = 0; }
                if (Math.abs(logicDx) > 15 && Math.abs(logicDy) < 25) { logicDy = 0; }
            } else {
                logicDx = 0;
                logicDy = 0;
            }

            const manualX = elSpeedX ? parseFloat(elSpeedX.value) : 0.5;
            const manualY = elSpeedY ? parseFloat(elSpeedY.value) : 0.3;

            targetV = -(logicDy / maxTransRadius) * manualX;
            targetY = -(logicDx / maxTransRadius) * manualY;
        }

        function resetTrans() {
            isTransDragging = false;
            t_visualDx = 0; t_visualDy = 0;
            targetV = 0; targetY = 0;

            transStick.classList.add('returning');
            transStick.style.transform = `translate(0px, 0px)`;

            if (transStatus) {
                transStatus.textContent = "等待操作...";
                transStatus.style.color = "";
            }
            stopSending();
        }

        transBase.addEventListener('pointerdown', (e) => {
            isTransDragging = true;
            transBase.setPointerCapture(e.pointerId);
            transStick.classList.remove('returning');

            calculateTransTarget(e.clientX, e.clientY);
            if (!renderFrameId) renderLoop();
            startSending();
        });

        transBase.addEventListener('pointermove', (e) => {
            if (!isTransDragging) return;
            calculateTransTarget(e.clientX, e.clientY);
        });

        function handleTransUp(e) {
            if (isTransDragging) {
                transBase.releasePointerCapture(e.pointerId);
                resetTrans();
            }
        }
        transBase.addEventListener('pointerup', handleTransUp);
        transBase.addEventListener('pointercancel', handleTransUp);


        // ================= 转向摇杆逻辑 (已重构以支持多实例) =================
        function calculateRotTarget(clientX, baseElement) {
            const rect = baseElement.getBoundingClientRect();
            const centerX = rect.left + rect.width / 2;

            let dx = clientX - centerX;

            if (dx > activeMaxRotRadius) dx = activeMaxRotRadius;
            if (dx < -activeMaxRotRadius) dx = -activeMaxRotRadius;

            r_visualDx = dx;
            const manualW = 1.0;
            targetW = -(dx / activeMaxRotRadius) * manualW;
        }

        function resetRot() {
            isRotDragging = false;
            r_visualDx = 0; targetW = 0;

            if (activeRotStick) {
                activeRotStick.classList.add('returning');
                activeRotStick.style.transform = `translate(0px, 0px)`;
            }

            if (activeRotStatus) {
                activeRotStatus.textContent = "等待操作...";
                activeRotStatus.style.color = "";
            }
            stopSending();
        }

        function startSending() {
            if (sendTimer) return;
            sendTimer = setInterval(() => {
                if (typeof sendMoveCommand === 'function') {
                    sendMoveCommand(targetV, targetW, targetY, 0.0, 0.0, false);
                }
            }, 100);
        }

        function stopSending() {
            if (!isTransDragging && !isRotDragging) {
                if (sendTimer) {
                    clearInterval(sendTimer);
                    sendTimer = null;
                }
                if (typeof sendMoveCommand === 'function') {
                    sendMoveCommand(0.0, 0.0, 0.0, 0.0, 0.0, true);
                }
            }
        }

        // 遍历并为所有面板里的转向摇杆绑定独立事件
        rotBases.forEach((rotBase, index) => {
            if (!rotBase) return;
            const rotStick = rotSticks[index];
            const rotStatus = rotStatuses[index];

            rotBase.addEventListener('pointerdown', (e) => {
                isRotDragging = true;
                // 将激活句柄切换到当前操作的这套摇杆上
                activeRotStick = rotStick;
                activeRotStatus = rotStatus;

                // 动态计算该摇杆的最大游动半径 (兼容小摇杆和长摇杆)
                activeMaxRotRadius = (rotBase.clientWidth - rotStick.clientWidth) / 2;

                rotBase.setPointerCapture(e.pointerId);
                rotStick.classList.remove('returning');

                calculateRotTarget(e.clientX, rotBase);

                if (!renderFrameId) renderLoop();
                startSending();
            });

            rotBase.addEventListener('pointermove', (e) => {
                if (!isRotDragging) return;
                calculateRotTarget(e.clientX, rotBase);
            });

            function handleRotUp(e) {
                if (isRotDragging) {
                    rotBase.releasePointerCapture(e.pointerId);
                    resetRot();
                }
            }
            rotBase.addEventListener('pointerup', handleRotUp);
            rotBase.addEventListener('pointercancel', handleRotUp);
        });

        // 急停按钮绑定保持不变
        const btnEmergency = document.getElementById('btnEmergencyStop');
        if (btnEmergency) {
            const newBtn = btnEmergency.cloneNode(true);
            btnEmergency.parentNode.replaceChild(newBtn, btnEmergency);
            newBtn.addEventListener('click', function (e) {
                e.preventDefault();
                if (typeof emergencyStop === 'function') emergencyStop();
            });
        }
    })();
    // ==========================================
    // ✅ 新增：管线仪控制弹窗与按键逻辑
    // ==========================================
    document.addEventListener("DOMContentLoaded", function () {
        const btnShowPipelineKeyModal = document.getElementById('btnShowPipelineKeyModal');
        const pipelineKeyModal = document.getElementById('pipelineKeyModal');
        const closePipelineKeyModal = document.getElementById('closePipelineKeyModal');
        const pipelineKeyStatus = document.getElementById('pipelineKeyStatus');
        const pipelineKeyButtons = Array.from(document.querySelectorAll('.pipeline-key-btn[data-pipeline-key]'));

        function setPipelineKeyStatus(text, type) {
            if (!pipelineKeyStatus) return;
            pipelineKeyStatus.textContent = text;
            pipelineKeyStatus.classList.remove('success', 'error');
            if (type) pipelineKeyStatus.classList.add(type);
        }

        function setPipelineKeyButtonsDisabled(disabled) {
            pipelineKeyButtons.forEach(btn => {
                btn.disabled = disabled;
            });
        }

        async function sendPipelineKeyCommand(keyName, activeButton) {
            if (!keyName) return;

            setPipelineKeyButtonsDisabled(true);
            if (activeButton) activeButton.classList.add('sending');
            setPipelineKeyStatus(`正在发送 ${keyName} 指令...`, "");
            setStatus(`管线仪：发送 ${keyName} 指令`, "");

            try {
                const res = await fetch(`/api/pipeline-key/key?name=${encodeURIComponent(keyName)}`, {
                    method: "POST",
                    cache: "no-store"
                });
                const contentType = res.headers.get("content-type") || "";
                const data = contentType.includes("application/json") ? await res.json() : { message: await res.text() };

                if (!res.ok || data.success === false) {
                    throw new Error(data.message || `管线仪 ${keyName} 指令发送失败`);
                }

                setPipelineKeyStatus(`${keyName} 指令发送成功`, "success");
                setStatus(`管线仪：${keyName} 指令发送成功`, "success");
            } catch (err) {
                const message = err && err.message ? err.message : String(err);
                setPipelineKeyStatus(message, "error");
                setStatus(`管线仪：${keyName} 指令发送失败`, "error");
                addSystemMessage(`管线仪 ${keyName} 指令发送失败：${message}`, "error");
                console.error("管线仪按键指令发送失败:", err);
            } finally {
                if (activeButton) activeButton.classList.remove('sending');
                setPipelineKeyButtonsDisabled(false);
            }
        }

        if (btnShowPipelineKeyModal && pipelineKeyModal && closePipelineKeyModal) {
            btnShowPipelineKeyModal.addEventListener('click', function (e) {
                e.preventDefault();
                pipelineKeyModal.style.display = 'block';
            });

            closePipelineKeyModal.addEventListener('click', function () {
                pipelineKeyModal.style.display = 'none';
            });

            window.addEventListener('click', function (event) {
                if (event.target === pipelineKeyModal) {
                    pipelineKeyModal.style.display = 'none';
                }
            });
        }

        pipelineKeyButtons.forEach(btn => {
            btn.addEventListener('click', function () {
                sendPipelineKeyCommand(this.dataset.pipelineKey, this);
            });
        });
    });

    // ==========================================
    // ✅ 新增：机械臂控制弹窗逻辑
    // ==========================================
    document.addEventListener("DOMContentLoaded", function () {
        const btnShowArmModal = document.getElementById('btnShowArmModal');
        const armControlModal = document.getElementById('armControlModal');
        const closeArmModal = document.getElementById('closeArmModal');

        if (btnShowArmModal && armControlModal && closeArmModal) {
            // 点击底栏按钮打开弹窗
            btnShowArmModal.addEventListener('click', function (e) {
                e.preventDefault();
                armControlModal.style.display = 'block';
            });

            // 点击右上角 X 关闭弹窗
            closeArmModal.addEventListener('click', function () {
                armControlModal.style.display = 'none';
            });

            // 点击弹窗以外的黑色遮罩层自动关闭弹窗
            window.addEventListener('click', function (event) {
                if (event.target === armControlModal) {
                    armControlModal.style.display = 'none';
                }
            });
        }
    });

    // ==========================================
    // ✅ 新增：抓拍图片弹窗控制逻辑 (不阻塞UI版)
    // ==========================================
    document.addEventListener("DOMContentLoaded", function () {
        const btnShowCaptureModal = document.getElementById('btnShowCaptureModal');
        const captureImageModal = document.getElementById('captureImageModal');
        const closeCaptureModal = document.getElementById('closeCaptureModal');

        if (btnShowCaptureModal && captureImageModal && closeCaptureModal) {

            // 1. 点击【查看抓拍图片】按钮，显示弹窗
            btnShowCaptureModal.addEventListener('click', function (e) {
                e.preventDefault();

                // 切换显示/隐藏状态
                if (captureImageModal.style.display === 'none' || captureImageModal.style.display === '') {
                    captureImageModal.style.display = 'block';
                } else {
                    captureImageModal.style.display = 'none';
                }
            });

            // 2. 点击弹窗右上角的【X】关闭弹窗
            closeCaptureModal.addEventListener('click', function () {
                captureImageModal.style.display = 'none';
            });

            // 注意：由于我们在 CSS 中使用了 pointer-events: none; 
            // 遮罩层不再接收点击事件，因此无需（也无法）监听点击黑色遮罩关闭弹窗。
            // 这正是为了保证你能点击到主界面的“抓拍按钮”！
        }
    });

})(); 
