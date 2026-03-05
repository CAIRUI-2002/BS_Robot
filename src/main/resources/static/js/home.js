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
    const ARM_API_BASE = "http://10.21.31.200:5000";

    // ====== 🌍 GPS 与 SLAM 坐标映射全局变量 ======
    window.gpsAnchor = null;        // 起点真实的 GPS 坐标 {lat, lon}
    window.slamAnchor = null;       // 起点对应的 SLAM 坐标 {x, y}
    window.headingOffset = 0;       // 两个坐标系的角度差 (弧度)
    window.isGpsCalibrated = false; // 是否已完成标定
    window.currentGps = null;       // 机器狗当前的最新 GPS

    // ====== 🌊 自适应检测全局变量 ======
    const HISTORY_SIZE = 50;  // 记录最近 50 帧的数据作为背景参考
    let signalBaselineQueue = []; // 存储历史信号强度
// ====== 🌊 [新] AUC 算法全局配置 ======
    // 对应 Python 代码中的核心参数
    const AUC_CONFIG = {
        SAMPLING_RATE: 10000,   // 采样率
        PULSE_THRESHOLD: 300,   // 粗筛：主脉冲识别阈值 (绝对值 > 500)
        DISTANCE_MIN: 1000,     // 粗筛：最小脉冲间隔 (防止重复触发)
        
        // --- 核心判定参数 ---
        NOISE_THRESHOLD: 3.5,   // Z-score 阈值 (由滑条控制)
        BURST_THRESHOLD: 6,     // 异常点数量阈值
        AUC_THRESHOLD: 2500.0,  // 能量积分阈值 (幅度 * 数量)
        
        // --- 窗口设置 ---
        WINDOW_BEFORE: 2000,    // 峰值前取点
        WINDOW_AFTER: 5000      // 峰值后取点 (等待数据攒够)
    };

    // ==========================================
    // ✅ 新增：AUC 算法参数微调控制
    // ==========================================
    window.adjustAucParam = function(type, delta) {
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

    // ====== 🌊 峰值监测全局变量 ======
// 存储最近 50 帧的 { deviation: 偏离度, val: 信号强度 }
    let mutationWindow = [];
// 用于界面滚动显示的队列 [历史2, 历史1, 当前]
    let mutationDisplayHistory = [0, 0, 0];


    // GPR 相关
    let showAnnotation = false;      // 标注开关状态
    let annotation3DGroups = [];     // 存储 3D 标注对象
    let gprResult = null;            // 缓存 GPR 结果

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
        brightnessThreshold: 200,
        minRadius: 3,
        radiusPadding: 6
    };

    // 管道连接配置
    const LINK_CONFIG = {
        maxZDiff: 0.1,
        maxYDiff: 0.1
    };

    // ==========================================
    // 1. 辅助算法与工具函数
    // ==========================================

    function $(id) { return document.getElementById(id); }

    function setStatus(text, type) {
        const statusSpan = $("gprStatus");
        // 如果在 GPR 面板没找到状态栏，尝试找机械臂面板的状态栏(如果有的话)
        // 这里主要更新 GPR 面板的状态
        if (!statusSpan) {
            console.log(`[系统状态] ${text}`); // 备用日志
            return;
        }
        statusSpan.textContent = text || "";
        statusSpan.classList.remove("error", "success");
        if (type) statusSpan.classList.add(type);
    }
    // ✅ 新增：更新界面上的数值显示 (挂载到 window 以便 HTML 调用)
    window.updateSpeedLabel = function(id, val) {
        const el = document.getElementById(id);
        if(el) el.textContent = val;
        // 自动模式的数值显示也在这里顺便更新，但发送请求单独处理
        if(id === 'valAutoX') document.getElementById('valAutoX').textContent = val;
        if(id === 'valAutoY') document.getElementById('valAutoY').textContent = val;
    };

    // ✅ 新增：发送自动巡线速度给后端 (挂载到 window 以便 HTML 调用)
    window.updateAutoSpeed = function() {
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

    // 连通域检测算法 (GPR)
    function detectBlobs(matrix) {
        if (!matrix || !matrix.length) return [];
        const rows = matrix.length;
        const cols = matrix[0].length;
        const visited = new Uint8Array(rows * cols);
        const blobs = [];

        let min = Infinity, max = -Infinity;
        for (let r = 0; r < rows; r++) {
            for (let c = 0; c < cols; c++) {
                const v = matrix[r][c];
                if (v < min) min = v;
                if (v > max) max = v;
            }
        }

        const range = max - min + 1e-9;
        const realThreshold = min + (CONFIG.brightnessThreshold / 255.0) * range;

        for (let r = 0; r < rows; r++) {
            for (let c = 0; c < cols; c++) {
                const idx = r * cols + c;
                if (visited[idx]) continue;
                if (matrix[r][c] > realThreshold) {
                    const blob = floodFill(matrix, r, c, rows, cols, visited, realThreshold);
                    if (blob.radius > CONFIG.minRadius) {
                        blobs.push(blob);
                    }
                }
            }
        }
        return blobs;
    }

    function floodFill(matrix, startR, startC, rows, cols, visited, threshold) {
        const queue = [{ r: startR, c: startC }];
        visited[startR * cols + startC] = 1;
        let minR = startR, maxR = startR;
        let minC = startC, maxC = startC;
        while (queue.length > 0) {
            const curr = queue.shift();
            if (curr.r < minR) minR = curr.r;
            if (curr.r > maxR) maxR = curr.r;
            if (curr.c < minC) minC = curr.c;
            if (curr.c > maxC) maxC = curr.c;
            const neighbors = [
                { r: curr.r - 1, c: curr.c }, { r: curr.r + 1, c: curr.c },
                { r: curr.r, c: curr.c - 1 }, { r: curr.r, c: curr.c + 1 }
            ];
            for (let n of neighbors) {
                if (n.r >= 0 && n.r < rows && n.c >= 0 && n.c < cols) {
                    const nIdx = n.r * cols + n.c;
                    if (!visited[nIdx] && matrix[n.r][n.c] > threshold) {
                        visited[nIdx] = 1;
                        queue.push(n);
                    }
                }
            }
        }
        const centerR = (minR + maxR) / 2;
        const centerC = (minC + maxC) / 2;
        const radius = Math.max((maxR - minR), (maxC - minC)) / 2;
        return { row: centerR, col: centerC, radius: radius };
    }

    // SLAM 本地坐标 (x,y) 转 全局 GPS 经纬度 (lat,lon)
    window.slamToGps = function(targetX, targetY) {
        if (!window.isGpsCalibrated || !window.gpsAnchor || !window.slamAnchor) {
            return null; // 尚未标定完成
        }

        // 1. 计算目标点相对于锚点的位移向量 (单位：米)
        const dx = targetX - window.slamAnchor.x;
        const dy = targetY - window.slamAnchor.y;

        // 2. 将向量旋转到真实地球的正东、正北方向
        const trueDx = dx * Math.cos(window.headingOffset) - dy * Math.sin(window.headingOffset);
        const trueDy = dx * Math.sin(window.headingOffset) + dy * Math.cos(window.headingOffset);

        // 3. 地理常数换算：将米转换回经纬度偏移
        // 纬度 1度 ≈ 111320米，经度 1度 ≈ 111320 * cos(纬度) 米
        const latOffset = trueDy / 111320;
        const lonOffset = trueDx / (111320 * Math.cos(window.gpsAnchor.lat * Math.PI / 180));

        // 4. 返回叠加后的绝对经纬度
        return {
            lat: window.gpsAnchor.lat + latOffset,
            lon: window.gpsAnchor.lon + lonOffset
        };
    };

// ==========================================
    // 2. 页面初始化逻辑 (DOMContentLoaded)
    // ==========================================

    document.addEventListener("DOMContentLoaded", function () {
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
        const segmentSelect = $("segmentSelect");
        const btnAnno = $("btnToggleAnnotation");

        const btnDogStand = $("btnDogStand");  // 机器狗站立
        const btnDogWalk = $("btnDogControl"); // 运动控制

        // ✅ G. 站立/趴下 切换按钮 (保持 Mode 1 和 Mode 4)
        if (btnDogStand) {
            let isDogStanding = false;

            btnDogStand.addEventListener('click', function() {
                if (!isDogStanding) {
                    // === 站立 ===
                    console.log("🚀 指令: 机器狗站立 (Mode 1)");
                    sendMoveCommand(0.0, 0.0, 0.0, MODES.STAND, true);

                    this.textContent = "机器狗趴下";
                    this.classList.add("active");
                    setStatus("发送指令：站立模式", "success");
                    isDogStanding = true;
                } else {
                    // === 趴下 ===
                    console.log("🚀 指令: 机器狗趴下 (Mode 4)");
                    sendMoveCommand(0.0, 0.0, 0.0, MODES.LIE_DOWN, true);

                    this.textContent = "机器狗站立";
                    this.classList.remove("active");
                    setStatus("发送指令：趴下模式", "warning");
                    isDogStanding = false;
                }
            });
        }

        // ✅ H. 运动控制按钮 (回退到 Mode 3)
        if (btnDogWalk) {
            btnDogWalk.addEventListener('click', function() {
                console.log("点击: 进入常规运动模式 (Mode 3)");
                // 发送 Mode 3
                sendMoveCommand(0.0, 0.0, 0.0, MODES.WALK, true);
                setStatus("发送指令：常规运动模式", "success");
            });
        }

        // --- B. 初始化 WebSocket 连接 (机器狗) ---
        connectWebSocket();

        // --- C. 面板切换逻辑 (升级版) ---
        function switchMainPanel(target) {
            console.log("切换面板模式至:", target);

            // 1. 隐藏所有面板
            if (gprPanel) { gprPanel.classList.add("hidden"); gprPanel.style.display = 'none'; }
            if (robotPanel) { robotPanel.classList.add("hidden"); robotPanel.style.display = 'none'; }
            if (armPanel) { armPanel.classList.add("hidden"); armPanel.style.display = 'none'; }
            if (pathFaultPanel) { pathFaultPanel.classList.add("hidden"); pathFaultPanel.style.display = 'none'; } // ✅ 隐藏新面板

            // 2. 显示目标面板
            if (target === 'gpr' && gprPanel) {
                gprPanel.classList.remove("hidden");
                gprPanel.style.display = 'flex';
                scrollToMain();
            } else if (target === 'robot' && robotPanel) {
                robotPanel.classList.remove("hidden");
                robotPanel.style.display = 'flex';
                scrollToMain();
            } else if (target === 'arm' && armPanel) {
                armPanel.classList.remove("hidden");
                armPanel.style.display = 'flex';
                scrollToMain();
            } else if (target === 'pathFault' && pathFaultPanel) { // ✅ 新增：显示故障面板
                pathFaultPanel.classList.remove("hidden");
                pathFaultPanel.style.display = 'flex';
                scrollToMain();
            }
        }

        function scrollToMain() {
            const mainArea = $("mainArea");
            if (mainArea) mainArea.scrollIntoView({ behavior: "smooth", block: "start" });
        }

        // --- D. 绑定底部导航按钮 ---
        if (btn3D) btn3D.addEventListener("click", () => switchMainPanel('gpr'));
        if (btnRobotNav) btnRobotNav.addEventListener("click", () => switchMainPanel('robot'));
        // ✅ 绑定左侧系统栏“设备”按钮 -> 切换到机械臂
        if (btnShowArm) {
            btnShowArm.addEventListener("click", () => switchMainPanel('arm'));
        }

        if (btnPathFault) {
            btnPathFault.addEventListener("click", () => switchMainPanel('pathFault'));
        }

        // --- E. 绑定 GPR 按钮 ---
        if (btnStart) btnStart.addEventListener("click", startGprProcessing);

        if (btnAnno) {
            btnAnno.addEventListener("click", function () {
                showAnnotation = !showAnnotation;
                this.textContent = "显示标注: " + (showAnnotation ? "ON" : "OFF");
                this.classList.toggle("active", showAnnotation);
                if (segmentSelect && !segmentSelect.disabled) {
                    const idx = parseInt(segmentSelect.value, 10);
                    if (!isNaN(idx)) showSegment(idx);
                }
                annotation3DGroups.forEach(g => g.visible = showAnnotation);
            });
        }
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
        if (btnDogStand) {
            // 使用 dataset 在 DOM 元素上存储状态，防止变量污染
            // 0 = 趴下, 1 = 站立
            btnDogStand.dataset.status = "0";

            // ⚡️ 核心修改：使用 onclick 而不是 addEventListener
            // 这样即使 JS 被加载两遍，第二次的也会覆盖第一次，保证只有一个事件生效
            btnDogStand.onclick = function() {

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
                    console.log("🚀 [执行] 切换到: 站立 (Mode 1)");

                    // 发送指令
                    sendMoveCommand(0.0, 0.0, 0.0, MODES.STAND, true);

                    // 更新 UI
                    this.textContent = "机器狗趴下";
                    this.classList.add("active");
                    setStatus("发送指令：站立模式", "success");

                    // 更新状态
                    this.dataset.status = "1";

                } else {
                    // === 当前是站立 -> 切换为趴下 ===
                    console.log("🚀 [执行] 切换到: 趴下 (Mode 4)");

                    // 发送指令
                    sendMoveCommand(0.0, 0.0, 0.0, MODES.LIE_DOWN, true);

                    // 更新 UI
                    this.textContent = "机器狗站立";
                    this.classList.remove("active");
                    setStatus("发送指令：趴下模式", "warning");

                    // 更新状态
                    this.dataset.status = "0";
                }
            };
        }

        if (btnDogWalk) {
            btnDogWalk.addEventListener('click', function() {
                console.log("点击: 运动控制模式");
                // 发送控制指令：Mode = 6
                sendMoveCommand(0.0, 0.0, 0.0, 3.0, true); // 3.0 对应后端的 "常规模式"
                setStatus("发送指令：运动控制模式", "success");
            });
        }
        // ==========================================
        // ✅ 新增：定时获取机械臂实时角度
        // ==========================================

        // 1. 确保端口正确 (你的 Python 代码是 5000)
        // 如果你之前的 ARM_BACKEND_URL 定义在文件开头且是 8080，请修改它，或者在这里覆盖它
        // ✅ 新代码 (正确：指向机械臂的 IP 10.21.31.200)
        const ARM_API = "http://10.21.31.200:5000";
        // 🔴 状态标志位：是否已记录过错误（防止日志重复打印）
        let hasLoggedArmError = false;

        function fetchArmAngles() {
            // 1. 如果正在忙于发送控制指令（防止串口冲突），跳过本次读取，稍后重试
            if (typeof isArmBusy !== 'undefined' && isArmBusy) {
                setTimeout(fetchArmAngles, 100);
                return;
            }

            // 2. 定义下一次请求的延迟时间 (默认 100ms = 10Hz)
            let nextDelay = 100;

            // 设置超时控制器 (500ms 超时)
            const controller = new AbortController();
            const timeoutId = setTimeout(() => controller.abort(), 500);

            // 发起请求
            fetch(ARM_API_BASE + "/get_angles", { signal: controller.signal })
                .then(res => {
                    clearTimeout(timeoutId); // 清除超时
                    if (!res.ok) {
                        throw new Error("Network response was not ok");
                    }
                    return res.json();
                })
                .then(data => {
                    // --- ✅ 连接成功逻辑 ---

                    // 如果之前报过错，说明刚从断线中恢复
                    if (hasLoggedArmError) {
                        console.log("%c✅ 机械臂连接已恢复！切换回高速模式。", "color: green; font-weight: bold;");
                        hasLoggedArmError = false; // 重置错误锁

                        // 恢复界面亮度 (可选)
                        const container = document.getElementById('armDataDisplay');
                        if(container) container.style.opacity = "1";
                    }

                    // 更新界面数值
                    if (Array.isArray(data) && data.length === 6) {
                        for(let i=0; i<6; i++) {
                            // 注意：你的HTML里ID是 disp_j1 ... disp_j6
                            const el = document.getElementById('disp_j' + (i+1));
                            if(el) {
                                let val = data[i];
                                // 视觉死区过滤：小于 0.5 度显示为 0.0，防止数字乱跳
                                if (Math.abs(val) < 0.5) val = 0.0;
                                el.textContent = val.toFixed(1);
                            }
                        }
                    }
                })
                .catch(err => {
                    clearTimeout(timeoutId);

                    // --- ❌ 连接失败逻辑 ---

                    // 只有当“之前没报过错”时，才打印警告，防止控制台黄字刷屏
                    if (!hasLoggedArmError) {
                        console.warn("⚠️ 机械臂未连接 (后端未启动?)，转为低速心跳模式 (2s/次) 以防刷屏。");
                        hasLoggedArmError = true; // 上锁

                        // 让界面变暗表示离线 (可选)
                        const container = document.getElementById('armDataDisplay');
                        if(container) container.style.opacity = "0.5";
                    }

                    // 🔥 关键：连接失败时，将下次请求推迟到 2000ms 后
                    // 这能让浏览器的红色报错频率降低 20 倍！
                    nextDelay = 2000;
                })
                .finally(() => {
                    // 🔄 递归调用：无论成功失败，都通过 setTimeout 发起下一次请求
                    setTimeout(fetchArmAngles, nextDelay);
                });
        }

        // 🔥 启动第一次请求 (不要再用 setInterval 了！)
        fetchArmAngles();

        // ==========================================
        // 🧪 [室内测试专用] 强制虚拟 GPS 标定 (快捷键 Shift + C)
        // ==========================================
        document.addEventListener('keydown', function(event) {
            // 忽略输入框内的按键
            if (document.activeElement.tagName === 'INPUT' || document.activeElement.tagName === 'TEXTAREA') return;

            // 按下 Shift + C 触发室内测试标定
            if (event.code === 'KeyC' && event.shiftKey) {
                // 确保机器狗已经上线并且有 SLAM 坐标
                if (window.currentRobotPose && window.currentRobotPose.x !== -999 && window.currentRobotPose.x !== undefined) {
                    
                    // 1. 设置一个虚拟的基准 GPS 锚点 (这里以武汉某地为例，你可以随便改)
                    window.gpsAnchor = { lat: 30.522222, lon: 114.354444 }; 
                    
                    // 2. 将机器狗当前的 SLAM 坐标与虚拟 GPS 强行绑定
                    window.slamAnchor = { x: window.currentRobotPose.x, y: window.currentRobotPose.y };
                    
                    // 3. 假设 SLAM 坐标系的 X 轴正好指向正东方向 (偏角设为0)
                    window.headingOffset = 0; 
                    
                    // 4. 强制修改状态锁
                    window.isGpsCalibrated = true;
                    window.currentGps = window.gpsAnchor; // 伪造当前信号，防止 UI 显示“标定中”

                    console.log("✅ [室内测试] 虚拟 GPS 标定已强制完成！");
                    alert("📍 虚拟 GPS 标定成功！现在可以点击地图上的电缆测试经纬度换算了。");
                } else {
                    console.warn("⚠️ 尚未获取到机器狗的 SLAM 坐标，请先让机器狗连上并稍微动一下。");
                    alert("⚠️ 尚未获取到机器狗坐标，请先遥控机器狗走一步再标定。");
                }
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
            // 🌟 新增：订阅管线仪数据 (关键修改在这里)
            // ==================================================
            stompClient.subscribe('/topic/pipeline', function (message) {
                try {
                    // 后端 template.convertAndSend 发送的是整数 (0, 1, 2)
                    // message.body 也是字符串 "0", "1", "2"
                    var signal = parseInt(message.body);

                    // 如果解析失败，或者后端改发了 JSON，做个兼容
                    if (isNaN(signal)) {
                        var data = JSON.parse(message.body);
                        signal = data.signal || data;
                    }

                    // 调用你在文件末尾定义的那个函数
                    updatePipelineDisplay(signal);
                    // ✅✅✅ [新增] 将信号存入全局变量，供 Three.js 读取
                    window.g_pipelineSignal = signal;

                    // (可选) 打印日志方便调试
                    // console.log("收到管线信号:", signal);
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
                        window.currentGps = gpsData; 

                        // ⚠️ 假设 currentRobotPose 在这里可以获取到 (它在下方定义的)
                        const pose = window.currentRobotPose; 

                        // --- 自动标定逻辑 ---
                        if (!window.isGpsCalibrated && pose && pose.x !== -999) {
                            if (!window.gpsAnchor) {
                                // 1. 钉下锚点
                                window.gpsAnchor = { lat: gpsData.lat, lon: gpsData.lon };
                                window.slamAnchor = { x: pose.x, y: pose.y };
                                console.log("📍 GPS基准锚点已记录!");
                            } else {
                                // 2. 机器狗走出 1 米后，计算角度差
                                const dx = pose.x - window.slamAnchor.x;
                                const dy = pose.y - window.slamAnchor.y;
                                const dist = Math.hypot(dx, dy);

                                if (dist > 1.0) { 
                                    const slamAngle = Math.atan2(dy, dx);
                                    
                                    const latMeters = (gpsData.lat - window.gpsAnchor.lat) * 111320;
                                    const lonMeters = (gpsData.lon - window.gpsAnchor.lon) * (111320 * Math.cos(window.gpsAnchor.lat * Math.PI / 180));
                                    const gpsAngle = Math.atan2(latMeters, lonMeters); // Y(北), X(东)

                                    window.headingOffset = gpsAngle - slamAngle;
                                    window.isGpsCalibrated = true;
                                    console.log(`🧭 GPS标定完成! 偏角: ${(window.headingOffset * 180 / Math.PI).toFixed(2)}度`);
                                }
                            }
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
            // ✅ 修复：第4个参数(mode/z) 必须是数字 0.0
            sendMoveCommand(0.0, 0.0, 0.0, 0.0, false); // 第5个参数 false 代表不打印日志

            // 20ms后再补一发，确保停止
            setTimeout(() => {
                sendMoveCommand(0.0, 0.0, 0.0, 0.0, true);
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
            sendMoveCommand(v, w, y, 0.0, true);

            // 6. 开启心跳包 (长按连续发送)
            motionTimer = setInterval(() => {
                sendMoveCommand(v, w, y, 0.0, false);
            }, 150);
        }
    }

    // ✅ 修改后的发送函数：增加 mode (通过 linear.z 传递)
    // 参数：v(前后), w(旋转), y(横移), mode(站立=1/运动=6), showLog
    // ✅ 修改后的发送函数：增加了 UI 状态更新
    function sendMoveCommand(v, w, y, mode = 0.0, showLog = true) {
        if (stompClient && stompClient.connected) {
            const payload = JSON.stringify({
                'v': v,
                'w': w,
                'y': y || 0.0,
                'z': mode || 0.0
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
                    case 4.0: text = "趴下 (休眠)"; break;
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
    function startGprProcessing() {
        const input = $("folderPathInput");
        if (!input) return;

        const folderPath = input.value.trim();
        if (!folderPath) {
            alert("请先输入服务器上的数据文件夹路径！");
            return;
        }

        setStatus("正在处理，请稍候……", null);

        fetch("/api/gpr/process?folderPath=" + encodeURIComponent(folderPath), {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({})   // 如需自定义参数，可在这里传 GprConfig
        })
            .then(function (res) {
                if (!res.ok) {
                    throw new Error("后端返回错误状态：" + res.status);
                }
                return res.json();
            })
            .then(function (data) {
                if (!data || !data.segments || !data.segments.length) {
                    throw new Error("返回数据为空或格式不正确");
                }
                setStatus("处理完成。当前显示的是第 1 段 B 扫结果。", "success");
                renderGprResults(data);
            })
            .catch(function (err) {
                console.error(err);
                setStatus("处理失败：" + err.message, "error");
            });
    }

    // 渲染一次完整结果：填充下拉框 + 默认显示第 1 段 + 画 Z 字 3D
    function renderGprResults(result) {
        if (!result || !result.segments || !result.segments.length) {
            throw new Error("返回数据中没有 segments");
        }
        gprResult = result;

        // ✅ 数据加载成功后，显示标注按钮
        const btnAnno = $("btnToggleAnnotation");
        if (btnAnno) btnAnno.style.display = "inline-block";

        const segmentSelect = $("segmentSelect");
        if (segmentSelect) {
            segmentSelect.innerHTML = "";
            result.segments.forEach(function (seg, idx) {
                const opt = document.createElement("option");
                opt.value = String(idx);
                const vStr = seg.vBest != null ? (" v*=" + seg.vBest.toFixed(4) + " m/ns") : "";
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

        showSegment(0);
        render3DZigzagAll(result, true);
    }

    // 显示指定索引的剖面（2D）
    function showSegment(index) {
        if (!gprResult || !gprResult.segments) return;
        if (index < 0 || index >= gprResult.segments.length) return;

        const seg = gprResult.segments[index];

        if (seg.rawData) {
            render2DCanvas("raw2dCanvas", seg.rawData);
        }
        if (seg.migratedData) {
            render2DCanvas("mig2dCanvas", seg.migratedData);
        }

        setStatus("处理完成。当前显示的是第 " + (index + 1) + " 段 B 扫结果。", "success");
    }

    // ====================== 2D 灰度图绘制 ======================

    function render2DCanvas(canvasId, matrix) {
        const canvas = $(canvasId);
        if (!canvas || !matrix || !matrix.length || !matrix[0].length) return;

        const rows = matrix.length;
        const cols = matrix[0].length;
        const maxCanvasWidth = 500;
        const maxCanvasHeight = 500;

        let scaleX = 1, scaleY = 1;
        if (cols > maxCanvasWidth) scaleX = cols / maxCanvasWidth;
        if (rows > maxCanvasHeight) scaleY = rows / maxCanvasHeight;

        const stepX = Math.ceil(scaleX);
        const stepY = Math.ceil(scaleY);
        const w = Math.ceil(cols / stepX);
        const h = Math.ceil(rows / stepY);

        canvas.width = w;
        canvas.height = h;

        const ctx = canvas.getContext("2d");
        const imgData = ctx.createImageData(w, h);

        let min = Infinity, max = -Infinity;
        for (let r = 0; r < rows; r += stepY) {
            for (let c = 0; c < cols; c += stepX) {
                const v = matrix[r][c];
                if (v < min) min = v;
                if (v > max) max = v;
            }
        }
        if (max === min) max = min + 1e-9;

        let idx = 0;
        for (let y = 0, r = 0; r < rows; r += stepY, y++) {
            for (let x = 0, c = 0; c < cols; c += stepX, x++) {
                const v = matrix[r][c];
                let norm = (v - min) / (max - min);
                norm = Math.pow(norm, 0.7);
                const gray = Math.max(0, Math.min(255, Math.round(norm * 255)));
                imgData.data[idx++] = gray;
                imgData.data[idx++] = gray;
                imgData.data[idx++] = gray;
                imgData.data[idx++] = 255;
            }
        }
        ctx.putImageData(imgData, 0, 0);

        // ✅ 绘制标注红圈 (仅针对偏移后剖面 mig2dCanvas)
        if (canvasId === "mig2dCanvas" && showAnnotation) {
            const blobs = detectBlobs(matrix);

            ctx.lineWidth = 1;
            ctx.strokeStyle = "#ff0000"; // 红色
            ctx.shadowColor = "red";
            ctx.shadowBlur = 5;

            blobs.forEach(blob => {
                ctx.beginPath();
                const cx = blob.col / stepX;
                const cy = blob.row / stepY;
                const r = (blob.radius + CONFIG.radiusPadding) / Math.max(stepX, stepY);

                ctx.arc(cx, cy, r, 0, 2 * Math.PI);
                ctx.stroke();
            });
        }
    }

    // ====================== Z 字型三维拼接 ======================

    // 渲染所有剖面，按 Z 字路径拼接到一个三维坐标系（带交互 + 轴）
    function render3DZigzagAll(result, useMigrated) {
        const container = $("threeContainer");
        if (!container) return;

        if (typeof THREE === "undefined") {
            setStatus("three.js 未成功加载，无法显示三维视图。", "error");
            console.error("THREE is not defined. 检查 three.min.js 是否正确引入。");
            return;
        }

        const segments = result.segments || [];
        if (!segments.length) return;

        // 清空旧内容
        container.innerHTML = "";
        annotation3DGroups = []; // ✅ 清空旧的标注组

        // 记录每个剖面的标注圆心和半径，用于后续用圆柱连接
        const segmentAnnoCenters = [];
        const allCenters = [];
        let globalCenterId = 0;

        // 1. 统计所有剖面的幅值范围（统一色标）
        let vMin = Infinity;
        let vMax = -Infinity;
        segments.forEach(seg => {
            const mat = useMigrated ? seg.migratedData : seg.rawData;
            if (!mat) return;
            const rows = mat.length;
            const cols = mat[0].length;
            for (let r = 0; r < rows; r++) {
                for (let c = 0; c < cols; c++) {
                    const v = mat[r][c];
                    if (v < vMin) vMin = v;
                    if (v > vMax) vMax = v;
                }
            }
        });
        if (vMax === vMin) vMax = vMin + 1e-9;

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
        scene.background = new THREE.Color(0xffffff);

        const camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 10000);

        const renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(width, height);
        renderer.toneMapping = THREE.NoToneMapping;
        container.appendChild(renderer.domElement);

        // 所有剖面和路径统一放在 world 组里
        const world = new THREE.Group();
        scene.add(world);

        // 3. 计算几何包围盒 + 最大深度（单位：m）
        const bounds = {
            minX: Infinity, maxX: -Infinity,
            minY: Infinity, maxY: -Infinity,
            minZ: Infinity, maxZ: -Infinity
        };
        let maxDepth = 0.0;  // 真实深度（正值，单位 m）

        // 3.1 逐段构建 Z 字平面
        segments.forEach((seg, segIndex) => {
            const mat = useMigrated ? seg.migratedData : seg.rawData;
            const geo = seg.geometry;
            if (!mat || !geo) return;

            // 记录该段最大深度
            if (geo.zAxis && geo.zAxis.length > 0) {
                const d = geo.zAxis[geo.zAxis.length - 1]; // 单位 m
                if (d > maxDepth) maxDepth = d;
            }

            const mesh = buildSegmentMesh(mat, geo, vMin, vMax, bounds);
            world.add(mesh);

            // ✅ 3D 标注圆环 + 记录圆心和半径
            if (useMigrated && seg.migratedData) {
                const blobs = detectBlobs(seg.migratedData);
                const annoGroup = new THREE.Group();

                const phi = geo.phiRad || 0.0;
                const startX = geo.startX || 0.0;
                const startY = geo.startY || 0.0;
                const cosPhi = Math.cos(phi);
                const sinPhi = Math.sin(phi);

                const rows = seg.migratedData.length;
                const cols = seg.migratedData[0].length;

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
                    const uVal = uAxis[cIdx];

                    const worldX = startX + uVal * cosPhi;
                    const worldZ = startY + uVal * sinPhi; // 注意：这里在 addAxes 里被解释为 y(m)
                    const worldY = -depth;                 // 深度向下为负（worldY）

                    // --- B. 半径动态计算 ---
                    const pixelRadius = blob.radius + CONFIG.radiusPadding;
                    const ringRadius = (pixelRadius * metersPerPixel) * 0.6;
                    const tubeThickness = 0.01;

                    const geometry = new THREE.TorusGeometry(ringRadius, tubeThickness, 16, 32);
                    const material = new THREE.MeshBasicMaterial({ color: 0xff0000 });
                    const ring = new THREE.Mesh(geometry, material);

                    // --- C. 姿态与定位 ---
                    ring.position.set(worldX, worldY, worldZ);
                    ring.rotation.y = -phi;

                    annoGroup.add(ring);

                    // 记录圆心和半径 + 在坐标轴上的投影
                    const centerObj = {
                        id: globalCenterId++,
                        center: new THREE.Vector3(worldX, worldY, worldZ),
                        radius: ringRadius,
                        zCoord: depth,   // 对应 z 轴的坐标值 (m, down)
                        yCoord: worldZ,  // 对应 y 轴的坐标值 (m)
                        segIndex: segIndex
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
        if (segmentAnnoCenters.length >= 2) {
            const edges = [];

            // 1) 构建“相邻剖面”的连接边
            for (let i = 0; i < segmentAnnoCenters.length - 1; i++) {
                const listA = segmentAnnoCenters[i];
                const listB = segmentAnnoCenters[i + 1];

                listA.forEach(a => {
                    listB.forEach(b => {
                        // “同一水平面”：z 轴投影差得小
                        const zDiff = Math.abs(a.zCoord - b.zCoord);
                        // “同一垂直面”：y 轴投影差得小
                        const yDiff = Math.abs(a.yCoord - b.yCoord);

                        if (zDiff <= LINK_CONFIG.maxZDiff &&
                            yDiff <= LINK_CONFIG.maxYDiff) {

                            const radius = (a.radius + b.radius) / 2;
                            const cylinder = createHollowCylinderBetween(
                                a.center,
                                b.center,
                                radius
                            );
                            if (cylinder) {
                                edges.push({
                                    aId: a.id,
                                    bId: b.id,
                                    mesh: cylinder
                                });
                            }
                        }
                    });
                });
            }

            // 2) 用并查集根据 edges 做“管道分组”（连通分量）
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
                const pipelinesMap = new Map(); // root -> [meshes]
                edges.forEach(e => {
                    const root = find(e.aId);
                    let arr = pipelinesMap.get(root);
                    if (!arr) {
                        arr = [];
                        pipelinesMap.set(root, arr);
                    }
                    arr.push(e.mesh);
                });

                // 3) 给每条管道分配不同颜色，并单独建组
                const connPipelinesGroup = new THREE.Group();
                let pipelineIndex = 0;
                const pipelineCount = pipelinesMap.size;

                pipelinesMap.forEach(meshes => {
                    const pipelineGroup = new THREE.Group();

                    // 使用 HSL 均匀取色，颜色区分度比较高
                    const color = new THREE.Color().setHSL(
                        pipelineCount > 0 ? (pipelineIndex / pipelineCount) : 0,
                        0.8,
                        0.5
                    );
                    pipelineIndex++;

                    meshes.forEach(mesh => {
                        if (mesh.material && mesh.material.color) {
                            mesh.material.color.copy(color);
                        }
                        pipelineGroup.add(mesh);
                    });

                    connPipelinesGroup.add(pipelineGroup);
                });

                connPipelinesGroup.visible = showAnnotation;
                world.add(connPipelinesGroup);
                annotation3DGroups.push(connPipelinesGroup);
            }
        }

        // 3.3 路径折线（放在底部）
        if (Array.isArray(result.pathX) && Array.isArray(result.pathY) &&
            result.pathX.length === result.pathY.length) {

            const n = result.pathX.length;
            const points = [];
            for (let i = 0; i < n; i++) {
                const x = result.pathX[i];  // x 方向（m）
                const z = result.pathY[i];  // 横向（m）
                const y = -maxDepth;        // 底部（最大深度处）
                points.push(new THREE.Vector3(x, y, z));
            }
            const pathGeom = new THREE.BufferGeometry().setFromPoints(points);
            const pathMat = new THREE.LineBasicMaterial({ color: 0x000000 });
            world.add(new THREE.Line(pathGeom, pathMat));
        }

        // 3.4 画三维坐标轴（x(m), y(m), z(m,down)）
        if (maxDepth > 0) {
            addAxes(scene, bounds, maxDepth);
        }

        // 4. 相机围绕“数据中心”绕圈观察
        const centerX = (bounds.minX + bounds.maxX) / 2;
        const centerY = (bounds.minY + bounds.maxY) / 2;
        const centerZ = (bounds.minZ + bounds.maxZ) / 2;
        const center = new THREE.Vector3(centerX, centerY, centerZ);

        const spanX = bounds.maxX - bounds.minX;
        const spanZ = bounds.maxZ - bounds.minZ;
        const spanY = bounds.maxY - bounds.minY;
        const span = Math.max(spanX, spanZ, spanY) || 1.0;

        let radius = span * 2.0;
        let theta = Math.PI / 4; // 绕 Y 轴
        let phi = Math.PI / 4;   // 仰角

        function updateCamera() {
            const x = center.x + radius * Math.cos(phi) * Math.cos(theta);
            const y = center.y + radius * Math.sin(phi);
            const z = center.z + radius * Math.cos(phi) * Math.sin(theta);
            camera.position.set(x, y, z);
            camera.lookAt(center);
        }
        updateCamera();

        // ========== 交互：左键旋转，中键平移，滚轮缩放 ==========

        const dom = renderer.domElement;
        let isRotating = false;
        let isPanning = false;
        let lastX = 0;
        let lastY = 0;

        dom.addEventListener("mousedown", e => {
            if (e.button === 0) {           // 左键旋转
                isRotating = true;
                isPanning = false;
            } else if (e.button === 1) {    // 中键平移
                isPanning = true;
                isRotating = false;
            } else {
                return;
            }
            lastX = e.clientX;
            lastY = e.clientY;
        });

        window.addEventListener("mouseup", () => {
            isRotating = false;
            isPanning = false;
        });

        dom.addEventListener("contextmenu", e => e.preventDefault());

        dom.addEventListener("mousemove", e => {
            const dx = e.clientX - lastX;
            const dy = e.clientY - lastY;

            if (isRotating) {
                theta += dx * 0.005;
                phi += dy * 0.005;
                const eps = 0.1;
                phi = Math.max(eps, Math.min(Math.PI - eps, phi));
                updateCamera();
            } else if (isPanning) {
                const panSpeed = span * 0.002;
                const dir = new THREE.Vector3();
                camera.getWorldDirection(dir);
                const up = new THREE.Vector3(0, 1, 0);
                const right = new THREE.Vector3().crossVectors(dir, up).normalize().negate();
                const camUp = new THREE.Vector3().crossVectors(right, dir).normalize();

                center.add(right.multiplyScalar(dx * panSpeed));
                center.add(camUp.multiplyScalar(-dy * panSpeed));
                updateCamera();
            }

            lastX = e.clientX;
            lastY = e.clientY;
        });

        dom.addEventListener("wheel", e => {
            e.preventDefault();
            const factor = e.deltaY > 0 ? 1.1 : 0.9;
            radius *= factor;
            radius = Math.max(span * 0.5, Math.min(span * 5.0, radius));
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
            requestAnimationFrame(animate);
            renderer.render(scene, camera);
        }
        animate();
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
            color: 0xff0000
        });

        const mesh = new THREE.Mesh(geometry, material);

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
    function buildSegmentMesh(matrix, geometry, vMin, vMax, bounds) {
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

        const cosPhi = Math.cos(phi);
        const sinPhi = Math.sin(phi);

        const vertexCount = rows * cols;
        const positions = new Float32Array(vertexCount * 3);
        const colors = new Float32Array(vertexCount * 3);

        let idxV = 0;
        let idxC = 0;
        for (let r = 0; r < rows; r++) {
            const depth = zAxis[r];      // >=0
            const worldY = -depth;       // 深度向下为负

            for (let c = 0; c < cols; c++) {
                const u = uAxis[c];

                // 沿着 Z 字路径的水平位置
                const xHoriz = startX + u * cosPhi;
                const yHoriz = startY + u * sinPhi;

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

                let norm = (v - vMin) / (vMax - vMin); // [0,1]
                norm = Math.max(0, Math.min(1, norm));
                norm = Math.pow(norm, 1.2);

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
            toneMapped: false
        });

        const mesh = new THREE.Mesh(geom, material);
        console.log("GPR mesh material type =", mesh.material.type);
        return mesh;
    }

    // 在场景中添加一个三维坐标系：
    // x (m)   : 沿电缆方向
    // y (m)   : 横向偏移
    // z (m,down) : 深度，向下为正（从 0 到 maxDepth）
    function addAxes(scene, bounds, maxDepth) {
        if (!isFinite(maxDepth) || maxDepth <= 0) return;

        const axisGroup = new THREE.Group();
        scene.add(axisGroup);

        const xLen = bounds.maxX - bounds.minX;   // 沿电缆方向长度（m）
        const yLen = bounds.maxZ - bounds.minZ;   // 横向宽度（m）
        const zLen = maxDepth;                    // 最大深度（m）

        const span = Math.max(xLen, yLen, zLen) || 1.0;
        const tickSize = span * 0.03;
        const labelSize = span * 0.06;
        const nTicks = 5;

        // 顶部原点（深度 0）
        const originTop = new THREE.Vector3(bounds.minX, 0, bounds.minZ);
        // 底部原点（最大深度）
        const originBottom = new THREE.Vector3(bounds.minX, -zLen, bounds.minZ);

        function axisLine(start, end) {
            const geom = new THREE.BufferGeometry().setFromPoints([start, end]);
            const mat = new THREE.LineBasicMaterial({ color: 0x000000 });
            return new THREE.Line(geom, mat);
        }

        // ================= Z 轴：深度（0 -> maxDepth，向下为正）=================
        const zEnd = originBottom.clone();
        axisGroup.add(axisLine(originTop, zEnd));

        for (let i = 0; i <= nTicks; i++) {
            const d = (zLen / nTicks) * i; // 深度值
            const pos = originTop.clone().add(new THREE.Vector3(0, -d, 0));

            const tickStart = pos.clone().add(new THREE.Vector3(-tickSize * 0.5, 0, 0));
            const tickEnd = pos.clone().add(new THREE.Vector3(tickSize * 0.5, 0, 0));
            axisGroup.add(axisLine(tickStart, tickEnd));

            const label = makeTextSprite(d.toFixed(1));
            label.scale.set(labelSize, labelSize, 1);
            label.position.copy(pos).add(new THREE.Vector3(0, 0, -tickSize * 2.0));
            axisGroup.add(label);
        }

        const zLabel = makeTextSprite("z (m, down)");
        zLabel.scale.set(labelSize * 1.2, labelSize * 1.2, 1);
        zLabel.position.copy(zEnd).add(new THREE.Vector3(0, 0, -tickSize * 3.0));
        axisGroup.add(zLabel);

        // ================= Y 轴：横向偏移，从 Z 轴终点（底部）开始 =================
        const yEnd = originBottom.clone().add(new THREE.Vector3(0, 0, yLen));
        axisGroup.add(axisLine(originBottom, yEnd));

        for (let i = 0; i <= nTicks; i++) {
            const t = (yLen / nTicks) * i;
            const val = bounds.minZ + t;  // 实际偏移（m）
            const pos = originBottom.clone().add(new THREE.Vector3(0, 0, t));

            const tickStart = pos.clone().add(new THREE.Vector3(-tickSize * 0.5, 0, 0));
            const tickEnd = pos.clone().add(new THREE.Vector3(tickSize * 0.5, 0, 0));
            axisGroup.add(axisLine(tickStart, tickEnd));

            const label = makeTextSprite(val.toFixed(1));
            label.scale.set(labelSize, labelSize, 1);
            label.position.copy(pos).add(new THREE.Vector3(0, -tickSize * 1.5, 0));
            axisGroup.add(label);
        }

        const yLabel = makeTextSprite("y (m)");
        yLabel.scale.set(labelSize * 1.2, labelSize * 1.2, 1);
        yLabel.position.copy(yEnd).add(new THREE.Vector3(0, -tickSize * 2.0, labelSize * 0.8));
        axisGroup.add(yLabel);

        // ================= X 轴：以 “Y 轴终点” 作为起点 =================
        const originX = yEnd.clone();  // ★ 这里：X 轴起点 = Y 轴终点
        const xEnd = originX.clone().add(new THREE.Vector3(xLen, 0, 0));
        axisGroup.add(axisLine(originX, xEnd));

        for (let i = 0; i <= nTicks; i++) {
            const t = (xLen / nTicks) * i;
            const pos = originX.clone().add(new THREE.Vector3(t, 0, 0));

            const tickStart = pos.clone().add(new THREE.Vector3(0, 0, -tickSize * 0.5));
            const tickEnd = pos.clone().add(new THREE.Vector3(0, 0, tickSize * 0.5));
            axisGroup.add(axisLine(tickStart, tickEnd));

            const label = makeTextSprite(t.toFixed(1));
            label.scale.set(labelSize, labelSize, 1);
            label.position.copy(pos).add(new THREE.Vector3(0, -tickSize * 1.5, 0));
            axisGroup.add(label);
        }

        const xLabel = makeTextSprite("x (m)");
        xLabel.scale.set(labelSize * 1.2, labelSize * 1.2, 1);
        xLabel.position.copy(xEnd).add(new THREE.Vector3(labelSize * 0.8, -tickSize * 2.0, 0));
        axisGroup.add(xLabel);
    }

    // 用 Canvas 生成一个简单的文字 Sprite
    function makeTextSprite(text) {
        const canvas = document.createElement("canvas");
        const size = 256;
        canvas.width = size;
        canvas.height = size;
        const ctx = canvas.getContext("2d");

        ctx.clearRect(0, 0, size, size);
        ctx.fillStyle = "rgba(255,255,255,0)"; // 透明背景
        ctx.fillRect(0, 0, size, size);
        ctx.font = "72px Arial";
        ctx.fillStyle = "#000000";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(text, size / 2, size / 2);

        const texture = new THREE.CanvasTexture(canvas);
        texture.needsUpdate = true;

        const material = new THREE.SpriteMaterial({
            map: texture,
            transparent: true
        });

        const sprite = new THREE.Sprite(material);
        return sprite;
    }

    // ==========================================
    // 5. ✅ 机械臂控制函数 (已适配 Java Spring Boot)
    // ==========================================

    // 🔴 全局状态：记录当前6个关节的角度 (初始化为0度)
    // 因为 Java 接口每次都需要发送所有关节的绝对位置
    var currentAngles = [0, 0, 0, 0, 0, 0];

    // 辅助函数：限制角度范围 (防止撞机)
    function clampAngle(angle) {
        // 这里的范围 -170 到 170 是 myCobot 的物理极限参考
        return Math.max(-170, Math.min(170, angle));
    }

    // 5.1 末端坐标控制 (XYZ)
    // 注意：目前的 Java 后端脚本只接受 6 个关节角度。
    // 如果要支持 XYZ，需要在 Python 脚本中实现逆运动学 (IK)。
    window.controlArm = function(x, y, z) {
        console.warn("⚠️ 后端暂仅支持关节角度控制，XYZ 控制需集成逆运动学。");
        alert("XYZ 控制暂时不可用，请使用下方的关节控制面板。");

        // 如果未来实现了 IK，调用方式大概如下：
        // fetch(`${ARM_BACKEND_URL}/arm/move_ik?x=${x}&y=${y}&z=${z}...`)
    };

    // 5.2 关节控制 (点击按钮步进)
    // index: 0-5 (对应 J1-J6), direction: 1 (正向) 或 -1 (反向)
    window.controlJoint = function(index, direction) {
        // 1. 定义步长 (每次点击转动多少度)
        const step = 5;

        // 2. 更新本地记录的角度状态
        let newAngle = currentAngles[index] + (direction > 0 ? step : -step);

        // 限制角度范围
        newAngle = clampAngle(newAngle);

        // 更新全局状态
        currentAngles[index] = newAngle;

        const jointName = "J" + (index + 1);
        console.log(`🚀 准备发送指令: ${jointName} -> ${newAngle}° (当前全状态: ${currentAngles})`);

        // 3. 拼接 URL 参数
        // 目标格式: /arm/move?j1=0&j2=0&...&speed=50
        // 使用 map 和 join 快速生成查询字符串
        const queryParams = currentAngles.map((angle, i) => `j${i+1}=${angle}`).join("&");
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
// 5.3 ✅ 一键测试序列 (添加到 controlJoint 函数下面)
// ==========================================

    window.testArmSequence = function() {
        console.log("🧪 开始机械臂一键测试...");

        // 1. 定义动作 A：所有关节去 20 度
        const target20 = [20, 20, 20, 20, 20, 20];
        const speed = 50;

        // 更新全局状态 (很重要！否则下次点击单关节控制时位置会乱)
        if (typeof currentAngles !== 'undefined') {
            currentAngles = [...target20];
        }

        // 拼接 URL 并发送第一波指令
        // URL: /arm/move?j1=20&j2=20...&speed=50
        const query20 = target20.map((a, i) => `j${i+1}=${a}`).join("&");
        const url20 = `${ARM_BACKEND_URL}/arm/move?${query20}&speed=${speed}`;

        console.log("🚀 动作1: 前往全 20 度");
        fetch(url20).then(res => res.text()).then(msg => console.log("反馈:", msg));

        // 2. 延时 1000 毫秒 (1秒) 后执行动作 B：归零
        // 给机械臂足够的时间运动到位
        setTimeout(() => {
            console.log("🔄 动作2: 1秒已到，执行自动归零");

            const target0 = [0, 0, 0, 0, 0, 0];

            // 再次更新全局状态
            if (typeof currentAngles !== 'undefined') {
                currentAngles = [...target0];
            }

            const query0 = target0.map((a, i) => `j${i+1}=${a}`).join("&");
            const url0 = `${ARM_BACKEND_URL}/arm/move?${query0}&speed=${speed}`;

            fetch(url0).then(res => res.text()).then(msg => console.log("归零反馈:", msg));

            // 简单的弹窗提示（可选）
            // alert("测试序列完成！");
        }, 1000);
    };
    // 全局变量，记录状态
    let isStopping = false; // 是否正在停止中
    // ==========================================
// 5.4 ✅ 终极版：状态锁控制 (State Machine)
// ==========================================

// 🔴 核心变量：记录机械臂是否正在运动
// 只有当它是 true 时，我们才允许发 stop
    let isJogging = false;

    window.startArmJog = function(index, direction, event) {
        // 1. 阻止浏览器默认行为 (防止拖拽按钮、选中文字)
        if (event) {
            event.preventDefault();
            event.stopPropagation();
        }

        // 2. 如果已经在运动中，防止重复发送 Start (屏蔽微小抖动)
        if (isJogging) return;

        // 3. 立即上锁
        isJogging = true;

        console.log(`🚀 [前端] 开始运动: J${index+1} 方向 ${direction}`);

        // 4. 发送 Start 请求
        const url = `${ARM_BACKEND_URL}/arm/jog?index=${index}&direction=${direction}`;
        fetch(url).catch(err => {
            console.error("Start 失败:", err);
            // 如果发送失败，必须把锁打开，否则用户没法再触发了
            isJogging = false;
        });
    };

    window.stopArmJog = function() {
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

        // --- 1. 数据存储 (保留双通道) ---
        let channel1Data = []; // 磁信号 (用于绘图)
        let channel2Data = []; // 声信号 (用于绘图 - ✅ 保持不变)

        // 历史数值缓存 (UI显示用 - ✅ 保持不变)
        let magHistory = [0, 0, 0];
        let sndHistory = [0, 0, 0];

        // --- 2. [新增] AUC 算法专用变量 (仅针对磁信号) ---
        const BUFFER_SIZE = 20000;
        const ringBuffer = new Float32Array(BUFFER_SIZE);
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
        function processMagAlgorithm(newDataChunk) {
            for (let i = 0; i < newDataChunk.length; i++) {
                const rawVal = newDataChunk[i];
                const adjustedVal = rawVal - 2048; // 去直流
                
                // 1. 写入环形缓冲区
                ringBuffer[writePtr] = adjustedVal;
                writePtr = (writePtr + 1) % BUFFER_SIZE;
                totalSamples++;

                // 2. 实时触发检测 (Trigger)
                if (algoState === 'IDLE') {
                    if (Math.abs(adjustedVal) > AUC_CONFIG.PULSE_THRESHOLD) {
                        // 冷却时间检查
                        if (totalSamples - lastTriggerTime > AUC_CONFIG.DISTANCE_MIN) {
                            // console.log(`⚡ [AUC触发] T=${totalSamples}`);
                            algoState = 'WAITING_DATA';
                            triggerIndex = totalSamples;
                            lastTriggerTime = totalSamples;
                        }
                    }
                }
            }

            // 3. 检查数据是否攒够 (Window Full?)
            if (algoState === 'WAITING_DATA') {
                if (totalSamples >= triggerIndex + AUC_CONFIG.WINDOW_AFTER) {
                    performAUCAnalysis();
                    algoState = 'IDLE';
                }
            }
        }

        // 执行 AUC 积分分析
        function performAUCAnalysis() {
            // 提取窗口数据: [Trigger - BEFORE, Trigger + AFTER]
            const startAbs = triggerIndex - AUC_CONFIG.WINDOW_BEFORE;
            const len = AUC_CONFIG.WINDOW_BEFORE + AUC_CONFIG.WINDOW_AFTER;
            const segment = new Float32Array(len);
            
            // 从环形缓冲区复制 (处理回绕)
            for (let i = 0; i < len; i++) {
                let idx = (startAbs + i) % BUFFER_SIZE;
                if (idx < 0) idx += BUFFER_SIZE;
                segment[i] = ringBuffer[idx];
            }

            // 1. 计算 Mean & Std
            let sumAbs = 0;
            const absSeg = new Float32Array(len);
            for(let i=0; i<len; i++) {
                absSeg[i] = Math.abs(segment[i]);
                sumAbs += absSeg[i];
            }
            const mean = sumAbs / len;

            let sumSqDiff = 0;
            for(let i=0; i<len; i++) {
                sumSqDiff += (absSeg[i] - mean) ** 2;
            }
            const std = Math.sqrt(sumSqDiff / len);

            if (std <= 0) return;

            // 2. Z-score 积分
            let noisePointsCount = 0;
            let aucSum = 0;
            // 🔥 这里使用了全局配置中的 NOISE_THRESHOLD (由滑条控制)
            const threshold = AUC_CONFIG.NOISE_THRESHOLD;

            for(let i=0; i<len; i++) {
                const zScore = (absSeg[i] - mean) / std;
                if (zScore > threshold) {
                    noisePointsCount++;
                    aucSum += absSeg[i];
                }
            }

            // 3. 最终判定
            const isDischarge = (noisePointsCount >= AUC_CONFIG.BURST_THRESHOLD) && 
                                (aucSum >= AUC_CONFIG.AUC_THRESHOLD);

            // 4. 结果输出
            if (isDischarge) {
                console.log(`🔴 [故障判定] AUC:${aucSum.toFixed(0)} > ${AUC_CONFIG.AUC_THRESHOLD}, Pts:${noisePointsCount}`);
                
                window.g_isFaultDetected = true;
                // ✅✅✅ [新增] 专门给二分法等待期使用的信号触发标志
                window.g_faultTriggeredInBS = true;

                // 为了让 UI 显示数值，取触发点的原始值 (加回 2048)
                // 也可以直接用 aucSum 来代表强度，看你需求
                const peakVal = Math.abs(segment[AUC_CONFIG.WINDOW_BEFORE]) + 2048;
                window.g_realtimeMag = peakVal;

                // UI 变红提示
                const elMag = document.getElementById('magSignalVal');
                if(elMag) elMag.style.color = "#ff4444";
                
                // 0.5秒后自动复位状态，防止一直锁定
                setTimeout(() => { window.g_isFaultDetected = false; }, 500);
            }
        }


        // ============================================================
        // 🎨 绘图逻辑 (保留原有双通道绘制)
        // ============================================================
        function drawSignal() {
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

            // ✅ 绘制 Ch1 (磁信号)
            if (channel1Data && channel1Data.length > 0) {
                drawChannel(channel1Data, 0, halfH, "#00ccff");
            }

            // ✅ 绘制 Ch2 (声信号 - 保持不变)
            if (channel2Data && channel2Data.length > 0) {
                drawChannel(channel2Data, halfH, halfH, "#ff3333");
            }
            
            // 更新 UI 数值
            updateUIValues();
            
            requestAnimationFrame(drawSignal);
        }

        // 通用单通道绘图函数 (✅ 保持不变)
        function drawChannel(data, topY, height, color) {
            ctx.strokeStyle = color;
            ctx.lineWidth = 1.5;
            ctx.beginPath();

            const len = data.length;
            const w = canvas.width;

            for (let i = 0; i < len; i++) {
                const x = (i / len) * w;
                let val = data[i];
                if (val > Y_MAX) val = Y_MAX;
                if (val < 0) val = 0;

                const normalized = val / Y_MAX;
                const y = topY + (height - (normalized * height));

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
            // 1. 磁信号数值 (取最新数据的最大值，或者由 AUC 结果驱动)
            if (channel1Data.length > 0) {
                // 为了界面跳动效果，我们依然取当前帧的一个代表值
                // 这里取当前帧最大值作为显示
                const currVal = Math.max(...channel1Data);
                magHistory.push(Math.floor(currVal));
                if (magHistory.length > 3) magHistory.shift();
                
                const elMag = document.getElementById('magSignalVal');
                if(elMag) {
                    elMag.textContent = magHistory.join('   ');
                    if (!window.g_isFaultDetected) elMag.style.color = "#00e5ff";
                }
            }

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

        // --- 4. WebSocket 连接 (✅ 保持不变) ---
        const wsUrl = "ws://10.21.31.104:8001/ws/signal";
        const ws = new WebSocket(wsUrl);

        ws.onopen = () => { 
            console.log("📡 AUC Signal Engine Connected");
            if(statusDiv) {
                statusDiv.textContent = "● AUC LIVE";
                statusDiv.style.color = "#0f0";
            }
            resizeCanvas();
        };

        ws.onmessage = (event) => {
            try {
                const msg = JSON.parse(event.data);
                if (msg.type === "mixed_signal" || msg.type === "signal") {
                    // 1. 处理磁信号 (Ch1) -> 送入 AUC 算法 & 绘图缓存
                    if (msg.mag_data) {
                        channel1Data = msg.mag_data; // 存下来画图用
                        processMagAlgorithm(msg.mag_data); // 送去算 AUC
                    } else if (msg.ch1) {
                        channel1Data = msg.ch1;
                        processMagAlgorithm(msg.ch1);
                    }

                    // 2. ✅ 处理声信号 (Ch2) -> 仅存下来画图 (保持不变)
                    if (msg.snd_data) {
                        channel2Data = msg.snd_data;
                    } else if (msg.ch2) {
                        channel2Data = msg.ch2;
                    }
                }
            } catch (e) {
                console.error("Data Parse Error", e);
            }
        };

        ws.onerror = (err) => {
            if(statusDiv) { statusDiv.textContent = "● ERROR"; statusDiv.style.color = "#f00"; }
        };

        ws.onclose = () => {
            if(statusDiv) { statusDiv.textContent = "● OFFLINE"; statusDiv.style.color = "#f00"; }
        };
    })();

// ==========================================
// I. 自动巡线按钮逻辑
// ==========================================
    const btnAutoTrack = document.getElementById("btnAutoTrack");

// 定义状态：是否正在巡线
    let isAutoTracking = false;

    if (btnAutoTrack) {
        // 使用 onclick 避免重复绑定
        btnAutoTrack.onclick = function() {
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
                        if(lbl) {
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
                        if(lbl) {
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
            sendMoveCommand(0.0, 0.0, 0.0, 0.0, true);

            // 双重保险：50ms 后再发一次，防止丢包
            setTimeout(() => {
                sendMoveCommand(0.0, 0.0, 0.0, 0.0, false);
            }, 50);
        } else {
            console.error("❌ 无法调用 sendMoveCommand，请检查代码作用域");
        }

        // --- B. 重置“自动巡线”状态 (逻辑急停) ---
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
    document.addEventListener('DOMContentLoaded', function() {
        // 获取两个 STOP 按钮
        const btnSideStop = document.getElementById('btnSideStop'); // 侧边栏 STOP
        const btnStopMove = document.getElementById('btnStopMove'); // 控制盘 STOP

        // 绑定点击事件
        // 注意：这两个按钮在上面的 motionBtnIds 里已经绑定了 mousedown/mouseup 用于停止手动运动
        // 这里绑定 click 是为了处理“自动巡线”状态的复位
        if (btnSideStop) {
            btnSideStop.addEventListener('click', function(e) {
                e.preventDefault();
                emergencyStop();
            });
        }

        if (btnStopMove) {
            btnStopMove.addEventListener('click', function(e) {
                e.preventDefault();
                emergencyStop();
            });
        }

        // 绑定空格键作为急停快捷键
        document.addEventListener('keydown', function(event) {
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
                sendMoveCommand(v, w, y, 0.0, false);
            }
        }

        document.addEventListener('keydown', function(event) {
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

        document.addEventListener('keyup', function(event) {
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
                        sendMoveCommand(0.0, 0.0, 0.0, 0.0, true);
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

    let mapCursor = 0;
    let lastKeyframePose = { x: -999, y: -999 };

    // ✅ 新增：用于连续破损区域追踪的状态变量
    let lastFaultMarker = null;    // 存储当前最新的那个红球/闪电 Mesh
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
    let lastFaultRobotDist = -1; // 上一次正常测到信号时，机器狗在偏移路径上的行驶距离
    let offsetPathDists = [];    // 缓存偏移路径上每个点到起点的累加距离

    // 1. 计算整条路径的距离积分
    function computePathDists(path) {
        offsetPathDists = [0];
        let d = 0;
        for(let i = 1; i < path.length; i++) {
            d += Math.hypot(path[i].x - path[i-1].x, path[i].y - path[i-1].y);
            offsetPathDists.push(d);
        }
    }

    // 2. 计算当前坐标在路径上所处的“里程数” (距离起点的距离)
    function getDistOnPath(pose, path) {
        if(!path || path.length < 2) return 0;
        let minDist = Infinity, bestT = 0, bestIdx = 0;
        for(let i = 0; i < path.length - 1; i++) {
            const p1 = path[i], p2 = path[i+1];
            const dx = p2.x - p1.x, dy = p2.y - p1.y;
            const len2 = dx*dx + dy*dy;
            let t = 0;
            if (len2 > 0) t = ((pose.x - p1.x)*dx + (pose.y - p1.y)*dy) / len2;
            t = Math.max(0, Math.min(1, t));
            const px = p1.x + t*dx, py = p1.y + t*dy;
            const dist = Math.hypot(pose.x - px, pose.y - py);
            if(dist < minDist) {
                minDist = dist; bestT = t; bestIdx = i;
            }
        }
        return offsetPathDists[bestIdx] + bestT * Math.hypot(path[bestIdx+1].x - path[bestIdx].x, path[bestIdx+1].y - path[bestIdx].y);
    }

    // 3. 根据给定的“里程数”，反推该点的 (x, y) 坐标
    function getPoseAtDist(targetDist, path) {
        if(targetDist <= 0) return {x: path[0].x, y: path[0].y};
        if(targetDist >= offsetPathDists[offsetPathDists.length-1]) return {x: path[path.length-1].x, y: path[path.length-1].y};
        for(let i = 0; i < offsetPathDists.length - 1; i++) {
            if(targetDist >= offsetPathDists[i] && targetDist <= offsetPathDists[i+1]) {
                const segLen = offsetPathDists[i+1] - offsetPathDists[i];
                let t = 0;
                if (segLen > 0) t = (targetDist - offsetPathDists[i]) / segLen;
                return {
                    x: path[i].x + t * (path[i+1].x - path[i].x),
                    y: path[i].y + t * (path[i+1].y - path[i].y)
                };
            }
        }
        return {x: path[path.length-1].x, y: path[path.length-1].y};
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
        smooth: function(points) {
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
            for(let i=0; i<16; i++) Q[i] *= process_noise;
            const R = this.eye(2);
            for(let i=0; i<4; i++) R[i] *= measurement_noise;

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
    m20Model.position.set(0, 0, 0.35); 
            
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

    // B1. 实时雷达层
    const scanGeom = new THREE.BufferGeometry();
    const scanPositions = new Float32Array(5000 * 3);
    const scanColors = new Float32Array(5000 * 3); 
    scanGeom.setAttribute('position', new THREE.BufferAttribute(scanPositions, 3));
    scanGeom.setAttribute('color', new THREE.BufferAttribute(scanColors, 3)); 

    const scanMesh = new THREE.Points(scanGeom, new THREE.PointsMaterial({
        vertexColors: true, 
        size: 0.03,         
        sizeAttenuation: true
    }));
    scanMesh.frustumCulled = false;
    scene.add(scanMesh);

    // B2. 历史地图层
    const mapMaxPoints = 1000000;
    const mapGeom = new THREE.BufferGeometry();
    const mapPositions = new Float32Array(mapMaxPoints * 3);
    const mapColors = new Float32Array(mapMaxPoints * 3); 

    mapGeom.setAttribute('position', new THREE.BufferAttribute(mapPositions, 3));
    mapGeom.setAttribute('color', new THREE.BufferAttribute(mapColors, 3)); 

    for(let i=0; i<mapMaxPoints*3; i++) mapPositions[i] = 99999;

    const mapMesh = new THREE.Points(mapGeom, new THREE.PointsMaterial({
        vertexColors: true,
        size: 0.03,
        sizeAttenuation: true
    }));
    mapMesh.frustumCulled = false;
    scene.add(mapMesh);

    

    // C. 机器狗轨迹
    const trailGroup = new THREE.Group();
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
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 1, 8), new THREE.MeshBasicMaterial({color:0xffffff}));
    pole.rotation.x = Math.PI/2; pole.position.z = 0.5;
    const flag = new THREE.Mesh(new THREE.ShapeGeometry(new THREE.Shape().moveTo(0,0).lineTo(0,0.4).lineTo(0.6,0.2).lineTo(0,0)), new THREE.MeshBasicMaterial({color:0xff0000, side:THREE.DoubleSide}));
    flag.rotation.x = Math.PI/2; flag.position.set(0,0,0.6);
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
    let currentRobotPose = { x: 0, y: 0, yaw: 0 };
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
            const dist = Math.sqrt(dx*dx + dy*dy);

            if (elNavDist) elNavDist.textContent = dist.toFixed(2);
            if (elNavPanel) elNavPanel.style.display = 'block';
        } else {
            if (elNavPanel) elNavPanel.style.display = 'none';
        }
    }

    // --- ✅ A* 寻路算法 ---
    const GRID_SIZE = 0.2;

    function worldToGrid(x, y) {
        return { x: Math.round(x / GRID_SIZE), y: Math.round(y / GRID_SIZE) };
    }
    function gridToWorld(gx, gy) {
        return { x: gx * GRID_SIZE, y: gy * GRID_SIZE };
    }

    function planPathAStar(start, end) {
        console.log("🔍 A* 规划中...");
        const gridStart = worldToGrid(start.x, start.y);
        let gridEnd = worldToGrid(end.x, end.y);

        const obstacles = new Set();
        const addObstacle = (wx, wy, wz) => {
            if (wz > 0.2 && wz < 1.0) {
                const g = worldToGrid(wx, wy);
                for(let i=-1; i<=1; i++) {
                    for(let j=-1; j<=1; j++) {
                        obstacles.add(`${g.x+i},${g.y+j}`);
                    }
                }
            }
        };

        const scanArr = scanMesh.geometry.attributes.position.array;
        for(let i=0; i<scanArr.length; i+=3) {
            if(scanArr[i] < 9000) addObstacle(scanArr[i], scanArr[i+1], scanArr[i+2]);
        }
        const mapArr = mapMesh.geometry.attributes.position.array;
        for(let i=0; i<mapCursor*3; i+=3) {
            if(mapArr[i] < 9000) addObstacle(mapArr[i], mapArr[i+1], mapArr[i+2]);
        }

        const endKey = `${gridEnd.x},${gridEnd.y}`;
        if(obstacles.has(endKey)) {
            console.warn("目标点不可达，搜索附近...");
            let found = false, minD = Infinity, bestG = null;
            for(let dx=-3; dx<=3; dx++) {
                for(let dy=-3; dy<=3; dy++) {
                    const nx = gridEnd.x+dx, ny = gridEnd.y+dy;
                    if(!obstacles.has(`${nx},${ny}`)) {
                        const d = dx*dx + dy*dy;
                        if(d < minD) { minD=d; bestG={x:nx, y:ny}; found=true; }
                    }
                }
            }
            if(found) gridEnd = bestG;
            else { alert("❌ 目标区域完全堵塞！"); return null; }
        }

        const openSet = [];
        const closedSet = new Set();
        const cameFrom = {};
        const gScore = {};
        const fScore = {};

        const startKey = `${gridStart.x},${gridStart.y}`;
        openSet.push(gridStart);
        gScore[startKey] = 0;
        fScore[startKey] = Math.hypot(gridStart.x - gridEnd.x, gridStart.y - gridEnd.y);

        let iterations = 0;
        while(openSet.length > 0) {
            if(iterations++ > 5000) { console.warn("A* 超时"); break; }

            openSet.sort((a,b) => (fScore[`${a.x},${a.y}`]||Infinity) - (fScore[`${b.x},${b.y}`]||Infinity));
            const current = openSet.shift();
            const currentKey = `${current.x},${current.y}`;

            if(current.x === gridEnd.x && current.y === gridEnd.y) {
                const path = [];
                let currKey = currentKey;
                while(cameFrom[currKey]) {
                    const coords = currKey.split(',').map(Number);
                    path.unshift({x: coords[0]*GRID_SIZE, y: coords[1]*GRID_SIZE});
                    currKey = cameFrom[currKey];
                }
                path.push({x: gridEnd.x*GRID_SIZE, y: gridEnd.y*GRID_SIZE});
                return simplifyPath(path);
            }

            closedSet.add(currentKey);
            const neighbors = [{x:0,y:1}, {x:0,y:-1}, {x:1,y:0}, {x:-1,y:0}, {x:1,y:1}, {x:1,y:-1}, {x:-1,y:1}, {x:-1,y:-1}];

            for(let n of neighbors) {
                const neighbor = {x: current.x + n.x, y: current.y + n.y};
                const nKey = `${neighbor.x},${neighbor.y}`;
                if(closedSet.has(nKey) || obstacles.has(nKey)) continue;

                const cost = (n.x===0 || n.y===0) ? 1.0 : 1.414;
                const tG = gScore[currentKey] + cost;

                if(tG < (gScore[nKey] || Infinity)) {
                    cameFrom[nKey] = currentKey;
                    gScore[nKey] = tG;
                    fScore[nKey] = tG + Math.hypot(neighbor.x - gridEnd.x, neighbor.y - gridEnd.y);
                    if(!openSet.some(o => o.x===neighbor.x && o.y===neighbor.y)) openSet.push(neighbor);
                }
            }
        }
        alert("❌ 无法找到路径"); return null;
    }

    function simplifyPath(path) {
        if(path.length < 3) return path;
        const newPath = [path[0]];
        for(let i=1; i<path.length-1; i++) {
            const last = newPath[newPath.length-1];
            const curr = path[i];
            if(Math.hypot(curr.x-last.x, curr.y-last.y) > 0.4) {
                newPath.push(curr);
            }
        }
        newPath.push(path[path.length-1]);
        return newPath;
    }

    function calculateAvgError() {
        if (navOriginalPath.length < 2 || activeNavTrail.length === 0) {
            if (elNavError) elNavError.textContent = "--";
            return;
        }
        function distToSegmentSquared(p, v, w) {
            const l2 = (w.x - v.x)**2 + (w.y - v.y)**2;
            if (l2 === 0) return (p.x - v.x)**2 + (p.y - v.y)**2;
            let t = ((p.x - v.x)*(w.x - v.x) + (p.y - v.y)*(w.y - v.y)) / l2;
            t = Math.max(0, Math.min(1, t));
            return (p.x - (v.x + t*(w.x - v.x)))**2 + (p.y - (v.y + t*(w.y - v.y)))**2;
        }
        function distToPolyline(p, polyline) {
            let minDst2 = Infinity;
            for(let i=0; i<polyline.length-1; i++) {
                const d2 = distToSegmentSquared(p, polyline[i], polyline[i+1]);
                if(d2 < minDst2) minDst2 = d2;
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
                    navGoal = path[path.length-1];
                    flagObj.position.set(navGoal.x, navGoal.y, 0);
                    flagObj.visible = true;

                    isPicking = false;
                    const btn = document.getElementById('btnSetNavPoint');
                    if(btn) btn.classList.remove('active');
                    renderer.domElement.style.cursor = 'default';

                    if(elNavError) elNavError.textContent = "--";
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

            if (window.savedCablePath && window.savedCablePath.length > 1) {
                for (let i = 0; i < window.savedCablePath.length - 1; i++) {
                    const p1 = window.savedCablePath[i];
                    const p2 = window.savedCablePath[i+1];
                    const dx = p2.x - p1.x, dy = p2.y - p1.y, dz = p2.z - p1.z;
                    const segLenSq = dx*dx + dy*dy + dz*dz; 
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
                    }
                    currentAccumulatedDist += segLen;
                }
            }

            let gpsText = "未就绪 (等待标定)";
            const calculatedGps = window.slamToGps(hitPoint.x, hitPoint.y);
            if (calculatedGps) gpsText = `${calculatedGps.lat.toFixed(6)}, ${calculatedGps.lon.toFixed(6)}`;
            else if (window.currentGps) gpsText = "标定中...请移动机器狗>1米";

            document.getElementById('popupDist').textContent = bestAccumulatedDist.toFixed(2);
            document.getElementById('popupDepth').textContent = "待同步"; 
            document.getElementById('popupGps').textContent = gpsText; 
            
            const elPopup = document.getElementById('cableInfoPopup');
            if (elPopup) elPopup.style.display = 'block';

            // ✅ 核心：更新右上角固定的实景抓拍预留区
            const frontImg = document.getElementById('dispCaptureFront');
            const rearImg = document.getElementById('dispCaptureRear');
            
            if (frontImg && rearImg) {
                if (clickedFaultId && window.faultImagesMap && window.faultImagesMap.has(clickedFaultId)) {
                    // 1. 点中了破损点，且存在抓拍图片 -> 显示图片
                    const imgs = window.faultImagesMap.get(clickedFaultId);
                    frontImg.src = imgs.front;
                    rearImg.src = imgs.rear;
                } else {
                    // 2. 点中了普通电缆，或者该破损点尚未抓拍 -> 清空 src，恢复等待状态
                    frontImg.src = "";
                    rearImg.src = "";
                }
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
        if(isPicking) return;
        e.preventDefault();
        if (e.button === 0) isRotating = true;
        else if (e.button === 1) isPanning = true;
        prevPos = { x: e.offsetX, y: e.offsetY };
    });

    renderer.domElement.addEventListener('mousemove', e => {
        if(isPicking) return;
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

    if(btnSet) {
        btnSet.onclick = function() {
            isPicking = !isPicking;
            if(isPicking) {
                this.classList.add('active');
                renderer.domElement.style.cursor = 'crosshair';
            } else {
                this.classList.remove('active');
                renderer.domElement.style.cursor = 'default';
            }
        };
    }

    if(btnClear) {
        btnClear.onclick = function() {
            stopNav(false);
            navPath = [];
            navGoal = null;
            flagObj.visible = false;
            navLine.visible = false;
            updateDistanceUI();
            if(elNavError) elNavError.textContent = "--";
        };
    }

    if(btnStart) {
        btnStart.onclick = function() {
            if(navPath.length === 0) {
                alert("请先点击 [📍选点] 生成避障路径！");
                return;
            }
            if(isNavigating) return;
            startNav();
        };
    }

    const stopBtnIds = ['btnStopMove', 'btnSideStop'];
    stopBtnIds.forEach(id => {
        const btn = document.getElementById(id);
        if(btn) {
            btn.addEventListener('click', function() {
                if(isNavigating) {
                    console.log("🛑 检测到手动停止");
                    stopNav(false);
                }
            });
        }
    });

    function startNav() {
        console.log("🚀 开始导航...");
        isNavigating = true;
        
        // 只有当不是电缆跟踪且不是寻找破损模式时，才记录 A* 原始路径
        if (!isCableTrackingMode && !isFaultFindingMode) {
            navOriginalPath = [...navPath];
            navOriginalPath.unshift({x: currentRobotPose.x, y: currentRobotPose.y});
            activeNavTrail = [];
            if(elNavError) elNavError.textContent = "运行中...";
        }

        if(typeof sendMoveCommand === 'function') sendMoveCommand(0, 0, 0, 3.0, true);
        if(navInterval) clearInterval(navInterval);
        navInterval = setInterval(controlLoop, 100);
    }

    function stopNav(isSuccess) {
        isNavigating = false;
        if(navInterval) { clearInterval(navInterval); navInterval = null; }
        if(typeof sendMoveCommand === 'function') sendMoveCommand(0, 0, 0, 0.0, true);
        console.log("🛑 导航停止");
        
        // ✅✅✅ 修正：电缆跟踪与寻找破损模式停止逻辑
        if (isCableTrackingMode || isFaultFindingMode) {
            if (cableTrackingState === 'TO_START' && isSuccess) {
                console.log("✅ 已到达起始切入点，开始沿线运行...");
                cableTrackingState = 'FOLLOWING';
                
                if (isCableTrackingMode && window.savedCablePath) {
                    navPath = window.savedCablePath.map(p => ({x: p.x, y: p.y}));
                } else if (isFaultFindingMode && window.offsetCablePath) {
                    navPath = window.offsetCablePath; // 使用计算好的偏移路径
                }
                startNav();
            } else {
                console.log("⏹️ 自动跟踪/寻找任务结束或停止");
                
                isCableTrackingMode = false;
                isFaultFindingMode = false;
                cableTrackingState = 'IDLE';

                // 恢复两个按钮 UI
                const btn1 = document.getElementById("btnTrackCableOverlay");
                if(btn1) {
                    btn1.innerText = "⚡ 跟踪电缆";
                    btn1.style.color = "#00ccff";
                    btn1.style.border = "1px solid #00ccff";
                }
                const btn2 = document.getElementById("btnFindFaultOverlay");
                if(btn2) {
                    btn2.innerText = "🔍 寻找破损点";
                    btn2.style.color = "#ffaa00";
                    btn2.style.border = "1px solid #ffaa00";
                }
            }
        } else {
            // 普通 A* 导航停止
            if (isSuccess) calculateAvgError();
        }
    }

    // ✅ 核心控制：纯追踪 (Pure Pursuit) + 二分法拦截
    function controlLoop() {
        if(navPath.length === 0) { stopNav(true); return; }

        // =========================================================
        // 🌟🌟🌟 [新增] 二分法破损点精确定位逻辑拦截
        // =========================================================
        if (isFaultFindingMode && window.offsetCablePath && window.offsetCablePath.length > 0) {
            if (offsetPathDists.length === 0) computePathDists(window.offsetCablePath);
            const currentDist = getDistOnPath(currentRobotPose, window.offsetCablePath);

            // 触发条件：正常前进状态，且上一个破损点已记录，且走出 2m 还没新破损点
            if (bsState === 'INACTIVE' && lastFaultRobotDist >= 0 && (currentDist - lastFaultRobotDist > 2.0)) {
                console.log("🛑 [二分法触发] 超过 2m 未发现新破损点，准备启动精确定位...");
                bsState = 'INIT_WAIT';
                bsGoodDist = lastFaultRobotDist; // 上次有信号的距离
                bsBadDist = currentDist;         // 当前无信号的距离
                window.g_faultTriggeredInBS = false; 
                bsWaitEndTime = Date.now() + 5000; // 原地等待 5 秒
                sendMoveCommand(0, 0, 0, 0, true); // 立即停车
                return;
            }

            // 如果已经进入了二分法状态，接管控制，不再执行下面的常规纯追踪
            if (bsState !== 'INACTIVE') {
                handleBinarySearchControl(currentDist);
                return; 
            }
        }

        const LOOKAHEAD_DIST = 0.8;
        let targetIdx = 0;
        let target = navPath[0];

        for(let i=0; i<navPath.length; i++) {
            const p = navPath[i];
            const d = Math.hypot(p.x - currentRobotPose.x, p.y - currentRobotPose.y);
            if(d > LOOKAHEAD_DIST) {
                target = p;
                targetIdx = i;
                break;
            }
            if(i === navPath.length-1) {
                target = p;
                targetIdx = i;
            }
        }

        if(targetIdx > 0) {
            navPath.splice(0, targetIdx);
        }

        const dx = target.x - currentRobotPose.x;
        const dy = target.y - currentRobotPose.y;
        const dist = Math.sqrt(dx*dx + dy*dy);

        if(navPath.length === 1 && dist < 0.2) {
            stopNav(true);
            return;
        }

        const targetYaw = Math.atan2(dy, dx);
        let diff = targetYaw - currentRobotPose.yaw;
        while(diff > Math.PI) diff -= 2*Math.PI;
        while(diff < -Math.PI) diff += 2*Math.PI;

        let v = 0, w = 0;
        if(Math.abs(diff) > 0.8) {
            v = 0.1;
            w = diff > 0 ? 0.6 : -0.6;
        } else {
            v = 0.4 * (1.0 - Math.abs(diff)/1.5);
            v = Math.max(0.15, v);
            w = diff * 1.5;
        }

        if(typeof sendMoveCommand === 'function') {
            sendMoveCommand(v, w, 0, 0, false);
        }
    }
    function handleBinarySearchControl(currentDist) {
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

        // 阶段 2: 到达某个测试点后，等待 5秒 评估是否有信号
        if (bsState === 'EVAL_WAIT') {
            if (now < bsWaitEndTime) return;

            // 5秒评估结束，看期间有没有触发过信号
            if (window.g_faultTriggeredInBS) {
                console.log(`✅ [二分定位] 在 ${currentDist.toFixed(2)}m 处检测到信号！更新有信号边界。`);
                bsGoodDist = currentDist; // 更新有信号边界
            } else {
                console.log(`❌ [二分定位] 在 ${currentDist.toFixed(2)}m 处无信号！更新无信号边界。`);
                bsBadDist = currentDist;  // 更新无信号边界
            }

            // 检查误差是否满足退出条件 (小于 0.1m)
            if (Math.abs(bsBadDist - bsGoodDist) <= 0.1) {
                console.log(`🎯 [二分定位] 定位完成！误差已 <= 0.1m。最终真实破损点位于: ${bsGoodDist.toFixed(2)}m`);
                bsState = 'DONE';
                setStatus("🎯 破损点精准定位完成！", "success");
                stopNav(true);
                return;
            }

            // 计算新的目标点并进入移动状态
            bsTargetDist = (bsGoodDist + bsBadDist) / 2.0;
            bsState = 'MOVING';

            // ✅ 新增：在每次发车前清空上一轮的信号，确保在去下一个点的路上干净地重新检测
            window.g_faultTriggeredInBS = false;
            const action = (bsTargetDist > currentDist) ? "向前移动" : "倒退移动";
            console.log(`🤖 [二分定位] 准备${action}至: ${bsTargetDist.toFixed(2)}m (搜索区间: ${Math.min(bsGoodDist, bsBadDist).toFixed(2)} ~ ${Math.max(bsGoodDist, bsBadDist).toFixed(2)})`);
            return;
        }

        // 阶段 3: 向目标点移动 (支持倒车防偏移)
        if (bsState === 'MOVING') {
            const distError = bsTargetDist - currentDist;
            
            // 如果误差小于 0.1m，视为到达目标
            if (Math.abs(distError) < 0.1) {
                sendMoveCommand(0, 0, 0, 0, true); // 停车
                bsState = 'EVAL_WAIT';
                // ❌ 删除下面这行！不要在刚停下来的时候清空移动中好不容易抓到的信号！
                // window.g_faultTriggeredInBS = false; 
                bsWaitEndTime = Date.now() + 5000; // 等待 5秒
                console.log("⏱️ [二分定位] 到达目标点，原地等待 5 秒检测信号...");
                return;
            }

            // 获取目标坐标
            const targetPose = getPoseAtDist(bsTargetDist, window.offsetCablePath);
            const dx = targetPose.x - currentRobotPose.x;
            const dy = targetPose.y - currentRobotPose.y;
            
            let targetYaw = Math.atan2(dy, dx);
            let diff = targetYaw - currentRobotPose.yaw;
            while(diff > Math.PI) diff -= 2*Math.PI;
            while(diff < -Math.PI) diff += 2*Math.PI;

            let v = 0.0, w = 0.0;

            if (distError < 0) {
                // === 倒车模式 ===
                v = -0.15; // 慢速倒退
                // 倒车时，机器狗的前方是远离目标的，所以要把目标偏航角反转 180 度进行纠偏
                let reverseYaw = targetYaw + Math.PI;
                while(reverseYaw > Math.PI) reverseYaw -= 2*Math.PI;
                while(reverseYaw < -Math.PI) reverseYaw += 2*Math.PI;

                diff = reverseYaw - currentRobotPose.yaw;
                while(diff > Math.PI) diff -= 2*Math.PI;
                while(diff < -Math.PI) diff += 2*Math.PI;
            } else {
                // === 前进模式 ===
                v = 0.15; // 慢速前进
            }

            // 结合车头误差计算转向
            if(Math.abs(diff) > 0.8) {
                v = (distError < 0) ? -0.05 : 0.05; // 角度偏大时降速
                w = diff > 0 ? 0.4 : -0.4;
            } else {
                w = diff * 1.0;
            }

            if(typeof sendMoveCommand === 'function') {
                sendMoveCommand(v, w, 0, 0, false);
            }
        }
    }

    // 5. 旧清除按钮 (修改后：只清除蓝色机器狗轨迹)
    const btnClearDog = document.getElementById("btnClearDogPath");
    if (btnClearDog) {
        btnClearDog.onclick = function() {
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

    const btnCaptureFault = document.getElementById("btnCaptureFault");
    if (btnCaptureFault) {
        btnCaptureFault.onclick = function() {
            // 确保当前有破损点存在
            if (!lastFaultMarker || !lastFaultMarker.userData.faultId) {
                alert("⚠️ 当前没有检测到破损点，无法进行抓拍绑定！");
                return;
            }

            const faultId = lastFaultMarker.userData.faultId;
            const originalText = this.innerHTML;
            this.innerHTML = "⏳ 抓拍读取中...";
            this.disabled = true;

            // 调用 Python 后端的抓拍接口
            fetch("http://10.21.31.104:5000/api/capture")
                .then(res => {
                    if (!res.ok) throw new Error("网络响应异常");
                    return res.json();
                })
                .then(data => {
                    if (data.status === "success") {
                        // 将图片存入字典
                        window.faultImagesMap.set(faultId, { 
                            front: data.front || "", 
                            rear: data.rear || "" 
                        });
                        this.innerHTML = "✅ 原帧绑定成功";
                    } else {
                        alert("❌ 后端抓拍失败: " + data.message);
                        this.innerHTML = "❌ 抓拍失败";
                    }
                })
                .catch(err => {
                    console.error("抓拍出错:", err);
                    this.innerHTML = "❌ 请求超时";
                })
                .finally(() => {
                    setTimeout(() => { 
                        this.innerHTML = originalText; 
                        this.disabled = false;
                    }, 2000);
                });
        };
    }

    // ✅ 新增: 清除点云数据按钮逻辑
    const btnClearCloud = document.getElementById("btnClearCloud");
    if (btnClearCloud) {
        btnClearCloud.onclick = function() {
            console.log("🧹 清除所有点云数据...");
            const scanAttr = scanMesh.geometry.attributes.position;
            const scanArr = scanAttr.array;
            for(let i=0; i<scanArr.length; i++) scanArr[i] = 99999;
            scanAttr.needsUpdate = true;

            const mapAttr = mapMesh.geometry.attributes.position;
            const mapArr = mapAttr.array;
            for(let i=0; i<mapArr.length; i++) mapArr[i] = 99999;
            mapAttr.needsUpdate = true;

            mapCursor = 0;
            lastKeyframePose = { x: -999, y: -999 };
        };
    }
    // ✅ [新增] 清除破损点按钮逻辑
    const btnClearFaults = document.getElementById("btnClearFaults");
    if (btnClearFaults) {
        btnClearFaults.onclick = function() {
            console.log("🧹 清除所有破损点标记...");
            // ✅ 新增：清除绑定的图片数据及界面显示
            if (window.faultImagesMap) window.faultImagesMap.clear();
            const frontImg = document.getElementById('dispCaptureFront');
            const rearImg = document.getElementById('dispCaptureRear');
            if (frontImg && rearImg) {
                frontImg.src = "";
                rearImg.src = "";
            }

            maxFaultIdx = -1;  // 🌟 新增：重置最远探测进度
            bsState = 'INACTIVE'; // ✅ 重置二分法状态
            lastFaultRobotDist = -1;

            // 1. 重置连续追踪的状态变量 (防止红线乱连)
            if (typeof lastFaultMarker !== 'undefined') lastFaultMarker = null;
            if (typeof lastFaultCablePos !== 'undefined') lastFaultCablePos = null;
            maxFaultIdx = -1;  // 🌟 新增：重置最远探测进度

            // 2. 清除红色球体组 (faultMarkerGroup)
            if (faultMarkerGroup) {
                while(faultMarkerGroup.children.length > 0){
                    const obj = faultMarkerGroup.children[0];
                    faultMarkerGroup.remove(obj);
                    // 递归释放内存 (兼容 Group 和 单个 Mesh)
                    obj.traverse(child => {
                        if(child.geometry) child.geometry.dispose();
                        if(child.material) {
                            if(child.material.map) child.material.map.dispose();
                            child.material.dispose();
                        }
                    });
                }
            }

            // 3. 清除红色电缆路径组 (redCableGroup)
            // 注意：redCableGroup 是在 initSlamVis 开头定义的
            if (redCableGroup) {
                while(redCableGroup.children.length > 0){
                    const mesh = redCableGroup.children[0];
                    redCableGroup.remove(mesh);
                    // 释放内存
                    if(mesh.geometry) mesh.geometry.dispose();
                    if(mesh.material) mesh.material.dispose();
                }
            }
        };
    }

    const btnClearCable = document.getElementById("btnClearCablePath");
    if (btnClearCable) {
        btnClearCable.onclick = function() {
            cablePoints.length = 0;
            segmentPoints.length = 0;
            lastCablePos.set(-999, -999, 0);
            
            if (currentCableMesh) {
                cableGroup.remove(currentCableMesh);
                currentCableMesh.geometry.dispose();
                currentCableMesh = null;
            }
            while(redCableGroup.children.length > 0){
                const mesh = redCableGroup.children[0];
                redCableGroup.remove(mesh);
                if(mesh.geometry) mesh.geometry.dispose();
            }
            // 追加这行：同时隐藏信息弹窗
            document.getElementById('cableInfoPopup').style.display = 'none';
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
        btnTrack.onclick = function() {
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
            const pathPoints = window.savedCablePath.map(p => ({x: p.x, y: p.y}));
            
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
                for(let i=0; i<pathPoints.length; i++) {
                    const d = Math.hypot(pathPoints[i].x - currentRobotPose.x, pathPoints[i].y - currentRobotPose.y);
                    if(d < minD) {
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
        // ✅✅✅ 新增：注入“寻找破损点”悬浮按钮逻辑
        // ========================================================
        const btnFault = document.createElement('button');
        btnFault.innerText = "🔍 寻找破损点";
        btnFault.id = "btnFindFaultOverlay";
        btnFault.style.position = "absolute";
        btnFault.style.top = "240px"; // 放在跟踪电缆按钮下方
        btnFault.style.right = "10px";
        btnFault.style.zIndex = "999";
        btnFault.style.padding = "8px 12px";
        btnFault.style.backgroundColor = "rgba(0, 0, 0, 0.6)";
        btnFault.style.border = "1px solid #ffaa00"; // 橙黄色主题
        btnFault.style.color = "#ffaa00";
        btnFault.style.borderRadius = "4px";
        btnFault.style.cursor = "pointer";
        btnFault.style.fontSize = "14px";
        btnFault.style.fontWeight = "bold";
        btnFault.style.transition = "all 0.3s";

        btnFault.onmouseover = () => { btnFault.style.backgroundColor = "rgba(255, 170, 0, 0.2)"; };
        btnFault.onmouseout = () => { btnFault.style.backgroundColor = "rgba(0, 0, 0, 0.6)"; };

        container.appendChild(btnFault);

        btnFault.onclick = function() {
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
            lastFaultRobotDist = -1;
            offsetPathDists = [];
            maxFaultIdx = -1;
            isFaultFindingMode = true;
            btnFault.innerText = "⏹ 停止寻找";
            btnFault.style.color = "#ff4444";
            btnFault.style.border = "1px solid #ff4444";

            // === 核心算法：计算左右偏移 0.3m 的路径 ===
            const pathPoints = window.savedCablePath.map(p => ({x: p.x, y: p.y}));
            const n = pathPoints.length;
            const normals = [];

            // 1. 求出路径上每个点的“向左”法向量
            for (let i = 0; i < n; i++) {
                let dx, dy;
                // 用相邻点作差求切向量
                if (i === 0) {
                    dx = pathPoints[1].x - pathPoints[0].x;
                    dy = pathPoints[1].y - pathPoints[0].y;
                } else if (i === n - 1) {
                    dx = pathPoints[n-1].x - pathPoints[n-2].x;
                    dy = pathPoints[n-1].y - pathPoints[n-2].y;
                } else {
                    dx = pathPoints[i+1].x - pathPoints[i-1].x;
                    dy = pathPoints[i+1].y - pathPoints[i-1].y;
                }
                const len = Math.hypot(dx, dy);
                if (len > 0) { dx /= len; dy /= len; }
                
                // 将切向量 (dx, dy) 逆时针旋转 90 度得到向左的法向量 (-dy, dx)
                normals.push({nx: -dy, ny: dx});
            }

            // 2. 叉乘法：判断机器狗在电缆左侧还是右侧
            const p0 = pathPoints[0];
            const pNext = pathPoints[Math.min(5, n - 1)]; 
            const dirX = pNext.x - p0.x;
            const dirY = pNext.y - p0.y;
            const rx = currentRobotPose.x - p0.x;
            const ry = currentRobotPose.y - p0.y;
            
            // 计算 2D 叉积
            const cross = dirX * ry - dirY * rx;
            let offsetDist = 0.3; // 默认向左偏移 0.3m
            if (cross < 0) {
                offsetDist = -0.3; // 如果叉积小于0，说明在右侧，向右偏移 0.3m
                console.log("👉 机器狗位于电缆右侧，生成右侧 0.3m 平行路径");
            } else {
                console.log("👈 机器狗位于电缆左侧，生成左侧 0.3m 平行路径");
            }

            // 3. 生成平行偏移的新路径
            const offsetPath = [];
            for (let i = 0; i < n; i++) {
                offsetPath.push({
                    x: pathPoints[i].x + normals[i].nx * offsetDist,
                    y: pathPoints[i].y + normals[i].ny * offsetDist
                });
            }
            window.offsetCablePath = offsetPath; // 存入全局供 TO_START 状态调用

            // === 柔性并入逻辑 (以偏移路径为基准) ===
            const op0 = offsetPath[0];
            const odirX = offsetPath[Math.min(5, n-1)].x - op0.x;
            const odirY = offsetPath[Math.min(5, n-1)].y - op0.y;
            const orx = currentRobotPose.x - op0.x;
            const ory = currentRobotPose.y - op0.y;
            
            const dotProduct = orx * odirX + ory * odirY;
            const distToStart = Math.hypot(orx, ory);

            const isBehindStart = (dotProduct < -0.1) && (distToStart > 1.0);

            if (isBehindStart) {
                console.log("🚀 位于偏移路径起点后方，A* 导航至平行起点...");
                cableTrackingState = 'TO_START';
                const path = planPathAStar(currentRobotPose, op0);
                if (path) {
                    navPath = path;
                    startNav();
                } else {
                     alert("⚠️ 无法规划路径到平行起点，尝试直接平行切入...");
                     cableTrackingState = 'FOLLOWING';
                     navPath = offsetPath;
                     startNav();
                }
            } else {
                console.log("✅ 启用柔性平行并入 (Flexible Parallel Merge)...");
                cableTrackingState = 'FOLLOWING';
                let minD = Infinity;
                let closestIdx = 0;
                for(let i=0; i<offsetPath.length; i++) {
                    const d = Math.hypot(offsetPath[i].x - currentRobotPose.x, offsetPath[i].y - currentRobotPose.y);
                    if(d < minD) {
                        minD = d;
                        closestIdx = i;
                    }
                }
                
                if (closestIdx >= offsetPath.length - 1) {
                    alert("⚠️ 已经到达电缆终点附近！");
                    stopNav(false);
                    return;
                }
                
                navPath = offsetPath.slice(closestIdx);
                startNav();
            }
        };
    })();

    // --- 6. WebSocket ---
    const ROBOT_IP = "10.21.31.104";
    const wsUrl = "ws://" + ROBOT_IP + ":9090";
    console.log("正在尝试连接 WebSocket:", wsUrl);

    const ws = new WebSocket(wsUrl);
    let hasLoggedSlamSuccess = false;
    let isFirstFrame = true;

    ws.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            if (data.type === 'slam_data') {
                if (!hasLoggedSlamSuccess) {
                    console.log("%c🚀 SLAM 数据传输成功！...", "color: #00e5ff; font-weight: bold;");
                    hasLoggedSlamSuccess = true;
                }
                updateVis(data);
            }
        } catch (e) { }
    };

    function updateVis(data) {
        const pose = data.robot;
        if (pose) {
            const scaleFactor = 1.0;
            const px = pose.x * scaleFactor;
            const py = pose.y * scaleFactor;
            // ✅ 新增逻辑：提取真实的 Z 轴高度，减去 0.3m 的补偿值
            const pz = ((pose.z !== undefined ? pose.z : 0.05) - 0.25) * scaleFactor;

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

            currentRobotPose = { x: px, y: py, yaw: pose.yaw };
            window.currentRobotPose = currentRobotPose; // 🌟 增加这一行：每次机器狗移动时，实时同步给全局变量

            updateNavLine();
            updateDistanceUI();

            if (isNavigating) {
                const len = activeNavTrail.length;
                if (len === 0) activeNavTrail.push({x: px, y: py});
                else {
                    const lastP = activeNavTrail[len - 1];
                    if (Math.hypot(px - lastP.x, py - lastP.y) > 0.02) activeNavTrail.push({x: px, y: py});
                }
            }

            dogGroup.position.set(px, py, pz);
            // ✅ 新增提取 pitch 角度，如果没有则默认为 0
            const pitch = pose.pitch !== undefined ? pose.pitch : 0.0;
            
            // ✅ 核心修改：同时设置 pitch(绕Y轴) 和 yaw(绕Z轴)
            // 'ZYX' 是旋转顺序，必须指定，否则在同时有偏航和俯仰时模型会发生万向节死锁导致倾斜
            dogGroup.rotation.set(0, pitch, pose.yaw, 'ZYX');

            if (typeof gridHelper !== 'undefined') {
                gridHelper.position.set(px, py, 0);
                if(clickPlane) clickPlane.position.set(px, py, 0);
            }

            if (isFirstFrame) {
                orbitTarget.x = px; orbitTarget.y = py; orbitTarget.z = 0;
                updateCamera();
                isFirstFrame = false;
            }

            const currentPos = new THREE.Vector3(px, py, pz);
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
                        const p2 = window.savedCablePath[i+1];
                        
                        // 计算线段的向量方向
                        const dx = p2.x - p1.x;
                        const dy = p2.y - p1.y;
                        const dz = p2.z - p1.z;
                        const segLenSq = dx*dx + dy*dy + dz*dz; 
                        
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
                        // 🌟 核心修正：严格执行“单向向前推进”逻辑
                        // 判断规则：只有当新探测到的点（closestIdx）大于上一次记录的最远点（maxFaultIdx）时，才允许向前方更新！
                        // 这样即使机器狗后退到变红的区域内探测到信号，也会被这里拦截，标志保持在最前端不动。
                        if (maxFaultIdx === -1 || closestIdx > maxFaultIdx) {
                            
                            // ✅ 只有在常规追踪模式下，才记录最新信号的距离
                            if (bsState === 'INACTIVE') {
                                if (window.offsetCablePath && offsetPathDists.length === 0) computePathDists(window.offsetCablePath);
                                if (window.offsetCablePath) lastFaultRobotDist = getDistOnPath(currentPos, window.offsetCablePath);
                            }

                            // 距离去重，防止原地微小抖动频繁重绘
                            if (!lastFaultCablePos || closestPoint.distanceTo(lastFaultCablePos) > 0.02) {
                                
                                // --- A. 删除上一个临时点 ---
                                if (lastFaultMarker) {
                                    faultMarkerGroup.remove(lastFaultMarker);
                                    lastFaultMarker.traverse(child => {
                                        if(child.geometry) child.geometry.dispose();
                                        if(child.material) {
                                            if(child.material.map) child.material.map.dispose();
                                            child.material.dispose();
                                        }
                                    });
                                    lastFaultMarker = null;
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
                                lastFaultMarker = faultGroup;

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
                                
                                // 🌟 关键锁死：将当前的最远进度记录下来。下次只要没超过这个进度，坚决不更新！
                                maxFaultIdx = closestIdx;

                                if (bsState === 'INACTIVE') {
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
            if (!isCableTrackingMode) {
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
                        const curve = new THREE.CatmullRomCurve3(trailPoints);
                        const tube = new THREE.TubeGeometry(curve, trailPoints.length * 3, 0.03, 6, false);
                        currentTrailMesh = new THREE.Mesh(tube, trailMat);
                        trailGroup.add(currentTrailMesh);
                    }
                }
            }

            // =========================================================
            // ✅✅✅ [核心修改] 特征点提取 + 卡尔曼滤波平滑
            // =========================================================
            const sig = (typeof window.g_pipelineSignal !== 'undefined') ? window.g_pipelineSignal : -1;
            
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

            // ✅✅✅ 修改：增加 isAutoTracking 判断
            // 只有在 (是特征点) AND (不是在跟踪回放模式) AND (正在自动巡线录制模式) 时，才绘制电缆
            // ✅✅✅ 修改：增加 isFaultFindingMode 判断
            if (isFeaturePoint && !isCableTrackingMode && !isFaultFindingMode && isAutoTracking) {
                const dist = currentPos.distanceTo(lastCablePos);
                if (dist > 0.1) {
                    const cablePt = new THREE.Vector3(px, py, 0.08);
                    cablePoints.push(cablePt);
                    lastCablePos = cablePt;

                    if (cablePoints.length > 2000) cablePoints.shift();
                    if (cablePoints.length > 3) {
                        if (currentCableMesh) {
                            cableGroup.remove(currentCableMesh);
                            currentCableMesh.geometry.dispose();
                            currentCableMesh = null;
                        }
                        // 调用卡尔曼平滑
                        let displayPoints = KalmanSmoother.smooth(cablePoints);
                        
                        // ✅✅✅ 保存平滑后的路径供跟踪使用
                        window.savedCablePath = displayPoints;

                        // 绘制平滑曲线
                        const curve = new THREE.CatmullRomCurve3(displayPoints);
                        const tube = new THREE.TubeGeometry(curve, displayPoints.length * 8, 0.03, 8, false);
                        currentCableMesh = new THREE.Mesh(tube, cableMat);
                        cableGroup.add(currentCableMesh);
                    }
                }
            }
            lastSignal = sig;
        }

        const scan = data.scan;
        if (scan && scan.length > 0) {
            const scanAttr = scanMesh.geometry.attributes.position;
            const colAttr = scanMesh.geometry.attributes.color; 

            const scanPos = scanAttr.array;
            const scanCol = colAttr.array;

            let c = 0; 
            let d = 0; 
            let scaleFactor = 1.0;
            if (data.robot && (Math.abs(data.robot.x) > 100)) scaleFactor = 0.001;

            const isFlat = (scan.length > 0 && typeof scan[0] === 'number');

            if (isFlat) {
                for (let i = 0; i < scan.length; i += 4) {
                    if (c >= 5000 * 3) break;

                    scanPos[c++] = scan[i] * scaleFactor;
                    scanPos[c++] = scan[i+1] * scaleFactor;
                    let valZ = (i + 2 < scan.length) ? scan[i+2] : 0.1;
                    scanPos[c++] = valZ * scaleFactor;

                    const intensity = (i + 3 < scan.length) ? scan[i+3] : 0;
                    const maxVal = 10.0;
                    let val = Math.min(Math.max(intensity, 0), maxVal) / maxVal;
                    const color = new THREE.Color();
                    color.setHSL((1.0 - val) * 0.66, 1.0, 0.5);

                    scanCol[d++] = color.r;
                    scanCol[d++] = color.g;
                    scanCol[d++] = color.b;
                }
            }

            for (let i = c; i < 5000 * 3; i++) scanPos[i] = 99999;
            scanMesh.geometry.attributes.position.needsUpdate = true;
            scanMesh.geometry.attributes.color.needsUpdate = true;

            if (pose) {
                const poseX = pose.x * scaleFactor;
                const poseY = pose.y * scaleFactor;
                const dist = Math.sqrt(Math.pow(poseX - lastKeyframePose.x, 2) + Math.pow(poseY - lastKeyframePose.y, 2));
                if (dist > 0.2) {
                    const mapPos = mapMesh.geometry.attributes.position.array;
                    const mapCol = mapMesh.geometry.attributes.color.array; 

                    for (let k = 0; k < c; k++) {
                        if (mapCursor >= mapMaxPoints * 3) mapCursor = 0;
                        mapPos[mapCursor] = scanPos[k];
                        mapCol[mapCursor] = scanCol[k]; 
                        mapCursor++;
                    }
                    mapMesh.geometry.attributes.position.needsUpdate = true;
                    mapMesh.geometry.attributes.color.needsUpdate = true; 
                    lastKeyframePose = { x: poseX, y: poseY };
                }
            }
        }
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
        renderer.render(scene, camera); 
        updatePopupPosition(); // ✅ 每一帧实时刷新弹窗位置，使其跟随地图拖拽
    }
    animate();

    new ResizeObserver(() => {
        const w = container.clientWidth, h = container.clientHeight;
        if(w&&h) { renderer.setSize(w,h); camera.aspect=w/h; camera.updateProjectionMatrix(); }
    }).observe(container);
})();

})(); // ✅✅✅ [关键修复] 这里加上了最外层的闭包结束括号！
