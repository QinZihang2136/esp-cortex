let lastMagPacket = null; // 缓存最新的磁力计数据
// ================= 配置区 =================
// 【重要】true = 电脑模拟模式; false = 真实连接模式
const IS_SIMULATION = false;
const ESP_IP = window.location.host; // 自动获取当前IP

// ================= 全局变量定义 =================
let ws = null;
let allParams = [];

// 图表数据缓存
let chartData = { labels: [], pitch: [], roll: [], yaw: [] };

// 3D 场景变量
let robotMesh = null;
let renderer = null, scene = null, camera = null, controls = null;

// === 磁力计校准专用变量 ===
let isCapturingMag = false;
let magRawData = []; // 存储采集的原始点 {x,y,z}
let octantFlags = new Array(8).fill(false); // 8个卦限的覆盖标志位

// === IMU (Accel/Gyro) 校准专用变量 ===
let imuCalibData = {
    // 6面校准用的数据池
    sides: { z_up: [], z_down: [], x_up: [], x_down: [], y_up: [], y_down: [] },
    // 陀螺仪静止校准用的数据池
    gyro: []
};
let currentCalibSide = null; // 当前正在采集哪个面 (Accel)
let isCalibratingGyro = false; // 是否正在校准陀螺仪
let imuSidesStatus = { z_up: false, z_down: false, x_up: false, x_down: false, y_up: false, y_down: false };

// 当前仪表盘视图模式 ('chart' 或 '3d')
let currentDashMode = 'chart';
// === 方向校准专用变量 ===
let orientSamples = { level: null, noseUp: null }; // 存储采集的平均向量 {x,y,z}
let orientRawBuffer = []; // 临时缓冲
let isCapturingOrient = false;
let targetOrientStep = null; // 'level' or 'noseUp'

// 标准旋转列表 (参考 ArduPilot/PX4 定义)
// 这里的 index 对应固件里 enum Rotation 的整数值
const ROTATION_LIST = [
    { id: 0, name: "None (默认)", roll: 0, pitch: 0, yaw: 0 },
    { id: 1, name: "Yaw 45°", roll: 0, pitch: 0, yaw: 45 },
    { id: 2, name: "Yaw 90°", roll: 0, pitch: 0, yaw: 90 },
    { id: 3, name: "Yaw 135°", roll: 0, pitch: 0, yaw: 135 },
    { id: 4, name: "Yaw 180°", roll: 0, pitch: 0, yaw: 180 },
    { id: 5, name: "Yaw 225°", roll: 0, pitch: 0, yaw: 225 },
    { id: 6, name: "Yaw 270°", roll: 0, pitch: 0, yaw: 270 },
    { id: 7, name: "Yaw 315°", roll: 0, pitch: 0, yaw: 315 },
    { id: 8, name: "Roll 180°", roll: 180, pitch: 0, yaw: 0 },
    { id: 9, name: "Roll 180°, Yaw 45°", roll: 180, pitch: 0, yaw: 45 },
    { id: 10, name: "Roll 180°, Yaw 90°", roll: 180, pitch: 0, yaw: 90 },
    { id: 11, name: "Roll 180°, Yaw 135°", roll: 180, pitch: 0, yaw: 135 },
    { id: 12, name: "Pitch 180°", roll: 0, pitch: 180, yaw: 0 },
    { id: 13, name: "Roll 180°, Yaw 225°", roll: 180, pitch: 0, yaw: 225 },
    { id: 14, name: "Roll 180°, Yaw 270°", roll: 180, pitch: 0, yaw: 270 },
    { id: 15, name: "Roll 180°, Yaw 315°", roll: 180, pitch: 0, yaw: 315 },
    { id: 16, name: "Roll 90°", roll: 90, pitch: 0, yaw: 0 },
    { id: 17, name: "Roll 90°, Yaw 45°", roll: 90, pitch: 0, yaw: 45 },
    { id: 18, name: "Roll 90°, Yaw 90°", roll: 90, pitch: 0, yaw: 90 },
    { id: 19, name: "Roll 90°, Yaw 135°", roll: 90, pitch: 0, yaw: 135 },
    { id: 20, name: "Roll 270°", roll: 270, pitch: 0, yaw: 0 },
    { id: 21, name: "Roll 270°, Yaw 45°", roll: 270, pitch: 0, yaw: 45 },
    { id: 22, name: "Roll 270°, Yaw 90°", roll: 270, pitch: 0, yaw: 90 },
    { id: 23, name: "Roll 270°, Yaw 135°", roll: 270, pitch: 0, yaw: 135 },
    // 特殊情况：Pitch 90 (Nose Down) 等... 这里仅列举常用组合，完整列表需匹配 C++
    { id: 24, name: "Pitch 90°", roll: 0, pitch: 90, yaw: 0 },
    { id: 25, name: "Pitch 270° (Nose Up)", roll: 0, pitch: 270, yaw: 0 }
];
// ================= 1. 程序入口 =================
window.onload = function () {
    logToTerminal("GUI System Booting...");

    // 初始化各个模块
    initSidebar();
    initChart();
    init3DScene();
    initMagPlot();

    // 默认显示波形图
    switchDashView('chart');

    if (IS_SIMULATION) {
        setConnStatus("模拟模式 (Simulation)", "bg-warning text-dark");
        document.getElementById("sim-badge").style.display = "inline-block";
        logToTerminal("Mode: Simulation enabled (FRD Frame).");
        startSimulationDataLoop();
        allParams = generateMockParams(20);
    } else {
        setConnStatus("正在连接...", "bg-secondary");
        connectWebSocket();
    }
};

// ================= 2. 界面视图切换 =================
window.switchDashView = function (mode) {
    currentDashMode = mode;
    const divChart = document.getElementById("dash-content-chart");
    const div3D = document.getElementById("dash-content-3d");

    if (mode === 'chart') {
        divChart.style.display = 'block';
        div3D.style.display = 'none';
    } else {
        divChart.style.display = 'none';
        div3D.style.display = 'block';
        // 切换到 3D 界面时，必须重置渲染器大小，否则画布会变形
        if (renderer && camera) {
            const w = div3D.clientWidth;
            const h = div3D.clientHeight;
            renderer.setSize(w, h);
            camera.aspect = w / h;
            camera.updateProjectionMatrix();
        }
    }
};

window.showPage = function (pageName, btnElement) {
    document.querySelectorAll(".view-section").forEach(el => el.style.display = 'none');
    document.getElementById("view-" + pageName).style.display = 'block';

    document.querySelectorAll(".list-group-item").forEach(el => el.classList.remove("active"));
    if (btnElement) btnElement.classList.add("active");

    if (pageName === 'dashboard') switchDashView(currentDashMode);
    if (pageName === 'calibration') Plotly.relayout('mag-plot', { autosize: true });

    // [修正点] 这里必须使用 pageName，而不是 pageId
    if (pageName === 'monitor') {
        if (typeof initMonitor === 'function') {
            initMonitor();
        } else {
            console.error("initMonitor function not found!");
        }
    }
};

window.switchCalibTab = function (type) {
    document.querySelectorAll(".calib-content").forEach(el => el.style.display = 'none');
    document.getElementById("tab-content-" + type).style.display = 'block';
    document.querySelectorAll("#calibTab .nav-link").forEach(el => el.classList.remove("active"));
    document.getElementById("tab-btn-" + type).classList.add("active");
    if (type === 'mag') Plotly.relayout('mag-plot', { autosize: true });
};

// ================= 3. 核心数据处理 (路由分发) =================
function handleDataPacket(jsonObj) {
    const type = jsonObj.type;
    const payload = jsonObj.payload;

    // 1. 遥测数据 (Telemetry)
    if (type === "telem") {
        // 更新姿态数值
        if (document.getElementById("val-bat")) document.getElementById("val-bat").innerText = payload.voltage.toFixed(1) + " V";
        if (document.getElementById("val-pitch")) document.getElementById("val-pitch").innerText = payload.pitch.toFixed(1) + "°";
        if (document.getElementById("val-roll")) document.getElementById("val-roll").innerText = payload.roll.toFixed(1) + "°";
        if (document.getElementById("val-yaw")) document.getElementById("val-yaw").innerText = payload.yaw.toFixed(1) + "°";

        // 更新系统状态 (Heap & Time)
        if (payload.sys) {
            if (document.getElementById("val-heap")) {
                document.getElementById("val-heap").innerText = payload.sys.heap.toFixed(1);
            }
            if (document.getElementById("val-uptime")) {
                let totalSec = payload.sys.time;
                let h = Math.floor(totalSec / 3600).toString().padStart(2, '0');
                let m = Math.floor((totalSec % 3600) / 60).toString().padStart(2, '0');
                let s = (totalSec % 60).toString().padStart(2, '0');
                document.getElementById("val-uptime").innerText = `${h}:${m}:${s}`;
            }
        }

        // 更新图表数据
        updateChart(payload);

        // 更新 3D 模型
        if (currentDashMode === '3d') {
            update3DObject(payload.roll, payload.pitch, payload.yaw);
        }

        // --- 磁力计数据处理 ---
        if (payload.mag) {
            lastMagPacket = payload.mag; // [Fix] 缓存最新数据

            // 磁力计校准数据流
            if (isCapturingMag) {
                updateMagCalibrationLogic(payload.mag);
            }

            // [New] 分步方向检测采集 (仅使用 Mag 数据)
            if (isAxisDetecting) {
                collectAxisData(payload.mag);
            }
        }

        // --- IMU (Accel/Gyro) 数据处理 ---
        if (payload.imu) {
            // 陀螺仪采集
            if (isCalibratingGyro) {
                // 保存 {x, y, z} 格式
                imuCalibData.gyro.push({ x: payload.imu.gx, y: payload.imu.gy, z: payload.imu.gz });
            }
            // 加速度计 6面采集
            if (currentCalibSide) {
                // 保存 {x, y, z} 格式
                imuCalibData.sides[currentCalibSide].push({ x: payload.imu.ax, y: payload.imu.ay, z: payload.imu.az });
            }

            // --- 方向自动检测采集逻辑 (IMU部分) ---
            if (isCapturingOrient && targetOrientStep) {
                // 采集 50 个点 (约 0.5~1秒)
                if (orientRawBuffer.length < 50) {
                    orientRawBuffer.push({ x: payload.imu.ax, y: payload.imu.ay, z: payload.imu.az });
                } else {
                    finishOrientStep();
                }
            }

            // [New] 驱动双向验证条 UI
            updateShakeTestUI(payload.imu);

            // [New] 磁力计方向检查逻辑 (使用缓存的 Mag 数据)
            if (isCheckingMag && lastMagPacket) {
                processMagCheck(payload.imu, lastMagPacket);
            }

            // [New] 驱动原始数据监控页面 (Monitor)
            // 这里调用我们新写的更新函数，传入 imu 和 mag
            updateMonitor(payload.imu, payload.mag);
        }
    }
    // 2. 参数列表
    else if (type === "param_list") {
        logToTerminal(`Received ${payload.length} parameters.`);
        allParams = payload; // 更新本地缓存
        renderParamTable(payload);
    }
    // 3. 系统日志
    else if (type === "log") {
        logToTerminal("[ESP] " + payload.msg);
    }
}
// ================= 4. 图表功能 (Chart.js) =================
let mainChart = null;
function initChart() {
    const ctx = document.getElementById('mainChart').getContext('2d');
    mainChart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: [],
            datasets: [
                { label: 'Pitch', data: [], borderColor: '#ff6384', tension: 0.3, pointRadius: 0 },
                { label: 'Roll', data: [], borderColor: '#36a2eb', tension: 0.3, pointRadius: 0 },
                { label: 'Yaw', data: [], borderColor: '#4bc0c0', tension: 0.3, pointRadius: 0 }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: false,
            scales: { x: { display: false }, y: { min: -180, max: 180 } }
        }
    });
}

function updateChart(data) {
    const now = new Date().toLocaleTimeString();
    chartData.labels.push(now);
    chartData.pitch.push(data.pitch);
    chartData.roll.push(data.roll);
    chartData.yaw.push(data.yaw);

    if (chartData.labels.length > 100) {
        chartData.labels.shift();
        chartData.pitch.shift();
        chartData.roll.shift();
        chartData.yaw.shift();
    }

    if (currentDashMode === 'chart') {
        mainChart.data.labels = chartData.labels;
        mainChart.data.datasets[0].data = chartData.pitch;
        mainChart.data.datasets[1].data = chartData.roll;
        mainChart.data.datasets[2].data = chartData.yaw;
        mainChart.update();
    }
}

// ================= 5. 3D 场景 (Three.js + OrbitControls) =================
function init3DScene() {
    const container = document.getElementById('three-container');
    const w = container.clientWidth || 800;
    const h = container.clientHeight || 500;

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf8f9fa);

    const light = new THREE.DirectionalLight(0xffffff, 1);
    light.position.set(2, 5, 2);
    scene.add(light);
    scene.add(new THREE.AmbientLight(0x404040));

    camera = new THREE.PerspectiveCamera(45, w / h, 0.1, 100);
    camera.position.set(3, 3, 3);
    camera.lookAt(0, 0, 0);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(w, h);
    container.appendChild(renderer.domElement);

    // 鼠标交互控制器
    if (THREE.OrbitControls) {
        controls = new THREE.OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.05;
    }

    // 机器人模型
    const geometry = new THREE.BoxGeometry(1.6, 0.2, 1);
    const materials = [
        new THREE.MeshLambertMaterial({ color: 0xcccccc }),
        new THREE.MeshLambertMaterial({ color: 0xcccccc }),
        new THREE.MeshLambertMaterial({ color: 0xeeeeee }),
        new THREE.MeshLambertMaterial({ color: 0x666666 }),
        new THREE.MeshLambertMaterial({ color: 0xff3333 }), // Front (Red)
        new THREE.MeshLambertMaterial({ color: 0xcccccc })
    ];
    robotMesh = new THREE.Mesh(geometry, materials);

    scene.add(new THREE.AxesHelper(3));
    scene.add(robotMesh);
    scene.add(new THREE.GridHelper(10, 10));

    const animate = function () {
        requestAnimationFrame(animate);
        if (controls) controls.update();
        renderer.render(scene, camera);
    };
    animate();
}

function update3DObject(roll, pitch, yaw) {
    if (!robotMesh) return;
    const d2r = Math.PI / 180;
    robotMesh.rotation.x = pitch * d2r;
    robotMesh.rotation.z = -roll * d2r;
    robotMesh.rotation.y = -yaw * d2r;
}

// ================= 6. 磁力计智能校准 (Smart Calibration) =================

function initMagPlot() {
    const layout = {
        margin: { t: 0, b: 0, l: 0, r: 0 },
        scene: { aspectmode: 'cube' },
        paper_bgcolor: 'rgba(0,0,0,0)',
        plot_bgcolor: 'rgba(0,0,0,0)'
    };
    Plotly.newPlot('mag-plot', [{
        type: 'scatter3d',
        mode: 'markers',
        x: [], y: [], z: [],
        marker: { size: 3, color: '#1f77b4' }
    }], layout, { responsive: true });
}

window.toggleMagCapture = function () {
    const btn = document.getElementById("btn-mag-start");

    if (!isCapturingMag) {
        isCapturingMag = true;
        magRawData = [];
        octantFlags.fill(false);
        updateOctantUI();
        document.getElementById("mag-progress-bar").style.width = "0%";
        document.getElementById("mag-result-box").style.display = "none";
        document.getElementById("btn-mag-calc").disabled = true;
        document.getElementById("btn-mag-calc").classList.replace("btn-success", "btn-secondary");

        btn.innerHTML = '<i class="fas fa-stop me-2"></i>停止采集';
        btn.classList.replace("btn-outline-primary", "btn-danger");
        Plotly.restyle('mag-plot', { x: [[]], y: [[]], z: [[]] });

        const statusText = document.getElementById("mag-status-text");
        statusText.innerText = "正在采集... 请全方位旋转设备，点亮所有方块。";
        statusText.className = "text-primary fw-bold";

        logToTerminal("Mag Capture Started. Resetting ESP32 params to raw mode...");
        if (!IS_SIMULATION) {
            sendJson({ cmd: "set_param", key: "MAG_OFFSET_X", val: 0 });
            sendJson({ cmd: "set_param", key: "MAG_OFFSET_Y", val: 0 });
            sendJson({ cmd: "set_param", key: "MAG_OFFSET_Z", val: 0 });
            sendJson({ cmd: "set_param", key: "MAG_SCALE_X", val: 1 });
            sendJson({ cmd: "set_param", key: "MAG_SCALE_Y", val: 1 });
            sendJson({ cmd: "set_param", key: "MAG_SCALE_Z", val: 1 });
        }
    } else {
        isCapturingMag = false;
        btn.innerHTML = '<i class="fas fa-play me-2"></i>重新采集';
        btn.classList.replace("btn-danger", "btn-outline-primary");
        const statusText = document.getElementById("mag-status-text");
        statusText.innerText = "采集暂停。";
        statusText.className = "text-muted";
    }
};

function updateMagCalibrationLogic(magData) {
    if (!isCapturingMag) return;
    magRawData.push(magData);
    if (magRawData.length > 5000) magRawData.shift();
    document.getElementById('mag-points-count').innerText = magRawData.length;

    if (magRawData.length % 5 === 0) {
        const xArr = magRawData.map(p => p.x);
        const yArr = magRawData.map(p => p.y);
        const zArr = magRawData.map(p => p.z);
        Plotly.restyle('mag-plot', { x: [xArr], y: [yArr], z: [zArr] });
        checkOctantCoverage(xArr, yArr, zArr);
    }
}

function checkOctantCoverage(xArr, yArr, zArr) {
    let sumX = 0, sumY = 0, sumZ = 0;
    const len = xArr.length;
    for (let i = 0; i < len; i++) { sumX += xArr[i]; sumY += yArr[i]; sumZ += zArr[i]; }
    const cx = sumX / len;
    const cy = sumY / len;
    const cz = sumZ / len;

    const lastIdx = len - 1;
    const dx = xArr[lastIdx] - cx;
    const dy = yArr[lastIdx] - cy;
    const dz = zArr[lastIdx] - cz;

    let octantIdx = 0;
    if (dx > 0) octantIdx |= 4;
    if (dy > 0) octantIdx |= 2;
    if (dz > 0) octantIdx |= 1;

    if (!octantFlags[octantIdx]) {
        octantFlags[octantIdx] = true;
        updateOctantUI();
    }

    const coveredCount = octantFlags.filter(Boolean).length;
    const progress = (coveredCount / 8) * 100;
    document.getElementById("mag-progress-bar").style.width = progress + "%";

    if (coveredCount === 8 && len > 300) {
        const calcBtn = document.getElementById("btn-mag-calc");
        calcBtn.disabled = false;
        calcBtn.classList.replace("btn-secondary", "btn-success");
        calcBtn.innerHTML = '<i class="fas fa-check-circle me-2"></i>数据就绪，点击计算';
    }
}

function updateOctantUI() {
    for (let i = 0; i < 8; i++) {
        const el = document.getElementById(`oct-${i}`);
        if (octantFlags[i]) el.classList.add("active");
        else el.classList.remove("active");
    }
}

window.calcMagCalibration = function () {
    if (magRawData.length < 100) { alert("数据点不足"); return; }
    logToTerminal(`Calculating calibration with ${magRawData.length} raw points...`);

    let sumX = 0, sumY = 0, sumZ = 0;
    magRawData.forEach(p => { sumX += p.x; sumY += p.y; sumZ += p.z; });
    const tempCx = sumX / magRawData.length;
    const tempCy = sumY / magRawData.length;
    const tempCz = sumZ / magRawData.length;

    const radii = magRawData.map(p => {
        return Math.sqrt(Math.pow(p.x - tempCx, 2) + Math.pow(p.y - tempCy, 2) + Math.pow(p.z - tempCz, 2));
    });
    const meanR = radii.reduce((a, b) => a + b, 0) / radii.length;
    const variance = radii.reduce((a, b) => a + Math.pow(b - meanR, 2), 0) / radii.length;
    const stdDev = Math.sqrt(variance);

    const threshold = 2.5 * stdDev;
    const cleanData = magRawData.filter((p, i) => Math.abs(radii[i] - meanR) < threshold);
    const removedCount = magRawData.length - cleanData.length;
    logToTerminal(`Filter: Removed ${removedCount} noise points.`);

    if (cleanData.length < 100) { alert("去噪后数据不足"); return; }

    let minX = Infinity, maxX = -Infinity;
    let minY = Infinity, maxY = -Infinity;
    let minZ = Infinity, maxZ = -Infinity;

    cleanData.forEach(p => {
        if (p.x < minX) minX = p.x; if (p.x > maxX) maxX = p.x;
        if (p.y < minY) minY = p.y; if (p.y > maxY) maxY = p.y;
        if (p.z < minZ) minZ = p.z; if (p.z > maxZ) maxZ = p.z;
    });

    const offX = (maxX + minX) / 2;
    const offY = (maxY + minY) / 2;
    const offZ = (maxZ + minZ) / 2;

    const chordX = maxX - minX;
    const chordY = maxY - minY;
    const chordZ = maxZ - minZ;

    if (chordX === 0 || chordY === 0 || chordZ === 0) { alert("数据范围错误"); return; }

    const avgChord = (chordX + chordY + chordZ) / 3;
    const scaleX = avgChord / chordX;
    const scaleY = avgChord / chordY;
    const scaleZ = avgChord / chordZ;

    document.getElementById("mag-result-box").style.display = "block";
    document.getElementById("res-offset").innerText = `(${offX.toFixed(1)}, ${offY.toFixed(1)}, ${offZ.toFixed(1)})`;
    document.getElementById("res-scale").innerText = `(${scaleX.toFixed(2)}, ${scaleY.toFixed(2)}, ${scaleZ.toFixed(2)})`;
    document.getElementById("res-err").innerText = `Clean: ${cleanData.length}/${magRawData.length}`;

    if (confirm(`计算成功！是否写入 Flash？\nOffset: ${offX.toFixed(1)}, ...\nScale: ${scaleX.toFixed(2)}, ...`)) {
        if (!IS_SIMULATION) {
            sendJson({ cmd: "set_param", key: "MAG_OFFSET_X", val: offX });
            sendJson({ cmd: "set_param", key: "MAG_OFFSET_Y", val: offY });
            sendJson({ cmd: "set_param", key: "MAG_OFFSET_Z", val: offZ });
            sendJson({ cmd: "set_param", key: "MAG_SCALE_X", val: scaleX });
            sendJson({ cmd: "set_param", key: "MAG_SCALE_Y", val: scaleY });
            sendJson({ cmd: "set_param", key: "MAG_SCALE_Z", val: scaleZ });
        }
        logToTerminal("Mag parameters sent.");
        alert("参数已写入！");
        if (isCapturingMag) toggleMagCapture();
    }
};

// ================= 7. IMU 校准 (智能版：FRD 适配) =================

// --- 7.1 陀螺仪静止校准 ---
window.startGyroCalibration = function () {
    if (isCalibratingGyro) return;

    if (!IS_SIMULATION) {
        // 先重置参数，保证采集到原始数据
        sendJson({ cmd: "set_param", key: "GYRO_OFFSET_X", val: 0 });
        sendJson({ cmd: "set_param", key: "GYRO_OFFSET_Y", val: 0 });
        sendJson({ cmd: "set_param", key: "GYRO_OFFSET_Z", val: 0 });
    }

    const btn = document.querySelector("button[onclick*='calib_imu_level']") || document.getElementById("btn-gyro-start");

    let originalHtml = "";
    if (btn) {
        originalHtml = btn.innerHTML;
        btn.disabled = true;
        btn.innerHTML = '<i class="fas fa-spinner fa-spin me-2"></i>静止采集中 (3s)...';
        btn.classList.add('disabled');
    }

    imuCalibData.gyro = [];
    isCalibratingGyro = true;
    logToTerminal("Gyro Calib: Keep device ABSOLUTELY STILL!");

    setTimeout(() => {
        isCalibratingGyro = false;
        finishGyroCalibration();
        if (btn) {
            btn.disabled = false;
            btn.innerHTML = originalHtml || '<i class="fas fa-level-down-alt me-2"></i>水平校准';
            btn.classList.remove('disabled');
        }
    }, 3000);
};

function finishGyroCalibration() {
    const data = imuCalibData.gyro;
    if (data.length < 20) { alert("采集点数不足，请重试"); return; }

    const statsX = getArrayStats(data.map(p => p.x));
    const statsY = getArrayStats(data.map(p => p.y));
    const statsZ = getArrayStats(data.map(p => p.z));

    const maxStd = Math.max(statsX.std, statsY.std, statsZ.std);
    // 这里取 0.1 作为防抖阈值
    if (maxStd > 0.1) {
        alert(`校准失败：检测到晃动 (StdDev: ${maxStd.toFixed(3)})！\n请将设备固定在桌面上重试。`);
        return;
    }

    const offX = statsX.mean;
    const offY = statsY.mean;
    const offZ = statsZ.mean;

    if (confirm(`静止检测通过。\n陀螺仪零偏:\nX: ${offX.toFixed(4)}\nY: ${offY.toFixed(4)}\nZ: ${offZ.toFixed(4)}\n\n是否写入?`)) {
        if (!IS_SIMULATION) {
            sendJson({ cmd: "set_param", key: "GYRO_OFFSET_X", val: offX });
            sendJson({ cmd: "set_param", key: "GYRO_OFFSET_Y", val: offY });
            sendJson({ cmd: "set_param", key: "GYRO_OFFSET_Z", val: offZ });
        }
        logToTerminal("Gyro parameters updated.");
        alert("陀螺仪校准成功！");
    }
}

// --- 7.2 加速度计 6面校准 (带防抖与方向检查 - FRD 适配) ---

window.captureImuSide = function (sideName) {
    const btnId = `btn-side-${sideName.replace('_', '-')}`;
    const btn = document.getElementById(btnId);

    if (!btn.dataset.origHtml) btn.dataset.origHtml = btn.innerHTML;

    btn.className = "btn btn-info w-100 py-3 text-white shadow-sm imu-side-btn";
    btn.innerHTML = `<div class="spinner-border spinner-border-sm mb-2"></div><br>采集中...<br><span class="small">请保持静止</span>`;
    btn.disabled = true;

    imuCalibData.sides[sideName] = [];
    currentCalibSide = sideName; // 开闸

    logToTerminal(`Start capturing Accel side: ${sideName}`);

    if (IS_SIMULATION) window.simTargetSide = sideName;

    setTimeout(() => {
        finishImuSideCapture(sideName, btn);
    }, 2500);
};

function finishImuSideCapture(sideName, btnElement) {
    currentCalibSide = null; // 关闸
    if (IS_SIMULATION) window.simTargetSide = null;

    const data = imuCalibData.sides[sideName];

    if (data.length < 20) {
        alert("数据点不足，请重试！");
        resetSideBtn(btnElement);
        return;
    }

    const sX = getArrayStats(data.map(p => p.x));
    const sY = getArrayStats(data.map(p => p.y));
    const sZ = getArrayStats(data.map(p => p.z));

    const maxStd = Math.max(sX.std, sY.std, sZ.std);
    if (maxStd > 0.1) {
        alert(`采集失败：检测到晃动！\n标准差: ${maxStd.toFixed(3)}\n请将设备放置在稳固平面上。`);
        resetSideBtn(btnElement);
        return;
    }

    // 方向检测 (核心修正逻辑)
    if (!checkOrientationValid(sideName, sX.mean, sY.mean, sZ.mean)) {
        resetSideBtn(btnElement);
        return;
    }

    logToTerminal(`Side ${sideName} OK. Avg: [${sX.mean.toFixed(2)}, ${sY.mean.toFixed(2)}, ${sZ.mean.toFixed(2)}]`);
    markSideAsDone(sideName);
}

function resetSideBtn(btn) {
    btn.className = "btn btn-outline-secondary w-100 py-3 imu-side-btn";
    btn.innerHTML = btn.dataset.origHtml || "重试";
    btn.disabled = false;
}

function getArrayStats(arr) {
    const n = arr.length;
    if (n === 0) return { mean: 0, std: 0 };
    const mean = arr.reduce((a, b) => a + b, 0) / n;
    const variance = arr.reduce((a, b) => a + Math.pow(b - mean, 2), 0) / n;
    return { mean: mean, std: Math.sqrt(variance) };
}

// [修正] 适配 FRD 坐标系的方向检查
function checkOrientationValid(side, x, y, z) {
    const G_TARGET = 1.0;
    const TOLERANCE = 0.4;

    let valid = false;
    let errorMsg = "姿态不符";

    // FRD 定义: Z轴向下为正，X轴向前为正，Y轴向右为正
    // 加速度计测量的是比力 (Support Force)，静止时方向向上(对抗重力)
    // 1. 平放 (Body Z Down): Support Force Up -> 指向 Body -Z -> 读数 -1g
    // 2. 倒扣 (Body Z Up):   Support Force Up -> 指向 Body +Z -> 读数 +1g
    // 3. 抬头 (Body X Up):   Support Force Up -> 指向 Body +X -> 读数 +1g
    // 4. 低头 (Body X Down): Support Force Up -> 指向 Body -X -> 读数 -1g
    // 5. 右侧下 (Body Y Down): Support Force Up -> 指向 Body -Y -> 读数 -1g
    // 6. 左侧下 (Body Y Up):   Support Force Up -> 指向 Body +Y -> 读数 +1g

    switch (side) {
        case 'z_down': // 平放
            if (z < -G_TARGET + TOLERANCE) valid = true; // Expect Z ≈ -1
            else errorMsg = `期望 Z轴 ≈ -1.0 (平放)，实际 Z=${z.toFixed(2)}。请平放设备！`; break;
        case 'z_up':   // 倒扣
            if (z > G_TARGET - TOLERANCE) valid = true;  // Expect Z ≈ +1
            else errorMsg = `期望 Z轴 ≈ +1.0 (倒扣)，实际 Z=${z.toFixed(2)}。请倒扣设备！`; break;

        case 'x_up':   // 机头朝上
            if (x > G_TARGET - TOLERANCE) valid = true;  // Expect X ≈ +1
            else errorMsg = `期望 X轴 ≈ +1.0 (机头朝上)，实际 X=${x.toFixed(2)}。请机头朝天！`; break;
        case 'x_down': // 机头朝下
            if (x < -G_TARGET + TOLERANCE) valid = true; // Expect X ≈ -1
            else errorMsg = `期望 X轴 ≈ -1.0 (机头朝下)，实际 X=${x.toFixed(2)}。请机头朝地！`; break;

        case 'y_up':   // 左侧朝下 (左翼触地，Y轴指向天)
            if (y > G_TARGET - TOLERANCE) valid = true;  // Expect Y ≈ +1
            else errorMsg = `期望 Y轴 ≈ +1.0 (左侧朝下)，实际 Y=${y.toFixed(2)}。请左侧朝下！`; break;
        case 'y_down': // 右侧朝下 (右翼触地，Y轴指向地)
            if (y < -G_TARGET + TOLERANCE) valid = true; // Expect Y ≈ -1
            else errorMsg = `期望 Y轴 ≈ -1.0 (右侧朝下)，实际 Y=${y.toFixed(2)}。请右侧朝下！`; break;
    }

    if (!valid && !IS_SIMULATION) {
        alert(`校准失败：${errorMsg}`);
        return false;
    }
    return true;
}

function markSideAsDone(sideName) {
    imuSidesStatus[sideName] = true;
    const btnId = `btn-side-${sideName.replace('_', '-')}`;
    const btn = document.getElementById(btnId);

    btn.className = "btn btn-success w-100 py-3 shadow-sm imu-side-btn";
    btn.innerHTML = `<i class="fas fa-check-circle fs-2 mb-1"></i><br>已完成 (Done)`;
    btn.disabled = false;

    checkImuAllDone();
}

function checkImuAllDone() {
    const allDone = Object.values(imuSidesStatus).every(status => status === true);
    const calcBtn = document.getElementById("btn-imu-calc");

    if (allDone) {
        calcBtn.disabled = false;
        calcBtn.classList.replace("btn-secondary", "btn-primary");
        calcBtn.innerHTML = `<i class="fas fa-microchip me-2"></i>6面数据就绪，点击计算并保存`;
    }
}

window.finishImuCalibration = function () {
    if (!confirm("确认根据采集的6面数据计算并写入参数吗？")) return;

    const getAvg = (side) => {
        const d = imuCalibData.sides[side];
        let x = 0, y = 0, z = 0;
        d.forEach(p => { x += p.x; y += p.y; z += p.z; });
        return { x: x / d.length, y: y / d.length, z: z / d.length };
    };

    const z_up_inv = getAvg('z_up');     // 倒扣 (+1g)
    const z_down_nml = getAvg('z_down'); // 平放 (-1g)

    const x_up_nose = getAvg('x_up');    // 头上 (+1g)
    const x_down_nose = getAvg('x_down');// 头下 (-1g)

    const y_up_left = getAvg('y_up');    // 左下 (+1g)
    const y_down_right = getAvg('y_down');// 右下 (-1g)

    // Min-Max 算法修正 (FRD)
    // Offset = (Max + Min) / 2
    // Scale = 1.0 / ((Max - Min) / 2)

    // Z Axis: Max=Inv(+1), Min=Norm(-1)
    const offZ = (z_up_inv.z + z_down_nml.z) / 2;
    const scaZ = 1.0 / ((z_up_inv.z - z_down_nml.z) / 2);

    // X Axis: Max=Up(+1), Min=Down(-1)
    const offX = (x_up_nose.x + x_down_nose.x) / 2;
    const scaX = 1.0 / ((x_up_nose.x - x_down_nose.x) / 2);

    // Y Axis: Max=Left(+1), Min=Right(-1)
    const offY = (y_up_left.y + y_down_right.y) / 2;
    const scaY = 1.0 / ((y_up_left.y - y_down_right.y) / 2);

    const msg = `计算结果 (FRD):\n
    Offset: (${offX.toFixed(3)}, ${offY.toFixed(3)}, ${offZ.toFixed(3)})
    Scale : (${scaX.toFixed(3)}, ${scaY.toFixed(3)}, ${scaZ.toFixed(3)})
    
    是否写入?`;

    if (confirm(msg)) {
        if (!IS_SIMULATION) {
            sendJson({ cmd: "set_param", key: "ACCEL_OFFSET_X", val: offX });
            sendJson({ cmd: "set_param", key: "ACCEL_OFFSET_Y", val: offY });
            sendJson({ cmd: "set_param", key: "ACCEL_OFFSET_Z", val: offZ });

            sendJson({ cmd: "set_param", key: "ACCEL_SCALE_X", val: scaX });
            sendJson({ cmd: "set_param", key: "ACCEL_SCALE_Y", val: scaY });
            sendJson({ cmd: "set_param", key: "ACCEL_SCALE_Z", val: scaZ });
        }
        logToTerminal("Accel parameters updated.");
        alert("参数已写入！");
    }
};

// ================= 8. 参数管理 (通用) =================
window.requestAllParams = function () {
    logToTerminal("Requesting parameter list...");
    const tbody = document.getElementById("param-table-body");
    tbody.innerHTML = '<tr><td colspan="4" class="text-center text-primary"><i class="fas fa-spinner fa-spin me-2"></i>Loading...</td></tr>';

    if (IS_SIMULATION) {
        setTimeout(() => renderParamTable(allParams), 500);
    } else {
        sendJson({ cmd: "get_params" });
    }
};

function renderParamTable(params) {
    const tbody = document.getElementById("param-table-body");
    tbody.innerHTML = "";
    document.getElementById("param-count-label").innerText = `共 ${params.length} 个参数`;

    params.sort((a, b) => a.name.localeCompare(b.name));

    params.forEach(p => {
        const tr = document.createElement("tr");
        const typeBadge = p.type === 1
            ? '<span class="badge bg-info text-dark">FLOAT</span>'
            : '<span class="badge bg-secondary">INT32</span>';
        const step = p.type === 1 ? "0.01" : "1";
        const safeVal = (p.val !== undefined && p.val !== null) ? p.val : 0;

        tr.innerHTML = `
            <td class="ps-4 fw-bold text-primary font-monospace">${p.name}</td>
            <td>${typeBadge}</td>
            <td>
                <input type="number" class="form-control form-control-sm" style="max-width: 150px;"
                       id="input-${p.name}" value="${safeVal}" step="${step}">
            </td>
            <td class="text-center">
                <button class="btn btn-sm btn-outline-success border-0" onclick="saveSingleParam('${p.name}')">
                    <i class="fas fa-save fs-5"></i>
                </button>
            </td>
        `;
        tbody.appendChild(tr);
    });
}

window.filterParams = function () {
    const keyword = document.getElementById("param-search").value.toLowerCase();
    const rows = document.getElementById("param-table-body").getElementsByTagName("tr");
    for (let row of rows) {
        if (row.cells.length > 0) {
            const keyText = row.cells[0].innerText.toLowerCase();
            row.style.display = keyText.includes(keyword) ? "" : "none";
        }
    }
};

window.saveSingleParam = function (key) {
    const input = document.getElementById(`input-${key}`);
    const newVal = parseFloat(input.value);
    logToTerminal(`Set ${key} = ${newVal}`);
    if (IS_SIMULATION) {
        const p = allParams.find(x => x.name === key);
        if (p) p.val = newVal;
    } else {
        sendJson({ cmd: "set_param", key: key, val: newVal });
    }
};

// ================= 9. 辅助工具与模拟器 =================
function logToTerminal(msg) {
    const box = document.getElementById("terminal-box");
    const time = new Date().toLocaleTimeString();
    box.innerHTML += `<div><span class="text-secondary">[${time}]</span> ${msg}</div>`;
    box.scrollTop = box.scrollHeight;
}

function setConnStatus(text, bgClass) {
    const el = document.getElementById("connection-status");
    el.innerText = text;
    el.className = `badge ${bgClass}`;
}

function initSidebar() {
    const el = document.getElementById("wrapper");
    const toggleButton = document.getElementById("menu-toggle");
    if (toggleButton) toggleButton.onclick = function () { el.classList.toggle("toggled"); };
}

function connectWebSocket() {
    const url = `ws://${ESP_IP}/ws`;
    logToTerminal("Connecting to " + url);
    ws = new WebSocket(url);

    ws.onopen = () => {
        setConnStatus("在线 (Online)", "bg-success");
        logToTerminal("WebSocket Connected");

        // 【新增】连接建立后，立即发送一条指令来“激活”链路
        // 同时这也顺便解决了“网页刚打开时参数表是空的”这个问题
        sendJson({ cmd: "get_params" });
    };

    ws.onclose = () => {
        setConnStatus("断开 (Offline)", "bg-danger");
        logToTerminal("WebSocket Closed");

        // 可选：断开后尝试 3秒后 自动重连
        setTimeout(connectWebSocket, 3000);
    };

    ws.onmessage = (e) => {
        try { handleDataPacket(JSON.parse(e.data)); }
        catch (err) { console.error(err); }
    };
}

function sendJson(obj) {
    if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
}

// 简单的模拟器
function generateMockParams(count) {
    const arr = [];
    const groups = ["PID_PITCH", "MAG_OFFSET", "SYS_CONF"];
    for (let i = 0; i < count; i++) {
        arr.push({ name: `${groups[i % 3]}_${i}`, type: 1, val: Math.random() * 10 });
    }
    return arr;
}

window.simTargetSide = null; // 模拟器目标面

function startSimulationDataLoop() {
    let t = 0;
    setInterval(() => {
        t += 0.05;

        // 模拟 IMU 数据 (默认平放)
        // FRD: 平放时 Z = -1
        let simAccel = { x: 0, y: 0, z: -1 };

        // 如果正在校准某个面，生成对应面的数据 (符合 FRD 规范)
        if (window.simTargetSide) {
            switch (window.simTargetSide) {
                case 'z_down': simAccel = { x: 0, y: 0, z: -1 }; break; // 平放
                case 'z_up': simAccel = { x: 0, y: 0, z: 1 }; break; // 倒扣
                case 'x_up': simAccel = { x: 1, y: 0, z: 0 }; break; // 抬头
                case 'x_down': simAccel = { x: -1, y: 0, z: 0 }; break; // 低头
                case 'y_up': simAccel = { x: 0, y: 1, z: 0 }; break; // 左侧下 (Y朝天)
                case 'y_down': simAccel = { x: 0, y: -1, z: 0 }; break; // 右侧下 (Y朝地)
            }
            // 模拟随机抖动
            simAccel.x += (Math.random() - 0.5) * 0.02;
            simAccel.y += (Math.random() - 0.5) * 0.02;
            simAccel.z += (Math.random() - 0.5) * 0.02;
        }

        handleDataPacket({
            type: "telem",
            payload: {
                voltage: 12.0 + Math.random() * 0.1,
                pitch: Math.sin(t) * 20, roll: Math.cos(t) * 15, yaw: t * 10 % 360,
                sys: { heap: 200 + Math.sin(t) * 10, time: Math.floor(t * 100) },
                mag: {
                    x: Math.sin(t) * 300 + Math.random() * 20,
                    y: Math.cos(t) * 300 + Math.random() * 20,
                    z: Math.random() * 100
                },
                imu: {
                    // 陀螺仪零偏
                    gx: (Math.random() - 0.5) * 0.01,
                    gy: (Math.random() - 0.5) * 0.01,
                    gz: (Math.random() - 0.5) * 0.01,
                    // 加速度计
                    ax: simAccel.x, ay: simAccel.y, az: simAccel.z
                }
            }
        });
    }, 50);
}
// ================= 10. 安装方向自动检测逻辑 =================

window.captureOrientStep = function (step) {
    if (isCapturingOrient) return;

    // UI 更新
    const btn = document.getElementById(step === 'level' ? 'btn-orient-level' : 'btn-orient-up');
    const originalText = btn.innerHTML;
    btn.innerHTML = '<div class="spinner-border spinner-border-sm"></div> 采集中...';
    btn.disabled = true;

    // 重置状态
    orientRawBuffer = [];
    targetOrientStep = step;
    isCapturingOrient = true;

    // 3秒超时保护 (防止 WebSocket 断连卡死)
    setTimeout(() => {
        if (isCapturingOrient) {
            isCapturingOrient = false;
            btn.innerHTML = originalText;
            btn.disabled = false;
            alert("采集超时，请检查连接");
        }
    }, 3000);
};

function finishOrientStep() {
    isCapturingOrient = false;

    // 计算均值
    let sum = { x: 0, y: 0, z: 0 };
    orientRawBuffer.forEach(p => { sum.x += p.x; sum.y += p.y; sum.z += p.z; });
    const avg = {
        x: sum.x / orientRawBuffer.length,
        y: sum.y / orientRawBuffer.length,
        z: sum.z / orientRawBuffer.length
    };

    // 保存结果
    orientSamples[targetOrientStep] = avg;

    // UI 更新
    const step = targetOrientStep;
    const btn = document.getElementById(step === 'level' ? 'btn-orient-level' : 'btn-orient-up');
    btn.innerHTML = '<i class="fas fa-check"></i> 重新采集';
    btn.disabled = false;
    btn.classList.replace('btn-outline-primary', 'btn-success');
    btn.classList.replace('btn-outline-info', 'btn-success');

    document.getElementById(step === 'level' ? 'res-orient-level' : 'res-orient-up').style.display = 'block';

    // 检查是否两个都完成了
    if (orientSamples.level && orientSamples.noseUp) {
        const calcBtn = document.getElementById('btn-orient-calc');
        calcBtn.disabled = false;
        calcBtn.classList.replace('btn-secondary', 'btn-primary');
    }

    targetOrientStep = null;
}

// ================= 修复后的 calcOrientation 函数 =================
window.calcOrientation = function () {
    if (!orientSamples.level || !orientSamples.noseUp) return;

    // 1. 定义目标向量 (标准 Body Frame FRD)
    // Level: Z轴向下 (-1g)，因为加速度计测量的是支持力(向上)，所以在 FRD 里指向 -Z => [0, 0, -1]
    const target_level = new THREE.Vector3(0, 0, -1);
    // NoseUp: 机头向上，支持力向上(沿机头方向)，在 FRD 里指向 +X => [1, 0, 0]
    const target_noseup = new THREE.Vector3(1, 0, 0);

    let bestIdx = -1;
    let minErr = Infinity;

    // 辅助函数：旋转并归一化向量
    const getRotatedNormVec = (vRaw, roll, pitch, yaw) => {
        // A. 先转为 Three.js 向量
        const vec = new THREE.Vector3(vRaw.x, vRaw.y, vRaw.z);

        // B. 【关键修复】必须归一化！把 9.8 变成 1.0
        vec.normalize();

        // C. 应用旋转
        const d2r = Math.PI / 180;
        const euler = new THREE.Euler(roll * d2r, pitch * d2r, yaw * d2r, 'ZYX');
        vec.applyEuler(euler);

        return vec;
    };

    ROTATION_LIST.forEach(rot => {
        // 2. 将采集到的原始数据，尝试旋转到标准 FRD 坐标系
        const r_level = getRotatedNormVec(orientSamples.level, rot.roll, rot.pitch, rot.yaw);
        const r_noseup = getRotatedNormVec(orientSamples.noseUp, rot.roll, rot.pitch, rot.yaw);

        // 3. 计算误差 (向量距离)
        // 现在的比较是公平的：都是单位向量
        const err1 = r_level.distanceTo(target_level);
        const err2 = r_noseup.distanceTo(target_noseup);

        const totalErr = err1 + err2;

        // debug: 打印一下看看误差分布 (可选)
        // console.log(`Rot ${rot.id} (${rot.name}): Err=${totalErr.toFixed(3)}`);

        if (totalErr < minErr) {
            minErr = totalErr;
            bestIdx = rot.id;
        }
    });

    // 4. 判定阈值
    // 归一化后，两个单位向量如果偏差 5度，距离约为 0.08。
    // 两个动作加起来，误差 < 0.5 是非常宽松的（允许约 15-20 度的总偏差）。
    // 如果想要更宽松，可以改到 0.8
    if (bestIdx !== -1 && minErr < 0.6) {
        const match = ROTATION_LIST.find(r => r.id === bestIdx);
        const msg = `检测结果：\n\nID: ${bestIdx}\n名称: ${match.name}\n(误差: ${minErr.toFixed(3)})\n\n是否应用此方向设置？\n注意：应用后请重新校准加速度计。`;

        if (confirm(msg)) {
            sendJson({ cmd: "set_param", key: "SENS_BOARD_ROT", val: bestIdx });
            alert("参数已发送！请手动重启或在控制台确认生效。");

            // 更新 UI
            document.getElementById('orient-current-val').innerText = match.name;
            const icon = document.getElementById('orient-icon-preview');
            icon.style.transform = `rotate(${match.yaw}deg)`;
        }
    } else {
        alert(`检测失败！未能匹配标准方向 (最小误差: ${minErr.toFixed(2)})\n\n可能原因：\n1. 采集时未保持静止\n2. 摆放姿态偏差过大\n3. 传感器本身有巨大零偏 (请先做一次粗略的 IMU 校准)`);
    }
};
// ================= 11. 摇晃验证 UI 驱动 (双向版) =================
function updateShakeTestUI(imu) {
    const box = document.getElementById('shakeTestBox');
    if (!box || !box.classList.contains('show')) return;

    // 设定满量程 (rad/s), 比如 200度/秒 ≈ 3.5 rad/s
    const MAX_VAL = 3.5;

    // 辅助函数：更新双向条
    const setBiBar = (axis, val) => {
        // 限制范围 -MAX ~ +MAX
        let v = Math.max(-MAX_VAL, Math.min(MAX_VAL, val));

        // 计算百分比 (0~100)
        let pct = (Math.abs(v) / MAX_VAL) * 100;

        const elNeg = document.getElementById(`bar-shake-${axis}-neg`);
        const elPos = document.getElementById(`bar-shake-${axis}-pos`);

        if (v < 0) {
            // 负值：左条增长，右条为0
            elNeg.style.width = pct + '%';
            elPos.style.width = '0%';
        } else {
            // 正值：右条增长，左条为0
            elNeg.style.width = '0%';
            elPos.style.width = pct + '%';
        }
    };

    // 应用到三个轴 (注意：这里的 imu.gx/gy/gz 已经是固件 FRD 转换后的)
    setBiBar('x', imu.gx);
    setBiBar('y', imu.gy);
    setBiBar('z', imu.gz);
}

// ================= 12. 磁力计方向检查逻辑 (完整修正版) =================

let isCheckingMag = false;
let magCheckData = {
    gyroInteg: 0, // 陀螺仪积分角度 (Yaw)
    startMagHeading: 0,
    lastTime: 0
};

// 开始检查按钮点击事件
window.startMagCheck = function () {
    const btn = document.getElementById('btn-mag-check');
    if (isCheckingMag) {
        // 停止并结算
        finishMagCheck();
        return;
    }

    // 初始化状态
    isCheckingMag = true;
    magCheckData.gyroInteg = 0;
    magCheckData.lastTime = Date.now();
    magCheckData.startMagHeading = null; // 等待第一帧数据

    // 更新按钮状态
    btn.innerHTML = '<i class="fas fa-stop me-2"></i>停止并分析';
    btn.classList.replace('btn-outline-dark', 'btn-danger');

    // 更新提示信息
    document.getElementById('mag-check-result').innerHTML = "正在重置...";
    document.getElementById('mag-orient-status').className = "badge bg-warning text-dark";
    document.getElementById('mag-orient-status').innerText = "检测中";

    // 重置进度条到中间 (0)
    updateMagCheckUI(0, 0);

    // 稍后提示操作
    setTimeout(() => {
        if (isCheckingMag) document.getElementById('mag-check-result').innerText = "请水平左右晃动机身...";
    }, 500);
};

// 结束检查并输出结论
function finishMagCheck() {
    isCheckingMag = false;
    const btn = document.getElementById('btn-mag-check');
    btn.innerHTML = '<i class="fas fa-sync me-2"></i>重新检查';
    btn.classList.replace('btn-danger', 'btn-outline-dark');

    // 最终判定逻辑
    const resultBox = document.getElementById('mag-check-result');

    // 根据最后的状态文字来决定弹窗内容
    if (resultBox.innerHTML.includes("方向一致")) {
        alert("✅ 检测通过！\n\n磁力计旋转方向与 IMU 一致。\n您可以放心进行 8 卦限校准。");
    } else if (resultBox.innerHTML.includes("方向相反")) {
        alert("❌ 检测失败：方向相反！\n\n磁力计的 Y 轴可能反了，或者安装面倒置。\n请检查 'Board Rotation' 或 'Mag Rotation' 参数。");
    } else if (resultBox.innerHTML.includes("不足")) {
        alert("⚠️ 数据不足\n\n旋转幅度太小，请至少左右转动 30 度以上。");
    } else {
        alert("⚠️ 检测存疑：轴向不匹配！\n\n可能是 90 度偏差。\n请尝试修改磁力计方向参数。");
    }
}

// 核心处理函数 (在 handleDataPacket 中调用)
function processMagCheck(imu, mag) {
    if (!isCheckingMag) return;

    const now = Date.now();
    const dt = (now - magCheckData.lastTime) / 1000;
    magCheckData.lastTime = now;

    if (dt > 1.0) return; // 丢包或卡顿太大，忽略此帧积分

    // 1. 陀螺仪积分 (Yaw)
    // imu.gz 单位是 rad/s，积分得到角度变化
    // 假设 imu.gz 已经是校准过方向的 Body Frame 数据 (FRD: Z向下，右转为正)
    magCheckData.gyroInteg += (imu.gz * 180 / Math.PI) * dt;

    // 2. 磁力计航向计算 (简单的 atan2，假设水平)
    // 如果固件传上来的是经过 SENS_BOARD_ROT 变换后的 Body Frame Mag，那直接算 atan2(y, x)
    // 注意: atan2(y, x) 是标准的数学定义，但在导航中 Heading 通常定义为 atan2(y, x) 或 atan2(-y, x) 取决于坐标系
    // 这里我们只关心相对变化，公式本身不影响旋转的"方向性"判断，只要前后一致即可
    const magHeading = Math.atan2(mag.y, mag.x) * 180 / Math.PI;

    if (magCheckData.startMagHeading === null) {
        magCheckData.startMagHeading = magHeading;
    }

    // 计算磁力计的累积角度变化 (处理 -180/180 跳变)
    let magDelta = magHeading - magCheckData.startMagHeading;
    // 处理过零点问题 (比如从 179 跳到 -179，实际变了 +2度，直接减是 -358)
    if (magDelta > 180) magDelta -= 360;
    if (magDelta < -180) magDelta += 360;

    // 3. 更新双向进度条 UI
    updateMagCheckUI(magCheckData.gyroInteg, magDelta);

    // 4. 实时判定与反馈
    const resDiv = document.getElementById('mag-check-result');
    const statusBadge = document.getElementById('mag-orient-status');

    // 阈值判断 (如果转动超过 15度才开始评判，避免噪音)
    if (Math.abs(magCheckData.gyroInteg) > 15) {
        // 同号 (乘积>0) 且 误差小于 30度
        // 这里放宽误差是因为未校准的磁力计可能有软磁干扰，导致椭圆失真，角度不线性
        if (magCheckData.gyroInteg * magDelta > 0 && Math.abs(magCheckData.gyroInteg - magDelta) < 45) {
            resDiv.innerHTML = '<span class="text-success fw-bold"><i class="fas fa-check-circle"></i> 方向一致</span>';
            statusBadge.className = "badge bg-success";
            statusBadge.innerText = "正常";
        }
        // 异号 (乘积<0) -> 反向
        else if (magCheckData.gyroInteg * magDelta < 0) {
            resDiv.innerHTML = '<span class="text-danger fw-bold"><i class="fas fa-times-circle"></i> 方向相反！</span><br><small>磁力计需翻转/修正</small>';
            statusBadge.className = "badge bg-danger";
            statusBadge.innerText = "异常";
        }
        // 其他情况 (如同号但误差极大) -> 轴向错误 (比如转了90度)
        else {
            resDiv.innerHTML = '<span class="text-warning fw-bold"><i class="fas fa-exclamation-triangle"></i> 轴向不匹配</span><br><small>可能是90度偏差</small>';
            statusBadge.className = "badge bg-warning text-dark";
            statusBadge.innerText = "异常";
        }
    } else {
        resDiv.innerHTML = '正在采集... 请继续旋转';
    }
}

// 辅助 UI 更新函数 (适配新的双向进度条)
function updateMagCheckUI(gyroVal, magVal) {
    // 映射范围：+/- 90 度满格
    const MAX_ANG = 90;

    const setBar = (type, val) => {
        // 限制在 -90 ~ 90
        let v = Math.max(-MAX_ANG, Math.min(MAX_ANG, val));
        // 转为百分比
        let pct = (Math.abs(v) / MAX_ANG) * 100;

        const elNeg = document.getElementById(`bar-chk-${type}-neg`);
        const elPos = document.getElementById(`bar-chk-${type}-pos`);

        if (v < 0) {
            elNeg.style.width = pct + '%';
            elPos.style.width = '0%';
        } else {
            elNeg.style.width = '0%';
            elPos.style.width = pct + '%';
        }
    };

    setBar('gyro', gyroVal);
    setBar('mag', magVal);
}
// ================= 13. 分步轴向推断 (Axis Correlation Method - Gauss适配版) =================

let isAxisDetecting = false;
let currentDetectAxis = null; // 'pitch' or 'roll'
let axisSamples = { pitch: null, roll: null };
let axisBuffer = []; // 存储磁力计原始向量
let axisDetectStartTime = 0;
const AXIS_DETECT_DURATION = 5000; // 5秒

window.startAxisDetect = function (axis) {
    if (isAxisDetecting) return;

    isAxisDetecting = true;
    currentDetectAxis = axis;
    axisBuffer = []; // 清空缓存
    axisDetectStartTime = Date.now();

    // UI 更新
    const btn = document.getElementById(`btn-step-${axis}`);
    btn.innerHTML = '<div class="spinner-border spinner-border-sm"></div> 晃动中...';
    btn.disabled = true;

    const bar = document.getElementById(`prog-step-${axis}`);
    bar.classList.add('progress-bar-striped', 'progress-bar-animated');
    bar.classList.replace('bg-secondary', axis === 'pitch' ? 'bg-primary' : 'bg-success');

    updateAxisProgress();
};

function updateAxisProgress() {
    if (!isAxisDetecting) return;

    const elapsed = Date.now() - axisDetectStartTime;
    const pct = Math.min(100, (elapsed / AXIS_DETECT_DURATION) * 100);
    const bar = document.getElementById(`prog-step-${currentDetectAxis}`);
    bar.style.width = pct + '%';

    if (elapsed < AXIS_DETECT_DURATION) {
        requestAnimationFrame(updateAxisProgress);
    } else {
        finishAxisDetect();
    }
}

// 在 handleDataPacket 中调用
function collectAxisData(mag) {
    if (!isAxisDetecting) return;

    // [修复] 针对 Gauss 单位 (0.x 级别) 大幅降低去重阈值
    // 之前是 2.0，现在改为 0.01 (10 mGauss)
    if (axisBuffer.length > 0) {
        const last = axisBuffer[axisBuffer.length - 1];
        const dist = Math.abs(mag.x - last.x) + Math.abs(mag.y - last.y) + Math.abs(mag.z - last.z);
        if (dist < 0.01) return; // 忽略极其微小的抖动
    }

    axisBuffer.push(new THREE.Vector3(mag.x, mag.y, mag.z));
}

function finishAxisDetect() {
    isAxisDetecting = false;
    const axis = currentDetectAxis;

    // UI 恢复
    const btn = document.getElementById(`btn-step-${axis}`);
    const bar = document.getElementById(`prog-step-${axis}`);
    bar.classList.remove('progress-bar-striped', 'progress-bar-animated');

    // --- 核心算法 ---

    let rotationAxisSum = new THREE.Vector3(0, 0, 0);
    let validPairs = 0;

    // Debug: 打印采集到的点数
    console.log(`[AxisDetect] Collected ${axisBuffer.length} samples for ${axis}`);

    for (let i = 1; i < axisBuffer.length; i++) {
        const v1 = axisBuffer[i - 1];
        const v2 = axisBuffer[i];

        // 叉乘得到法向量 (即旋转轴)
        const cross = new THREE.Vector3().crossVectors(v1, v2);

        // [修复] 针对 Gauss 单位大幅降低有效性阈值
        // 0.2 * 0.2 * sin(theta) ≈ 0.00x 级别
        // 之前是 5.0，现在改为 0.002
        if (cross.length() > 0.002) {
            // 累加绝对值分量，判断主轴
            rotationAxisSum.x += Math.abs(cross.x);
            rotationAxisSum.y += Math.abs(cross.y);
            rotationAxisSum.z += Math.abs(cross.z);
            validPairs++;
        }
    }

    console.log(`[AxisDetect] Valid pairs: ${validPairs}, SumVec:`, rotationAxisSum);

    // 如果有效数据太少，提示重试
    if (validPairs < 5) {
        alert(`检测失败：动作幅度太小 (${validPairs} 帧有效)。\n\n请确保大幅度抬头/低头或摇摆。`);
        btn.innerHTML = '<i class="fas fa-redo"></i> 重试';
        btn.disabled = false;
        bar.innerText = "数据不足";
        return;
    }

    // 成功
    btn.innerHTML = '<i class="fas fa-check"></i> 完成';
    bar.innerText = "采集完成";

    // 归一化
    rotationAxisSum.normalize();
    axisSamples[axis] = rotationAxisSum;

    // 逻辑流转
    if (axis === 'pitch') {
        document.getElementById('btn-step-roll').disabled = false;
        document.getElementById('prog-step-roll').innerText = "请点击开始";
    } else if (axis === 'roll') {
        calculateFinalOrientation();
    }
}

function calculateFinalOrientation() {
    if (!axisSamples.pitch || !axisSamples.roll) return;

    const resDiv = document.getElementById('axis-detect-result');
    resDiv.style.display = 'block';

    const pitchVec = axisSamples.pitch; // 对应 Body Y
    const rollVec = axisSamples.roll;  // 对应 Body X

    let bestRotId = -1;
    let maxScore = -Infinity;

    ROTATION_LIST.forEach(rot => {
        const d2r = Math.PI / 180;
        const euler = new THREE.Euler(rot.roll * d2r, rot.pitch * d2r, rot.yaw * d2r, 'ZYX');
        const q = new THREE.Quaternion().setFromEuler(euler);

        // 验证逻辑：
        // 将测量的 Sensor Frame 下的旋转轴，转到 Body Frame
        const rollInBody = rollVec.clone().applyQuaternion(q);
        const pitchInBody = pitchVec.clone().applyQuaternion(q);

        // 评分：Body X 应该接近 (1,0,0)，Body Y 应该接近 (0,1,0)
        // 取绝对值是因为用户可能正向摇，也可能反向摇，但轴线是不变的
        const score = Math.abs(rollInBody.x) + Math.abs(pitchInBody.y);

        if (score > maxScore) {
            maxScore = score;
            bestRotId = rot.id;
        }
    });

    const match = ROTATION_LIST.find(r => r.id === bestRotId);

    // [调整] 评分阈值，因为实际测量会有噪声，1.4 分（满分2.0）通常就比较可信了
    if (maxScore > 1.4) {
        resDiv.innerHTML = `
            <h5 class="text-success"><i class="fas fa-check-circle"></i> 识别成功</h5>
            <div class="fs-4 fw-bold mb-2">${match.name}</div>
            <div class="text-muted small mb-3">置信度: ${(maxScore / 2 * 100).toFixed(0)}% (ID: ${bestRotId})</div>
            <button class="btn btn-success" onclick="applyMagAutoParam(${bestRotId}, '${match.name}')">
                应用并保存
            </button>
        `;
    } else {
        resDiv.innerHTML = `
            <h5 class="text-danger"><i class="fas fa-times-circle"></i> 识别失败</h5>
            <p class="mb-0">数据特征不明显 (得分 ${maxScore.toFixed(2)})。<br>可能是动作不够标准。</p>
            <button class="btn btn-outline-dark btn-sm mt-2" onclick="location.reload()">重新开始</button>
        `;
    }
}

// ================= 14. 原始数据监控 (Monitor & Scope) =================

let monitorInited = false;
let isChartPaused = false;
let currentScopeType = 'acc'; // acc, gyro, mag

// 3D 对象引用
let vecRenderer, vecScene, vecCamera, vecArrowAcc, vecArrowMag;
let attRenderer, attScene, attCamera, attBoxAcc, attBoxGyro;
let gyroQuat = new THREE.Quaternion(); // 陀螺仪积分姿态

// 波形数据缓存 (用于 Plotly)
let scopeData = {
    x: [],
    y0: [], y1: [], y2: [] // X, Y, Z
};
const MAX_SCOPE_POINTS = 300; // 保留最近 300 个点

// --- 初始化入口 ---
function initMonitor() {
    if (monitorInited) return;
    monitorInited = true;

    // 1. 初始化 Plotly 波形图
    const layout = {
        margin: { t: 20, r: 20, b: 40, l: 50 },
        showlegend: true,
        legend: { orientation: 'h', y: 1.1 },
        xaxis: { title: 'Time' },
        // [关键] Y轴自适应设置
        yaxis: {
            autorange: true,   // 开启自适应
            fixedrange: false, // 允许用户手动缩放
            title: 'Value'
        }
    };

    // 初始化三条线 (X, Y, Z)
    Plotly.newPlot('plotly-scope', [
        { y: [], mode: 'lines', name: 'X-Axis', line: { color: '#dc3545' } },
        { y: [], mode: 'lines', name: 'Y-Axis', line: { color: '#198754' } },
        { y: [], mode: 'lines', name: 'Z-Axis', line: { color: '#0d6efd' } }
    ], layout, { responsive: true, displayModeBar: true });

    // 2. 初始化 3D 视图 (向量)
    initVectorScene();

    // 3. 初始化 3D 视图 (姿态对比)
    initAttitudeScene();

    // 启动动画循环
    animateMonitor();
}

// 切换波形类型
window.switchScopeTrace = function (type) {
    currentScopeType = type;
    // 清空当前图表数据以避免混乱
    Plotly.restyle('plotly-scope', { y: [[], [], []] });
    scopeData = { x: [], y0: [], y1: [], y2: [] };
};

window.toggleChartPause = function () {
    isChartPaused = document.getElementById('btn-pause-chart').checked;
};

window.resetGyroIntegration = function () {
    gyroQuat.set(0, 0, 0, 1); // 重置为 Identity
};

// --- 核心更新逻辑 (在 handleDataPacket 中调用) ---
// --- 核心更新逻辑 (在 handleDataPacket 中调用) ---
function updateMonitor(imu, mag) {
    if (!monitorInited || document.getElementById('view-monitor').style.display === 'none') return;

    // 1. 更新数值仪表盘 (保持不变)
    document.getElementById('raw-ax').innerText = imu.ax.toFixed(2);
    document.getElementById('raw-ay').innerText = imu.ay.toFixed(2);
    document.getElementById('raw-az').innerText = imu.az.toFixed(2);
    const aNorm = Math.sqrt(imu.ax ** 2 + imu.ay ** 2 + imu.az ** 2);
    document.getElementById('raw-a-norm').innerText = aNorm.toFixed(2);

    document.getElementById('raw-gx').innerText = imu.gx.toFixed(2);
    document.getElementById('raw-gy').innerText = imu.gy.toFixed(2);
    document.getElementById('raw-gz').innerText = imu.gz.toFixed(2);

    if (mag) {
        document.getElementById('raw-mx').innerText = mag.x.toFixed(2);
        document.getElementById('raw-my').innerText = mag.y.toFixed(2);
        document.getElementById('raw-mz').innerText = mag.z.toFixed(2);
        const mNorm = Math.sqrt(mag.x ** 2 + mag.y ** 2 + mag.z ** 2);
        document.getElementById('raw-m-norm').innerText = mNorm.toFixed(2);
    }

    // 2. 向量视图 (保持你刚才满意的状态)
    if (vecArrowAcc) {
        // Accel: FRD z-down is gravity. So -az is up.
        // Map: y -> x (Right), -z -> y (Up), x -> z (Forward)
        const vAcc = new THREE.Vector3(imu.ay, -imu.az, imu.ax).normalize();
        vecArrowAcc.setDirection(vAcc);
        vecArrowAcc.setLength(1.5);
    }
    if (mag && vecArrowMag) {
        const vMag = new THREE.Vector3(mag.y, -mag.z, mag.x).normalize();
        vecArrowMag.setDirection(vMag);
        vecArrowMag.setLength(1.5);
    }

    // =========================================================
    // [修正重点 1] Accel (线框) 轴向交换修正
    // =========================================================

    // 计算倾角 (保持公式不变)
    // accRoll: 绕 X 轴旋转 (左右倾斜)
    const accRoll = Math.atan2(imu.ay, imu.az);
    // accPitch: 绕 Y 轴旋转 (抬头低头)
    const accPitch = Math.atan2(-imu.ax, Math.sqrt(imu.ay ** 2 + imu.az ** 2));

    if (attBoxAcc) {
        // Three.js rotation.set(x, y, z) 对应的是 绕X轴, 绕Y轴, 绕Z轴
        // 在我们的视图映射中：
        // 视觉 X轴 (红色) = 机体 Pitch 轴
        // 视觉 Y轴 (绿色) = 机体 Yaw 轴 (Accel测不到)
        // 视觉 Z轴 (蓝色) = 机体 Roll 轴

        // 之前写反了，现在交换位置：
        // 参数1 (Rot X) <- accPitch (之前是 accRoll)
        // 参数3 (Rot Z) <- accRoll  (之前是 accPitch)
        // 注意：如果方向反了，试着给 accRoll 加负号，即 -accRoll
        attBoxAcc.rotation.set(-accPitch, 0, -accRoll);
    }

    // =========================================================
    // [修正重点 2] Gyro (蓝色实体) Roll 反向修正
    // =========================================================

    const dt = 0.05;
    const wx = imu.gx;
    const wy = imu.gy;
    const wz = imu.gz;

    const omega = Math.sqrt(wx * wx + wy * wy + wz * wz);
    if (omega > 0.001) {
        // 轴向映射：
        // wy (Pitch) -> x
        // -wz (Yaw)  -> y
        // wx (Roll)  -> z

        // [修改] 之前是 wx，现在改为 -wx (取反)
        // 这样可以反转 Roll 的旋转方向，解决"反向"问题
        const axis = new THREE.Vector3(wy, -wz, -wx).normalize();

        const angle = omega * dt;
        const dq = new THREE.Quaternion().setFromAxisAngle(axis, angle);
        gyroQuat.multiply(dq);
        gyroQuat.normalize();
    }

    if (attBoxGyro) {
        attBoxGyro.setRotationFromQuaternion(gyroQuat);
    }

    // 4. 波形图更新 (保持不变)
    if (!isChartPaused) {
        let val0 = 0, val1 = 0, val2 = 0;
        if (currentScopeType === 'acc') {
            val0 = imu.ax; val1 = imu.ay; val2 = imu.az;
        } else if (currentScopeType === 'gyro') {
            val0 = imu.gx; val1 = imu.gy; val2 = imu.gz;
        } else if (currentScopeType === 'mag' && mag) {
            val0 = mag.x; val1 = mag.y; val2 = mag.z;
        }
        Plotly.extendTraces('plotly-scope', {
            y: [[val0], [val1], [val2]]
        }, [0, 1, 2], MAX_SCOPE_POINTS);
    }
}

// --- 3D 初始化辅助函数 ---
function initVectorScene() {
    const container = document.getElementById('scene-vectors');
    vecScene = new THREE.Scene();
    vecScene.background = new THREE.Color(0xf8f9fa); // 浅灰背景

    vecCamera = new THREE.PerspectiveCamera(50, container.clientWidth / container.clientHeight, 0.1, 100);
    vecCamera.position.set(3, 2, 3);
    vecCamera.lookAt(0, 0, 0);

    vecRenderer = new THREE.WebGLRenderer({ antialias: true });
    vecRenderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(vecRenderer.domElement);

    const controls = new THREE.OrbitControls(vecCamera, vecRenderer.domElement);
    controls.enableZoom = false;

    // 坐标轴辅助 (RGB = XYZ)
    const axesHelper = new THREE.AxesHelper(2);
    vecScene.add(axesHelper);

    // 网格
    const gridHelper = new THREE.GridHelper(5, 10);
    vecScene.add(gridHelper);

    // 箭头对象
    const origin = new THREE.Vector3(0, 0, 0);
    // Accel 箭头 (绿)
    vecArrowAcc = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), origin, 1.5, 0x198754);
    vecScene.add(vecArrowAcc);
    // Mag 箭头 (红)
    vecArrowMag = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), origin, 1.5, 0xdc3545);
    vecScene.add(vecArrowMag);
}

function initAttitudeScene() {
    const container = document.getElementById('scene-attitude');
    attScene = new THREE.Scene();
    attScene.background = new THREE.Color(0xf0f0f0);

    attCamera = new THREE.PerspectiveCamera(50, container.clientWidth / container.clientHeight, 0.1, 100);
    attCamera.position.set(0, 3, 5); // 后上方视角
    attCamera.lookAt(0, 0, 0);

    attRenderer = new THREE.WebGLRenderer({ antialias: true });
    attRenderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(attRenderer.domElement);

    // 灯光
    const light = new THREE.DirectionalLight(0xffffff, 1);
    light.position.set(5, 10, 7);
    attScene.add(light);
    attScene.add(new THREE.AmbientLight(0x404040));

    // 物体 1: Accel 姿态 (半透明线框盒子)
    const geo = new THREE.BoxGeometry(2, 0.2, 1.5); // 像个机翼
    const matAcc = new THREE.MeshBasicMaterial({ color: 0x999999, wireframe: true, transparent: true, opacity: 0.5 });
    attBoxAcc = new THREE.Mesh(geo, matAcc);
    attScene.add(attBoxAcc);

    // 物体 2: Gyro 姿态 (实心盒子)
    const matGyro = new THREE.MeshPhongMaterial({ color: 0x0d6efd });
    attBoxGyro = new THREE.Mesh(geo, matGyro);
    attScene.add(attBoxGyro);

    const gridHelper = new THREE.GridHelper(10, 10);
    gridHelper.position.y = -2;
    attScene.add(gridHelper);
}

function animateMonitor() {
    requestAnimationFrame(animateMonitor);
    if (document.getElementById('view-monitor').style.display !== 'none') {
        if (vecRenderer && vecScene && vecCamera) vecRenderer.render(vecScene, vecCamera);
        if (attRenderer && attScene && attCamera) attRenderer.render(attScene, attCamera);
    }
}