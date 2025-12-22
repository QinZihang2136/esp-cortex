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

        // --- 磁力计校准数据流 ---
        if (isCapturingMag && payload.mag) {
            updateMagCalibrationLogic(payload.mag);
        }

        // --- IMU (Accel/Gyro) 校准数据流 ---
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
    ws.onopen = () => { setConnStatus("在线 (Online)", "bg-success"); logToTerminal("WebSocket Connected"); };
    ws.onclose = () => { setConnStatus("断开 (Offline)", "bg-danger"); logToTerminal("WebSocket Closed"); };
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