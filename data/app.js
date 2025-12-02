// ================= 配置区 =================
// 【重要】true = 电脑模拟模式; false = 真实连接模式
const IS_SIMULATION = true;
const ESP_IP = "192.168.4.1"; // 真实机器人的IP

// ================= 全局变量定义 =================
let ws = null;
let allParams = [];
// 图表数据缓存
let chartData = { labels: [], pitch: [], roll: [], yaw: [] };
// 6面校准状态记录
let imuSidesStatus = { z_up: false, z_down: false, x_up: false, x_down: false, y_up: false, y_down: false };

// 3D 场景变量
let robotMesh = null;
let renderer = null, scene = null, camera = null;

// 磁力计采集变量
let isCapturingMag = false;
let magPoints = { x: [], y: [], z: [] };

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
        logToTerminal("Mode: Simulation enabled.");

        // 启动模拟数据生成器
        startSimulationDataLoop();
        // 生成一些假参数用于测试
        allParams = generateMockParams(50);
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

// ================= 3. 核心数据处理 (路由分发) =================
function handleDataPacket(jsonObj) {
    const type = jsonObj.type;
    const payload = jsonObj.payload;

    // 1. 遥测数据 (Telemetry)
    if (type === "telem") {
        // 更新顶部卡片数值
        if (document.getElementById("val-bat")) document.getElementById("val-bat").innerText = payload.voltage.toFixed(1) + " V";
        if (document.getElementById("val-pitch")) document.getElementById("val-pitch").innerText = payload.pitch.toFixed(1) + "°";
        if (document.getElementById("val-roll")) document.getElementById("val-roll").innerText = payload.roll.toFixed(1) + "°";
        if (document.getElementById("val-yaw")) document.getElementById("val-yaw").innerText = payload.yaw.toFixed(1) + "°";

        // 更新图表数据
        updateChart(payload);

        // 如果当前是 3D 模式，更新模型姿态
        if (currentDashMode === '3d') {
            update3DObject(payload.roll, payload.pitch, payload.yaw);
        }

        // 如果正在采集磁力计，更新点云
        if (isCapturingMag && payload.mag) {
            updateMagPlot(payload.mag);
        }
    }
    // 2. 参数列表
    else if (type === "param_list") {
        logToTerminal(`Received ${payload.length} parameters.`);
        renderParamTable(payload);
    }
    // 3. 系统日志
    else if (type === "log") {
        logToTerminal("[ESP] " + payload.msg);
    }
    // 4. 标定确认信号 (ACK) - 关键逻辑
    else if (type === "calib_ack") {
        logToTerminal(`Calibration ACK received: ${payload.side}`);
        markSideAsDone(payload.side);
    }
}

// ================= 4. IMU 6面校准逻辑 (修复版) =================

// 用户点击按钮 -> 进入“采集中”状态
window.captureImuSide = function (sideName) {
    // 转换ID格式 (z_up -> btn-side-z-up)
    const btnId = `btn-side-${sideName.replace('_', '-')}`;
    const btn = document.getElementById(btnId);

    // 视觉反馈：变成蓝色 Loading 状态
    btn.className = "btn btn-info w-100 py-3 text-white shadow-sm imu-side-btn";
    btn.innerHTML = `<div class="spinner-border spinner-border-sm mb-2" role="status"></div><br>采集中 (Sampling)...`;
    btn.disabled = true; // 防止重复点击

    logToTerminal(`Cmd: Start capturing ${sideName}...`);

    if (IS_SIMULATION) {
        // 模拟：延迟 2秒 后，假装收到 ESP32 的完成信号
        setTimeout(() => {
            // 构造一个假的 ACK 包，喂给处理函数
            handleDataPacket({
                type: "calib_ack",
                payload: { side: sideName }
            });
        }, 2000);
    } else {
        // 真实：发送指令给 ESP32
        sendJson({ cmd: "calib_imu_side", side: sideName });
        // 注意：真实设备需要在采集完成后发送 {"type":"calib_ack", "payload":{"side":"..."}}
    }
};

// 收到完成信号 -> 变成绿色 OK 状态
function markSideAsDone(sideName) {
    imuSidesStatus[sideName] = true;

    const btnId = `btn-side-${sideName.replace('_', '-')}`;
    const btn = document.getElementById(btnId);

    // 恢复图标映射
    const icons = {
        'z_up': 'fa-arrow-up', 'z_down': 'fa-arrow-down',
        'x_up': 'fa-arrow-right', 'x_down': 'fa-arrow-left',
        'y_up': 'fa-redo', 'y_down': 'fa-undo'
    };

    // 视觉反馈：变绿
    btn.className = "btn btn-success w-100 py-3 shadow-sm imu-side-btn";
    btn.innerHTML = `<i class="fas fa-check-circle fs-2 mb-1"></i><br>已完成 (Done)`;
    btn.disabled = false;

    // 检查是否全部完成
    checkImuAllDone();
}

function checkImuAllDone() {
    // 只有当所有 6 个状态都为 true 时，才启用计算按钮
    const allDone = Object.values(imuSidesStatus).every(status => status === true);
    const calcBtn = document.getElementById("btn-imu-calc");

    if (allDone) {
        calcBtn.disabled = false;
        calcBtn.classList.remove("btn-secondary");
        calcBtn.classList.add("btn-primary"); // 变蓝高亮
        calcBtn.innerHTML = `<i class="fas fa-microchip me-2"></i>6面数据就绪，点击计算并保存`;
        logToTerminal("All 6 sides captured. Ready to calculate.");
    }
}

window.finishImuCalibration = function () {
    if (!confirm("确认使用当前采集的数据进行计算并写入 Flash 吗？")) return;

    logToTerminal("Calculating IMU offsets...");
    if (IS_SIMULATION) {
        setTimeout(() => {
            alert("[模拟] 标定成功！参数已更新。");
            location.reload();
        }, 1000);
    } else {
        sendJson({ cmd: "calib_imu_finish" });
    }
};

// ================= 5. 图表功能 (Chart.js) =================
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
                { label: 'Yaw', data: [], borderColor: '#4bc0c0', tension: 0.3, pointRadius: 0 } // Yaw 曲线
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

    // 保持最近 100 个点
    if (chartData.labels.length > 100) {
        chartData.labels.shift();
        chartData.pitch.shift();
        chartData.roll.shift();
        chartData.yaw.shift();
    }

    mainChart.data.labels = chartData.labels;
    mainChart.data.datasets[0].data = chartData.pitch;
    mainChart.data.datasets[1].data = chartData.roll;
    mainChart.data.datasets[2].data = chartData.yaw;

    if (currentDashMode === 'chart') mainChart.update();
}

// ================= 6. 3D 场景 (Three.js) =================
function init3DScene() {
    const container = document.getElementById('three-container');
    // 给个默认大小防止出错
    const w = container.clientWidth || 800;
    const h = container.clientHeight || 500;

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf8f9fa);

    // 灯光
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

    // 创建长方体机器人
    const geometry = new THREE.BoxGeometry(1.6, 0.2, 1);
    const materials = [
        new THREE.MeshLambertMaterial({ color: 0xcccccc }), // Right
        new THREE.MeshLambertMaterial({ color: 0xcccccc }), // Left
        new THREE.MeshLambertMaterial({ color: 0xeeeeee }), // Top
        new THREE.MeshLambertMaterial({ color: 0x666666 }), // Bottom
        new THREE.MeshLambertMaterial({ color: 0xff3333 }), // Front (红色车头)
        new THREE.MeshLambertMaterial({ color: 0xcccccc })  // Back
    ];
    robotMesh = new THREE.Mesh(geometry, materials);

    scene.add(new THREE.AxesHelper(2)); // 坐标轴
    scene.add(robotMesh);
    scene.add(new THREE.GridHelper(10, 10)); // 网格地面

    const animate = function () {
        requestAnimationFrame(animate);
        renderer.render(scene, camera);
    };
    animate();
}

function update3DObject(roll, pitch, yaw) {
    if (!robotMesh) return;
    const d2r = Math.PI / 180;
    // 注意：这里可能需要根据你的实际传感器坐标系调整正负号
    robotMesh.rotation.x = pitch * d2r;
    robotMesh.rotation.z = -roll * d2r;
    robotMesh.rotation.y = -yaw * d2r;
}

// ================= 7. 磁力计点云 (Plotly.js) =================
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

function updateMagPlot(magData) {
    if (magPoints.x.length > 2000) return; // 限制点数

    magPoints.x.push(magData.x);
    magPoints.y.push(magData.y);
    magPoints.z.push(magData.z);

    if (document.getElementById('mag-points-count')) {
        document.getElementById('mag-points-count').innerText = magPoints.x.length;
    }

    // 降低刷新频率优化性能
    if (magPoints.x.length % 5 === 0) {
        Plotly.restyle('mag-plot', {
            x: [magPoints.x],
            y: [magPoints.y],
            z: [magPoints.z]
        });
    }
}

window.startMagCapture = function () {
    isCapturingMag = !isCapturingMag;
    const btn = event.target.closest('button'); // 获取点击的按钮

    if (isCapturingMag) {
        // 开始
        magPoints = { x: [], y: [], z: [] };
        Plotly.restyle('mag-plot', { x: [[]], y: [[]], z: [[]] });
        btn.innerHTML = '<i class="fas fa-stop me-2"></i>停止采集';
        btn.classList.add('btn-danger');
        btn.classList.remove('btn-outline-primary');
        logToTerminal("Started Mag data capture...");
    } else {
        // 停止
        btn.innerHTML = '<i class="fas fa-play me-2"></i>开始采集';
        btn.classList.remove('btn-danger');
        btn.classList.add('btn-outline-primary');
        logToTerminal(`Stopped. Points: ${magPoints.x.length}`);
    }
};

// ================= 8. 参数管理 =================
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
                <button class="btn btn-sm btn-outline-success border-0" title="保存" onclick="saveSingleParam('${p.name}')">
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
        // alert(`[模拟] 参数 ${key} 已更新!`); // 嫌烦可以注释掉
    } else {
        sendJson({ cmd: "set_param", key: key, val: newVal });
    }
};

// ================= 9. 辅助工具与模拟器 =================
function generateMockParams(count) {
    const arr = [];
    const groups = ["PID_PITCH", "PID_ROLL", "PID_YAW", "MAG_OFFSET", "MAG_SCALE", "SYS_CONFIG"];
    for (let i = 0; i < count; i++) {
        const grp = groups[Math.floor(Math.random() * groups.length)];
        const isFloat = Math.random() > 0.4;
        arr.push({
            name: `${grp}_${i}`,
            type: isFloat ? 1 : 0,
            val: isFloat ? parseFloat((Math.random() * 10).toFixed(2)) : Math.floor(Math.random() * 100)
        });
    }
    return arr;
}

window.startSimulationDataLoop = function () {
    let t = 0;
    setInterval(() => {
        t += 0.05;
        // 模拟数据包
        handleDataPacket({
            type: "telem",
            payload: {
                voltage: 12.0 + Math.random() * 0.1,
                pitch: Math.sin(t) * 30,
                roll: Math.cos(t) * 20,
                yaw: t * 10 % 360, // 模拟 Yaw 旋转
                mag: {
                    x: Math.sin(t) * 300 + Math.random() * 50,
                    y: Math.cos(t) * 300 + Math.random() * 50,
                    z: Math.random() * 100
                }
            }
        });
    }, 50); // 20Hz 刷新
};

// 页面切换
window.showPage = function (pageName, btnElement) {
    document.querySelectorAll(".view-section").forEach(el => el.style.display = 'none');
    document.getElementById("view-" + pageName).style.display = 'block';

    document.querySelectorAll(".list-group-item").forEach(el => el.classList.remove("active"));
    if (btnElement) btnElement.classList.add("active");

    // 特殊处理
    if (pageName === 'dashboard') switchDashView(currentDashMode);
    if (pageName === 'calibration') Plotly.relayout('mag-plot', { autosize: true });
};

// 标定Tab切换
window.switchCalibTab = function (type) {
    document.querySelectorAll(".calib-content").forEach(el => el.style.display = 'none');
    document.getElementById("tab-content-" + type).style.display = 'block';
    document.querySelectorAll("#calibTab .nav-link").forEach(el => el.classList.remove("active"));
    document.getElementById("tab-btn-" + type).classList.add("active");
    if (type === 'mag') Plotly.relayout('mag-plot', { autosize: true });
};

// 终端日志
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

// WebSocket 连接
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
    else logToTerminal("Error: No connection.");
}