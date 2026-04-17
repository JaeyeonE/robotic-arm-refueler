// ── Admin Monitor API 연동 (station 선택 기반) ──

let selectedStationId = null;
let torqueChart = null;

// 시계
function updateClock() {
    const now = new Date();
    document.getElementById('aClock').textContent =
        [now.getHours(), now.getMinutes(), now.getSeconds()]
            .map(v => String(v).padStart(2, '0')).join(':');
}
setInterval(updateClock, 1000);
updateClock();

// ── 로그아웃 ──
async function doLogout() {
    await fetch('/api/auth/logout', { method: 'POST' });
    window.location.href = '/login';
}

// ── 지점 / 스테이션 선택 ──
async function loadBranches() {
    try {
        const res = await fetch('/api/branches');
        const branches = await res.json();
        const sel = document.getElementById('aBranchSelect');
        sel.innerHTML = '<option value="">지점 선택...</option>' +
            branches.map(b => `<option value="${b.id}">${b.name}</option>`).join('');
        if (branches.length === 1) {
            sel.value = branches[0].id;
            onBranchChange();
        }
    } catch (e) { /* silent */ }
}

async function onBranchChange() {
    const branchId = document.getElementById('aBranchSelect').value;
    const list = document.getElementById('aStationList');
    if (!branchId) {
        list.innerHTML = '<div style="color:#94a3b8;font-size:11px;">지점을 선택하세요</div>';
        selectStation(null);
        return;
    }
    try {
        const res = await fetch(`/api/branches/${branchId}/stations`);
        const stations = await res.json();
        if (stations.length === 0) {
            list.innerHTML = '<div style="color:#94a3b8;font-size:11px;">등록된 스테이션 없음</div>';
            selectStation(null);
            return;
        }
        list.innerHTML = stations.map(s => {
            const stCls = s.status === 'busy' ? 'sb-busy' : s.status === 'offline' ? 'sb-offline' : 'sb-idle';
            return `<div class="station-card" data-sid="${s.id}" onclick="selectStation(${s.id})">
                <div class="station-card-title">
                    Station #${s.station_no}
                    <span class="station-badge ${stCls}">${s.status.toUpperCase()}</span>
                </div>
                <div class="station-card-meta">${s.robot_model} · ${s.camera_model}</div>
            </div>`;
        }).join('');

        // 첫 번째 스테이션 자동 선택
        selectStation(stations[0].id);
    } catch (e) { /* silent */ }
}

function selectStation(stationId) {
    selectedStationId = stationId;

    // 카드 active 표시
    document.querySelectorAll('.station-card').forEach(c => {
        c.classList.toggle('active', Number(c.dataset.sid) === stationId);
    });

    const grid = document.getElementById('aGrid');
    const noSt = document.getElementById('aNoStation');

    if (!stationId) {
        grid.style.display = 'none';
        noSt.style.display = 'flex';
        return;
    }

    grid.style.display = 'grid';
    noSt.style.display = 'none';

    // 즉시 데이터 로드
    loadStatus(); loadRobot(); loadFrame(); loadVision(); loadLogs();

    // 차트 초기화
    if (!torqueChart) initChart();
}

function SQ() { return `station_id=${selectedStationId}`; }

// ── 시스템 상태 배지 (3초) ──
async function loadStatus() {
    if (!selectedStationId) return;
    try {
        const res = await fetch(`/api/system/status?${SQ()}`);
        const d = await res.json();

        const bRobot = document.getElementById('badgeRobot');
        const bTask = document.getElementById('badgeTask');
        const bDart = document.getElementById('badgeDart');
        const dot = document.getElementById('aBrandDot');

        if (d.robot_mode && d.robot_mode !== 'OFFLINE') {
            bRobot.textContent = 'ROBOT ONLINE';
            bRobot.className = 'a-badge a-badge-green';
            dot.style.background = '#4ade80';
        } else {
            bRobot.textContent = 'ROBOT OFFLINE';
            bRobot.className = 'a-badge a-badge-red';
            dot.style.background = '#f87171';
        }

        bTask.textContent = d.task_status ? d.task_status.toUpperCase() : 'IDLE';
        bDart.textContent = d.dart_connected ? 'DART CONNECTED' : 'DART DISCONNECTED';
        bDart.className = 'a-badge ' + (d.dart_connected ? 'a-badge-blue' : 'a-badge-red');
    } catch (e) { /* silent */ }
}

// ── 관절 상태 (1.5초) ──
function fmt(v, digits = 1) {
    return (v === null || v === undefined) ? '--' : Number(v).toFixed(digits);
}

function renderJoints(joints) {
    document.getElementById('aJoints').innerHTML = joints.map(j => {
        const angle = (j.angle === null || j.angle === undefined) ? 0 : j.angle;
        const pct = Math.min(100, (Math.abs(angle) / 180 * 100)).toFixed(0);
        return `<div class="jrow">
            <span class="jlabel">${j.label}</span>
            <div class="jbar-bg"><div class="jbar" style="width:${pct}%"></div></div>
            <span class="jval">${fmt(j.angle)}°</span>
            <span class="jtorq">${fmt(j.torque)} Nm</span>
        </div>`;
    }).join('');
}

function renderTcp(tcp) {
    const axes = [
        { axis: 'X', val: tcp.x, unit: 'mm' },
        { axis: 'Y', val: tcp.y, unit: 'mm' },
        { axis: 'Z', val: tcp.z, unit: 'mm' },
        { axis: 'A', val: tcp.a, unit: '°' },
        { axis: 'B', val: tcp.b, unit: '°' },
        { axis: 'C', val: tcp.c, unit: '°' },
    ];
    document.getElementById('aTcp').innerHTML = axes.map(t =>
        `<div class="tcp-tile">
            <div class="tcp-axis-lbl">${t.axis}</div>
            <div class="tcp-num">${fmt(t.val)}<span class="tcp-u">${t.unit}</span></div>
        </div>`
    ).join('');
}

function initChart() {
    const ctx = document.getElementById('aTorqueChart').getContext('2d');
    torqueChart = new Chart(ctx, {
        type: 'bar',
        data: {
            labels: ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'],
            datasets: [{
                label: 'Torque (Nm)',
                data: [0, 0, 0, 0, 0, 0],
                backgroundColor: 'rgba(37,99,235,0.12)',
                borderColor: '#2563eb',
                borderWidth: 1.5,
                borderRadius: 3,
            }]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            plugins: { legend: { display: false } },
            scales: {
                x: { ticks: { color: '#94a3b8', font: { size: 10 } }, grid: { color: '#f1f5f9' } },
                y: { ticks: { color: '#94a3b8', font: { size: 10 } }, grid: { color: '#f1f5f9' }, beginAtZero: true, max: 35 }
            }
        }
    });
}

async function loadRobot() {
    if (!selectedStationId) return;
    try {
        const [jRes, tRes] = await Promise.all([
            fetch(`/api/robot/joints?${SQ()}`),
            fetch(`/api/robot/tcp?${SQ()}`),
        ]);
        const jData = await jRes.json();
        const tData = await tRes.json();

        if (jData.joints && jData.joints.length > 0) {
            renderJoints(jData.joints);
            if (torqueChart) {
                torqueChart.data.datasets[0].data = jData.joints.map(j =>
                    (j.torque === null || j.torque === undefined) ? 0 : j.torque
                );
                torqueChart.update('none');
            }
        }

        if (tData.tcp && Object.keys(tData.tcp).length > 0) {
            renderTcp(tData.tcp);
        }
    } catch (e) { /* silent */ }
}

// ── 카메라 프레임 (1초) ──
function loadFrame() {
    if (!selectedStationId) return;
    const img = document.getElementById('aVidImg');
    const ph = document.getElementById('aVidPlaceholder');
    const newImg = new Image();
    newImg.onload = function () {
        img.src = newImg.src;
        img.style.display = 'block';
        if (ph) ph.style.display = 'none';
    };
    newImg.src = `/api/vision/frame/${selectedStationId}?t=` + Date.now();
}

// ── 비전 인식 (2초) ──
async function loadVision() {
    if (!selectedStationId) return;
    try {
        const res = await fetch(`/api/vision/current?${SQ()}`);
        const d = await res.json();

        if (d.label) {
            const confPct = (d.confidence * 100).toFixed(1);
            document.getElementById('vBboxTag').textContent = `${d.label} ${confPct}%`;
            document.getElementById('vDepth').textContent = `YOLO v8 · D: ${d.depth ? d.depth.toFixed(2) : '--'} m`;

            const confEl = document.getElementById('aConf');
            confEl.textContent = confPct + ' %';
            confEl.className = 'err-val ' + (d.confidence >= 0.90 ? 'err-ok' : 'err-warn');

            const angleEl = document.getElementById('aAngle');
            if (d.handle_angle !== null && d.handle_angle !== undefined) {
                angleEl.textContent = d.handle_angle.toFixed(1) + ' °';
                angleEl.style.color = '#1e40af';
            } else {
                angleEl.textContent = '-- °';
                angleEl.style.color = '#64748b';
            }
        }
    } catch (e) { /* silent */ }
}

// ── 시스템 로그 (3초) ──
const LV_MAP = { 'OK': 'ok', 'INFO': 'info', 'WARN': 'warn', 'WARNING': 'warn', 'ERROR': 'err' };

async function loadLogs() {
    if (!selectedStationId) return;
    try {
        const res = await fetch(`/api/logs?${SQ()}&limit=30`);
        const rows = await res.json();
        const box = document.getElementById('aLog');

        box.innerHTML = rows.map(r => {
            const ts = r.timestamp ? r.timestamp.split('T').pop().split('.')[0] : '';
            const lvCls = LV_MAP[r.level] || 'info';
            return `<div class="log-entry">
                <span class="log-ts">${ts}</span>
                <span class="log-lv lv-${lvCls}">${r.level}</span>
                <span class="log-msg">${r.message}</span>
            </div>`;
        }).join('') || '<div style="color:#94a3b8;font-size:11px;">로그 없음</div>';

        box.scrollTop = box.scrollHeight;
    } catch (e) { /* silent */ }
}

// ── 초기화 + polling ──
loadBranches();

setInterval(loadFrame, 1000);
setInterval(loadRobot, 1500);
setInterval(loadVision, 2000);
setInterval(loadStatus, 3000);
setInterval(loadLogs, 3000);
