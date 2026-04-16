// ── Kiosk API 연동 (station_id 기반) ──

const SQ = `station_id=${STATION_ID}`;

let selectedFuel = 'gasoline';
let selectedMode = 'amount';

// 스테이션 라벨 표시
document.getElementById('kStationLabel').textContent =
    `B${BRANCH_ID} · S${STATION_NO}`;

// ── UI 인터랙션 ──
function pickFuel(el) {
    document.querySelectorAll('.fuel-opt').forEach(b => b.classList.remove('active'));
    el.classList.add('active');
    selectedFuel = el.dataset.fuel;
}

function pickMode(el) {
    document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
    el.classList.add('active');
    selectedMode = el.dataset.mode;
    document.getElementById('kAmtInput').value = selectedMode === 'amount' ? '50,000' : '30.0';
}

// ── 로그아웃 ──
async function doLogout() {
    await fetch('/api/auth/logout', { method: 'POST' });
    window.location.href = '/login';
}

// ── 주유 시작 ──
async function doStart() {
    const rawVal = document.getElementById('kAmtInput').value.replace(/,/g, '');
    const targetValue = parseFloat(rawVal);
    if (isNaN(targetValue) || targetValue <= 0) return;

    const btn = document.getElementById('kBtnStart');
    btn.disabled = true;
    btn.textContent = '주유 진행 중...';

    try {
        const res = await fetch('/api/task/start', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                station_id: STATION_ID,
                fuel_type: selectedFuel,
                input_mode: selectedMode,
                target_value: targetValue,
            }),
        });
        const data = await res.json();
        if (data.ok) {
            document.getElementById('kGuide').textContent =
                '주유가 시작되었습니다 (Task #' + data.task_id + ')';
        }
    } catch (e) {
        btn.disabled = false;
        btn.textContent = '주유 시작 →';
    }
}

// ── 비상 정지 ──
async function doEstop() {
    try {
        await fetch('/api/task/estop', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ station_id: STATION_ID }),
        });
        const btn = document.querySelector('.btn-estop');
        btn.textContent = '⚠ 비상 정지 실행됨 — 로봇 즉시 정지';
        btn.style.background = '#7f1d1d';

        document.getElementById('kBtnStart').disabled = false;
        document.getElementById('kBtnStart').textContent = '주유 시작 →';

        document.querySelectorAll('.step-item').forEach(s => s.style.opacity = '0.3');
    } catch (e) { /* silent */ }
}

// ── 시스템 상태 (3초) ──
async function loadStatus() {
    try {
        const res = await fetch(`/api/system/status?${SQ}`);
        const d = await res.json();

        const el = document.getElementById('kStatus');
        const txt = document.getElementById('kStatusText');

        if (d.robot_mode && d.robot_mode !== 'OFFLINE') {
            el.className = 'k-status';
            txt.textContent = '시스템 정상 운행 중';
        } else {
            el.className = 'k-status offline';
            txt.textContent = '시스템 오프라인';
        }
    } catch (e) { /* silent */ }
}

// ── 카메라 프레임 (1초) ──
function loadFrame() {
    const img = document.getElementById('kCamImg');
    const ph = document.getElementById('kCamPlaceholder');
    const newImg = new Image();
    newImg.onload = function () {
        img.src = newImg.src;
        img.style.display = 'block';
        if (ph) ph.style.display = 'none';
    };
    newImg.src = `/api/vision/frame/${STATION_ID}?t=` + Date.now();
}

// ── 프레임 저장 ──
async function doCapture() {
    try {
        const res = await fetch(`/api/vision/capture/${STATION_ID}`, { method: 'POST' });
        const d = await res.json();
        if (d.ok) {
            document.getElementById('kGuide').textContent = '프레임 저장 완료: ' + d.filename;
        } else {
            document.getElementById('kGuide').textContent = d.error || '저장 실패';
        }
    } catch (e) { /* silent */ }
}

// ── 비전 인식 (2초) ──
async function loadVision() {
    try {
        const res = await fetch(`/api/vision/current?${SQ}`);
        const d = await res.json();

        if (d.label) {
            const confPct = (d.confidence * 100).toFixed(1);
            document.getElementById('kCamInfo').textContent =
                `RealSense D455 · 30fps · D:${d.depth ? d.depth.toFixed(2) : '--'}m`;
            document.getElementById('kCamBadge').textContent = `${d.label} ${confPct}%`;
        }
    } catch (e) { /* silent */ }
}

// ── 작업 현황 + 메트릭 (1초) ──
const STEP_SUBS = {
    0: ['대기 중', '대기 중', '대기 중', '대기 중'],
    1: ['비전 인식 중...', '대기 중', '대기 중', '대기 중'],
    2: ['비전 인식 완료 · 좌표 확정', 'TCP → 목표 좌표 이동 중...', '대기 중', '대기 중'],
    3: ['비전 인식 완료 · 좌표 확정', '로봇암 접근 완료', '주유 노즐 삽입 · 주유 중...', '대기 중'],
    4: ['비전 인식 완료 · 좌표 확정', '로봇암 접근 완료', '주유 완료', '로봇 복귀 중...'],
};

const GUIDE_MSGS = {
    0: '시스템 준비 중...',
    1: '주유구를 탐색하고 있습니다',
    2: '주유구 위치 확인 완료 — 로봇암 접근 준비 중',
    3: '주유가 진행되고 있습니다',
    4: '주유 완료 — 로봇이 원위치로 복귀 중입니다',
};

async function loadTask() {
    try {
        const [taskRes, metricRes] = await Promise.all([
            fetch(`/api/task/current?${SQ}`),
            fetch(`/api/task/metrics?${SQ}`),
        ]);
        const task = await taskRes.json();
        const metric = await metricRes.json();

        if (!task.id) {
            document.getElementById('kBtnStart').disabled = false;
            document.getElementById('kBtnStart').textContent = '주유 시작 →';
            return;
        }

        // Step 표시
        const step = task.current_step || 0;
        const items = document.querySelectorAll('.step-item');
        const subs = STEP_SUBS[step] || STEP_SUBS[0];

        items.forEach((item, i) => {
            const s = i + 1;
            const subEl = item.querySelector('.step-sub');
            subEl.textContent = subs[i];

            if (s < step) {
                item.className = 'step-item done';
                item.querySelector('.step-num').textContent = '✓';
            } else if (s === step) {
                item.className = 'step-item active';
                item.querySelector('.step-num').textContent = String(s);
            } else {
                item.className = 'step-item';
                item.querySelector('.step-num').textContent = String(s);
            }
        });

        // Progress bar (step 2)
        const prog = document.getElementById('kStepProgress');
        if (step === 2) {
            prog.style.width = ((task.step_progress || 0) * 100) + '%';
        } else if (step > 2) {
            prog.style.width = '100%';
        } else {
            prog.style.width = '0%';
        }

        // 가이드
        document.getElementById('kGuide').textContent = GUIDE_MSGS[step] || '';

        // 메트릭
        if (metric.liters !== undefined) {
            document.getElementById('kLiters').innerHTML =
                metric.liters.toFixed(1) + '<span class="metric-tile-unit"> L</span>';
            document.getElementById('kMoney').innerHTML =
                metric.cost.toLocaleString() + '<span class="metric-tile-unit"> ₩</span>';

            const em = String(Math.floor(metric.elapsed_sec / 60)).padStart(2, '0');
            const es = String(metric.elapsed_sec % 60).padStart(2, '0');
            document.getElementById('kTime').textContent = em + ':' + es;

            const rm = String(Math.floor(metric.remaining_sec / 60)).padStart(2, '0');
            const rs = String(metric.remaining_sec % 60).padStart(2, '0');
            document.getElementById('kRem').textContent = rm + ':' + rs;
        }

        // 버튼 상태
        if (task.status === 'running') {
            document.getElementById('kBtnStart').disabled = true;
            document.getElementById('kBtnStart').textContent = '주유 진행 중...';
        }

        // 완료/실패 처리
        if (task.status === 'success' || task.status === 'fail' || task.status === 'estop') {
            document.getElementById('kBtnStart').disabled = false;
            document.getElementById('kBtnStart').textContent = '주유 시작 →';
        }
    } catch (e) { /* silent */ }
}

// ── 로그 (3초) ──
const LV_MAP = { 'OK': 'ok', 'INFO': 'info', 'WARN': 'warn', 'WARNING': 'warn', 'ERROR': 'err' };

async function loadLogs() {
    try {
        const res = await fetch(`/api/logs?source=kiosk&${SQ}&limit=10`);
        const rows = await res.json();
        const box = document.getElementById('kSyslog');

        box.innerHTML = rows.map(r => {
            const lvCls = LV_MAP[r.level] || 'info';
            return `<div class="syslog-line">
                <span class="tag tag-${lvCls}">${r.level}</span>
                <span>${r.message}</span>
            </div>`;
        }).join('') || '<div>로그 없음</div>';
    } catch (e) { /* silent */ }
}

// ── 초기화 + polling ──
loadStatus(); loadFrame(); loadVision(); loadTask(); loadLogs();

setInterval(loadFrame, 1000);
setInterval(loadTask, 1000);
setInterval(loadVision, 2000);
setInterval(loadStatus, 3000);
setInterval(loadLogs, 3000);
