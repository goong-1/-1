console.log("[main.js] loaded");

window.socket = window.socket || io();

window.socket.on("connect", () => {
    console.log("[socket] connected:", window.socket.id);
    addLog("SYSTEM", "Socket connected", "info");
});

window.socket.on("connect_error", (err) => {
    console.error("[socket] connect_error:", err);
});
/**
 * 관제 시스템 메인 클라이언트 스크립트
 * 기능: 소켓 통신, 로봇별 수동 제어(WASD/방향키), 실시간 로그 및 데이터 시각화
 */

const socket = io(); // 서버와 실시간 양방향 통신을 위한 Socket.IO 객체
window.robotState = { x: 0, y: 0, step: 25 }; // 시각화용 가상 좌표 (필요 시 확장용)
const MANUAL_ROBOT_ID = 0; // 수동 제어 기본 대상은 로봇 0으로 설정

/**
 * [함수: syncCalibration]
 * 의미: UI에서 변경된 보정값(계수)을 서버에 즉시 전송하여 저장
 */
const syncCalibration = (robotId) => {
    // 해당 로봇 ID에 맞는 input 엘리먼트에서 값을 가져옴
    const factor = document.getElementById(`calib-${robotId}`)?.value || "1.0";
    socket.emit('update_calibration', { id: robotId, factor: factor });
};

/**
 * [대표 함수: moveRobot]
 * 상세: 로봇 0(타겟 로봇)의 주행 명령을 생성하고 서버로 전송
 * 특징: PWM 베이스 값에 보정 계수를 곱하여 좌우 모터의 균형을 맞춤
 */
window.moveRobot = (direction) => {
    console.log("[moveRobot] direction:", direction);
    // 1. 설정값 로드: 기본 속도(PWM) 및 보정 계수
    const basePwm = parseInt(document.getElementById('manual-pwm')?.value || "85");
    const factor0 = parseFloat(document.getElementById('calib-0')?.value || "1.0");
    
    // 2. 보정 계산: 오른쪽 모터(d)에만 계수를 곱해 직진성 확보
    const rightPwm = Math.round(basePwm * factor0);
    const halfBasePwm = Math.floor(basePwm / 2); // 회전 시 안쪽 바퀴 감속용
    const halfRightPwm = Math.round(halfBasePwm * factor0);
    
    let key = '';
    // 3. 방향별 명령 조합 (a: 왼쪽 모터, d: 오른쪽 모터)
    switch(direction) {
        case 'up':    key = `a+${basePwm},d+${rightPwm}`; break;
        case 'down':  key = `a-${basePwm},d-${rightPwm}`; break;
        case 'left':  key = `a+${halfBasePwm},d+${rightPwm}`; break; // 좌완만 감속
        case 'right': key = `a+${basePwm},d+${halfRightPwm}`; break; // 우완만 감속
        case 'turnL': key = `a-${basePwm},d+${rightPwm}`; break;     // 제자리 좌회전
        case 'turnR': key = `a+${basePwm},d-${rightPwm}`; break;     // 제자리 우회전
        case 'backLeft': key = `a-${halfBasePwm},d-${rightPwm}`; break; // 궁:후진하면서 좌회전
        case 'backRight': key = `a-${basePwm},d-${halfRightPwm}`; break; // 궁:후진하면서 우회전
        case 'stop':  key = 'a+0,d+0'; break;
    }

    console.log("[moveRobot] command:", key);

    if (key) {
        // 서버의 'drive_control' 이벤트로 로봇 ID와 명령 전송
        socket.emit('drive_control', { id: MANUAL_ROBOT_ID, command: key + '\n' });
        addLog('Control', key, 'info'); 
    }
};

/**
 * [추가 함수: sendEmergencyCmd]
 * 의미: 로봇 1(추종 로봇)의 자동 주행을 일시 중단하고 사용자가 직접 제어
 * 특징: 서버의 'emergency_control_robot1' 이벤트를 트리거하여 자동 로직보다 우선순위를 가짐
 */
function sendEmergencyCmd(command) {
    const pwmVal = document.getElementById('manual-pwm')?.value || 85;
    
    socket.emit('emergency_control_robot1', { 
        command: command,
        pwm: pwmVal
    });
    
    if (command === "stop") {
        addLog('Emergency', 'Robot 1 STOP', 'error');
    }
}

/**
 * [이벤트 초기화: DOMContentLoaded]
 * 상세: 페이지 로드 완료 후 각종 리스너(D-Pad, 키보드, 소켓 수신) 등록
 */
document.addEventListener('DOMContentLoaded', () => {
    
    // 보정값 변경 시 서버와 자동 동기화 설정
    document.getElementById('calib-0')?.addEventListener('change', () => syncCalibration(0));
    document.getElementById('calib-1')?.addEventListener('change', () => syncCalibration(1));
    // 초기 로딩 1초 후 현재 보정값을 서버에 한 번 찌러줌
    setTimeout(() => { syncCalibration(0); syncCalibration(1); }, 1000);

    let isMoving = false; // 키 중복 입력 방지용 플래그
    const dpad = document.querySelector('.d-pad');
    
    // --- 화면 UI(D-Pad) 버튼 제어 로직 ---
    if (dpad) {
        const handleStart = (e) => {
            if (isMoving) return;
            const btn = e.target.closest('button, .d-btn, .btn-center, .circle-btn, .turn-l, .turn-r');
            if (!btn) return;
            e.preventDefault();
            
            let dir = '';
            if (btn.classList.contains('btn-up')) dir = 'up';
            else if (btn.classList.contains('btn-down')) dir = 'down';
            else if (btn.classList.contains('btn-left')) dir = 'left';
            else if (btn.classList.contains('btn-right')) dir = 'right';
            else if (btn.classList.contains('btn-center')) dir = 'stop';
            else if (btn.classList.contains('turn-l')) dir = 'turnL';
            else if (btn.classList.contains('turn-r')) dir = 'turnR';
            
            if (dir) { isMoving = true; window.moveRobot(dir); }
        };
        // 버튼에서 손을 떼거나 영역을 벗어나면 정지
        const handleEnd = (e) => { if (isMoving) { isMoving = false; window.moveRobot('stop'); } };
        
        dpad.onmousedown = handleStart; dpad.onmouseup = handleEnd; dpad.onmouseleave = handleEnd;
        dpad.ontouchstart = handleStart; dpad.ontouchend = handleEnd;
    }

    /**
     * [키보드 통합 관리: onkeydown / onkeyup]
     * 1. WASD: 로봇 0 제어
     * 2. 방향키: 로봇 1 비상 제어 (자동 추적 강제 중단)
     * 3. Space: 전체 정지
     */
        /**
     * [키보드 상태 기반 제어]
     * keydown/keyup 단발 이벤트가 아니라 현재 눌려 있는 키 조합을 기준으로 명령을 계산한다.
     */
    const pressedKeys = new Set();
    let lastRobot0Dir = null;
    let lastRobot1Dir = null;

    function getRobot0Direction() {
        const w = pressedKeys.has('KeyW');
        const a = pressedKeys.has('KeyA');
        const s = pressedKeys.has('KeyS');
        const d = pressedKeys.has('KeyD');

        // 반대 방향 동시 입력
        if (w && s) return 'stop';
        if (a && d) {
            if (w) return 'up';
            if (s) return 'down';
            return 'stop';
        }

        // 조합 입력
        if (w && a) return 'left';
        if (w && d) return 'right';

        if (s && a) return 'backLeft';
        if (s && d) return 'backRight';

        // 단일 입력
        if (w) return 'up';
        if (s) return 'down';
        if (a) return 'left';
        if (d) return 'right';

        return 'stop';
    }

    function getRobot1Direction() {
        const up = pressedKeys.has('ArrowUp');
        const down = pressedKeys.has('ArrowDown');
        const left = pressedKeys.has('ArrowLeft');
        const right = pressedKeys.has('ArrowRight');

        if (up && down) return 'stop';
        if (left && right) {
            if (up) return 'up';
            if (down) return 'down';
            return 'stop';
        }

        if (up && left) return 'left';
        if (up && right) return 'right';

        if (down && left) return 'backLeft'; // 궁 추가
        if (down && right) return 'backRight'; // 궁 추가

        if (up) return 'up';
        if (down) return 'down';
        if (left) return 'left';
        if (right) return 'right';

        return 'stop';
    }

    function updateRobot0FromKeys() {
        const dir = getRobot0Direction();

        // 같은 명령 반복 전송 방지
        if (dir === lastRobot0Dir) return;

        lastRobot0Dir = dir;
        window.moveRobot(dir);
    }

    function updateRobot1FromKeys() {
        const dir = getRobot1Direction();

        // 같은 명령 반복 전송 방지
        if (dir === lastRobot1Dir) return;

        lastRobot1Dir = dir;
        sendEmergencyCmd(dir);
    }

    document.onkeydown = (e) => {
        if (e.target.tagName === 'INPUT') return;

        const controlKeys = [
            'KeyW', 'KeyA', 'KeyS', 'KeyD',
            'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight',
            'Space'
        ];

        if (!controlKeys.includes(e.code)) return;

        e.preventDefault();

        if (e.code === 'Space') {
            pressedKeys.clear();
            lastRobot0Dir = null;
            lastRobot1Dir = null;

            window.moveRobot('stop');
            sendEmergencyCmd('stop');
            return;
        }

        pressedKeys.add(e.code);

        // WASD → 로봇 0
        if (['KeyW', 'KeyA', 'KeyS', 'KeyD'].includes(e.code)) {
            updateRobot0FromKeys();
        }

        // 방향키 → 로봇 1
        if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.code)) {
            updateRobot1FromKeys();
        }
    };

    document.onkeyup = (e) => {
        const controlKeys = [
            'KeyW', 'KeyA', 'KeyS', 'KeyD',
            'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'
        ];

        if (!controlKeys.includes(e.code)) return;

        e.preventDefault();
        pressedKeys.delete(e.code);

        // WASD → 로봇 0
        if (['KeyW', 'KeyA', 'KeyS', 'KeyD'].includes(e.code)) {
            updateRobot0FromKeys();
        }

        // 방향키 → 로봇 1
        if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.code)) {
            updateRobot1FromKeys();
        }
    };

    // 브라우저 창 포커스를 잃으면 안전 정지
    window.addEventListener('blur', () => {
        pressedKeys.clear();
        lastRobot0Dir = null;
        lastRobot1Dir = null;

        window.moveRobot('stop');
        sendEmergencyCmd('stop');
    });

    // --- 실시간 수신 리스너 ---
    // vision_data: 서버에서 계산한 거리/각도를 UI 텍스트에 반영
    socket.on('vision_data', (data) => {
        document.getElementById('ui-dist').textContent = data.dist.toFixed(1);
        document.getElementById('ui-angle').textContent = data.angle.toFixed(1);
    });
    // log: 서버에서 발생하는 모든 로그(자동/수동/시스템) 수신
    socket.on('log', (data) => addLog(data.type, data.msg, data.status === 'success' ? 'info' : 'error'));
});

/**
 * [함수: addLog]
 * 상세: 웹 관제 UI의 로그 테이블에 새로운 로그 추가
 * 특징: 
 * - afterbegin을 사용해 최신 로그가 무조건 테이블 맨 위에 나타나게 함
 * - 최대 500개까지만 유지하여 브라우저 메모리 관리
 */
function addLog(activity, message, type = 'info') {
    const tbody = document.getElementById('log-tbody');
    if (!tbody) return;

    const time = new Date().toLocaleTimeString('ko-KR', { hour12: false });
    
    // 로그의 성격(성공/에러)에 따라 행의 스타일(CSS 클래스) 결정
    const row = `<tr ${type === 'error' ? 'class="log-error"' : ''}>
                    <td>${time}</td>
                    <td>${activity}</td>
                    <td><b>${message}</b></td>
                    <td>${type === 'error' ? '❌' : '✅'}</td>
                </tr>`;

    // 최신 로그를 테이블 본문(tbody)의 가장 첫 번째 자식으로 삽입
    tbody.insertAdjacentHTML('afterbegin', row);

    // 로그 개수 제한 (500개 초과 시 가장 아래에 있는 오래된 행 삭제)
    if (tbody.rows.length > 500) {
        tbody.deleteRow(tbody.rows.length - 1); 
    }
}
