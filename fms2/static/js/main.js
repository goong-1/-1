const socket = io();

// 1. 로봇 전역 상태 관리 (위치 및 화면 표시용 가짜 속도)
window.robotState = {
    x: 0, y: 0,
    step: 25,
    currentSpeed: 0.0 
};

document.addEventListener('DOMContentLoaded', () => {
    const robotMarker = document.getElementById('robot-1');
    const mapContainer = document.querySelector('.map-zone');

    // 화면에 로봇 위치 반영
    const updatePosition = () => {
        if (!robotMarker) return;
        robotMarker.style.left = `${window.robotState.x}px`;
        robotMarker.style.top = `${window.robotState.y}px`;
    };

    // 로봇을 지도의 정중앙으로 배치
    const setCenter = () => {
        if (!robotMarker || !mapContainer) return;
        window.robotState.x = (mapContainer.clientWidth / 2) - (robotMarker.clientWidth / 2);
        window.robotState.y = (mapContainer.clientHeight / 2) - (robotMarker.clientHeight / 2);
        updatePosition();
        addLog('System', 'CENTER_ALIGN', 'Admin');
    };

    // 2. 이동 함수: 조작 시에만 가짜 속도(currentSpeed) 발생
    window.moveRobot = (direction) => {
        // 1. 지도와 로봇의 현재 크기 파악
        const robotMarker = document.getElementById('robot-1');
        const mapContainer = document.querySelector('.map-zone');
        if (!robotMarker || !mapContainer) return;

        const mapW = mapContainer.clientWidth;
        const mapH = mapContainer.clientHeight;
        const robotW = robotMarker.clientWidth;
        const robotH = robotMarker.clientHeight;

        // 2. 속도 발생 (시각 효과)
        window.robotState.currentSpeed = parseFloat((Math.random() * 0.8 + 0.7).toFixed(2));
        
        // 3. 임시 좌표 계산 (이동하기 전 미리 계산)
        let nextX = window.robotState.x;
        let nextY = window.robotState.y;

        switch(direction) {
            case 'up':    nextY -= window.robotState.step; break;
            case 'down':  nextY += window.robotState.step; break;
            case 'left':  nextX -= window.robotState.step; break;
            case 'right': nextX += window.robotState.step; break;
            case 'stop':  window.robotState.currentSpeed = 0.0; break;
        }

        // 4. [핵심] 경계 검사 (Boundary Check)
        // 0보다 작아지거나, 지도 크기에서 로봇 크기를 뺀 값보다 커지지 않게 제한
        if (nextX >= 0 && nextX <= (mapW - robotW)) {
            window.robotState.x = nextX;
        } else {
            addLog('System', 'WALL_DETECTED', 'R1'); // 벽 충돌 로그
        }

        if (nextY >= 0 && nextY <= (mapH - robotH)) {
            window.robotState.y = nextY;
        } else {
            addLog('System', 'WALL_DETECTED', 'R1');
        }

        // 5. 실제 위치 반영 및 로그 출력
        updatePosition();
        addLog('ManualControl', `MOVE_${direction.toUpperCase()}`, 'User_1');

        setTimeout(() => { window.robotState.currentSpeed = 0.0; }, 400);

        // 🌟🌟 [여기서부터 새로 추가된 부분] 🌟🌟
        // 프론트엔드의 방향(up, down)을 파이썬/아두이노가 아는 숫자(1, 2, 3)로 번역
        let cmdCode = '3'; // 기본 정지
        if (direction === 'up') cmdCode = '1';       // 전진
        else if (direction === 'down') cmdCode = '2'; // 후진
        else if (direction === 'left') cmdCode = '4'; // 좌회전 (아두이노에 4번 코드 추가 시 작동)
        else if (direction === 'right') cmdCode = '5'; // 우회전 (아두이노에 5번 코드 추가 시 작동)

        // 파이썬 웹 서버의 API 창구로 주문서(POST) 날리기
        fetch('/update_position', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ direction: cmdCode }) // 번역된 숫자 명령을 전송
        })
        .then(response => response.json())
        .then(data => console.log('✅ 파이썬 서버 응답:', data))
        .catch(error => console.error('❌ 명령 전송 에러:', error));
        // 🌟🌟 [새로 추가된 부분 끝] 🌟🌟
        
    };

    // 3. 소켓 수신 (아두이노 조이스틱 연동)
    socket.on('serial_data', (data) => {
        const threshold = 300;
        if (data.joyY > 512 + threshold) moveRobot('up');
        else if (data.joyY < 512 - threshold) moveRobot('down');
        else if (data.joyX < 512 - threshold) moveRobot('left');
        else if (data.joyX > 512 + threshold) moveRobot('right');
    });

    socket.on('alert_message', function(data) {
    const logBox = document.getElementById('ai-detection-log');
    logBox.innerText = data.msg;
    logBox.style.color = '#ff4d4d';
    
    // 필요하다면 화면 전체를 붉게 깜빡이는 효과를 줄 수도 있습니다.
});

    // 4. 버튼 이벤트 바인딩 (footer.html의 클래스명과 일치시킴)
    // 4. 버튼 이벤트 바인딩 (명확하게 분리)
    const bindClick = (selector, direction) => {
        const btn = document.querySelector(selector);
        if (btn) {
            btn.onclick = (e) => {
                e.preventDefault(); // 중복 실행 방지
                moveRobot(direction);
            };
        }
    };

    // 방향키 연결
    bindClick('.btn-up', 'up');
    bindClick('.btn-down', 'down');
    bindClick('.btn-left', 'left');
    bindClick('.btn-right', 'right'); // 이제 'right'는 정상적으로 5번(또는 설정값)을 보냅니다.
    
    // 회전키 연결
    bindClick('.btn-turnL', 'left');
    bindClick('.btn-turnR', 'right');
    
    // 정지 버튼 (중앙 버튼) - 명확하게 딱 한 번만 정의
    const stopBtn = document.querySelector('.btn-center');
    if (stopBtn) {
        stopBtn.innerHTML = '🛑';
        stopBtn.style.cursor = 'pointer';
        stopBtn.onclick = () => moveRobot('stop'); // 오직 중앙 버튼만 'stop(3)'을 보냅니다.
    }
    // 초기 배치 실행
    setTimeout(setCenter, 100);
    
    // 🌟 스페이스바를 누르면 즉시 로봇이 정지하도록 키보드 이벤트 추가
    // 🌟 스페이스바 및 키보드 방향키 연동 이벤트
    document.addEventListener('keydown', (e) => {
        // 1. 스페이스바 (긴급 정지)
        if (e.code === 'Space') {
            e.preventDefault(); // 스페이스바 눌렀을 때 화면 스크롤 방지
            moveRobot('stop');
            console.log('🚨 스페이스바 긴급 정지 발동!');
        }
        // 2. 위쪽 화살표 (전진)
        else if (e.code === 'ArrowUp') {
            e.preventDefault(); // 방향키 스크롤 방지
            moveRobot('up');
        }
        // 3. 아래쪽 화살표 (후진)
        else if (e.code === 'ArrowDown') {
            e.preventDefault();
            moveRobot('down');
        }
        // 4. 왼쪽 화살표 (좌회전)
        else if (e.code === 'ArrowLeft') {
            e.preventDefault();
            moveRobot('left');
        }
        // 5. 오른쪽 화살표 (우회전)
        else if (e.code === 'ArrowRight') {
            e.preventDefault();
            moveRobot('right');
        }
    });
}); // <-- 기존에 있던 DOMContentLoaded 닫는 괄호

// 5. 실시간 로그 출력 함수
function addLog(scenario, activity, name) {
    const logTbody = document.getElementById('log-tbody');
    if (!logTbody) return;

    const now = new Date();
    const timeStr = now.getFullYear() + '.' + 
                    (now.getMonth()+1).toString().padStart(2, '0') + '.' + 
                    now.getDate().toString().padStart(2, '0') + ' ' + 
                    now.toLocaleTimeString('ko-KR', { hour12: false });

    const row = `
        <tr>
            <td style="font-size: 11px;">${timeStr}</td>
            <td>${scenario}</td>
            <td style="color: #3b82f6; font-weight: 600;">${activity}</td>
            <td>${name}</td>
            <td>✅</td>
        </tr>
    `;

    logTbody.insertAdjacentHTML('afterbegin', row);

    // 로그 개수 제한 (최신 10개)
    if (logTbody.rows.length > 10) {
        logTbody.deleteRow(-1);
    }
}