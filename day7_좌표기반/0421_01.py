import cv2
import cv2.aruco as aruco
import math
import numpy as np

# --- 1. 설정 및 초기화 ---
# 본인이 선택한 딕셔너리에 맞게 수정 (예: DICT_4X4_50)
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
PARAMETERS = aruco.DetectorParameters()

# 실제 거리 계산을 위한 비율 (카메라 높이에 따라 조정 필요)
# 테스트 방법: 10cm가 화면에서 몇 픽셀인지 재서 (10 / 픽셀수) 입력
#PIXEL_TO_CM = 0.1  
PIXEL_TO_CM = 0.1275    # NEED TO FIX

cap = cv2.VideoCapture(0)

def get_marker_info(corners, id_list):
    """모든 검출된 마크의 중심점과 방향(각도)을 딕셔너리로 반환"""
    info = {}
    for i, m_id in enumerate(id_list.flatten()):
        c = corners[i][0] # [[tl, tr, br, bl]]
        
        # 중심점 계산
        cX = int(np.mean(c[:, 0]))
        cY = int(np.mean(c[:, 1]))
        
        # 로봇의 현재 Heading(방향) 계산 (위쪽 변 중간과 아래쪽 변 중간 활용)
        top_mid = (c[0] + c[1]) / 2
        bottom_mid = (c[2] + c[3]) / 2
#        heading_rad = math.atan2(top_mid[1] - bottom_mid[1], top_mid[0] - bottom_mid[0])
        heading_rad = math.atan2(bottom_mid[1] - top_mid[1], bottom_mid[0] - top_mid[0])
        heading_deg = math.degrees(heading_rad)
        
        info[m_id] = {'center': (cX, cY), 'heading': heading_deg}
    return info

print("정지하려면 'q'를 누르세요...")

with np.load("calibration.npz") as data:
    mtx = data['mtx']
    dist = data['dist']
    
while True:
    ret, frame = cap.read()
    if not ret: break

    # --- 왜곡 보정(Undistort) 적용 ---
    h, w = frame.shape[:2]
    new_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    frame = cv2.undistort(frame, mtx, dist, None, new_mtx)
    # ------------------------------

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(gray, ARUCO_DICT, parameters=PARAMETERS)

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        markers = get_marker_info(corners, ids)

        # 주행 로봇(ID:0)과 타겟(ID:1)이 모두 화면에 있을 때
        if 0 in markers and 1 in markers:
            r_pos = markers[0]['center']
            r_head = markers[0]['heading']
            t_pos = markers[1]['center']

            # 1. 타겟까지의 거리 (cm)
            dist_px = math.sqrt((t_pos[0] - r_pos[0])**2 + (t_pos[1] - r_pos[1])**2)
            dist_cm = dist_px * PIXEL_TO_CM

            # 2. 주행 로봇이 타겟을 바라보기 위한 '목표 각도'
            target_rad = math.atan2(t_pos[1] - r_pos[1], t_pos[0] - r_pos[0])
            target_deg = math.degrees(target_rad)

            # 3. 회전 오차 (Error Angle)
            # 내가 지금 보는 방향과 타겟 방향의 차이
            error_angle = target_deg - r_head
            
            # 각도 범위를 -180 ~ 180 사이로 정규화 (가까운 방향으로 돌게 함)
            if error_angle > 180: error_angle -= 360
            if error_angle < -180: error_angle += 360

            # 화면 출력용 텍스트 및 선
            cv2.line(frame, r_pos, t_pos, (255, 0, 0), 2)
            cv2.putText(frame, f"Dist: {dist_cm:.1f}cm", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Angle Error: {error_angle:.1f}deg", (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # 이 값을 나중에 ESP32로 보냅니다.
            # print(f"CMD >> Dist: {dist_cm:.1f}, Turn: {error_angle:.1f}")

    cv2.imshow("Top-view Robot Tracking", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()