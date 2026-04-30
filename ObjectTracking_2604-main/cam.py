import cv2
import numpy as np
import os

# --- 설정 ---
CALIB_FILE = "calibration.npz"
CHESS_W, CHESS_H = 9, 6  # 본인의 체스판 코너 개수
objp = np.zeros((CHESS_W * CHESS_H, 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESS_W, 0:CHESS_H].T.reshape(-1, 2)

objpoints, imgpoints = [], []
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

# --- [1단계] 사진 촬영 ---
print("\n📸 [1단계] 왜곡 보정용 촬영을 시작합니다.")
print("- 체스판을 비추고 'S' 키를 눌러 저장하세요 (20장 권장).")
print("- 다 찍었으면 'Q'를 눌러 계산을 시작하세요.")

count = 0
while True:
    ret, frame = cap.read()
    if not ret: break
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret_c, corners = cv2.findChessboardCorners(gray, (CHESS_W, CHESS_H), None)
    
    display = frame.copy()
    if ret_c:
        cv2.drawChessboardCorners(display, (CHESS_W, CHESS_H), corners, ret_c)
        
    cv2.putText(display, f"Captured: {count}/20", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Step 1: Calibration", display)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('s') and ret_c:
        objpoints.append(objp)
        imgpoints.append(corners)
        count += 1
        print(f"✅ {count}번째 사진 저장 성공!")
    elif key == ord('q'):
        if count < 10:
            print("⚠️ 사진이 너무 적습니다. 최소 10장 이상 찍어주세요.")
            continue
        break

# --- 보정 계수 계산 ---
print("\n" + "="*50)
print("⏳ 왜곡 보정 계수 계산 중... (약 10~20초 소요)")
print("⚠️ '응답 없음'이 떠도 창을 끄지 말고 잠시만 기다려주세요!")
print("="*50)

ret, mtx, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

if ret:
    print("\n🎉 왜곡 보정 데이터 생성 완료!")
    print("📢 [안내] 이제 바닥에 30cm 자를 똑바로 놓아주세요.")
    print("📢 준비가 되었으면 아무 키나 눌러 측정을 시작합니다...")
    cv2.waitKey(0) # 사용자가 준비될 때까지 대기
else:
    print("❌ 보정에 실패했습니다. 사진을 다시 찍어주세요.")
    cap.release()
    cv2.destroyAllWindows()
    exit()

# --- [2단계] 30cm 측정 ---
points = []
new_p2c = 0.0

def click_event(event, x, y, flags, param):
    global points, img_undistorted, new_p2c
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))
        cv2.circle(img_undistorted, (x, y), 6, (0, 0, 255), -1)
        cv2.imshow("Step 2: Distance Measurement", img_undistorted)
        
        if len(points) == 2:
            px_dist = np.sqrt((points[1][0]-points[0][0])**2 + (points[1][1]-points[0][1])**2)
            new_p2c = 30.0 / px_dist
            
            # 최종 결과 출력 및 저장
            print("\n" + "*"*40)
            print(f"🎯 측정 성공! Pixel to CM: {new_p2c:.8f}")
            print("*"*40)
            
            np.savez(CALIB_FILE, mtx=mtx, dist=dist, pixel_to_cm=new_p2c)
            print(f"💾 '{CALIB_FILE}'에 모든 정보가 안전하게 저장되었습니다.")
            print("이제 ESC를 눌러 종료하고 프로젝트를 진행하세요!")

cv2.destroyWindow("Step 1: Calibration")

while True:
    ret, frame = cap.read()
    if not ret: break
    
    # 왜곡 보정 적용 (alpha=0으로 깔끔하게)
    h, w = frame.shape[:2]
    new_mtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 0, (w, h))
    img_undistorted = cv2.undistort(frame, mtx, dist, None, new_mtx)
    
    cv2.putText(img_undistorted, "CLICK 0cm AND 30cm POINTS", (30, 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    if len(points) == 2:
        cv2.putText(img_undistorted, f"Result P2C: {new_p2c:.6f}", (30, 110), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)

    cv2.imshow("Step 2: Distance Measurement", img_undistorted)
    cv2.setMouseCallback("Step 2: Distance Measurement", click_event)
    
    if cv2.waitKey(1) == 27: # ESC 키
        break

cap.release()
cv2.destroyAllWindows()