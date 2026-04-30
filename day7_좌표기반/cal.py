import numpy as np
import cv2
import math
import os

# --- 설정 ---
CALIB_FILE = "calibration.npz"
CHECKERBOARD = (6, 9)  # 본인의 체스판 내부 교차점 개수
REAL_DIST_CM = 30.0    # 사용할 자의 실제 길이
WIDTH, HEIGHT = 960, 540

def run_setup():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

    # [PART 1] 체스판 왜곡 보정 단계
    print("\n📸 [STEP 1] 왜곡 보정 단계")
    print("- 체스판에 무지개색 선이 생기면 'S'를 누르세요.")
    print("- 화면 구석구석에서 30장을 찍으세요. 다 찍으면 자동으로 다음 단계로 넘어갑니다.")

    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objpoints, imgpoints = [], []

    while len(objpoints) < 30:
        ret, frame = cap.read()
        if not ret: break
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret_c, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
        
        display = frame.copy()
        if ret_c:
            cv2.drawChessboardCorners(display, CHECKERBOARD, corners, ret_c)
        
        cv2.putText(display, f"Captured: {len(objpoints)}/30", (10, 40), 1, 2, (255, 0, 0), 2)
        cv2.imshow('Camera Setup - Step 1', display)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s') and ret_c:
            objpoints.append(objp)
            imgpoints.append(corners)
            print(f"카운트: {len(objpoints)}/30")
        elif key == ord('q'):
            print("중도 종료합니다.")
            cap.release(); cv2.destroyAllWindows(); return

    print("⌛ 왜곡 수치 계산 중... 잠시만 기다려 주세요.")
    ret, mtx, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, (WIDTH, HEIGHT), None, None)

    # [PART 2] 거리 계수(P2C) 설정 단계
    print("\n📏 [STEP 2] 거리 설정 단계")
    print("- 주행장 바닥에 30cm 자를 놓으세요.")
    print("- 자의 양 끝 지점 2곳을 마우스로 클릭하세요.")
    print("- 초록색 선이 생기면 'S'를 눌러 최종 저장하세요.")

    clicked = []
    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked.append((x, y))

    cv2.destroyWindow('Camera Setup - Step 1')
    cv2.namedWindow('Camera Setup - Step 2')
    cv2.setMouseCallback('Camera Setup - Step 2', mouse_callback)

    p2c_final = 0.15 # 기본값
    while True:
        ret, frame = cap.read()
        if not ret: break

        # STEP 1에서 구한 mtx, dist로 화면 보정
        new_mtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (WIDTH, HEIGHT), 1, (WIDTH, HEIGHT))
        undistorted = cv2.undistort(frame, mtx, dist, None, new_mtx)

        for p in clicked:
            cv2.circle(undistorted, p, 5, (0, 0, 255), -1)

        if len(clicked) >= 2:
            p1, p2 = clicked[0], clicked[1]
            cv2.line(undistorted, p1, p2, (0, 255, 0), 2)
            px_dist = math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
            p2c_final = REAL_DIST_CM / px_dist
            cv2.putText(undistorted, f"P2C: {p2c_final:.6f} | Press 'S' to SAVE", (10, 50), 1, 2, (0, 255, 0), 2)

        cv2.imshow('Camera Setup - Step 2', undistorted)
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('s') and len(clicked) >= 2:
            np.savez(CALIB_FILE, mtx=mtx, dist=dist, pixel_to_cm=p2c_final)
            print(f"\n✅ 모든 설정이 완료되었습니다!")
            print(f"저장된 파일: {CALIB_FILE}")
            break
        elif key == ord('r'):
            clicked = []
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run_setup()