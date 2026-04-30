import numpy as np
import cv2
import glob

# 체스판의 가로, 세로 내부 코너 개수 (가로 9, 세로 6 예시)
CHECKERBOARD = (9, 6)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objpoints = [] # 실제 세계의 3D 점
imgpoints = [] # 이미지 상의 2D 점

# 3D 점 생성 (0,0,0), (1,0,0), (2,0,0) ...
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

cap = cv2.VideoCapture(0)
found_count = 0

print("체스판을 다양한 각도에서 비추고 's'를 눌러 캡처하세요. (20장 권장)")

while found_count < 20:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret == True:
        # 화면에 코너 그려주기
        cv2.drawChessboardCorners(frame, CHECKERBOARD, corners, ret)
        cv2.putText(frame, f"Saved: {found_count}/20", (10, 30), 1, 2, (0,255,0), 2)

    cv2.imshow('Calibration', frame)
    key = cv2.waitKey(1)
    if key == ord('s') and ret == True:
        objpoints.append(objp)
        imgpoints.append(corners)
        found_count += 1
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# 결과 계산 및 저장
if len(objpoints) > 0:
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    np.savez("calibration.npz", mtx=mtx, dist=dist)
    print("캘리브레이션 완료! 'calibration.npz' 저장됨.")