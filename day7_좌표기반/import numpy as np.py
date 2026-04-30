import numpy as np
import os  # <--- 이 줄이 빠져서 에러가 났던 거예요!

# 파일 로드
CALIB_FILE = "calibration.npz"

if os.path.exists(CALIB_FILE):
    # mmap_mode=None을 사용하여 안전하게 로드
    data = dict(np.load(CALIB_FILE, allow_pickle=True))
    
    print(f"--- {CALIB_FILE} 저장 데이터 목록 ---")
    for key in data:
        print(f"\n[{key}]")
        print(data[key])
    
    # 별도로 변수에 담아 확인하고 싶을 때
    if 'pixel_to_cm' in data:
        print(f"\n📍 최종 확인된 거리 계수(P2C): {data['pixel_to_cm']}")
else:
    print(f"❌ {CALIB_FILE} 파일이 존재하지 않습니다. 먼저 setup_camera.py를 실행하세요.")