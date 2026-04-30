import cv2
import cv2.aruco as aruco
import numpy as np
import math

class VisionSystem:
    def __init__(self, calibration_file="calibration.npz"):
        
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
        self.parameters = aruco.DetectorParameters()
        
        # 1. 작은 마커도 인식하게 함 (해상도 높였을 때 필수)
        self.parameters.minMarkerPerimeterRate = 0.02  # 기본값 0.03에서 하향

        # 2. 노이즈 제거 및 경계선 강화
        self.parameters.adaptiveThreshConstant = 5      # 빛 반사 대응 (약간 낮춤)
        self.parameters.minMarkerDistanceRate = 0.02    # 붙어 있는 마커 인식용
        self.parameters.adaptiveThreshWinSizeStep = 4   # 정밀 탐색
        self.parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX # 떨림 방지

        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 23
        
        # 마커의 모서리를 서브픽셀 단위로 정밀하게 잡아내서 각도 떨림을 줄여줍니다.
        self.parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.parameters.cornerRefinementWinSize = 5
        self.parameters.cornerRefinementMinAccuracy = 0.1
        
        # 기본값 설정
        self.pixel_to_cm = 0.21474087760610036 
        self.mtx, self.dist_coeffs = None, None
        self.new_mtx = None # 미리 계산해둘 행렬
        
        self.H_img_to_map = None
        self.map_width_m = 1.77
        self.map_height_m = 1.11
        
        try:
            data = np.load(calibration_file)
            self.mtx = data['mtx']
            self.dist_coeffs = data['dist']
            
            if 'pixel_to_cm' in data:
                self.pixel_to_cm = float(data['pixel_to_cm'])
                print(f"✅ 캘리브레이션 로드 완료! (P2C: {self.pixel_to_cm:.6f})")
            
            # [최적화] 보정용 행렬을 여기서 미리 한 번만 계산합니다 (960x540 기준)
            h, w = 540, 960
            self.new_mtx, _ = cv2.getOptimalNewCameraMatrix(
                self.mtx, self.dist_coeffs, (w, h), 0.0, (w, h)
            )
        except Exception as e:
            print(f"⚠️ 캘리브레이션 파일 로드 오류: {e}")

    def process_frame(self, frame):
        # 1. 왜곡 보정 (행렬이 로드된 경우만)
        if self.mtx is not None and self.new_mtx is not None:
            frame = cv2.undistort(frame, self.mtx, self.dist_coeffs, None, self.new_mtx)

        # 2. ArUco 마커 인식
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
      
        markers = {}
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            for i, m_id in enumerate(ids.flatten()):
                c = corners[i][0]
                # 중심점 계산
                cX, cY = int(np.mean(c[:, 0])), int(np.mean(c[:, 1]))
                
                # 마커 자체의 헤딩 (개별 마커 방향 알림용)
                top_mid = (c[0] + c[1]) / 2
                bottom_mid = (c[2] + c[3]) / 2
                heading = math.degrees(math.atan2(bottom_mid[1] - top_mid[1], bottom_mid[0] - top_mid[0]))
                
                markers[int(m_id)] = {'center': (cX, cY), 'heading': heading}
        
        return frame, markers

    def get_robot_pose(self, markers, left_id, right_id):
        """두 마커를 조합하여 로봇의 중심 좌표와 벡터 기반 헤딩 계산"""
        if left_id in markers and right_id in markers:
            l = markers[left_id]['center']
            r = markers[right_id]['center']
            
            # 로봇 중심 (좌우 마커의 중간)
            cX = (l[0] + r[0]) // 2
            cY = (l[1] + r[1]) // 2
            
            # 좌->우 벡터 기반 각도 계산 (로봇이 보는 방향 정렬)
            dx = r[0] - l[0]
            dy = r[1] - l[1]
            angle_rad = math.atan2(dy, dx)
            
            # 로봇 정면 보정 (-90도)
            # % 360 - 180 처리는 각도를 -180 ~ 180도 범위로 유지해줍니다.
            heading = (math.degrees(angle_rad) - 90 + 180) % 360 - 180
            
            return {'center': (cX, cY), 'heading': heading, 'detected': True, 'mode': 'DUAL'}
        
        # 한쪽 마커만 보일 때 (비상용)
        elif left_id in markers:
            return {**markers[left_id], 'detected': True, 'mode': 'SINGLE_L'}
        elif right_id in markers:
            return {**markers[right_id], 'detected': True, 'mode': 'SINGLE_R'}
            
        return {'detected': False}
    
    def set_homography_from_corners(self, image_corners):
        """
        image_corners:
        [
            [u_left_top, v_left_top],
            [u_right_top, v_right_top],
            [u_right_bottom, v_right_bottom],
            [u_left_bottom, v_left_bottom]
        ]

        map 좌표:
        left_bottom  = (0, 0)
        right_bottom = (1.77, 0)
        right_top    = (1.77, 1.11)
        left_top     = (0, 1.11)
        """
        img_pts = np.array(image_corners, dtype=np.float32)

        map_pts = np.array([
            [0.0, self.map_height_m],                 # left_top
            [self.map_width_m, self.map_height_m],    # right_top
            [self.map_width_m, 0.0],                  # right_bottom
            [0.0, 0.0],                               # left_bottom
        ], dtype=np.float32)

        self.H_img_to_map, _ = cv2.findHomography(img_pts, map_pts)
        print("✅ Homography 설정 완료")
        print(self.H_img_to_map)


    def image_to_map(self, point):
        """
        image pixel point: (u, v)
        return: map point (x, y), meter
        """
        if self.H_img_to_map is None:
            return None

        u, v = point
        src = np.array([[[u, v]]], dtype=np.float32)
        dst = cv2.perspectiveTransform(src, self.H_img_to_map)

        x = float(dst[0][0][0])
        y = float(dst[0][0][1])
        return (x, y)


    def get_robot_pose_map(self, markers, left_id, right_id):
        """
        ArUco image 좌표 markers를 map 좌표계 robot pose로 변환
        return:
        {
            'detected': True,
            'center': (x, y),
            'heading': theta_rad,
            'heading_deg': theta_deg,
            'mode': 'DUAL'
        }
        """
        if self.H_img_to_map is None:
            return {'detected': False, 'reason': 'homography not set'}

        if left_id in markers and right_id in markers:
            l_img = markers[left_id]['center']
            r_img = markers[right_id]['center']

            l_map = self.image_to_map(l_img)
            r_map = self.image_to_map(r_img)

            if l_map is None or r_map is None:
                return {'detected': False}

            x = (l_map[0] + r_map[0]) / 2.0
            y = (l_map[1] + r_map[1]) / 2.0

            dx = r_map[0] - l_map[0]
            dy = r_map[1] - l_map[1]

            side_theta = math.atan2(dy, dx)

            # 좌->우 벡터를 기준으로 로봇 전방 방향 계산
            theta = side_theta + math.pi / 2.0
            theta = (theta + math.pi) % (2 * math.pi) - math.pi

            theta_deg = math.degrees(theta)

            return {
                'detected': True,
                'center': (x, y),
                'heading': theta,
                'heading_deg': theta_deg,
                'mode': 'DUAL',
                'left_map': l_map,
                'right_map': r_map
            }

        return {'detected': False}