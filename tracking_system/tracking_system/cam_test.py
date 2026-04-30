
import cv2

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
print("opened:", cap.isOpened())

ret, frame = cap.read()
print("ret:", ret)

if ret:
    print("frame shape:", frame.shape)

cap.release()
