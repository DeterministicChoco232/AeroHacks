import cv2
import numpy as np

def open_first_camera(max_index=6):
    """Try camera indices from 0..max_index-1 and return the first working capture."""
    for idx in range(max_index):
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            cap.release()
            continue

        ok, _ = cap.read()
        if ok:
            print(f"Using camera index {idx}")
            return cap

        cap.release()

    return None


cap = open_first_camera()
if cap is None:
    raise RuntimeError(
        "No webcam frames received. Close other apps using the camera and try again."
    )

cv2.namedWindow('Trackbars')
cv2.namedWindow('Camera')
cv2.namedWindow('Mask (Only LED should be white)')

# Create sliders to find the LED color in the room
cv2.createTrackbar('V_Low', 'Trackbars', 200, 255, lambda x: None)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame from webcam.")
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        v_low = cv2.getTrackbarPos('V_Low', 'Trackbars')
        lower = np.array([0, 0, v_low])
        upper = np.array([180, 255, 255])

        mask = cv2.inRange(hsv, lower, upper)

        cv2.imshow('Camera', frame)
        cv2.imshow('Mask (Only LED should be white)', mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()