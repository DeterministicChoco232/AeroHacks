import cv2
import numpy as np

def open_camera_pair(max_index=8):
    """Find and return two working camera captures with their indices."""
    found = []
    for idx in range(max_index):
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            cap.release()
            continue

        ok, _ = cap.read()
        if ok:
            found.append((idx, cap))
            if len(found) == 2:
                break
            continue

        cap.release()

    if len(found) < 2:
        for _, cap in found:
            cap.release()
        return None, None, None, None

    (idx_a, cap_a), (idx_b, cap_b) = found
    return idx_a, cap_a, idx_b, cap_b


idx_front, cap_front, idx_side, cap_side = open_camera_pair()
if cap_front is None or cap_side is None:
    raise RuntimeError(
        "Could not find two working cameras. Connect both cameras and close other apps using camera access."
    )

print(f"Using front camera index {idx_front}")
print(f"Using side camera index {idx_side}")

cv2.namedWindow('Trackbars')
cv2.namedWindow('Front Camera')
cv2.namedWindow('Side Camera')
cv2.namedWindow('Front Mask')
cv2.namedWindow('Side Mask')

# Create sliders to find the LED color in the room
cv2.createTrackbar('V_Low', 'Trackbars', 200, 255, lambda x: None)

try:
    while True:
        ret_f, frame_f = cap_front.read()
        ret_s, frame_s = cap_side.read()

        if not ret_f or not ret_s:
            print("Failed to read from one or both cameras.")
            break

        hsv_f = cv2.cvtColor(frame_f, cv2.COLOR_BGR2HSV)
        hsv_s = cv2.cvtColor(frame_s, cv2.COLOR_BGR2HSV)

        v_low = cv2.getTrackbarPos('V_Low', 'Trackbars')
        lower = np.array([0, 0, v_low])
        upper = np.array([180, 255, 255])

        mask_f = cv2.inRange(hsv_f, lower, upper)
        mask_s = cv2.inRange(hsv_s, lower, upper)

        cv2.imshow('Front Camera', frame_f)
        cv2.imshow('Side Camera', frame_s)
        cv2.imshow('Front Mask', mask_f)
        cv2.imshow('Side Mask', mask_s)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
finally:
    cap_front.release()
    cap_side.release()
    cv2.destroyAllWindows()