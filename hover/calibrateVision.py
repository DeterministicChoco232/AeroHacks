import cv2
import numpy as np
import drone_rc

def main():
    # 1. Initialize Drone (Motors OFF, LEDs ON)
    try:
        drone_rc.set_mode(0)
        drone_rc.green_LED(1)
        drone_rc.red_LED(1)
        drone_rc.blue_LED(1)
        print("Drone Connected: LEDs ON, Motors SAFE.")
    except:
        print("Drone not connected. Running in Camera-Only mode.")

    # 2. Setup Variables [Hmin, Smin, Vmin, Hmax, Smax, Vmax]
    # Starting with a wide range
    hsv_vals = [42, 0, 250, 135, 255, 255] 
    cam_idx = 0
    cap = cv2.VideoCapture(cam_idx)

    print("\n--- KEYBOARD CALIBRATION ---")
    print("MIN: H:[Q/A]  S:[W/S]  V:[E/D]")
    print("MAX: H:[R/F]  S:[T/G]  V:[Y/H]")
    print("CAM: [C]  PRINT: [P]  QUIT: [ESC]")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera failed. Trying to reconnect...")
            cap.release()
            cap = cv2.VideoCapture(cam_idx)
            continue

        # Process HSV
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array(hsv_vals[0:3])
        upper = np.array(hsv_vals[3:6])
        mask = cv2.inRange(hsv_frame, lower, upper)
        
        # Side-by-side display
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        display = np.hstack((frame, mask_bgr))

        # Text Overlay
        info = f"MIN: {hsv_vals[0:3]}  MAX: {hsv_vals[3:6]}"
        cv2.putText(display, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(display, f"Camera: {cam_idx}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        cv2.imshow("Mac Calibration (Left: Real | Right: Mask)", display)

        key = cv2.waitKey(30) & 0xFF

        # --- KEYBOARD LOGIC ---
        # H-Min (Q/A)
        if key == ord('q'): hsv_vals[0] = min(179, hsv_vals[0] + 1)
        if key == ord('a'): hsv_vals[0] = max(0, hsv_vals[0] - 1)
        # S-Min (W/S)
        if key == ord('w'): hsv_vals[1] = min(255, hsv_vals[1] + 5)
        if key == ord('s'): hsv_vals[1] = max(0, hsv_vals[1] - 5)
        # V-Min (E/D)
        if key == ord('e'): hsv_vals[2] = min(255, hsv_vals[2] + 5)
        if key == ord('d'): hsv_vals[2] = max(0, hsv_vals[2] - 5)
        
        # H-Max (R/F)
        if key == ord('r'): hsv_vals[3] = min(179, hsv_vals[3] + 1)
        if key == ord('f'): hsv_vals[3] = max(0, hsv_vals[3] - 1)
        # S-Max (T/G)
        if key == ord('t'): hsv_vals[4] = min(255, hsv_vals[4] + 5)
        if key == ord('g'): hsv_vals[4] = max(0, hsv_vals[4] - 5)
        # V-Max (Y/H)
        if key == ord('y'): hsv_vals[5] = min(255, hsv_vals[5] + 5)
        if key == ord('h'): hsv_vals[5] = max(0, hsv_vals[5] - 5)

        # Camera Switch (C)
        if key == ord('c'):
            cam_idx = 1 if cam_idx == 0 else 0
            cap.release()
            cap = cv2.VideoCapture(cam_idx)
            print(f"Swapped to Camera {cam_idx}")

        # Print (P)
        if key == ord('p'):
            print(f"\n--- FINAL VALUES ---")
            print(f"LOWER = np.array([{hsv_vals[0]}, {hsv_vals[1]}, {hsv_vals[2]}])")
            print(f"UPPER = np.array([{hsv_vals[3]}, {hsv_vals[4]}, {hsv_vals[5]}])")

        # Quit (ESC)
        if key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()