import cv2
import numpy as np

# --- SETTINGS ---
# On a Mac, 0 is usually the built-in FaceTime camera. 
# 1 and 2 are usually USB cameras.
CAM_INDEX = 0 

# Initial HSV Guesses (Hmin, Smin, Vmin, Hmax, Smax, Vmax)
hsv_data = {
    0: {"name": "RED",   "vals": [0, 100, 100, 10, 255, 255]},
    1: {"name": "GREEN", "vals": [35, 50, 50, 90, 255, 255]},
    2: {"name": "BLUE",  "vals": [100, 150, 50, 140, 255, 255]},
    3: {"name": "WHITE", "vals": [0, 0, 240, 180, 30, 255]}
}

def nothing(x):
    pass

def main():
    # 1. Setup Windows (CRITICAL FOR MAC)
    cv2.namedWindow('Controls', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Controls', 400, 600)
    cv2.startWindowThread() # Helps prevent frozen sliders on macOS

    # 2. Create Sliders
    cv2.createTrackbar('COLOR_SELECT', 'Controls', 0, 3, nothing)
    cv2.createTrackbar('H_Min', 'Controls', 0, 179, nothing)
    cv2.createTrackbar('S_Min', 'Controls', 0, 255, nothing)
    cv2.createTrackbar('V_Min', 'Controls', 0, 255, nothing)
    cv2.createTrackbar('H_Max', 'Controls', 179, 179, nothing)
    cv2.createTrackbar('S_Max', 'Controls', 255, 255, nothing)
    cv2.createTrackbar('V_Max', 'Controls', 255, 255, nothing)

    cap = cv2.VideoCapture(CAM_INDEX)
    
    last_color_idx = -1

    print("--- MOCK CALIBRATION ACTIVE (NO DRONE) ---")
    print("1. Click the 'Controls' window to make it active.")
    print("2. Adjust 'COLOR_SELECT' to switch which LED color you are tuning.")
    print("3. Press 's' to print values to the terminal.")
    print("4. Press 'q' to quit.")

    while True:
        # Get Current Color Selection from slider
        color_idx = cv2.getTrackbarPos('COLOR_SELECT', 'Controls')
        
        # If we switched colors, reset sliders to the saved values for that color
        if color_idx != last_color_idx:
            v = hsv_data[color_idx]["vals"]
            cv2.setTrackbarPos('H_Min', 'Controls', v[0])
            cv2.setTrackbarPos('S_Min', 'Controls', v[1])
            cv2.setTrackbarPos('V_Min', 'Controls', v[2])
            cv2.setTrackbarPos('H_Max', 'Controls', v[3])
            cv2.setTrackbarPos('S_Max', 'Controls', v[4])
            cv2.setTrackbarPos('V_Max', 'Controls', v[5])
            last_color_idx = color_idx

        # Read Camera Frame
        ret, frame = cap.read()
        if not ret:
            # If no camera, show a grey screen
            frame = np.ones((480, 640, 3), dtype=np.uint8) * 50
            cv2.putText(frame, "No Camera Found", (150, 240), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # Update the local data based on current slider positions
        current_vals = [
            cv2.getTrackbarPos('H_Min', 'Controls'),
            cv2.getTrackbarPos('S_Min', 'Controls'),
            cv2.getTrackbarPos('V_Min', 'Controls'),
            cv2.getTrackbarPos('H_Max', 'Controls'),
            cv2.getTrackbarPos('S_Max', 'Controls'),
            cv2.getTrackbarPos('V_Max', 'Controls')
        ]
        hsv_data[color_idx]["vals"] = current_vals

        # Apply the HSV Mask
        lower = np.array(current_vals[0:3])
        upper = np.array(current_vals[3:6])
        
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, lower, upper)
        
        # Convert mask to BGR so we can stack it with the original frame
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # Stack Original and Mask side-by-side
        display = np.hstack((frame, mask_bgr))
        
        # Add labels for the user
        color_name = hsv_data[color_idx]['name']
        cv2.putText(display, f"Tuning: {color_name}", (20, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(display, "Left: Real | Right: Mask", (20, 90), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        cv2.imshow('Calibration View', display)

        # Mac-Friendly event loop (30ms provides enough time for UI updates)
        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            name = hsv_data[color_idx]['name']
            v = current_vals
            # Format this so it's easy to copy into a Python dictionary
            print(f'"{name.lower()}": ({v[0]}, {v[1]}, {v[2]}, {v[3]}, {v[4]}, {v[5]}),')

    cap.release()
    cv2.destroyAllWindows()
    # MacBook specific: force windows to close
    for _ in range(5): cv2.waitKey(1)

if __name__ == "__main__":
    main()