import cv2
import numpy as np
import drone_rc

# ======================================================
# MANUAL CONFIGURATION AREA
# Update these numbers, save the file, and watch the mask
# Format: (H_min, S_min, V_min, H_max, S_max, V_max)
# ======================================================
CONFIGS = {
    "1": {"name": "RED",   "hsv": (0, 100, 100, 10, 255, 255)},
    "2": {"name": "GREEN", "hsv": (35, 50, 50, 90, 255, 255)},
    "3": {"name": "BLUE",  "hsv": (100, 150, 50, 140, 255, 255)},
    "4": {"name": "WHITE", "hsv": (0, 0, 240, 180, 30, 255)}
}

def set_drone_led(choice):
    """Turns on only the LED we are currently calibrating"""
    drone_rc.red_LED(1 if choice == "1" else 0)
    drone_rc.green_LED(1 if choice == "2" else 0)
    drone_rc.blue_LED(1 if choice == "3" else 0)
    # Note: If 4 is a status white LED, it might always be on

def main():
    # Start in 'Safe' mode (Propellers OFF)
    try:
        drone_rc.set_mode(0)
    except:
        print("Drone not connected - checking camera only.")

    cap = cv2.VideoCapture(1) # Start with Camera 1
    current_choice = "1"
    set_drone_led(current_choice)

    print(f"--- CALIBRATION MODE ---")
    print(f"Press '1' for RED, '2' for GREEN, '3' for BLUE, '4' for WHITE")
    print(f"Press 'c' to swap between Camera 1 and Camera 2")
    print(f"Press 'q' to quit")

    while True:
        ret, frame = cap.read()
        if not ret: break

        # Get current color setup
        cfg = CONFIGS[current_choice]
        lower = np.array(cfg["hsv"][0:3])
        upper = np.array(cfg["hsv"][3:6])

        # Create Black & White Mask
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, lower, upper)
        
        # Clean the mask slightly for a better view
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=1)

        # Create a side-by-side view (Original and Mask)
        # Convert mask to BGR so we can stack it with the colored frame
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        stacked = np.hstack((frame, mask_bgr))

        # Add text overlay
        cv2.putText(stacked, f"Tuning: {cfg['name']} (Press 1-4 to switch)", 
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow("Calibration (Left: Normal | Right: Mask)", stacked)

        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        elif key in [ord('1'), ord('2'), ord('3'), ord('4')]:
            current_choice = chr(key)
            set_drone_led(current_choice)
            print(f"Switched to {CONFIGS[current_choice]['name']}")
        elif key == ord('c'):
            # Switch camera index
            new_idx = 2 if cap.get(cv2.CAP_PROP_POS_FRAMES) == 1 else 1
            cap.release()
            cap = cv2.VideoCapture(new_idx)
            print(f"Switched to Camera {new_idx}")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()