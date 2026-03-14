import cv2
import numpy as np
import time
from simple_pid import PID
import drone_rc  # Your updated library with LED support

# ==========================================
# CONFIGURATION
# ==========================================
CAM_FRONT_INDEX = 1 
CAM_SIDE_INDEX = 2

TARGET_X = 0.5 
TARGET_Y = 0.5
TARGET_Z = 0.5 

# --- GREEN LED TRACKING (HSV) ---
# These values are tuned for a glowing Green LED.
# If it's too dark, lower the third number (50).
LOWER_GREEN = np.array([40, 50, 50])  
UPPER_GREEN = np.array([90, 255, 255])

class CompetitionController:
    def __init__(self):
        self.is_running = True
        
        # PID Tunings
        # Scale for Mode 2 is small. Start here.
        self.pid_x = PID(15.0, 3.0, 1.5, setpoint=TARGET_X)   # Roll
        self.pid_y = PID(15.0, 3.0, 1.5, setpoint=TARGET_Y)   # Pitch
        self.pid_z = PID(40.0, 8.0, 10.0, setpoint=TARGET_Z)  # Thrust

        self.pid_x.output_limits = (-15, 15) 
        self.pid_y.output_limits = (-15, 15)
        self.pid_z.output_limits = (-50, 50) 

        self.base_thrust = 140 # ADJUST THIS: The power to hover (0-250)

    def get_led_pos(self, frame):
        if frame is None: return None, None
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
        
        # Noise reduction
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] > 0:
                return M["m10"]/M["m00"] / frame.shape[1], M["m01"]/M["m00"] / frame.shape[0]
        return None, None

def main():
    ctrl = CompetitionController()
    
    # 1. Initialize Drone and LEDs
    print("Connecting and turning on Green LED...")
    drone_rc.set_mode(2)
    drone_rc.green_LED(1) # Turn on Green LED for tracking
    drone_rc.red_LED(0)
    drone_rc.blue_LED(0)
    time.sleep(1)

    cap_f = cv2.VideoCapture(CAM_FRONT_INDEX)
    cap_s = cv2.VideoCapture(CAM_SIDE_INDEX)

    print("System Live. Press SPACE for Emergency Stop.")

    try:
        while ctrl.is_running:
            ret_f, frame_f = cap_f.read()
            ret_s, frame_s = cap_s.read()

            # --- SENSE ---
            curr_x, curr_z1 = ctrl.get_led_pos(frame_f)
            curr_y, curr_z2 = ctrl.get_led_pos(frame_s)

            # --- THINK & ACT ---
            if curr_x is not None and curr_y is not None:
                avg_z = (curr_z1 + curr_z2) / 2
                
                # PID Calculations
                roll = ctrl.pid_x(curr_x)
                pitch = ctrl.pid_y(curr_y)
                thrust_adj = ctrl.pid_z(1.0 - avg_z) # Inverted for screen coords
                
                final_thrust = int(ctrl.base_thrust + thrust_adj)
                final_thrust = max(0, min(250, final_thrust))

                # Send commands
                drone_rc.set_roll(roll)
                drone_rc.set_pitch(pitch)
                drone_rc.manual_thrusts(final_thrust, final_thrust, final_thrust, final_thrust)
                
                # Visual Debug
                cv2.circle(frame_f, (int(curr_x*frame_f.shape[1]), int(curr_z1*frame_f.shape[0])), 15, (0, 255, 0), 2)
                print(f"X:{curr_x:.2f} Y:{curr_y:.2f} Z:{avg_z:.2f} | T:{final_thrust}", end='\r')
            else:
                # Level out if LED lost
                drone_rc.set_roll(0)
                drone_rc.set_pitch(0)
                drone_rc.manual_thrusts(ctrl.base_thrust, ctrl.base_thrust, ctrl.base_thrust, ctrl.base_thrust)
                print("!!! LED LOST !!!                     ", end='\r')

            # --- UI ---
            if frame_f is not None: cv2.imshow('Front (X/Z)', frame_f)
            if frame_s is not None: cv2.imshow('Side (Y/Z)', frame_s)
            
            # The 20ms delay protects the Wi-Fi bandwidth
            key = cv2.waitKey(20) & 0xFF 
            if key == ord(' '): 
                print("\nEmergency Stop Triggered.")
                drone_rc.emergency_stop()
                drone_rc.green_LED(0)
                ctrl.is_running = False

    except Exception as e:
        print(f"\nError: {e}")
        drone_rc.emergency_stop()
    finally:
        cap_f.release()
        cap_s.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()