import cv2
import numpy as np
import time
from simple_pid import PID
import drone_rc 

# ==========================================
# CONFIGURATION
# ==========================================
CAM_FRONT_INDEX = 1 # Camera facing the X-Z plane
CAM_SIDE_INDEX = 2  # Camera facing the Y-Z plane

# Targets: 0.5 is the middle of the 1m cage
TARGET_X = 0.5 
TARGET_Y = 0.5
TARGET_Z = 0.5 

# Green LED HSV values
LOWER_GREEN = np.array([35, 50, 50])  
UPPER_GREEN = np.array([90, 255, 255])

class DroneController:
    def __init__(self):
        self.is_running = True
        
        # --- PID TUNING ---
        # Pitch/Roll PID: Units are small in Mode 2
        self.pid_x = PID(18.0, 4.0, 2.0, setpoint=TARGET_X)   # ROLL fixes X
        self.pid_y = PID(18.0, 4.0, 2.0, setpoint=TARGET_Y)   # PITCH fixes Y
        self.pid_z = PID(60.0, 10.0, 15.0, setpoint=TARGET_Z) # THRUST fixes Z

        self.pid_x.output_limits = (-15, 15) 
        self.pid_y.output_limits = (-15, 15)
        self.pid_z.output_limits = (-60, 60) 

        self.base_thrust = 145 # STARTING POINT - Tune this first!

    def get_drone_coords(self, frame):
        """Returns (horizontal, vertical) normalized 0-1"""
        if frame is None: return None, None
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
        
        # Noise filter
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] > 0:
                # Horizontal (X or Y), Vertical (Z)
                return M["m10"]/M["m00"] / frame.shape[1], M["m01"]/M["m00"] / frame.shape[0]
        return None, None

def main():
    ctrl = DroneController()
    
    # Init Hardware
    try:
        drone_rc.set_mode(2)
        drone_rc.green_LED(1)
        drone_rc.red_LED(0)
        drone_rc.blue_LED(0)
    except:
        print("Warning: Could not connect to drone. Check Wi-Fi!")

    cap_f = cv2.VideoCapture(CAM_FRONT_INDEX)
    cap_s = cv2.VideoCapture(CAM_SIDE_INDEX)

    print("System Running. Press SPACE to Emergency Stop.")

    try:
        while ctrl.is_running:
            ret_f, frame_f = cap_f.read()
            ret_s, frame_s = cap_s.read()

            # --- SENSE ---
            # Front camera gives us X and Height
            curr_x, z_from_front = ctrl.get_drone_coords(frame_f)
            # Side camera gives us Y and Height
            curr_y, z_from_side = ctrl.get_drone_coords(frame_s)

            # --- THINK ---
            if curr_x is not None and curr_y is not None:
                # Use the average of both cameras for height stability
                avg_z_pixel = (z_from_front + z_from_side) / 2
                curr_z = 1.0 - avg_z_pixel # Invert because 0 is top of screen
                
                # Calculate PID responses
                target_roll = ctrl.pid_x(curr_x)
                target_pitch = ctrl.pid_y(curr_y)
                thrust_adj = ctrl.pid_z(curr_z)
                
                final_thrust = int(ctrl.base_thrust + thrust_adj)
                final_thrust = max(0, min(250, final_thrust))

                # --- ACT ---
                drone_rc.set_roll(target_roll)
                drone_rc.set_pitch(target_pitch)
                drone_rc.manual_thrusts(final_thrust, final_thrust, final_thrust, final_thrust)
                
                # Visual Debugging (Draw on the frames)
                cv2.circle(frame_f, (int(curr_x*frame_f.shape[1]), int(z_from_front*frame_f.shape[0])), 10, (0,255,0), 2)
                cv2.circle(frame_s, (int(curr_y*frame_s.shape[1]), int(z_from_side*frame_s.shape[0])), 10, (0,255,0), 2)
                
                print(f"X:{curr_x:.2f} Y:{curr_y:.2f} Z:{curr_z:.2f} | Thrust:{final_thrust}", end='\r')
            else:
                # If LED is missing in either camera, stay level
                drone_rc.set_roll(0)
                drone_rc.set_pitch(0)
                drone_rc.manual_thrusts(ctrl.base_thrust, ctrl.base_thrust, ctrl.base_thrust, ctrl.base_thrust)
                print("LED LOST IN ONE OR BOTH CAMERAS          ", end='\r')

            # --- DISPLAY ---
            if frame_f is not None: cv2.imshow('Front (X-Z Axis)', frame_f)
            if frame_s is not None: cv2.imshow('Side (Y-Z Axis)', frame_s)
            
            if cv2.waitKey(20) & 0xFF == ord(' '): 
                drone_rc.emergency_stop()
                ctrl.is_running = False

    finally:
        cap_f.release()
        cap_s.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()