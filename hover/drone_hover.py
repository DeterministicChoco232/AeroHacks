import cv2
import numpy as np
import time
import csv
from datetime import datetime
from simple_pid import PID
import drone_rc 

# ==========================================
# 1. VISION CALIBRATION
# ==========================================
HSV_LOWER = np.array([42, 0, 200])  
HSV_UPPER = np.array([135, 255, 255])

# ==========================================
# 2. FLIGHT & AXIS SETTINGS
# ==========================================
CAM_BACK_INDEX = 0  # Tracks X (Left/Right) and Z
CAM_SIDE_INDEX = 1  # Tracks Y (Forward/Backward) and Z

# --- AXIS INVERSION (Change to True if drone flies away from center) ---
INVERT_X = False  # Set to True if "Left" in camera should be "Right" on drone
INVERT_Y = False  # Set to True if "Forward" in camera should be "Backward"
INVERT_Z = False  # Set to True if "Up" in camera should be "Down"

TARGET_X = 0.5 
TARGET_Y = 0.5
TARGET_Z = 0.5 

class DroneController:
    def __init__(self):
        self.is_running = True
        
        # --- PID VALUES (Tuned from your previous log) ---
        self.pid_x = PID(18.0, 2.5, 1.5, setpoint=TARGET_X) 
        self.pid_y = PID(18.0, 2.5, 1.5, setpoint=TARGET_Y) 
        self.pid_z = PID(22.0, 1.0, 18.0, setpoint=TARGET_Z) 

        self.pid_x.output_limits = (-15, 15) 
        self.pid_y.output_limits = (-15, 15)
        self.pid_z.output_limits = (-50, 50) 

        self.base_thrust = 155 

    def get_drone_center(self, frame):
        if frame is None: return None, None
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=1)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        points = []
        if contours:
            for c in contours:
                if cv2.contourArea(c) > 5:
                    M = cv2.moments(c)
                    if M["m00"] > 0:
                        points.append((M["m10"]/M["m00"], M["m01"]/M["m00"]))

        if len(points) > 0:
            avg_x = sum(p[0] for p in points) / len(points)
            avg_z = sum(p[1] for p in points) / len(points)
            cv2.circle(frame, (int(avg_x), int(avg_z)), 15, (0, 255, 0), 2)
            return avg_x / frame.shape[1], avg_z / frame.shape[0]
        return None, None

def main():
    ctrl = DroneController()
    
    # --- CSV LOGGING ---
    filename = datetime.now().strftime("backcam_log_%H%M%S.csv")
    csv_file = open(filename, mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(['Time', 'X_Cam', 'Roll_Cmd', 'IMU_Roll', 'Y_Cam', 'Pitch_Cmd', 'IMU_Pitch', 'Z_Cam', 'T_Adj', 'Total_T'])

    # --- CAMERAS ---
    cap_b = cv2.VideoCapture(CAM_BACK_INDEX)
    cap_s = cv2.VideoCapture(CAM_SIDE_INDEX)
    for c in [cap_b, cap_s]:
        c.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        c.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # --- INITIALIZATION ---
    print("PLACE DRONE FLAT ON FLOOR. ENSURE IT FACES AWAY FROM BACK CAMERA.")
    drone_rc.set_mode(0)
    time.sleep(1)
    drone_rc.set_mode(2) 
    drone_rc.reset_integral() 
    drone_rc.green_LED(1)
    drone_rc.red_LED(1)
    time.sleep(1)
    
    start_time = time.time()

    try:
        while ctrl.is_running:
            ret_b, frame_b = cap_b.read()
            ret_s, frame_s = cap_s.read()

            # --- SENSE ---
            x_raw, z_b = ctrl.get_drone_center(frame_b)
            y_raw, z_s = ctrl.get_drone_center(frame_s)

            if x_raw is not None and y_raw is not None:
                # Apply Axis Inversions if needed
                x_in = 1.0 - x_raw if INVERT_X else x_raw
                y_in = 1.0 - y_raw if INVERT_Y else y_raw
                z_raw = (z_b + z_s) / 2
                z_in = z_raw if INVERT_Z else (1.0 - z_raw)

                # Get IMU Feedback
                imu_pitch = drone_rc.get_pitch()
                imu_roll = drone_rc.get_roll()

                # --- THINK & ACT ---
                if z_in < 0.15:
                    roll_cmd, pitch_cmd = 0, 0
                    thrust_adj = 20
                else:
                    roll_cmd = ctrl.pid_x(x_in)
                    pitch_cmd = ctrl.pid_y(y_in)
                    thrust_adj = ctrl.pid_z(z_in)

                final_t = int(ctrl.base_thrust + thrust_adj)
                final_t = max(0, min(250, final_t))

                # Update Drone
                drone_rc.set_roll(roll_cmd)
                drone_rc.set_pitch(pitch_cmd)
                drone_rc.manual_thrusts(final_t, final_t, final_t, final_t)
                
                # --- LOG ---
                csv_writer.writerow([
                    round(time.time() - start_time, 3),
                    round(x_in, 3), round(roll_cmd, 2), round(imu_roll, 2),
                    round(y_in, 3), round(pitch_cmd, 2), round(imu_pitch, 2),
                    round(z_in, 3), round(thrust_adj, 2), final_t
                ])
                print(f"X:{x_in:.2f} Y:{y_in:.2f} Z:{z_in:.2f} | T:{final_t}", end='\r')

            else:
                drone_rc.set_roll(0)
                drone_rc.set_pitch(0)
                drone_rc.manual_thrusts(ctrl.base_thrust, ctrl.base_thrust, ctrl.base_thrust, ctrl.base_thrust)

            if frame_b is not None: cv2.imshow('Back Camera (X-Z)', frame_b)
            if frame_s is not None: cv2.imshow('Side Camera (Y-Z)', frame_s)
            
            if cv2.waitKey(20) & 0xFF == ord(' '): 
                ctrl.is_running = False

    finally:
        csv_file.close()
        drone_rc.emergency_stop()
        cap_b.release()
        cap_s.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()