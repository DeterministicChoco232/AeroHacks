import cv2
import numpy as np
import time
import csv
from datetime import datetime
from simple_pid import PID
import drone_rc 

# ==========================================
# 1. YOUR CALIBRATED HSV VALUES
# ==========================================
HSV_LOWER = np.array([42, 0, 250])  
HSV_UPPER = np.array([135, 255, 255])

# ==========================================
# 2. FLIGHT SETTINGS
# ==========================================
CAM_FRONT_INDEX = 0 
CAM_SIDE_INDEX = 1  

TARGET_X = 0.5 
TARGET_Y = 0.5
TARGET_Z = 0.5 

class DroneController:
    def __init__(self):
        self.is_running = True
        
        # --- PID TUNING ---
        self.pid_x = PID(12.0, 0.5, 1.0, setpoint=TARGET_X) 
        self.pid_y = PID(12.0, 0.5, 1.0, setpoint=TARGET_Y) 
        self.pid_z = PID(35.0, 1.5, 10.0, setpoint=TARGET_Z) 

        self.pid_x.output_limits = (-12, 12) 
        self.pid_y.output_limits = (-12, 12)
        self.pid_z.output_limits = (-50, 50) 

        self.base_thrust = 150 

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
    
    # --- SETUP CSV LOGGING ---
    filename = datetime.now().strftime("pid_log_%Y%m%d_%H%M%S.csv")
    csv_file = open(filename, mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    # Header row
    csv_writer.writerow([
        'Timestamp', 
        'Target_X', 'Current_X', 'Roll_Out',
        'Target_Y', 'Current_Y', 'Pitch_Out',
        'Target_Z', 'Current_Z', 'Thrust_Adj', 'Final_Thrust'
    ])

    # --- HARDWARE INITIALIZATION ---
    print("Pre-flight: PLACE DRONE FLAT ON FLOOR")
    drone_rc.set_mode(0)
    time.sleep(1)
    
    print("Calibrating Gyro (Mode 2)...")
    drone_rc.set_mode(2)
    drone_rc.reset_integral() 
    drone_rc.green_LED(1)
    drone_rc.red_LED(1)
    time.sleep(1)
    
    cap_f = cv2.VideoCapture(CAM_FRONT_INDEX)
    cap_s = cv2.VideoCapture(CAM_SIDE_INDEX)
    start_time = time.time()

    print(f"READY. Logging to {filename}. Press SPACE to Stop.")

    try:
        while ctrl.is_running:
            ret_f, frame_f = cap_f.read()
            ret_s, frame_s = cap_s.read()

            x_pos, z_f = ctrl.get_drone_center(frame_f)
            y_pos, z_s = ctrl.get_drone_center(frame_s)

            if x_pos is not None and y_pos is not None:
                avg_z = 1.0 - ((z_f + z_s) / 2)
                
                if avg_z < 0.15:
                    roll, pitch = 0, 0
                    thrust_adj = 20 # Jump thrust
                    final_thrust = ctrl.base_thrust + thrust_adj
                else:
                    roll = ctrl.pid_x(x_pos)
                    pitch = ctrl.pid_y(y_pos)
                    thrust_adj = ctrl.pid_z(avg_z)
                    final_thrust = int(ctrl.base_thrust + thrust_adj)

                final_thrust = max(0, min(250, final_thrust))

                # Command Drone
                drone_rc.set_roll(roll)
                drone_rc.set_pitch(pitch)
                drone_rc.manual_thrusts(final_thrust, final_thrust, final_thrust, final_thrust)
                
                # --- LOG TO CSV ---
                csv_writer.writerow([
                    round(time.time() - start_time, 3),
                    TARGET_X, round(x_pos, 3), round(roll, 2),
                    TARGET_Y, round(y_pos, 3), round(pitch, 2),
                    TARGET_Z, round(avg_z, 3), round(thrust_adj, 2), final_thrust
                ])

                print(f"X:{x_pos:.2f} Y:{y_pos:.2f} Z:{avg_z:.2f} | T:{final_thrust}", end='\r')
            else:
                drone_rc.set_roll(0)
                drone_rc.set_pitch(0)
                drone_rc.manual_thrusts(ctrl.base_thrust, ctrl.base_thrust, ctrl.base_thrust, ctrl.base_thrust)
                print("!!! SEARCHING FOR LEDs !!!          ", end='\r')

            if frame_f is not None: cv2.imshow('Front View (X-Z)', frame_f)
            if frame_s is not None: cv2.imshow('Side View (Y-Z)', frame_s)
            
            if cv2.waitKey(20) & 0xFF == ord(' '): 
                ctrl.is_running = False

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        print("\nSaving log and closing...")
        csv_file.close() # Important to save data!
        drone_rc.emergency_stop()
        cap_f.release()
        cap_s.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()