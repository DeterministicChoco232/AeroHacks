import time

import cv2
import numpy as np
from simple_pid import PID

import drone_rc

# ==========================================
# 1. YOUR CALIBRATED HSV VALUES
# ==========================================
# We use a small range for V (200-255) to ensure stability
# even if the LED dims slightly during flight. 42, 0, 250, 135, 255, 255
HSV_LOWER = np.array([42, 0, 250])
HSV_UPPER = np.array([135, 255, 255])

# ==========================================
# 2. FLIGHT SETTINGS
# ==========================================
CAM_FRONT_INDEX = 0  # Tracks X and Z
CAM_SIDE_INDEX = 1  # Tracks Y and Z

TARGET_X = 0.5
TARGET_Y = 0.5
TARGET_Z = 0.5


class DroneController:
    def __init__(self):
        self.is_running = True

        # --- PID TUNING ---
        # No wind = Lower gains for maximum smoothness
        self.pid_x = PID(12.0, 0.5, 1.0, setpoint=TARGET_X)  # Roll
        self.pid_y = PID(12.0, 0.5, 1.0, setpoint=TARGET_Y)  # Pitch
        self.pid_z = PID(40.0, 2.0, 5.0, setpoint=TARGET_Z)  # Thrust

        self.pid_x.output_limits = (-12, 12)
        self.pid_y.output_limits = (-12, 12)
        self.pid_z.output_limits = (-50, 50)

        # STARTING THRUST: Adjust this until drone stays at 0.5m
        self.base_thrust = 0  ##100

    def get_drone_center(self, frame):
        """Finds all bright LEDs and returns their averaged center"""
        if frame is None:
            return None, None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)

        # Clean mask
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        points = []
        if contours:
            # We take all blobs that look like LEDs (not just the largest)
            # This allows the "averaging" of Green and White LEDs
            for c in contours:
                if cv2.contourArea(c) > 5:  # Ignore tiny speckles
                    M = cv2.moments(c)
                    if M["m00"] > 0:
                        points.append((M["m10"] / M["m00"], M["m01"] / M["m00"]))

        if len(points) > 0:
            avg_x = sum(p[0] for p in points) / len(points)
            avg_z = sum(p[1] for p in points) / len(points)

            # Visual feedback for judges
            cv2.circle(frame, (int(avg_x), int(avg_z)), 15, (0, 255, 0), 2)
            return avg_x / frame.shape[1], avg_z / frame.shape[0]

        return None, None


def main():
    ctrl = DroneController()

    # Init Hardware
    print("Initializing Drone...")
    drone_rc.set_mode(2)
    drone_rc.green_LED(1)
    drone_rc.red_LED(1)  # We use both for better averaging
    time.sleep(1)

    cap_f = cv2.VideoCapture(CAM_FRONT_INDEX)
    cap_s = cv2.VideoCapture(CAM_SIDE_INDEX)

    print("READY. Press SPACE to Emergency Stop.")

    try:
        while ctrl.is_running:
            ret_f, frame_f = cap_f.read()
            ret_s, frame_s = cap_s.read()

            # --- SENSE ---
            x, z_f = ctrl.get_drone_center(frame_f)
            y, z_s = ctrl.get_drone_center(frame_s)

            # --- THINK & ACT ---
            if x is not None and y is not None:
                # Average height and invert (0 is top in OpenCV)
                avg_z = 1.0 - ((z_f + z_s) / 2)

                # PID Calculations
                roll = ctrl.pid_x(x)
                pitch = ctrl.pid_y(y)
                thrust_adj = ctrl.pid_z(avg_z)

                final_thrust = int(ctrl.base_thrust + thrust_adj)
                final_thrust = max(0, min(250, final_thrust))

                # Command Drone
                drone_rc.set_roll(roll)
                drone_rc.set_pitch(pitch)
                drone_rc.manual_thrusts(
                    final_thrust, final_thrust, final_thrust, final_thrust
                )

                print(f"X:{x:.2f} Y:{y:.2f} Z:{avg_z:.2f} | T:{final_thrust}", end="\r")
            else:
                # Level safety
                drone_rc.set_roll(0)
                drone_rc.set_pitch(0)
                drone_rc.manual_thrusts(
                    ctrl.base_thrust,
                    ctrl.base_thrust,
                    ctrl.base_thrust,
                    ctrl.base_thrust,
                )
                print("!!! SEARCHING FOR LEDs !!!          ", end="\r")

            # --- DISPLAY ---
            if frame_f is not None:
                cv2.imshow("Front View", frame_f)
            if frame_s is not None:
                cv2.imshow("Side View", frame_s)

            print(f"pitch: {drone_rc.get_gyro_pitch()}")
            print(f"roll: {drone_rc.get_gyro_roll()}")

            if cv2.waitKey(20) & 0xFF == ord(" "):
                drone_rc.emergency_stop()
                ctrl.is_running = False

    finally:
        cap_f.release()
        cap_s.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
