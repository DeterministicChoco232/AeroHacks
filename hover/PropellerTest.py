import drone_rc
import time

drone_rc.set_mode(2)
thrust = 150  # Start here

print("Finding Lift-Off Point. BE READY TO KILL POWER (Ctrl+C or Space)")

# --- DRONE INITIALIZATION ---
print("Step 1: Calibration. KEEP DRONE FLAT ON FLOOR.")
drone_rc.set_mode(0)
time.sleep(1)

drone_rc.set_mode(2) 
drone_rc.reset_integral() 
drone_rc.green_LED(1)
drone_rc.red_LED(1)
time.sleep(1)

try:
    while thrust <= 200:
        print(f"Current Thrust: {thrust}")
        # Send same thrust to all 4 motors
        drone_rc.manual_thrusts(thrust, thrust, thrust, thrust)
        
        time.sleep(1.0) # Stay at this level for 1 second
        thrust += 0     # Increase by 5
        
except KeyboardInterrupt:
    drone_rc.emergency_stop()