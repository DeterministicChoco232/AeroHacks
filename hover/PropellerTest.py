import drone_rc
import time

drone_rc.set_mode(2)
thrust = 0  # Start here

print("Finding Lift-Off Point. BE READY TO KILL POWER (Ctrl+C or Space)")

try:
    while thrust <= 200:
        print(f"Current Thrust: {thrust}")
        # Send same thrust to all 4 motors
        drone_rc.manual_thrusts(thrust, thrust, thrust, thrust)
        
        time.sleep(1.0) # Stay at this level for 1 second
        thrust += 5     # Increase by 5
        
except KeyboardInterrupt:
    drone_rc.emergency_stop()