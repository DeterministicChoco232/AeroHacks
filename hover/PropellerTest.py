import drone_rc
import time

def force_spin():
    print("--- MOTOR RECOVERY TEST ---")
    
    # 1. Start in Mode 0, then Mode 1
    drone_rc.set_mode(0)
    time.sleep(0.5)
    drone_rc.set_mode(1)
    time.sleep(0.5)
    
    # 2. Try Syntax A: The library's way (with a newline)
    print("Trying Syntax A (Library Default)...")
    drone_rc.manual_thrusts(180, 180, 180, 180)
    time.sleep(1)

    # 3. Try Syntax B: Space instead of Newline (Very common for ESP32)
    print("Trying Syntax B (Space Format)...")
    drone_rc.msg("manT 180,180,180,180")
    time.sleep(1)

    # 4. Try Syntax C: No Newline, No Space
    print("Trying Syntax C (Direct Format)...")
    drone_rc.msg("manT180,180,180,180")
    time.sleep(1)

    # 5. Try Syntax D: Increment Command
    print("Trying Syntax D (Increment)...")
    drone_rc.msg("incT 100,100,100,100")
    time.sleep(1)

    print("Test finished. Stopping...")
    drone_rc.emergency_stop()

if __name__ == "__main__":
    force_spin()