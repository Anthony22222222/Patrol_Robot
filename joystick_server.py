import socket, time, sys, threading
from gpiozero import OutputDevice, PWMOutputDevice
import RPi.GPIO as GPIO
import smbus2
from time import monotonic, sleep
from statistics import mean
import subprocess

# ================================================================
# MPU6050 GYROSCOPE CONFIGURATION
# Current mounting orientation:
#   Y axis -> Vehicle front
#   X axis -> Vehicle right side
#   Z axis -> Vehicle up
# ================================================================


class MPU6050:
    def __init__(self):
        self.bus = smbus2.SMBus(1)  # I2C bus 1
        self.address = 0x68         # MPU6050 address
        self.z_bias = 0            # Calibrated zero offset
        self.velocity_history = []  # For moving average
        print("\n=== MPU6050 Initialization ===")
        
        try:
            # Wake up the MPU6050
            self.bus.write_byte_data(self.address, 0x6B, 0)
            print("✓ MPU6050 powered on")
            
            # Configure gyro for ±250 degrees/second
            self.bus.write_byte_data(self.address, 0x1B, 0)
            print("✓ Gyro range set to ±250 deg/s")
            
            # Configure digital low-pass filter
            self.bus.write_byte_data(self.address, 0x1A, 0x06)
            print("✓ Low-pass filter enabled")
            
            # Verify communication by reading WHO_AM_I
            who = self.bus.read_byte_data(self.address, 0x75)
            if who != 0x68:
                raise Exception(f"Unexpected WHO_AM_I: 0x{who:02X}")
            print("✓ Device ID verified")
            
            # Calibrate the gyro
            print("\nCalibrating gyro (keep still)...")
            self.calibrate()
            print(f"✓ Calibration complete (Z bias: {self.z_bias:.2f} deg/s)")
            
            print("\nMPU6050 Initialization complete!")
            self.ok = True
            
        except Exception as e:
            print(f"MPU6050 init failed: {e}")
            self.ok = False
    
    def calibrate(self, samples=100):
        """Measure gyro bias by averaging readings while stationary"""
        z_samples = []
        for _ in range(samples):
            z_samples.append(self.get_gyro_dps()['z'])
            sleep(0.01)
        self.z_bias = mean(z_samples)
    
    def apply_moving_average(self, new_value, window_size=5):
        """Apply moving average filter to smooth readings"""
        self.velocity_history.append(new_value)
        if len(self.velocity_history) > window_size:
            self.velocity_history.pop(0)
        return mean(self.velocity_history)
    
    def read_word(self, reg):
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        value = (high << 8) + low
        return value if value < 32768 else value - 65536
    
    def get_gyro_z(self):
        """Optimized direct Z-axis reading in degrees per second"""
        z = self.read_word(0x47)  # Z axis only
        return z / 131.0 - self.z_bias  # Convert to dps and remove bias
    
    def get_gyro_raw(self):
        return {'z': self.read_word(0x47)}
    
    def get_gyro_dps(self):
        return {'z': self.get_gyro_z() + self.z_bias}

# Angle Tracking Globals
current_angle = 0.0
last_gyro_time = None
VELOCITY_DEADBAND = 0.5  # Ignore tiny movements (deg/s)


def update_angle():
    """Update current angle using optimized gyro data."""
    global current_angle, last_gyro_time
    if not gyro or not gyro.ok:
        return 0
    now = monotonic()
    if last_gyro_time is None:
        last_gyro_time = now
        return 0
    dt = now - last_gyro_time
    velocity = gyro.get_gyro_z()
    if abs(velocity) > VELOCITY_DEADBAND:
        current_angle += velocity * dt
    last_gyro_time = now
    return velocity

# Mode and State Tracking
MODE_MANUAL = 'manual'
MODE_PATROL = 'patrol'
current_mode = MODE_MANUAL

# Recording State
is_recording = False
recorded_path = []

# Patrol State
patrol_sequence = {
    'forward_path': [],
    'reverse_path': [],
    'current_index': 0,
    'going_forward': True
}

# Movement Types & Constants
MOVE_FORWARD = 'forward'
MOVE_BACKWARD = 'backward'
TURN_LEFT = 'left'
TURN_RIGHT = 'right'
FORWARD_SPEED = 60
MOVE_TIME = 2.0
BRAKE_POWER = 50
BRAKE_TIME = 0.4


# ================================================================
# TURNING PARAMETERS (ADJUST THESE)
# ================================================================

# Phase 1: Power Burst Parameters
BURST_POWER = 100      # Initial high power for strong start (0-100)
                       # Higher: stronger start on rough surfaces
                       # Lower: gentler start

BURST_PHASE = 0.4     # Portion of turn for initial burst phase (0.0-1.0)
                       # Higher: maintain high power longer
                       # Lower: shorter high-power phase

# Phase 2: Deceleration Parameters
MIN_POWER = 40        # Minimum power at end of deceleration (0-100)
                       # Higher: prevents stalling
                       # Lower: gentler approach to target

# Phase 3: Braking Parameters
BRAKE_THRESHOLD = 2.0  # Degrees from target to start braking
                       # Higher: earlier braking
                       # Lower: more precision (risk of overshoot)

BRAKE_POWER = 50       # Power for active braking pulse (0-100)
                       # Higher: stronger braking
                       # Lower: gentler braking

BRAKE_DURATION = 0.4  # Duration of brake pulse in seconds
                       # Higher: longer braking
                       # Lower: shorter braking

# Universal Stall Detection
STALL_THRESHOLD = 5.0  # Rotation rate below which is considered stalled (deg/sec)
                       # Higher: reduces false stall detections
                       # Lower: detects stalls more quickly

STALL_CHECK_TIME = 0.05 # Time between stall checks (seconds)
                       # Higher: less frequent checking
                       # Lower: faster stall response

STALL_BURST_POWER = 100 # Power for stall recovery (0-100)
                       # Maximum is 100
                       # Lower: gentler recovery

STALL_BURST_DURATION = 0.5 # Duration of recovery burst (seconds)
                       # Higher: longer recovery pulse
                       # Lower: shorter recovery pulse

# General Parameters
TURN_THRESHOLD = 2.0   # Final precision for turn completion (degrees)
                       # Higher: less precise turns
                       # Lower: more precise final position

# Movement Functions
def timed_move(direction, duration=1.0):
    """Move forward/back for a set duration with active braking"""
    print(f"\n=== timed_move called with direction: {direction}, duration: {duration} ===")
    if direction == "FWD":
        print("Starting forward movement...")
        drive(FORWARD_SPEED, FORWARD_SPEED)
    elif direction == "BACK":
        print("Starting backward movement...")
        drive(-FORWARD_SPEED, -FORWARD_SPEED)
    else:
        print(f"WARNING: Unknown direction '{direction}'")
        return
    
    print(f"Moving for {duration} seconds...")
    time.sleep(duration)
    
    # Apply active brake
    print("Applying active brake...")
    if direction == "FWD":
        drive(-BRAKE_POWER, -BRAKE_POWER)
    else:
        drive(BRAKE_POWER, BRAKE_POWER)
    time.sleep(BRAKE_TIME)
    
    print("Stopping motors...")
    stop_motors()

def turn_by_angle(angle_change):
    """Turn by relative angle change using simplified 3-phase approach:
       1. Power Burst: High power initial phase
       2. Deceleration: Gradual power decrease
       3. Active Braking: Quick stop at target
    """
    global current_angle
    start_angle = current_angle
    target_angle = start_angle + angle_change
    direction = 1 if angle_change > 0 else -1
    
    print(f"\n=== Starting {abs(angle_change)}° {'right' if angle_change > 0 else 'left'} turn ===")
    print(f"Start angle: {start_angle:.1f}°")
    print(f"Target angle: {target_angle:.1f}°")
    
    # Calculate phase transition points
    burst_end_angle = start_angle + (angle_change * BURST_PHASE)
    brake_start_angle = target_angle - (direction * BRAKE_THRESHOLD)
    
    # Setup monitoring variables
    last_check = monotonic()
    last_angle = current_angle
    last_stall_check = monotonic()
    in_stall_recovery = False
    stall_end_time = 0
    
    # Phase 1: Power Burst - High power to overcome initial friction
    print("PHASE 1: Power burst...")
    
    while ((direction > 0 and current_angle < burst_end_angle) or 
           (direction < 0 and current_angle > burst_end_angle)):
        
        now = monotonic()
        if now - last_check >= 0.005:  # 200Hz updates
            update_angle()
            last_check = now
            
            # Stall detection
            if not in_stall_recovery and now - last_stall_check >= STALL_CHECK_TIME:
                velocity = abs(gyro.get_gyro_z())
                if velocity < STALL_THRESHOLD:
                    # Apply recovery burst
                    print("Stall detected! Applying recovery burst...")
                    in_stall_recovery = True
                    stall_end_time = now + STALL_BURST_DURATION
                
                last_stall_check = now
                last_angle = current_angle
            
            # Apply power - either burst or recovery power
            if in_stall_recovery and now < stall_end_time:
                drive(STALL_BURST_POWER * direction, -STALL_BURST_POWER * direction)
            else:
                in_stall_recovery = False  # End recovery if time elapsed
                drive(BURST_POWER * direction, -BURST_POWER * direction)
        
        time.sleep(0.001)  # Prevent CPU hogging
    
    print(f"Completed burst phase at {current_angle:.1f}°")
    
    # Phase 2: Deceleration - Gradually decrease power as approaching target
    print("PHASE 2: Deceleration...")
    
    while ((direction > 0 and current_angle < brake_start_angle) or 
           (direction < 0 and current_angle > brake_start_angle)):
        
        now = monotonic()
        if now - last_check >= 0.005:  # 200Hz updates
            update_angle()
            last_check = now
            
            # Stall detection (same as in phase 1)
            if not in_stall_recovery and now - last_stall_check >= STALL_CHECK_TIME:
                velocity = abs(gyro.get_gyro_z())
                if velocity < STALL_THRESHOLD:
                    # Apply recovery burst
                    print("Stall detected! Applying recovery burst...")
                    in_stall_recovery = True
                    stall_end_time = now + STALL_BURST_DURATION
                
                last_stall_check = now
                last_angle = current_angle
            
            # Calculate power based on progress
            if in_stall_recovery and now < stall_end_time:
                # Apply recovery burst
                drive(STALL_BURST_POWER * direction, -STALL_BURST_POWER * direction)
            else:
                in_stall_recovery = False  # End recovery if time elapsed
                
                # Linear power decrease from BURST_POWER to MIN_POWER
                total_decel_angle = abs(brake_start_angle - burst_end_angle)
                progress = abs(current_angle - burst_end_angle) / total_decel_angle
                power = BURST_POWER - progress * (BURST_POWER - MIN_POWER)
                power = max(MIN_POWER, power)  # Ensure power doesn't go below MIN_POWER
                
                drive(power * direction, -power * direction)
        
        time.sleep(0.001)  # Prevent CPU hogging
    
    print(f"Completed deceleration phase at {current_angle:.1f}°")
    
    # Phase 3: Active Braking - Apply brake pulse then precise approach
    print("PHASE 3: Active braking...")
    
    # Apply active brake pulse
    print("Applying brake pulse...")
    for _ in range(3):  # Multiple short pulses for smoother braking
        if direction > 0:  # Right turn - reverse motors briefly
            drive(-BRAKE_POWER, BRAKE_POWER)
        else:  # Left turn - reverse motors briefly
            drive(BRAKE_POWER, -BRAKE_POWER)
        time.sleep(BRAKE_DURATION / 3)
    
    stop_motors()  # Brief stop after brake pulse
    
    # Final precision approach if needed
    if abs(current_angle - target_angle) > TURN_THRESHOLD:
        print("Fine adjustment to target...")
        while abs(target_angle - current_angle) > TURN_THRESHOLD:
            now = monotonic()
            if now - last_check >= 0.005:
                update_angle()
                last_check = now
                
                # Very gentle power for final approach
                final_power = MIN_POWER * 0.7  # 70% of minimum power
                remaining = target_angle - current_angle
                approach_direction = 1 if remaining > 0 else -1
                
                drive(final_power * approach_direction, -final_power * approach_direction)
        
        stop_motors()
    
    print(f"=== Turn complete! Final angle: {current_angle:.1f}° (target: {target_angle:.1f}°) ===")

# Path Execution Functions
def merge_path(path):
    """Merge consecutive forward/backward movements"""
    if not path:
        return path
    
    merged = []
    current = path[0].copy()
    current['duration'] = MOVE_TIME  # Initialize duration
    
    for next_move in path[1:]:
        if (current['type'] in [MOVE_FORWARD, MOVE_BACKWARD] and
            current['type'] == next_move['type']):
            # Merge consecutive forward/backward movements
            current['duration'] += MOVE_TIME
        else:
            merged.append(current)
            current = next_move.copy()
            current['duration'] = MOVE_TIME
    
    merged.append(current)
    return merged

def execute_movement(movement):
    """Execute a single movement command"""
    if movement['type'] == MOVE_FORWARD:
        print(f"\n=== Executing Forward Movement ({movement.get('duration', MOVE_TIME)}s) ===")
        timed_move('FWD', movement.get('duration', MOVE_TIME))
    elif movement['type'] == MOVE_BACKWARD:
        print(f"\n=== Executing Backward Movement ({movement.get('duration', MOVE_TIME)}s) ===")
        timed_move('BACK', movement.get('duration', MOVE_TIME))
    elif movement['type'] == TURN_LEFT:
        print(f"\n=== Executing Left Turn ===")
        turn_by_angle(-90)
    elif movement['type'] == TURN_RIGHT:
        print(f"\n=== Executing Right Turn ===")
        turn_by_angle(90)

def get_reverse_movement(movement):
    """Get the reverse of a movement command"""
    reverse_map = {
        MOVE_FORWARD: MOVE_BACKWARD,
        MOVE_BACKWARD: MOVE_FORWARD,
        TURN_LEFT: TURN_RIGHT,
        TURN_RIGHT: TURN_LEFT
    }
    reversed_move = {'type': reverse_map[movement['type']]}
    
    # Preserve duration for forward/backward movements
    if 'duration' in movement and movement['type'] in [MOVE_FORWARD, MOVE_BACKWARD]:
        reversed_move['duration'] = movement['duration']
    
    return reversed_move

def execute_patrol_sequence():
    """Execute one step of the patrol sequence"""
    global current_mode
    
    # Safety check
    if not patrol_sequence['forward_path']:
        print("No path recorded!")
        current_mode = MODE_MANUAL
        return
    
    # Get current path we're executing
    current_path = patrol_sequence['forward_path'] if patrol_sequence['going_forward'] else patrol_sequence['reverse_path']
    
    # Execute current movement
    if patrol_sequence['current_index'] < len(current_path):
        movement = current_path[patrol_sequence['current_index']]
        execute_movement(movement)
        patrol_sequence['current_index'] += 1
        time.sleep(1.0)  # Full second pause after movement
    else:
        # Reached end of current path, switch directions
        patrol_sequence['going_forward'] = not patrol_sequence['going_forward']
        patrol_sequence['current_index'] = 0
        time.sleep(1.0)  # Full second pause before switching direction

def patrol_runner():
    """Background thread that executes patrol sequence continuously"""
    while current_mode == MODE_PATROL:
        execute_patrol_sequence()
        time.sleep(1.0)  # Pause between iterations

# Hardware Setup (DO NOT MODIFY)
# Left motor
ENA = PWMOutputDevice(13)
IN1 = OutputDevice(23)
IN2 = OutputDevice(27)
# Right motor
ENB = PWMOutputDevice(19)
IN3 = OutputDevice(22)
IN4 = OutputDevice(24)
# Buzzer setup
GPIO.setmode(GPIO.BCM)  # Use BCM numbering
GPIO.setwarnings(False)  # Disable warnings
BUZZER_PIN = 21  # GPIO pin for buzzer
GPIO.setup(BUZZER_PIN, GPIO.OUT)

# Helper Functions
def stop_motors():
    ENA.value = ENB.value = 0
    IN1.off(); IN2.off(); IN3.off(); IN4.off()

def beep_mode_change():
    """Brief beep to indicate mode change - isolated from main logic"""
    try:
        GPIO.output(BUZZER_PIN, GPIO.HIGH)  # Turn buzzer on
        time.sleep(0.1)  # Beep for 100ms
        GPIO.output(BUZZER_PIN, GPIO.LOW)   # Turn buzzer off
    except Exception as e:
        print(f"Beep failed: {e}")  # Log but continue normally
        
def play_connected_sound():
    """Play 'connected' sound when controller connects"""
    try:
        # Non-blocking sound playback
        sound_file = "connected.wav"
        subprocess.Popen(['aplay', sound_file], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as e:
        print(f"Sound playback failed: {e}") #Log but continues normally
        
# Alternative connection sound using buzzer directly
# def play_connection_jingle():
#     """Play a short jingle when connection is established"""
#     try:
#         # Simple connection jingle (Ascending notes)
#         for i in range(3):
#             GPIO.output(BUZZER_PIN, GPIO.HIGH)
#             time.sleep(0.1)
#             GPIO.output(BUZZER_PIN, GPIO.LOW)
#             time.sleep(0.05)
#         # Final Longer Note
#         GPIO.output(BUZZER_PIN, GPIO.HIGH)
#         time.sleep(0.2)
#         GPIO.output(BUZZER_PIN, GPIO.LOW)
#     except Exception as e:
#         print(f"Connection jingle failed: {e}") # Log but continue normally

# ================================================================
# TANK DRIVE MOTOR CONTROL
# DO NOT MODIFY: Basic motor control with PWM working
# ================================================================

def drive(L, R):
    """Drive motors with speed values from -100 to +100"""
    # Left motor
    if L > 0:
        IN1.on(); IN2.off()
        ENA.value = L/100
    elif L < 0:
        IN1.off(); IN2.on()
        ENA.value = abs(L)/100
    else:
        IN1.off(); IN2.off()
        ENA.value = 0
    
    # Right motor
    if R > 0:
        IN3.on(); IN4.off()
        ENB.value = R/100
    elif R < 0:
        IN3.off(); IN4.on()
        ENB.value = abs(R)/100
    else:
        IN3.off(); IN4.off()
        ENB.value = 0

# ================================================================



# Server Implementation
PORT = 5000

def start_server():
    global current_angle, last_gyro_time, current_mode, is_recording, recorded_path
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(("", PORT))
    sock.listen(1)
    print("Robot server listening on port", PORT)
    
    current_angle = 0.0
    last_gyro_time = monotonic()
    
    while True:
        conn, addr = sock.accept()
        print("Connected by", addr)
        play_connected_sound()
        #play_connection_jingle()
        try:
                        with conn.makefile('r') as reader:
                for raw in reader:
                    # Update angle tracking
                    velocity = update_angle()
                    if abs(velocity) > VELOCITY_DEADBAND:
                        print(f"Angle: {current_angle:6.1f}°  (Rate: {velocity:6.1f}°/s)")
                    
                    # Parse commands
                    cmd = raw.strip()
                    if not cmd:
                        continue
                    
                    # Handle recording commands
                    if cmd == "record:start":
                        is_recording = True
                        recorded_path = []  # Clear previous recording
                        print("\n=== Started Recording Path ===")
                        continue
                    elif cmd == "record:stop":
                        is_recording = False
                        print(f"\n=== Recording Complete: {len(recorded_path)} movements ===")
                        
                        # Merge consecutive movements and prepare patrol paths
                        recorded_path = merge_path(recorded_path)
                        print(f"=== Path merged to {len(recorded_path)} movements ===")
                        
                        # Generate and store both paths for patrol mode
                        patrol_sequence['forward_path'] = recorded_path.copy()
                        patrol_sequence['reverse_path'] = merge_path([get_reverse_movement(m) for m in reversed(recorded_path)])
                        patrol_sequence['current_index'] = 0
                        patrol_sequence['going_forward'] = True
                        
                        # Execute reverse path to return to start
                        print("\n=== Returning to start position ===")
                        for movement in patrol_sequence['reverse_path']:
                            print(f"Executing: {movement['type']}")
                            execute_movement(movement)
                            time.sleep(0.5)  # Small pause between movements
                        print("=== Returned to start position ===")
                        continue
                    
                    # Handle mode switching
                    if cmd.startswith("mode:"):
                        mode = cmd.split(":")[1]
                        if mode == "manual":
                            current_mode = MODE_MANUAL
                            beep_mode_change()  # Beep to indicate mode change
                            print("\n=== Switching to MANUAL mode ===")
                        elif mode == "patrol":
                            if not patrol_sequence['forward_path']:
                                print("\nNo path recorded! Staying in manual mode.")
                            else:
                                current_mode = MODE_PATROL
                                patrol_sequence['current_index'] = 0
                                patrol_sequence['going_forward'] = True
                                beep_mode_change()  # Beep to indicate mode change
                                print("\n=== Switching to PATROL mode ===")
                                print(f"Forward path: {len(patrol_sequence['forward_path'])} moves")
                                print(f"Reverse path: {len(patrol_sequence['reverse_path'])} moves")
                                # Start patrol runner in background
                                threading.Thread(target=patrol_runner, daemon=True).start()
                    
                    # Handle patrol mode
                    if current_mode == MODE_PATROL and cmd.startswith("mode:manual"):
                        current_mode = MODE_MANUAL
                        beep_mode_change()  # Beep to indicate mode change
                        print("\n=== Switching to MANUAL mode ===")
                    
                    # Handle commands in manual mode
                    if current_mode == MODE_MANUAL:
                        if cmd.startswith("joystick:"):
                            try:
                                L, R = map(int, cmd[9:].split(","))
                                drive(L, R)
                                print(f"Drive L:{L:4d} R:{R:4d}")
                            except Exception as e:
                                print(f"Invalid joystick cmd: {e}")
                                stop_motors()
                    
                    # Handle button commands (works in any mode while recording)
                    if cmd.startswith("button:") and is_recording:
                        btn = cmd.split(":")[1]
                        movement = None
                        
                        if btn == "fwd":
                            movement = {'type': MOVE_FORWARD, 'duration': MOVE_TIME}
                            timed_move('FWD', MOVE_TIME)
                        elif btn == "back":
                            movement = {'type': MOVE_BACKWARD, 'duration': MOVE_TIME}
                            timed_move('BACK', MOVE_TIME)
                        elif btn == "left":
                            movement = {'type': TURN_LEFT}
                            turn_by_angle(-90)
                        elif btn == "right":
                            movement = {'type': TURN_RIGHT}
                            turn_by_angle(90)
                        
                        if movement:
                            recorded_path.append(movement)
                            print(f"Recorded movement: {movement['type']}")

        except Exception as e:
            print("Connection error:", e)
        finally:
            stop_motors()
            GPIO.output(BUZZER_PIN, GPIO.LOW)  # Ensure buzzer is off
            print("Connection closed")

if __name__ == "__main__":
    # Initialize gyroscope first
    gyro = MPU6050()
    if not gyro.ok:
        print("\nWARNING: Continuing without gyroscope")
        time.sleep(2)  # Give user time to read error
    
    # Set volume to maximum
    subprocess.call(['amixer', 'sset', 'Master', '100%'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    # Start robot server
    stop_motors()
    start_server()
