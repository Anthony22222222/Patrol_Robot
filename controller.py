import network, socket, time
from machine import Pin, ADC

# LCD Driver for HD44780 in 4-bit mode
rs = Pin(22, Pin.OUT)
e  = Pin(21, Pin.OUT)
d4 = Pin(20, Pin.OUT)
d5 = Pin(19, Pin.OUT)
d6 = Pin(18, Pin.OUT)
d7 = Pin(17, Pin.OUT)

def pulse_enable():
    e.off(); time.sleep_us(1)
    e.on();  time.sleep_us(1)
    e.off(); time.sleep_us(100)

def send_nibble(n):
    d4.value(n & 1)
    d5.value((n >> 1) & 1)
    d6.value((n >> 2) & 1)
    d7.value((n >> 3) & 1)
    pulse_enable()

def lcd_cmd(c):
    rs.off()
    send_nibble((c >> 4) & 0x0F)
    send_nibble(c & 0x0F)
    time.sleep_ms(2)

def lcd_data(d):
    rs.on()
    send_nibble((d >> 4) & 0x0F)
    send_nibble(d & 0x0F)
    time.sleep_ms(2)

def lcd_init():
    e.off(); rs.off(); time.sleep_ms(20)
    for cmd in (0x33,0x32,0x28,0x0C,0x06,0x01):
        lcd_cmd(cmd)
        if cmd == 0x01: time.sleep_ms(2)

def lcd_clear():
    lcd_cmd(0x01); time.sleep_ms(2)

def lcd_move(col, row):
    lcd_cmd(0x80 | (0x40 if row else 0) | col)

def lcd_putstr(s):
    for ch in s: lcd_data(ord(ch))

# initialize LCD
lcd_init()
lcd_clear()
lcd_move(0, 0)
lcd_putstr("Mode: Manual")

# Wi-Fi & Socket Connection
MAX_WIFI_WAIT = 10
wlan = network.WLAN(network.STA_IF)
wlan.active(True)

def connect_wifi():
    start = time.time()
    wlan.connect("RobotNet", "password123")
    while not wlan.isconnected():
        if time.time() - start > MAX_WIFI_WAIT:
            return False
        time.sleep(0.1)
    return True

def connect_robot():
    s = socket.socket()
    try:
        s.connect(("192.168.50.1", 5000))
        return s
    except:
        return None

# Establish connection (retry until successful)
while not connect_wifi():
    time.sleep(1)
sock = None
while not sock:
    sock = connect_robot()
    time.sleep(1)

def send(cmd):
    try:
        sock.send((cmd+"\n").encode())
    except:
        pass  # will reconnect in next send

# Hardware Pin Mappings
buttons = {
    "FWD":    Pin(0, Pin.IN, Pin.PULL_UP),
    "BACK":   Pin(1, Pin.IN, Pin.PULL_UP),
    "LEFT":   Pin(3, Pin.IN, Pin.PULL_UP),
    "RIGHT":  Pin(2, Pin.IN, Pin.PULL_UP),
    "mode":   Pin(4, Pin.IN, Pin.PULL_UP),
    "record": Pin(5, Pin.IN, Pin.PULL_UP),
}
joyL = ADC(Pin(26))
joyR = ADC(Pin(27))

# Control Parameters

# Joystick Control
DEADZONE = 3  # ignore tiny movements (percent)

# Mode Settings
MODE_MANUAL = 'manual'
MODE_PATROL = 'patrol'
current_mode = MODE_MANUAL

# Path Recording
is_recording = False
moves_recorded = 0  # Track number of recorded moves

# Show initial mode
lcd_clear()
lcd_move(0, 0)
lcd_putstr("Mode: Manual")

# Button debounce
last_press = {btn: time.time() for btn in buttons}
DEBOUNCE_TIME = 0.1  # Reduced from 0.2 for more responsive controls

while True:
    # Record button handling
    if not buttons['record'].value():  # Record button pressed
        if time.time() - last_press['record'] > DEBOUNCE_TIME:
            if not is_recording:
                is_recording = True
                moves_recorded = 0  # Reset counter
                send("record:start")
                lcd_clear()
                lcd_move(0, 0)
                lcd_putstr("Recording (#0)")
            else:
                is_recording = False
                send("record:stop")
                lcd_clear()
                lcd_move(0, 0)
                lcd_putstr("Returning to")
                lcd_move(0, 1)
                lcd_putstr("start...")
                time.sleep(2)  # Wait for return sequence
                lcd_clear()
                lcd_move(0, 0)
                lcd_putstr("Mode: Manual")
            last_press['record'] = time.time()
            time.sleep(0.1)  # Extra debounce
    
    # Mode button handling
    if not buttons['mode'].value():  # Mode button pressed
        if time.time() - last_press['mode'] > DEBOUNCE_TIME:
            if current_mode == MODE_MANUAL:
                current_mode = MODE_PATROL
                send("mode:patrol")
                lcd_clear()
                lcd_move(0, 0)
                lcd_putstr("Mode: Patrol")
            else:  # In patrol mode
                current_mode = MODE_MANUAL
                send("mode:manual")
                lcd_clear()
                lcd_move(0, 0)
                lcd_putstr("Mode: Manual")
            last_press['mode'] = time.time()
            time.sleep(0.1)  # Extra debounce
    
    # Manual mode joystick control
    if current_mode == MODE_MANUAL:
        # Read joysticks (-100 to +100)
        L = int((joyL.read_u16() - 32768) / 327.68)
        R = int((joyR.read_u16() - 32768) / 327.68)
        
        # Apply deadzone
        L = 0 if abs(L) < DEADZONE else L
        R = 0 if abs(R) < DEADZONE else R
        
        # Clamp values
        L = max(min(L, 100), -100)
        R = max(min(R, 100), -100)
        
        # Send to robot
        send(f"joystick:{L},{R}")
        
        # Update display
        lcd_move(0, 1)
        lcd_putstr(f"L:{L:4d} R:{R:4d}")
    
    # Direction buttons for path recording
    if is_recording:
        for btn, pin in buttons.items():
            if btn in ['FWD', 'BACK', 'LEFT', 'RIGHT']:
                if not pin.value():  # Button pressed
                    if time.time() - last_press[btn] > DEBOUNCE_TIME:
                        send(f"button:{btn.lower()}")
                        last_press[btn] = time.time()
                        
                        # Update move counter and display
                        moves_recorded += 1
                        lcd_move(0, 0)
                        lcd_putstr(f"Recording (#{moves_recorded})")
                        lcd_move(0, 1)
                        lcd_putstr(f"Last: {btn:6s}")
                        time.sleep(0.1)  # Extra debounce
    

    
    # Main loop timing delay
    time.sleep(0.02)
