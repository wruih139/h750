
down_car (STM32H750 + STM32duino) — 8-sensor PID Line Following
What this does
8-sensor I2C line following with PID
Route scripting: Forward(n intersections), turns, push blocks
Parameters centralized in config.h
Wiring used by code
Start button
PA1 (KEY_PIN), INPUT_PULLUP
Line follower (I2C)
Addr: 0x78 (7-bit Arduino Wire address)
SCL -> PB6
SDA -> PB7
Read reg 1, 1 byte
Bit i: 0 = black area (no line), 1 = white line detected
Motors (PWM + DIR)
Motor 0 (front-left):  PWM -> PD14,  DIR -> PD9
Motor 1 (front-right): PWM -> PA15,  DIR -> PD10
Motor 2 (rear-left):   PWM -> PB0,   DIR -> PB1
Motor 3 (rear-right):  PWM -> PA2,   DIR -> PA5
First bring-up checklist (important)
Lift the car and verify motor directions:
If forward command spins wheels backward or twists: change MOTOR_DIR_DEFAULT[4] in config.h。
Run only line following first:
Temporarily make the route just Forward(1) to tune PID before full strategy.
PID tuning
In config.h:

KP， KI， KD
MAX_TURN_CMD
TRACKING_SPEED
Typical tuning:

Oscillation: lower KP or raise KD
Slow response: raise KP
Usually keep KI = 0 first; only add small KI if there's steady bias.
Intersection detection
An intersection is detected when:

both edges (sensor 0 and 7) see white line (bit=1)
wide center (2..5) see white line (bit=1)
hit count >= INTERSECTION_MIN_HIT Plus debounce INTERSECTION_DEBOUNCE_MS。
If false triggers happen on thick lines:

increase INTERSECTION_MIN_HIT
increase INTERSECTION_DEBOUNCE_MS
