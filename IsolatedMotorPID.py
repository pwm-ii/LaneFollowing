
import time
import math
from machine import LED, Timer, Pin, PWM

# === Globals === #
redled = LED("LED_RED")
MeasuredVal = 0
Omega = 0.0  # Initialize to avoid NameError

# === Timer Callback === #
def tick(timer):
    global MeasuredVal
    global Omega
    Omega = (5 * math.pi * MeasuredVal) / 4
    print("Omega:", Omega)
    MeasuredVal = 0

# === Interrupt Handler === #
def isr(p):
    global MeasuredVal
    #redled.toggle()
    MeasuredVal += 1

# === PWM Setup === #
p7 = PWM("P7", freq=1600)  # DC Motor
p7.init(freq=1600)

p8 = PWM("P8", freq=100)  # Servo
p8.init(freq=100)

# === PID Controller === #
def pid_controller(setpoint, pv, kp, ki, kd, previous_error, integral, dt):
    error = setpoint - pv
    integral += error * dt
    derivative = (error - previous_error) / dt
    control = kp * error + ki * integral + kd * derivative
    return control, error, integral

# === PID Variables === #
setpoint = 168
kp = 100
ki = 10
kd = 50
previous_error = 0
integral = 0
dt = 0.05

# === PWM Output Function === #
def set_pwm(dc_motor_ns, servo_ns):
    p7.duty_ns(dc_motor_ns)
    #time.sleep_ms(15)
    p8.duty_ns(servo_ns)

# === Setup Timer and Interrupts === #
tim = Timer(-1, freq=1, callback=tick)
pin = Pin("P5", Pin.IN, Pin.PULL_UP)
pin.irq(trigger=Pin.IRQ_FALLING, handler=isr)

# === Run Loop === #
CONST_SERVO_NS = 1500000  # neutral position for servo

while True:
    # PID control using Omega
    control, previous_error, integral = pid_controller(
        setpoint, Omega, kp, ki, kd, previous_error, integral, dt
    )

    print("Angular Velocity:", Omega)
    print("Control Output:", control)

    # Motor PWM: map control to duty_ns (adjust scale as needed)
    motor_ns = int(control)
    print("motor PWM:", motor_ns)
    motor_ns = max(62500, min(312500, motor_ns))  # clamp to 10%-50%

    # Apply PWM signals
    #set_pwm(dc_motor_ns=motor_ns, servo_ns=CONST_SERVO_NS)
    p7.duty_ns(motor_ns + 90000)

    time.sleep(dt)
