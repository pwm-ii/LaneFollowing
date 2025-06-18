import time
import sensor
from machine import LED, PWM

# Threshold for white/grey line detection
GRAYSCALE_THRESHOLD = [(200, 255)]

ROIS = [
    (20, 20, 200, 20),
    (20, 40, 200, 20),
    (20, 60, 200, 20),
    (20, 80, 200, 20),
    (20, 100, 200, 20),
    (20, 120, 200, 20),
]

# PWM Setup
p0 = PWM("P0", freq=1600)  # DC Motor
p0.duty_ns(0)

p8 = PWM("P8", freq=100)   # Servo
p8.duty_ns(1500000)

CONST_SERVO_NS = 1500000  # Neutral position (1.5 ms pulse)
SAFE_DC_MOTOR_DUTY_NS = 81250  # Constant DC motor power

# Delay for hardware init
time.sleep(2)

# Sensor setup
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.HQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
clock = time.clock()

# LEDs
LED("LED_RED").off()
LED("LED_BLUE").off()
LED("LED_GREEN").off()

# PID controller
def pid_controller(setpoint, pv, kp, ki, kd, previous_error, integral, dt):
    error = setpoint - pv
    integral += error * dt
    integral = max(min(integral, 100), -100)  # Anti-windup
    derivative = (error - previous_error) / dt
    control = kp * error + ki * integral + kd * derivative
    return control, error, integral

# PID parameters
setpoint = 0
kp = 28500
ki = 0
kd = 200
previous_error = 0
integral = 30
dt = 0.01

# [MOD] Initialize last_blob_time
last_blob_time = time.ticks_ms()

try:
    while True:

        clock.tick()
        img = sensor.snapshot()

        current_time = time.ticks_ms()  # [MOD] Get current time
        blobs_found = False  # [MOD] Flag to check if any blob is detected

        p0.duty_ns(SAFE_DC_MOTOR_DUTY_NS)

        errorholder = []
        center_x = img.width() // 2
        midpoints = []

        for i, roi in enumerate(ROIS, start=1):
            blobs = img.find_blobs(GRAYSCALE_THRESHOLD, roi=roi, merge=True)
            filtered_blobs = [b for b in blobs if 100 < b.pixels() <= 350]

            if filtered_blobs:
                blobs_found = True  # [MOD] Blobs detected in this frame

                left_blob = None
                right_blob = None

                sorted_blobs = sorted(filtered_blobs, key=lambda b: abs(b.cx() - center_x))

                for b in sorted_blobs:
                    if b.cx() < center_x and left_blob is None:
                        left_blob = b
                    elif b.cx() > center_x and right_blob is None:
                        right_blob = b
                    if left_blob and right_blob:
                        break

                if left_blob and right_blob:
                    x_min, y_min = left_blob.cx(), left_blob.cy()
                    x_max, y_max = right_blob.cx(), right_blob.cy()

                    x_mid = (x_min + x_max) // 2
                    y_mid = (y_min + y_max) // 2

                    error = center_x - x_mid
                    errorholder.append(error)

                    midpoints.append((x_mid, y_mid))

                    img.draw_rectangle(roi, color=255, thickness=2)
                    img.draw_rectangle(left_blob.rect(), color=1)
                    img.draw_cross(x_min, y_min, color=1)
                    img.draw_rectangle(right_blob.rect(), color=1)
                    img.draw_cross(x_max, y_max, color=1)
                    img.draw_cross(center_x, y_mid, color=255, size=5, thickness=2)

        # Draw line connecting all x_mid, y_mid points
        for i in range(len(midpoints) - 1):
            x1, y1 = midpoints[i]
            x2, y2 = midpoints[i + 1]
            img.draw_line(x1, y1, x2, y2, color=255, thickness=2)

        # [MOD] Handle loss of blob detection
        if blobs_found:
            last_blob_time = current_time
        else:
            elapsed = time.ticks_diff(current_time, last_blob_time) / 1000  # ms to seconds
            if elapsed > 2:
                print("No blobs detected for 6 seconds. Stopping motor and resetting servo.")  # [MOD]
                p0.duty_ns(0)  # [MOD]
                p8.duty_ns(CONST_SERVO_NS)  # [MOD]
                time.sleep(dt)
                continue

        if errorholder:
            avg_error = sum(errorholder) / len(errorholder)
            print(f"\nAverage Horizontal Error: {avg_error:.2f}")

            # Adjust kp based on error magnitude
            if abs(avg_error) > 8:
                kp = 30800
            else:
                kp = 28500


        control, previous_error, integral = pid_controller(
            setpoint, avg_error, kp, ki, kd,
            previous_error, integral,
            dt
        )

        servo_ns = int(CONST_SERVO_NS - control)
        servo_ns = max(1000000, min(2000000, servo_ns))  # Clamp to 10%-20% duty cycle

        print(f"Control: {control:.3f}, Servo Pulse: {servo_ns}")

        p8.duty_ns(servo_ns)

        time.sleep(dt)

finally:
    p0.duty_ns(0)
    p8.duty_ns(CONST_SERVO_NS)
    p8.deinit()
    p0.deinit()
