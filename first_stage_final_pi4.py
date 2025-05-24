import RPi.GPIO as GPIO
import time

last_servo_duty = 7.5

# Setup GPIO for ultrasonic sensors
GPIO.setmode(GPIO.BCM)

# Define GPIO pins for ultrasonic sensors
RIGHT_TRIG = 11
RIGHT_ECHO = 22
LEFT_TRIG = 10
LEFT_ECHO = 17
CENTRAL_TRIG = 9
CENTRAL_ECHO = 27

# Define GPIO Motor control pins
IN1 = 15   # Motor forward
IN2 = 14   # Motor backward
ENA = 18  # PWM for speed

# Servo pin
SERVO_PIN = 12

#Setup pins
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)

GPIO.setup(RIGHT_TRIG, GPIO.OUT)
GPIO.setup(RIGHT_ECHO, GPIO.IN)
GPIO.setup(LEFT_TRIG, GPIO.OUT)
GPIO.setup(LEFT_ECHO, GPIO.IN)
GPIO.setup(CENTRAL_TRIG, GPIO.OUT)
GPIO.setup(CENTRAL_ECHO, GPIO.IN)

# PWM setup
pwm_motor = GPIO.PWM(ENA, 1000)  # 1kHz PWM
pwm_motor.start(35) 

pwm_servo = GPIO.PWM(SERVO_PIN, 50)  # 50Hz for servo
pwm_servo.start(35) 

def set_speed(speed_percent):
    pwm_motor.ChangeDutyCycle(speed_percent)

def set_servo_duty(duty):
    global last_servo_duty

    # Skip if duty hasn't changed
    if duty == last_servo_duty:
        return 
    
    pwm_servo.ChangeDutyCycle(duty)
    time.sleep(0.3)

    last_servo_duty = duty # Update the last duty

def go_forward():
    set_servo_duty(7.5)  # Forward
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    set_speed(35)  # Adjust speed (0–100%)

def turn_left(duty=5,sec=0.01):
    set_servo_duty(duty)  # Turn left
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    set_speed(35)
    time.sleep(sec)

def turn_right(duty=10,sec=0.01):
    set_servo_duty(duty)  # Turn right
    GPIO.output(IN1, True)
    GPIO.output(IN2, False)
    set_speed(35)
    time.sleep(sec)

def stop():
    GPIO.output(IN1, False)
    GPIO.output(IN2, False)
    set_speed(35)

def measure_distance(trig_pin, echo_pin):
    # Send 10us pulse to trigger
    GPIO.output(trig_pin, False)
    time.sleep(0.05)  # Let sensor settle
    GPIO.output(trig_pin, True)
    time.sleep(0.00001)  # 10 microseconds
    GPIO.output(trig_pin, False)

    # Wait for echo start
    timeout = time.time() + 1  # 1s timeout
    while GPIO.input(echo_pin) == 0:
        if time.time() > timeout:
            return None  # Timeout
    pulse_start = time.time()

    # Wait for echo end
    timeout = time.time() + 1
    while GPIO.input(echo_pin) == 1:
        if time.time() > timeout:
            return None
    pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start

    # Speed of sound is ~34300 cm/s
    distance_cm = pulse_duration * 17150
    return round(distance_cm, 2)

last_central_distance = measure_distance(CENTRAL_TRIG, CENTRAL_ECHO)
duration = 85

try:
    turn_time=time.time()
    while True:
        #Measure distance
        distance_central=measure_distance(CENTRAL_TRIG,CENTRAL_ECHO)
        distance_left=measure_distance(LEFT_TRIG,LEFT_ECHO)
        distance_right=measure_distance(RIGHT_TRIG,RIGHT_ECHO)

        print("Lijeva distanca je: ", distance_left)
        print("Centralna distanca je: ", distance_central)
        print("Desna distanca je: ", distance_right)

        #Movement logic
        if distance_left > distance_right:
          turn_left()
        elif distance_right > distance_left:
          turn_right()
        else:
          go_forward()
        if distance_central < 5 or time.time() - turn_time > duration:
          stop()
          break
        time.sleep(0.01)      
            
except KeyboardInterrupt:
    print("Interrupted")
finally:
    stop()
    pwm_motor.stop()
    pwm_servo.stop()
    GPIO.cleanup()
