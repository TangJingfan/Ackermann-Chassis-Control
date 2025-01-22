#!/usr/bin/env python3
import sys
import signal
import Hobot.GPIO as GPIO
import time

def signal_handler(signal, frame):
    sys.exit(0)

signal_pin = 33
min_angle=60
max_angle=120

GPIO.setwarnings(False)

def main():
    GPIO.setmode(GPIO.BOARD)
    p = GPIO.PWM(signal_pin, 50)
    p.start(7.5)

    print("Start servo")
    try:
        angle = min_angle
        increment = 10
        while True:
            duty_cycle = 5 + (angle / 180) * 5  
            p.ChangeDutyCycle(duty_cycle)
            p.start(duty_cycle)
            print(f"Servo is at {angle}.")
            time.sleep(1)
            if min_angle > angle+increment or max_angle< angle+increment:
                increment = -increment
            angle = angle + increment
    except KeyboardInterrupt:
        p.stop()
        GPIO.cleanup()
        print("Abort...")

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main()
