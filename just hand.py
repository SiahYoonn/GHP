import cv2
import sys
import os
import serial  
import time
#Set up the Arduino connection
port = 'COM3'  # Change to your Arduino's port
ser = serial.Serial(port, 9600)  # Adjust baud rate as needed

# Attach servos to their respective pins
servo_pins = [2, 6, 3, 5, 4]
finger_names = ["Thumb", "Index", "Middle", "Ring", "Pinky"]


def update_servo(pin, angle):
    if pin == 2:  # If the servo is the Thumb
        angle = 180 - int(angle)  # Reverse the angle

    # Send servo pin and angle as a string over the serial connection
    command = f"{pin}:{angle}\n"
    ser.write(command.encode())
    time.sleep(1)

#functions for each letter
def b_asl():
    update_servo(2, 26)
    update_servo(6, 180)
    update_servo(3, 180)
    update_servo(5, 180)
    update_servo(4, 180)
def c_asl():
    update_servo(2, 25)
    update_servo(6, 95)
    update_servo(3, 95)
    update_servo(5, 95)
    update_servo(4, 95)
def d_asl():
    update_servo(2, 0)
    update_servo(6, 180)
    update_servo(3, 0)
    update_servo(5, 0)
    update_servo(4,0)
def e_asl():
    update_servo(2, 0)
    update_servo(6, 0)
    update_servo(3, 0)
    update_servo(5, 0)
    update_servo(4, 0)

text_list = []
text = input("Type what you want to say: ")
x = 0
for letter in range(len(text)):
    if letter == "b":
        b_asl()
    if letter == "c":
        c_asl()
    if letter == "d":
        d_asl()
    if letter == "e":
        e_asl()
    x+=1
