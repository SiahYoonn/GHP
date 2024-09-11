import rospy
from sensor_msgs.msg import Image
import cv2
import sys
import time
import math
import numpy as np
from numpy import absolute, sqrt, square
import ros_numpy as rnp
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from array import array
from ultralytics import YOLO

#connect to the drone dronekit
vehicle = connect('udp:127.0.0.1:14551',wait_ready=True)

#parameters
vehicle.parameters['SIM_SONAR_SCALE']=10
vehicle.parameters['RNGFND1_TYPE']=1
vehicle.parameters['RNGFND1_SCALING']=10
vehicle.parameters['RNGFND1_PIN']=0
vehicle.parameters['RNGFND1_MAX_CM']=5000
vehicle.parameters['RNGFND1_MIN_CM']=0
vehicle.parameters['PLND_ENABLED']=1
vehicle.parameters['PLND_TYPE']=1
vehicle.parameters['PLND_EST_TYPE']=0
vehicle.parameters['LAND_SPEED']=30 ##cm/s

velocity=.3 #m/s
takeoff_height=13 #meters


#Set up a ROS publisher
newimg_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)



target_size = 121.9 ##CM

horizontal_resolution = 720
vertical_resolution = 576
horizontal_fov = 62.2*math.pi/180.0 ##degrees to radians
vertical_fov = 48.8*math.pi/180.0 ##degrees to radians THE MATH DONT USE RADIANS AHHHHHHHHHHHH

time_last=0
time_to_wait=.1 ##100 ms

#camera intrinsics
dist_coeffs = [0.0, 0.0, 0.0, 0.0, 0.0]
camera_matrix = [[1061.6538553425996, 0.0, 640.5],[0.0, 1061.6538553425996, 360.5],[0.0, 0.0, 1.0]]
np_camera_matrix = np.array(camera_matrix)
np_dist_coeffs = np.array(dist_coeffs)


#functions
def arm_and_takeoff(target_altitude):
    while vehicle.is_armable==False:
        print("Waiting for vehicle to become armable...")
        time.sleep(1)
    print('Vehicle is now armable')

    vehicle.mode = VehicleMode("GUIDED")

    while vehicle.mode!="GUIDED":
        print("Waiting for vehicle to enter GUIDED mode...")
        time.sleep(1)
    print('Vehicle is now in GUIDED mode')

    vehicle.armed = True
    while vehicle.armed== False:
        print("Waiting for vehicle to become armed...")
        time.sleep(1)
    print('Vehicle is now armed')

    vehicle.simple_takeoff(target_altitude)

    while True:
        print("Current altitude: %d" %vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >=.95*target_altitude:
            break
        time.sleep(1)
    print("Reached target altitude")

    return None

#send velocity command to UAV
def send_local_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0,
        0,
        0,
        vx,
        vy,
        vz,
        0,0,0,0,0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


def send_land_message(x,y):
    print("Sending land message with x angle: "+ str(x) +", y angle: "+ str(y))
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        0,#mavutil.mavlink.MAV_FRAME_BODY_NED,
        x*-1,
        y*-1,
        0,
        0,
        0
        )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def msg_receiver(message):
    global time_last, time_to_wait
    if time.time() - time_last > time_to_wait:
        frame = message 

        #import altitude
        alt = 6

        #threshold variables
        alt_switch = 3
        fps = 1

        #convert image to color binary
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        ROI_number = 0
        model = YOLO("yolov8.pt")
        results = model.predict(source=frame, show=True, conf=0.8)
        if results:
          for result in results:
            # Check if the result contains boxes
            for box in result.boxes:
              x1, y1, x2, y2 = box.xyxy[0]  # Get the bounding box coordinates
              # Calculate the center of the detected bounding box
              box_center_x = (x1 + x2) / 2
              box_center_y = (y1 + y2) / 2
              frame_center_x = 820
              frame_center_y =616
              # Calculate the error between the center of the frame and the center of the bounding box
              error_x = frame_center_x - box_center_x
              error_y = frame_center_y - box_center_y
        
        x_ang = (box_center_x - 360)*horizontal_fov/720
        #print(x_ang)
        y_ang = (box_center_y - 288)*vertical_fov/576
        #print(y_ang)
        if vehicle.mode != 'LAND':
            vehicle.mode = VehicleMode('LAND')
            while vehicle.mode!= 'LAND':
                time.sleep(.5)
            print('Vehicle in LAND mode')
            send_land_message(x_ang,y_ang)
        else:
            send_land_message(x_ang,y_ang)
        # Define thresholds for acceptable error range
        error_threshold = 5  # pixels
        SOUTH = -0.3
        NORTH = 0.3
        WEST = -0.3
        EAST = 0.3
        UP = -0.5  # Note: up is negative!
        DOWN = 0.5
        # Move the drone based on the error
        if abs(error_x) > error_threshold or abs(error_y) > error_threshold:
          if error_x > error_threshold:
            send_local_ned_velocity(SOUTH, 0, 0, 1)
          elif error_x < -error_threshold:
            send_local_ned_velocity(NORTH, 0, 0, 1)
          if error_y > error_threshold:
            send_local_ned_velocity(0, EAST, 0, 1)
          elif error_y < -error_threshold:
            send_local_ned_velocity(0, WEST, 0, 1)
          elif abs(error_x) < error_threshold or abs(error_y) < error_threshold:
            vehicle.mode = VehicleMode("LAND")
        #published process image back from a topic
        new_msg = rnp.msgify(Image, frame,encoding='rgb8')
        newimg_pub.publish(new_msg)
        time_last = time.time()
    else:
        return None
    

def subscriber(): #receives images from a topic
    rospy.init_node('drone_node', anonymous=False)
    sub = rospy.Subscriber('/camera/color/image_raw', Image, msg_receiver)
    rospy.spin()

if __name__ == "__main__":
    try:
        arm_and_takeoff(takeoff_height)
        time.sleep(1)
        send_local_ned_velocity(velocity,0,0)
        time.sleep(1)
        subscriber()
    except rospy.ROSInterruptException:
            pass





