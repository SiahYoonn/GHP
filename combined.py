import cv2
import numpy as np
import mediapipe as mp
import serial
import time

# Mediapipe setup
mp_holistic = mp.solutions.holistic
mp_drawing = mp.solutions.drawing_utils

# Serial setup for Arduino
port = 'COM3'  # Change to your Arduino's port
ser = serial.Serial(port, 9600)

# Servo setup
servo_pins = [2, 6, 3, 5, 4]
finger_names = ["Thumb", "Index", "Middle", "Ring", "Pinky"]

def update_servo(pin, angle):
    if pin == 2:  # Reverse angle for the thumb
        angle = 180 - int(angle)
    command = f"{pin}:{angle}\n"
    ser.write(command.encode())
    time.sleep(0.5)

# Servo control for ASL letters
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
    update_servo(4, 0)

def e_asl():
    update_servo(2, 0)
    update_servo(6, 0)
    update_servo(3, 0)
    update_servo(5, 0)
    update_servo(4, 0)

# Dictionary for text-to-ASL mapping
asl_map = {'b': b_asl, 'c': c_asl, 'd': d_asl, 'e': e_asl}

# Model and variables for detection
actions = ['b', 'c', 'd', 'e']  # Example actions
sequence = []
sentence = []
predictions = []
threshold = 0.8

# Mediapipe detection function
def mediapipe_detection(image, model):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = model.process(image)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image, results

# Keypoint extraction function
def extract_keypoints(results):
    # Simplified for example purposes
    return np.array([])

# Main loop
with mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:
    while True:
        # User input
        text = input("Type 'camera' to start recognition or enter text to translate to ASL: ").strip().lower()
        
        if text == "camera":
            cap = cv2.VideoCapture(0)
            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    break

                # Mediapipe detection
                image, results = mediapipe_detection(frame, holistic)
                draw_landmarks(image, results)

                # Prediction logic
                keypoints = extract_keypoints(results)
                sequence.append(keypoints)
                sequence = sequence[-30:]

                if len(sequence) == 30:
                    res = model.predict(np.expand_dims(sequence, axis=0))[0]
                    action = actions[np.argmax(res)]
                    predictions.append(np.argmax(res))

                    # Update sentence if confidence threshold is met
                    if np.unique(predictions[-10:])[0] == np.argmax(res) and res[np.argmax(res)] > threshold:
                        if not sentence or action != sentence[-1]:
                            sentence.append(action)

                    if len(sentence) > 5:
                        sentence = sentence[-5:]

                print("Detected signs:", ' '.join(sentence))
                if cv2.waitKey(10) & 0xFF == ord('q'):
                    break

            cap.release()
            cv2.destroyAllWindows()

        elif text in asl_map:
            for letter in text:
                asl_map[letter]()
        else:
            print("Invalid input. Try again.")
