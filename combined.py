import cv2
import numpy as np
import mediapipe as mp
import serial
import time
import speech_recognition as sr
from gtts import gTTS
import os
import playsound

# Mediapipe setup
mp_holistic = mp.solutions.holistic
mp_drawing = mp.solutions.drawing_utils

sentence = []
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
    sentence.append("b")
def c_asl():
    update_servo(2, 25)
    update_servo(6, 95)
    update_servo(3, 95)
    update_servo(5, 95)
    update_servo(4, 95)
    sentence.append("c")
def d_asl():
    update_servo(2, 0)
    update_servo(6, 180)
    update_servo(3, 0)
    update_servo(5, 0)
    update_servo(4, 0)
    sentence.append("d")
def e_asl():
    update_servo(2, 0)
    update_servo(6, 0)
    update_servo(3, 0)
    update_servo(5, 0)
    update_servo(4, 0)
    sentence.append("e")
def f_asl():
    update_servo(2, 0)
    update_servo(6, 0)
    update_servo(3, 180)
    update_servo(5, 180)
    update_servo(4, 180)
    sentence.append("f")
def g_asl():
    update_servo(2, 0)
    update_servo(6, 180)
    update_servo(3, 0)
    update_servo(5, 0)
    update_servo(4, 0)
    sentence.append("g")
def h_asl():
    update_servo(2, 0)
    update_servo(6, 180)
    update_servo(3, 180)
    update_servo(5, 0)
    update_servo(4, 0)
    sentence.append("h")
def i_asl():
    update_servo(2, 0)
    update_servo(6, 0)
    update_servo(3, 0)
    update_servo(5, 0)
    update_servo(4, 180)
    sentence.append("i")
def j_asl():
    update_servo(2, 0)
    update_servo(6, 0)
    update_servo(3, 0)
    update_servo(5, 0)
    update_servo(4, 180)
    sentence.append("j")
def l_asl():
    update_servo(2, 180)
    update_servo(6, 180)
    update_servo(3, 0)
    update_servo(5, 0)
    update_servo(4, 0)
    sentence.append("l")
def m_asl():
    update_servo(2, 0)
    update_servo(6, 0)
    update_servo(3, 0)
    update_servo(5, 0)
    update_servo(4, 0)
    sentence.append("m")
def p_asl():
    update_servo(2, 53)
    update_servo(6, 180)
    update_servo(3, 87)
    update_servo(5, 87)
    update_servo(4, 87)
    sentence.append("p")
def q_asl():
    update_servo(2, 87)
    update_servo(6, 87)
    update_servo(3, 87)
    update_servo(5, 0)
    update_servo(4, 87)
    sentence.append("q")
def u_asl():
    update_servo(2, 180)
    update_servo(6, 180)
    update_servo(3, 0)
    update_servo(5, 0)
    update_servo(4, 0)
    sentence.append("u")
def v_asl():
    update_servo(2, 0)
    update_servo(6, 180)
    update_servo(3, 180)
    update_servo(5, 0)
    update_servo(4, 0)
    sentence.append("v")
def w_asl():
    update_servo(2, 0)
    update_servo(6, 180)
    update_servo(3, 180)
    update_servo(5, 180)
    update_servo(4, 0)
    sentence.append("w")
def x_asl():
    update_servo(2, 0)
    update_servo(6, 90)
    update_servo(3, 0)
    update_servo(5, 0)
    update_servo(4, 0)
    sentence.append("x")
def y_asl():
    update_servo(2, 180)
    update_servo(6, 0)
    update_servo(3, 0)
    update_servo(5, 0)
    update_servo(4, 180)
    sentence.append("y")
def z_asl():
    update_servo(2, 0)
    update_servo(6, 180)
    update_servo(3, 0)
    update_servo(5, 0)
    update_servo(4, 0)
    sentence.append("z")
# Dictionary for text to ASL
asl_map = {'b': b_asl, 'c': c_asl, 'd': d_asl, 'e': e_asl, 'f': f_asl, 'g': g_asl, 'h': h_asl, 'i': i_asl, 'j': j_asl, 'l': l_asl, 'm': m_asl, 'p': p_asl, 'q': q_asl
           , 'u': u_asl, 'v': v_asl, 'w': w_asl, 'x': x_asl, 'y': y_asl, 'z': z_asl}

# Variables
actions = ['b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'l', 'm', 'p', 'q', 'u', 'v', 'w', 'x', 'y', 'z']  # Example actions
sequence = []
sentence = []
predictions = []
threshold = 0.8


def mediapipe_detection(image, model): #image is the frame
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) #color conversion (BlueGreenRed to RedGreenBlue)
    image.flags.writeable = False #sets image writable status to false
    results = model.process(image) #detecting & making prediction
    image.flags.writeable = True #image is now writeable
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR) #color conversion
    return image, results #return image and results

def draw_landmarks(image, results): 
    #draw face connections
    mp_drawing.draw_landmarks(image, results.face_landmarks, mp_holistic.FACEMESH_TESSELATION,
                              mp_drawing.DrawingSpec(color=(80,110,10), thickness=1, circle_radius=1), 
                              mp_drawing.DrawingSpec(color=(80,256,121), thickness=1, circle_radius=1) 
                              )
    #draw pose connections
    mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_holistic.POSE_CONNECTIONS,
                              mp_drawing.DrawingSpec(color=(80,22,10), thickness=2, circle_radius=2), 
                              mp_drawing.DrawingSpec(color=(80,44,121), thickness=2, circle_radius=2) 
                              )
    #draw left hand connections
    mp_drawing.draw_landmarks(image, results.left_hand_landmarks, mp_holistic.HAND_CONNECTIONS,
                              mp_drawing.DrawingSpec(color=(121,22,10), thickness=2, circle_radius=2), 
                              mp_drawing.DrawingSpec(color=(80,44,121), thickness=2, circle_radius=2) 
                              )
    #draw right hand connections
    mp_drawing.draw_landmarks(image, results.right_hand_landmarks, mp_holistic.HAND_CONNECTIONS,
                              mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=2), 
                              mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2)
                              )

def extract_keypoints(results):
    pose = np.array([[res.x, res.y, res.z, res.visibility] for res in results.pose_landmarks.landmark]).flatten() if results.pose_landmarks else np.zeros(33*4)#appends the np array for every landmark which is res x/y/z
    left_hand = np.array([[res.x, res.y, res.z] for res in results.left_hand_landmarks.landmark]).flatten() if results.left_hand_landmarks else np.zeros(21*3) #np zeros: replacement when there is no left/right hand in the frame with an empty array with the same shape
    right_hand = np.array([[res.x, res.y, res.z] for res in results.right_hand_landmarks.landmark]).flatten() if results.right_hand_landmarks else np.zeros(21*3)
    face = np.array([[res.x, res.y, res.z] for res in results.face_landmarks.landmark]).flatten() if results.face_landmarks else np.zeros(468*3) #486 landmarks with 3 values
    return np.concatenate([pose, face, left_hand, right_hand])

# Voice-to-text converter
def voice_to_text():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print("Listening...")
        try:
            audio = recognizer.listen(source, timeout=5)
            text = recognizer.recognize_google(audio)
            print(f"You said: {text}")
            return text
        except sr.UnknownValueError:
            print("Sorry, I did not understand that.")
        except sr.RequestError as e:
            print(f"Could not request results; {e}")
        except sr.WaitTimeoutError:
            print("Listening timed out. Please try again.")
    return None
# Sentence to collect translated letters
sentence = []

def speak_sentence(text):
    tts = gTTS(text=text, lang='en')
    filename = "temp_audio.mp3"
    tts.save(filename)
    playsound.playsound(filename)
    os.remove(filename)

# Main loop
with mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:
    while True:
        print("Type 'camera' to start recognition, 'voice' for voice input, or enter text to translate to ASL:")
        text = input("Type 'voice' to use voice input or text for ASL translation: ").strip().lower()

        if text == "voice":
            voice_input = voice_to_text()
            if voice_input and voice_input in asl_map:
                for letter in voice_input:
                    asl_map[letter]()
            elif voice_input:
                print("Voice input does not match ASL mapping.")

        elif text == "camera":
            cap = cv2.VideoCapture(0)
            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    break

                image, results = mediapipe_detection(frame, holistic)
                draw_landmarks(image, results)

                keypoints = extract_keypoints(results)
                sequence.append(keypoints)
                sequence = sequence[-30:]

                if len(sequence) == 30:
                    res = model.predict(np.expand_dims(sequence, axis=0))[0]
                    action = actions[np.argmax(res)]
                    predictions.append(np.argmax(res))

                    if np.unique(predictions[-10:])[0] == np.argmax(res) and res[np.argmax(res)] > threshold:
                        if not sentence or action != sentence[-1]:
                            sentence.append(action)

                    if len(sentence) > 5:
                        sentence = sentence[-5:]

                print("Detected signs:", ' '.join(sentence))
                speak_sentence(sentence)
                if cv2.waitKey(10) & 0xFF == ord('q'):
                    break

            cap.release()
            cv2.destroyAllWindows()

        elif text in asl_map:
            for letter in text:
                asl_map[letter]()
            text = ''.join(sentence)
            speak_sentence(text)
        else:
            print("Invalid input. Try again.")
