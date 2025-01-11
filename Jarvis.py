import schedule
import time
import threading
import cv2
import winsound
import tkinter as tk
from tkinter import scrolledtext
from tkinter import simpledialog
import speech_recognition as sr
import pyttsx3
import openai
import os
from dotenv import load_dotenv
from google.oauth2.credentials import Credentials
from google_auth_oauthlib.flow import InstalledAppFlow
from google.auth.transport.requests import Request
from googleapiclient.discovery import build
import datetime



def start_gui():
    messages = []

    def send_message():
        user_message = text_entry.get()
        if user_message:
            text_area.insert(tk.END, "You: " + user_message + "\n")
            messages.append({"role": "user", "content": user_message})
            response = send_to_chatGPT(messages)
            text_area.insert(tk.END, "Jarvis: " + response + "\n")
            SpeakText(response)
            text_entry.delete(0, tk.END)

    def record_voice():
        text_area.insert(tk.END, "Listening for voice input...\n")
        user_message = record_text()
        text_area.insert(tk.END, "You: " + user_message + "\n")
        if not user_message.startswith("Error:"):
            messages.append({"role": "user", "content": user_message})
            response = send_to_chatGPT(messages)
            text_area.insert(tk.END, "Jarvis: " + response + "\n")
            SpeakText(response)

    root = tk.Tk()
    root.title("Jarvis Assistant")

    text_area = scrolledtext.ScrolledText(root, wrap=tk.WORD, width=50, height=20, font=("Arial", 12))
    text_area.pack(pady=10)

    text_entry = tk.Entry(root, width=40, font=("Arial", 14))
    text_entry.pack(pady=5)

    send_button = tk.Button(root, text="Send", command=send_message, font=("Arial", 12))
    send_button.pack(pady=5)

    voice_button = tk.Button(root, text="Speak", command=record_voice, font=("Arial", 12))
    voice_button.pack(pady=5)

    root.mainloop()


def alarm_feature():
    def play_alarm():
        while not stop_alarm_event.is_set():
            winsound.Beep(2000, 1000) 

    def check_awake():
        global stop_alarm_event
        stop_alarm_event = threading.Event()

        face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
        eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_eye.xml")

        cap = cv2.VideoCapture(0)
        awake = False

        while not awake:
            ret, frame = cap.read()
            if not ret:
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

            for (x, y, w, h) in faces:
                roi_gray = gray[y:y + h, x:x + w]
                roi_color = frame[y:y + h, x:x + w]

                eyes = eye_cascade.detectMultiScale(roi_gray)
                if len(eyes) >= 1:  
                    awake = True
                    break

            cv2.imshow("Are you awake?", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
        stop_alarm_event.set()

    alarm_thread = threading.Thread(target=play_alarm)
    video_thread = threading.Thread(target=check_awake)
    alarm_thread.start()
    video_thread.start()
    video_thread.join()
    alarm_thread.join()



def schedule_alarm():
    schedule.every().day.at("07:00").do(alarm_feature)



def run_scheduler():
    while True:
        schedule.run_pending()
        time.sleep(1)


# GUI setup
def start_gui():
    threading.Thread(target=run_scheduler, daemon=True).start()  
    root = tk.Tk()
    root.title("Jarvis Assistant")

    text_area = scrolledtext.ScrolledText(root, wrap=tk.WORD, width=50, height=20, font=("Arial", 12))
    text_area.pack(pady=10)

    def start_manual_alarm():
        text_area.insert(tk.END, "Starting alarm manually...\n")
        alarm_feature()
        text_area.insert(tk.END, "Alarm stopped. You are awake!\n")

    manual_alarm_button = tk.Button(root, text="Manual Alarm", command=start_manual_alarm, font=("Arial", 12))
    manual_alarm_button.pack(pady=5)

    root.mainloop()



def SpeakText(command):
    engine.say(command)
    engine.runAndWait()



def record_text():
    try:
        with sr.Microphone() as source:
            r.adjust_for_ambient_noise(source, duration=0.2)
            print("Listening...")
            audio = r.listen(source)
            MyText = r.recognize_google(audio)  
            return MyText
    except sr.RequestError as e:
        return "Error: Unable to access Google Web Speech API."
    except sr.UnknownValueError:
        return "Error: Could not understand the audio."



def send_to_chatGPT(messages, model="gpt-3.5-turbo"):
    response = openai.ChatCompletion.create(
        model=model,
        messages=messages,
    )
    message = response['choices'][0]['message']['content']
    messages.append({"role": "assistant", "content": message})
    return message


SCOPES = ['https://www.googleapis.com/auth/calendar.readonly']


def get_calendar_service():
    creds = None
    if os.path.exists('token.json'):
        creds = Credentials.from_authorized_user_file('token.json', SCOPES)
    if not creds or not creds.valid:
        if creds and creds.expired and creds.refresh_token:
            creds.refresh(Request())
        else:
            flow = InstalledAppFlow.from_client_secrets_file('credentials.json', SCOPES)
            creds = flow.run_local_server(port=0)
        with open('token.json', 'w') as token:
            token.write(creds.to_json())
    return build('calendar', 'v3', credentials=creds)



def get_upcoming_events():
    service = get_calendar_service()
    now = datetime.datetime.utcnow().isoformat() + 'Z'  #UTC time
    events_result = service.events().list(
        calendarId='primary', timeMin=now,
        maxResults=10, singleEvents=True,
        orderBy='startTime'
    ).execute()
    events = events_result.get('items', [])
    if not events:
        return "No upcoming events found."

  
    events_str = "Upcoming Events:\n"
    for event in events:
        start = event['start'].get('dateTime', event['start'].get('date'))
        events_str += f"{start} - {event['summary']}\n"
    return events_str

def show_events():
    events = get_upcoming_events()
    text_area.insert(tk.END, events + "\n")

load_dotenv()
openai.api_key = "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"
r = sr.Recognizer()
engine = pyttsx3.init()

calendar_button = tk.Button(root, text="Show Calendar Events", command=show_events, font=("Arial", 12))
calendar_button.pack(pady=5)

if __name__ == "__main__":
    schedule_alarm()  
    start_gui()
    show_events()
