from PySide6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QLabel, QLineEdit, QPushButton, QTextEdit
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtCore import Qt, QTimer
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
class VideoApp(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Science Fair")

        self.output_box = QTextEdit(self)
        self.output_box.setPlaceholderText("Detections will be displayed here.")
        self.output_box.setAlignment(Qt.AlignmentFlag.AlignLeft)
        self.output_box.setReadOnly(True)

        self.video_label = QLabel(self)
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.user_input = QLineEdit(self)
        self.user_input.setPlaceholderText("Enter something...")
        self.user_input.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        self.user_input.returnPressed.connect(self.on_enter_pressed)
        

        self.start_button = QPushButton("Start Video", self)
        self.start_button.clicked.connect(self.start_video)

        layout = QVBoxLayout()
        layout.addWidget(self.video_label)
    
        layout.addWidget(self.user_input)
        layout.addWidget(self.start_button)
        layout.addWidget(self.output_box)
        
        container = QWidget()
        container.setLayout(layout
)
        self.setCentralWidget(container)

        #self.video_capture = cv2.VideoCapture(0)
        #self.timer = QTimer(self)
        #self.timer.timeout.connect(self.update_frame)
        
        self.close_button = QPushButton("Close", self)
        self.close_button.clicked.connect(self.close_application)
        layout.addWidget(self.close_button)
        
    def close_application(self):
        self.video_capture.release()
        self.close()

    def on_user_input_changed(self, text):
        self.user_input_text = text

    def on_enter_pressed(self):
        self.user_input_text = self.user_input.text()
        self.user_input.clear()
        user_input_list = self.user_input
        if user_input_list == "b":
            b_asl()
        if user_input_list == "c":
            c_asl()
        if user_input_list == "d":
            d_asl()
        if user_input_list == "e":
            e_asl()

    def start_video(self):
        self.timer.start(30)  # Update every 30 milliseconds

            
    def closeEvent(self, event):
        #self.video_capture.release()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = VideoApp()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
