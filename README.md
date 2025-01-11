GHP Portfolio:
My name is Siah Yoon and I am a junior. I hope to deepen my understanding of machine learning and its real-world applications. Currently, I am working on several machine learning projects, but I heavily rely on documentation as I’m still in the learning phase. I believe that the summer program will help me take a step further and advance my knowledge and gain practical skills. I also hope to learn how machine learning can be applied to other fields of study, because its various applications always astonish me. Last year I did a presentation on AI in healthcare, so I am especially looking forward to the application of machine learning in biomedical engineering.
I have a passion for programming/machine learning and especially in applications involving UAVs or tennis.
I've tried out the YOLO models as well as tensorflow

videoloop.py is a UAV camera script
1. Stabilized video: easier for pilots to assume center
2. Successfully connects the Raspberry Pi to the UAV & runs as the pi boots up
3. Visualization of flight data

AUTO LANDING 2024 (Gazebo).py is an autonomous landing software for a UAV, applied on the gazebo simulation software
1. Utilization of ROS Melodic to simulate a virtual UAV + camera with all required parameters
2. Python Dronekit Library to control UAV from the Raspberry Pi
3. YOLOv8 model to detect targets & algorithm to center the UAV to it (data for training was collected by pilot)

AUTO LANDING 2024 (UAV),py is an autonomous landing software for a UAV, applied on an actual UAV
1. Identical to the gazebo simulation however ROS is not required and the code is attatched onto the camera script

Sign Language Recognition.ipynb is the ASL Translator using machine learning (ASL to English)
1. Mediapipe Holistic model to plot landmarks
2. Numpy arrays to label them & collected 30 videos w/ 30 sequences
3. Tensorflow Keras LSTM model to analyze the change in landmarks

just hand.py is the English to ASL portion in the ASL translator
1. Pyserial library to connect python to the arduino
2. Adjustments of servo angles for mimicry
3. Slicing a string to express in order

combined.py combines all three parts of the ASL translator
1. ML model deployed for detection
2. Arduino mmimicry using lists & dictionaries
3. Text-to-speech and audio recognition

Jarvis.py is a personalized assistant I developed to help me access information, wake up better in the morning, and manage my schedule
1. Links to OpenAI using audio recognition
2. OpenCV for sleep detection & alarms using schedule & datetime
3. Google calendar for schedule management & refreshers in the morning

In-progress works:
1. Tennis shot detector & ML model to predict the type of shot (AP Research Project)
2. A more optimized autonomous landing software for latency on the raspberry pi 5 (UAS4STEM Competition 2025)
3. Upgrades to customized AI Assistant Jarvis 

