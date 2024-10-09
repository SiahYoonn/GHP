Code for my GHP Portfolio
I have a passion for programming/machine learning and especially in applications involving UAVs or tennis.
I've tried out the YOLO models as well as tensorflow

videoloop.py is the UAV camera script
1. Stabilized video: easier for pilots to assume center
2. Successfully connects the Raspberry Pi to the UAV & runs as the pi boots up
3. Visualization of flight data

AUTO LANDING 2024 (Gazebo),py is the autonomous landing software for a UAV, applied on the gazebo simulation software
1. Utilization of ROS Melodic to simulate a virtual UAV + camera with all required parameters
2. Python Dronekit Library to control UAV from the Raspberry Pi
3. YOLOv8 model to detect targets & algorithm to center the UAV to it (data for training was collected by pilot)

AUTO LANDING 2024 (UAV),py is the autonomous landing software for a UAV, applied on an actual UAV
1. Identical to the gazebo simulation however ROS is not required and the code is attatched onto the camera script

Sign Language Recognition,ipynb is the ASL Translator using machine learning (ASL to English)
1. Mediapipe Holistic model to plot landmarks
2. Numpy arrays to label them & collected 30 videos w/ 30 sequences
3. Tensorflow Keras LSTM model to analyze the change in landmarks

just hand.py is the English to ASL portion in the ASL translator (I swear i had a file with all of the letters in the alphabet where did that go)
1. Pyserial library to connect python to the arduino
2. Adjustments of servo angles for mimicry
3. Slicing a string to express in order



