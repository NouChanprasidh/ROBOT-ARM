import cv2
import mediapipe as mp


import numpy as np
import serial
import serial.tools.list_ports
import time

# Initialize MediaPipe Hand solution
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.5)
mp_draw = mp.solutions.drawing_utils

# Initialize the webcam
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# Function to find the Arduino port
def find_arduino_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if 'Arduino' in p.description:
            return p.device
    return None

# Try to connect to Arduino
arduino_port = find_arduino_port()
if arduino_port:
    try:
        arduino = serial.Serial(arduino_port, 9600, timeout=1)
        print(f"Connected to Arduino on port {arduino_port}")
        time.sleep(2)  # Wait for the connection to establish
    except serial.SerialException as e:
        print(f"Error connecting to Arduino: {e}")
        arduino = None
else:
    print("Arduino not found. Make sure it's connected and recognized by your computer.")
    arduino = None

# Function to smooth movements
def smooth_movement(new_value, prev_value, alpha=0.1):
    return (alpha * new_value) + ((1 - alpha) * prev_value)

# Initialize previous angles
prev_base_angle = 90
prev_elbow1_angle = 90
prev_elbow2_angle = 90
prev_gripper_angle = 90

def detect_hand_movement(frame):
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(frame_rgb)
    
    if results.multi_hand_landmarks:
        hand_landmarks = results.multi_hand_landmarks[0]
        
        wrist = hand_landmarks.landmark[0]
        index_tip = hand_landmarks.landmark[8]
        thumb_tip = hand_landmarks.landmark[4]
        middle_mcp = hand_landmarks.landmark[9]
        
        mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
        
        x_ratio = 1 - wrist.x  # Reverse the horizontal movement
        y_ratio = wrist.y
        z_ratio = middle_mcp.y
        
        distance = np.sqrt((thumb_tip.x - index_tip.x)**2 + (thumb_tip.y - index_tip.y)**2)
        
        return x_ratio, y_ratio, z_ratio, distance, True
    
    return 0, 0, 0, 0, False

def map_to_servo(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    frame = cv2.flip(frame, 1)
    
    x_ratio, y_ratio, z_ratio, finger_distance, hand_detected = detect_hand_movement(frame)
    
    if hand_detected:
        base_angle = map_to_servo(x_ratio, 0, 1, 0, 180)
        elbow1_angle = map_to_servo(y_ratio, 0, 1, 0, 180)
        elbow2_angle = map_to_servo(z_ratio, 0, 1, 0, 180)
        gripper_angle = map_to_servo(finger_distance, 0, 0.2, 180, 0)
        
        base_angle = smooth_movement(base_angle, prev_base_angle)
        elbow1_angle = smooth_movement(elbow1_angle, prev_elbow1_angle)
        elbow2_angle = smooth_movement(elbow2_angle, prev_elbow2_angle)
        gripper_angle = smooth_movement(gripper_angle, prev_gripper_angle)
        
        prev_base_angle = base_angle
        prev_elbow1_angle = elbow1_angle
        prev_elbow2_angle = elbow2_angle
        prev_gripper_angle = gripper_angle
        
        if arduino:
            command = f"{int(base_angle)},{int(elbow1_angle)},{int(elbow2_angle)},{int(gripper_angle)}\n"
            arduino.write(command.encode())
        
        cv2.putText(frame, f"Base: {int(base_angle)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Elbow1: {int(elbow1_angle)}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Elbow2: {int(elbow2_angle)}", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Gripper: {int(gripper_angle)}", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    cv2.namedWindow("Hand Tracking", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("Hand Tracking", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.imshow('Hand Tracking', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
if arduino:
    arduino.close()