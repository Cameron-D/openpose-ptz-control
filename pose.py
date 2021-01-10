import cv2, socket, time, socket, select, os
import numpy as np
import pyopenpose as op
import paho.mqtt.client as mqtt
from enum import Enum
from itertools import product
from signal import signal, SIGINT
from sys import exit

class Move(Enum):
    STOP = 0
    LEFT = 1
    RIGHT = 2

VISCA_IP = os.getenv('VISCA_IP', "192.168.1.134")
VISCA_PORT = int(os.getenv('VISCA_IP', 52381))

MQTT_HOST = os.getenv('MQTT_HOST', "10.1.1.175")

SHOW_UI = os.getenv('SHOW_UI', False)
CONTROL = os.getenv('CONTROL', False)

VISCA_SEQUENCE = '02 00 00 01 00 00 00 01 01'
VISCA_LEFT = '81 01 06 01 02 02 01 03 FF'
VISCA_RIGHT = '81 01 06 01 02 02 02 03 FF'
VISCA_STOP = '81 01 06 01 02 02 03 03 FF'

SCALE = int(os.getenv('SCALE', 3))
DIV = 1/SCALE

direction = Move.STOP
last_direction = direction

control_camera = True if CONTROL else False

# VISCA Setep
sequence_number = 1
visca_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # IPv4, UDP

# Processing Setup
bounding = [0, 0, 0, 0]
regions = []
process_this_frame = True
video_capture = cv2.VideoCapture(0)

# OpenPose Setep
params = dict()
params["model_folder"] = "/openpose/models"
params["net_resolution"] = "-1x128"

opWrapper = op.WrapperPython()
opWrapper.configure(params)
opWrapper.start()

def mqtt_message(client, userdata, message):
    global control_camera
    data = message.payload.decode("utf-8")
    print("Control message:", data)
    if data.startswith("control state"):
        mqttc.publish("PTZ_STATE", move_state())
        
    elif data.startswith("control on"):
        control_camera = True
        mqttc.publish("PTZ_STATE", move_state())
        
    elif data.startswith("control off"):
        control_camera = False
        mqttc.publish("PTZ_STATE", move_state())
        
    elif data.startswith("control toggle"):
        control_camera = not control_camera
        mqttc.publish("PTZ_STATE", move_state())
    

mqttc = mqtt.Client("PTZTrack")
mqttc.connect(MQTT_HOST)
frame_count = 0
mqttc.loop_start()
mqttc.subscribe("PTZ_SETSTATE")
mqttc.on_message=mqtt_message


def reset_sequence_number():
    global sequence_number
    visca_socket.sendto(bytearray.fromhex(VISCA_SEQUENCE), (VISCA_IP, VISCA_PORT))
    sequence_number = 1

def send_visca_packet(command):
    global sequence_number
    payload_type = bytearray.fromhex('01 00')
    payload = bytearray.fromhex(command)
    payload_length = len(payload).to_bytes(2, 'big')
    message = payload_type + payload_length + sequence_number.to_bytes(4, 'big') + payload
    sequence_number += 1
    visca_socket.sendto(message, (VISCA_IP, VISCA_PORT))

def move_state():
    return "on" if control_camera else "off"

mqttc.publish("PTZ_STATE", move_state())

def getKeypointsRectangle(keypoints, threshold=0.2):
    numberKeypoints = keypoints.shape[0]
    if numberKeypoints < 1:
        return "Number body parts must be > 0."
    
    minX =  minY = float('inf')
    maxX =  maxY = float('-inf')
    
    for keypoint in keypoints:
        score = keypoint[2]
        if score > threshold:
            x = keypoint[0]
            y = keypoint[1]
            
            if maxX < x:
                maxX = x
            if minX > x:
                minX = x
                
            if maxY < y:
                maxY = y
            if minY > y:
                minY = y
            
    if maxX >= minX and maxY >= minY:
        return int(minX), int(minY), int(maxX), int(maxY)


def sigint_handler(signal_received, frame):
    global control_camera
    print('Program exit requested... Exiting gracefully')
    control_camera = False
    mqttc.publish("PTZ_STATE", move_state())
    time.sleep(0.1)
    mqttc.loop_stop()
    video_capture.release() 
    cv2.destroyAllWindows()
    exit(0)
signal(SIGINT, sigint_handler)


while video_capture.isOpened():
    # Grab a single frame of video
    ret, frame = video_capture.read()
    
    frame = cv2.resize(frame, (1280, 720))

    if ret:
        l_edge = int(frame.shape[1] * .35)
        r_edge = frame.shape[1] - l_edge
        height = frame.shape[0]
        small_frame = frame
        small_frame = cv2.resize(small_frame, (0, 0), fx=DIV, fy=DIV)

        bounding[0] = frame.shape[1]
        bounding[1] = frame.shape[0]
        bounding[2] = 0
        bounding[3] = 0

        if process_this_frame and control_camera:
            regions = []

            datum = op.Datum()
            datum.cvInputData = small_frame
            opWrapper.emplaceAndPop(op.VectorDatum([datum]))
            
            small_frame = datum.cvOutputData
                        
            for i in range(0, datum.poseKeypoints.shape[0]):
                p = getKeypointsRectangle(datum.poseKeypoints[i], 0.1)
                regions.append([p[0], p[1], p[2]-p[0], p[3]-p[1]])
                #cv2.rectangle(small_frame, (p[0], p[1]), (p[2], p[3]), (0, 255, 0), 2)

        #process_this_frame = not process_this_frame

        # Drawing the regions in the
        # Image
        for (x, y, w, h) in regions:
            x *= SCALE
            y *= SCALE
            w *= SCALE
            h *= SCALE
            #cv2.rectangle(frame, (x, y),
            #              (x + w, y + h),
            #              (0, 0, 255), 2)
            if x < bounding[0]:
                bounding[0] = x
            if y < bounding[1]:
                bounding[1] = y
            if x+w > bounding[2]:
                bounding[2] = x+w
            if y+h > bounding[3]:
                bounding[3] = y+h
        

        bx1 = int(bounding[0])
        by1 = int(bounding[1])
        bx2 = int(bounding[2])
        by2 = int(bounding[3])
        cv2.rectangle(frame, (bx1, by1), (bx2, by2), (0, 200, 0), 2)

        lrmiddle = int(((bounding[2] - bounding[0]) / 2) + bounding[0])
        udmiddle = int(((bounding[3] - bounding[1]) / 2) + bounding[1])
        
        cv2.rectangle(frame, (lrmiddle-1, udmiddle-1), (lrmiddle+1, udmiddle+1), (255, 255, 0), 4)

        cv2.rectangle(frame, (l_edge, 0), (r_edge, height), (255, 0, 0), 4)
        
        if(lrmiddle < l_edge):
            direction = Move.LEFT
        elif (lrmiddle > r_edge):
            direction = Move.RIGHT
        else:
            direction = Move.STOP
        
        if direction != last_direction:
            if control_camera:
                reset_sequence_number()
                print("MOVE:", Move(direction))
                if direction == Move.LEFT:
                    send_visca_packet(VISCA_LEFT)
                elif direction == Move.RIGHT:
                    send_visca_packet(VISCA_RIGHT)
                else:
                    send_visca_packet(VISCA_STOP)
            last_direction = direction
   
        # Showing the output Image
        if SHOW_UI:
            frame = cv2.resize(frame, (1280, 720))
            cv2.imshow("Full", frame)
            cv2.imshow("Processed", small_frame)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break

        frame_count =+ 1
        if frame_count == 50:
            mqttc.publish("PTZ_STATE", move_state())
            frame_count = 0
        
    else:
        break

sigint_handler(None, None) #force tidy exit
