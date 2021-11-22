import cv2, socket, time, socket, select, os
import numpy as np
import pyopenpose as op
import paho.mqtt.client as mqtt
import time
from enum import IntEnum
from itertools import product
from signal import signal, SIGINT
from sys import exit

class Move(IntEnum):
    STOP = 0
    LEFT = 1
    RIGHT = 2

class Edge(IntEnum):
    LEFT = 0
    TOP = 1
    RIGHT = 2
    BOTTOM = 3

VISCA_IP = os.getenv('VISCA_IP', "192.168.1.134")
VISCA_PORT = int(os.getenv('VISCA_PORT', 52381))

MQTT_ENABLED = bool(int(os.getenv('MQTT_ENABLED', True)))
MQTT_HOST = os.getenv('MQTT_HOST', "10.1.1.175")

SHOW_UI = bool(int(os.getenv('SHOW_UI', False)))
CONTROL = bool(int(os.getenv('CONTROL', False)))

BOUNDARY = float(os.getenv('BOUNDARY', .35))
MIN_SPEED = int(os.getenv('MIN_SPEED', 1))
MAX_SPEED = int(os.getenv('MAX_SPEED', 12))

NET_RESOLUTION = os.getenv('NET_RESOLUTION', "-1x128")

VIDEO_DEVICE = int(os.getenv('VIDEO_DEVICE', 0))

control_camera = True if CONTROL else False

# VISCA Setep
sequence_number = 1
VISCA_SEQUENCE_RESET = '02 00 00 01 00 00 00 01 01'
VISCA_MOVE_HEADER = '81 01 06 01 '
VISCA_LEFT = ' 01 03 FF'
VISCA_RIGHT = ' 02 03 FF'
VISCA_STOP = ' 03 03 FF'
visca_socket = None

# Processing Setup
video_capture = None

openpose_wrapper = None
mqttc = None


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


def reset_sequence_number():
    global sequence_number
    visca_socket.sendto(bytearray.fromhex(VISCA_SEQUENCE_RESET), (VISCA_IP, VISCA_PORT))
    sequence_number = 1


def calculate_move_speed(smin, val, smax):
    speed_ratio = (val - smin) / smax
    speed = int(((MAX_SPEED - MIN_SPEED) * speed_ratio) + MIN_SPEED)
    return speed


def make_visca_move_command(direction_str, speed):
    # padded hex value without 0x
    spd_hex = "{0:0{1}x}".format(int(speed),2)
    return VISCA_MOVE_HEADER + spd_hex + spd_hex + direction_str


def do_visca_move(direction, speed):
    reset_sequence_number()
    print("MOVE:", Move(direction), "@ speed", speed, "({0:0{1}x})".format(int(speed),2))

    if direction == Move.LEFT:
        send_visca_packet(make_visca_move_command(VISCA_LEFT, speed))
    elif direction == Move.RIGHT:
        send_visca_packet(make_visca_move_command(VISCA_RIGHT, speed))
    else:
        send_visca_packet(make_visca_move_command(VISCA_STOP, speed))


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


def get_keypoints_rectangle(keypoints, threshold=0.2):
    numberKeypoints = keypoints.shape[0]
    if numberKeypoints < 1:
        return "Number body parts must be > 0."
    
    minX = minY = float('inf')
    maxX = maxY = float('-inf')
    
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
    if MQTT_ENABLED:
        mqttc.publish("PTZ_STATE", move_state())
        time.sleep(0.1)
        mqttc.loop_stop()
    video_capture.release() 
    cv2.destroyAllWindows()
    exit(0)


def do_setup():
    global visca_socket, openpose_wrapper, mqttc, video_capture

    visca_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # OpenPose Setep
    params = dict()
    params["model_folder"] = "/openpose/models"
    params["net_resolution"] = NET_RESOLUTION

    openpose_wrapper = op.WrapperPython()
    openpose_wrapper.configure(params)
    openpose_wrapper.start()

    video_capture = cv2.VideoCapture(VIDEO_DEVICE)

    #MQTT Setup
    if MQTT_ENABLED:
        mqttc = mqtt.Client("PTZTrack")
        mqttc.connect(MQTT_HOST)
        mqttc.loop_start()
        mqttc.subscribe("PTZ_SETSTATE")
        mqttc.on_message=mqtt_message
        mqttc.publish("PTZ_STATE", move_state())
        print("MQTT Connected")
    else:
        print("Skipping MQTT Connection")

    signal(SIGINT, sigint_handler)


def read_frame():
    ret, frame = video_capture.read()
    frame = cv2.resize(frame, (1280, 720))
    return ret, frame



def update_frame_count(fc):
    fc =+ 1
    
    if fc == 50:
        mqttc.publish("PTZ_STATE", move_state())
        fc = 0

    return fc


def show_ui(frame):
    if (frame.shape[0] != 720) and (frame.shape[1] != 1280):
        frame = cv2.resize(frame, (1280, 720))

    cv2.imshow("PTZTrack Frame", frame)

    if cv2.waitKey(25) & 0xFF == ord('q'):
        return True


def calculate_edges(frame_shape):
    l_edge = int(frame_shape[1] * BOUNDARY)
    r_edge = frame_shape[1] - l_edge
    height = frame_shape[0]
    width = frame_shape[1]

    bounding = [frame_shape[1], height, 0, 0]

    return l_edge, r_edge, height, width, bounding


def process_datum_keypoints(frame, datum):
    regions = []

    for i in range(0, datum.poseKeypoints.shape[0]):
        p = get_keypoints_rectangle(datum.poseKeypoints[i], 0.1)
        regions.append([p[0], p[1], p[2]-p[0], p[3]-p[1]])
        cv2.rectangle(frame, (p[0], p[1]), (p[2], p[3]), (0, 255, 255), 2)
    
    return frame, regions


def calculate_boundaries(bounding, regions):
    for (x, y, w, h) in regions:
        if x < bounding[Edge.LEFT]:
            bounding[Edge.LEFT] = x
        if y < bounding[Edge.TOP]:
            bounding[Edge.TOP] = y
        if x+w > bounding[Edge.RIGHT]:
            bounding[Edge.RIGHT] = x+w
        if y+h > bounding[Edge.BOTTOM]:
            bounding[Edge.BOTTOM] = y+h
    
    return bounding

def main_loop():
    bounding = []

    direction = Move.STOP
    last_direction = direction
    speed = 1
    last_speed = speed

    frame_count = 0

    while video_capture.isOpened():
        check, frame = read_frame()

        if not check:
            time.sleep(0.01)
            continue

        l_edge, r_edge, height, width, bounding = calculate_edges(frame.shape)

        openpose_datum = op.Datum()
        openpose_datum.cvInputData = frame
        openpose_wrapper.emplaceAndPop(op.VectorDatum([openpose_datum]))
        frame = openpose_datum.cvOutputData

        frame, regions = process_datum_keypoints(frame, openpose_datum)

        if len(regions) > 0:
            bounding = calculate_boundaries(bounding, regions)

            lrmiddle = int(((bounding[Edge.RIGHT] - bounding[Edge.LEFT]) / 2) + bounding[Edge.LEFT])
            udmiddle = int(((bounding[Edge.BOTTOM] - bounding[Edge.TOP]) / 2) + bounding[Edge.TOP])

            cv2.rectangle(frame, (bounding[Edge.LEFT], bounding[Edge.TOP]), (bounding[Edge.RIGHT], bounding[Edge.BOTTOM]), (0, 200, 0), 2)
            cv2.rectangle(frame, (lrmiddle-1, udmiddle-1), (lrmiddle+1, udmiddle+1), (255, 255, 0), 4)
            cv2.rectangle(frame, (l_edge, 0), (r_edge, height), (255, 0, 0), 4)

            if(lrmiddle < l_edge):
                speed = calculate_move_speed(0, l_edge - lrmiddle, l_edge)
                direction = Move.LEFT
            elif (lrmiddle > r_edge):
                speed = calculate_move_speed(r_edge, lrmiddle, width)
                direction = Move.RIGHT
            else:
                speed = MIN_SPEED
                direction = Move.STOP
        else:
            direction = Move.STOP
        
        if direction != last_direction or speed != last_speed:
            if control_camera:
                do_visca_move(direction, speed)
            last_direction = direction
            last_speed = speed
        
        # Showing the output Image
        if SHOW_UI:
            if show_ui(frame):
                break

        if MQTT_ENABLED:
            frame_count = update_frame_count(frame_count)


if __name__ == "__main__":
    do_setup()
    main_loop()
    sigint_handler(None, None) #force tidy exit