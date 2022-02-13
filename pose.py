import cv2, socket, time, socket, select, os
import numpy as np
import pyopenpose as op
import paho.mqtt.client as mqtt
import time
from enum import IntEnum
from itertools import product
from signal import signal, SIGINT
from sys import exit
import argparse


class Move(IntEnum):
    STOP = 0
    LEFT = 1
    RIGHT = 2


class Edge(IntEnum):
    LEFT = 0
    TOP = 1
    RIGHT = 2
    BOTTOM = 3


class BaseMoveControl:
    direction = None
    speed = None

    def __init__(self, args):
        raise NotImplementedError

    def do_move(self):
        raise NotImplementedError

    def set_direction(self, direction: Move):
        raise NotImplementedError

    def set_speed(self, speed: int):
        raise NotImplementedError


class ViscaControl(BaseMoveControl):
    SEQUENCE_RESET = "02 00 00 01 00 00 00 01 01"
    MOVE_HEADER = "81 01 06 01 "
    LEFT = " 01 03 FF"
    RIGHT = " 02 03 FF"
    STOP = " 03 03 FF"

    def __init__(self, args):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.host = args.visca_ip
        self.port = args.visca_port

        self.sequence_number = 1

        self.set_direction(Move.STOP)
        self.set_speed(args.speed_min)

    def set_direction(self, direction: Move):
        self.direction = direction

    def set_speed(self, speed: int):
        self.speed = speed

    def do_move(self):
        self.reset_sequence_number()

        if self.direction == Move.LEFT:
            self.send_packet(self.make_move_str(self.LEFT))
        elif self.direction == Move.RIGHT:
            self.send_packet(self.make_move_str(self.RIGHT))
        else:
            self.send_packet(self.make_move_str(self.STOP))

        print(
            "MOVE:",
            Move(self.direction),
            "@ speed",
            self.speed,
            "({0:0{1}x})".format(int(self.speed), 2),
        )

    def reset_sequence_number(self):
        self.socket.sendto(
            bytearray.fromhex(self.SEQUENCE_RESET), (self.host, self.port)
        )
        self.sequence_number = 1

    def make_move_str(self, direction_str: str):
        # returns a padded hex value without 0x
        spd_hex = "{0:0{1}x}".format(int(self.speed), 2)
        return self.MOVE_HEADER + spd_hex + spd_hex + direction_str

    def send_packet(self, command: str):
        payload_type = bytearray.fromhex("01 00")
        payload = bytearray.fromhex(command)
        payload_length = len(payload).to_bytes(2, "big")

        message = (
            payload_type
            + payload_length
            + self.sequence_number.to_bytes(4, "big")
            + payload
        )

        self.sequence_number += 1
        self.socket.sendto(message, (args.visca_ip, args.visca_port))


control_camera = None
move_control = None
video_capture = None
openpose_wrapper = None
mqttc = None
args = None


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


def calculate_move_speed(smin, val, smax):
    speed_ratio = (val - smin) / smax
    speed = int(((args.speed_max - args.speed_min) * speed_ratio) + args.speed_min)
    return speed


def move_state():
    return "on" if control_camera else "off"


def get_keypoints_rectangle(keypoints, threshold=0.2):
    numberKeypoints = keypoints.shape[0]
    if numberKeypoints < 1:
        return "Number body parts must be > 0."

    minX = minY = float("inf")
    maxX = maxY = float("-inf")

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

    print("Program exit requested... Exiting gracefully")
    control_camera = False
    if args.mqtt:
        mqttc.publish("PTZ_STATE", move_state())
        time.sleep(0.1)
        mqttc.loop_stop()
    video_capture.release()
    cv2.destroyAllWindows()
    exit(0)


def do_setup():
    global visca_socket, openpose_wrapper, mqttc, video_capture, move_control

    move_control = ViscaControl(args)

    # OpenPose Setep
    params = dict()
    params["model_folder"] = "/openpose/models"
    params["net_resolution"] = args.net_resolution

    openpose_wrapper = op.WrapperPython()
    openpose_wrapper.configure(params)
    openpose_wrapper.start()

    video_capture = cv2.VideoCapture(args.video_device)

    # MQTT Setup
    if args.mqtt:
        mqttc = mqtt.Client("PTZTrack")
        mqttc.connect(args.mqtt_host)
        mqttc.loop_start()
        mqttc.subscribe("PTZ_SETSTATE")
        mqttc.on_message = mqtt_message
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
    fc = +1

    if fc == 50:
        mqttc.publish("PTZ_STATE", move_state())
        fc = 0

    return fc


def show_ui(frame):
    if (frame.shape[0] != 720) and (frame.shape[1] != 1280):
        frame = cv2.resize(frame, (1280, 720))

    cv2.imshow("PTZTrack Frame", frame)

    if cv2.waitKey(25) & 0xFF == ord("q"):
        return True


def calculate_edges(frame_shape):
    l_edge = int(frame_shape[1] * args.boundary)
    r_edge = frame_shape[1] - l_edge
    height = frame_shape[0]
    width = frame_shape[1]

    bounding = [frame_shape[1], height, 0, 0]

    return l_edge, r_edge, height, width, bounding


def process_datum_keypoints(frame, datum):
    regions = []

    for i in range(0, datum.poseKeypoints.shape[0]):
        p = get_keypoints_rectangle(datum.poseKeypoints[i], 0.1)
        regions.append([p[0], p[1], p[2] - p[0], p[3] - p[1]])
        cv2.rectangle(frame, (p[0], p[1]), (p[2], p[3]), (0, 255, 255), 2)

    return frame, regions


def calculate_boundaries(bounding, regions):
    for (x, y, w, h) in regions:
        if x < bounding[Edge.LEFT]:
            bounding[Edge.LEFT] = x
        if y < bounding[Edge.TOP]:
            bounding[Edge.TOP] = y
        if x + w > bounding[Edge.RIGHT]:
            bounding[Edge.RIGHT] = x + w
        if y + h > bounding[Edge.BOTTOM]:
            bounding[Edge.BOTTOM] = y + h

    return bounding


def main_loop():
    bounding = []

    move_control.set_direction(Move.STOP)
    last_direction = move_control.direction

    move_control.set_speed(args.speed_min)
    last_speed = move_control.speed

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

            lrmiddle = int(
                ((bounding[Edge.RIGHT] - bounding[Edge.LEFT]) / 2) + bounding[Edge.LEFT]
            )
            udmiddle = int(
                ((bounding[Edge.BOTTOM] - bounding[Edge.TOP]) / 2) + bounding[Edge.TOP]
            )

            cv2.rectangle(
                frame,
                (bounding[Edge.LEFT], bounding[Edge.TOP]),
                (bounding[Edge.RIGHT], bounding[Edge.BOTTOM]),
                (0, 200, 0),
                2,
            )
            cv2.rectangle(
                frame,
                (lrmiddle - 1, udmiddle - 1),
                (lrmiddle + 1, udmiddle + 1),
                (255, 255, 0),
                4,
            )
            cv2.rectangle(frame, (l_edge, 0), (r_edge, height), (255, 0, 0), 4)

            if lrmiddle < l_edge:
                speed = calculate_move_speed(0, l_edge - lrmiddle, l_edge)
                move_control.set_direction(Move.LEFT)
            elif lrmiddle > r_edge:
                speed = calculate_move_speed(r_edge, lrmiddle, width)
                move_control.set_direction(Move.RIGHT)
            else:
                move_control.set_speed(args.speed_min)
                move_control.set_direction(Move.STOP)
        else:
            move_control.set_direction(Move.STOP)

        if move_control.direction != last_direction or move_control.speed != last_speed:
            if control_camera:
                move_control.do_move()
            last_direction = move_control.direction
            last_speed = move_control.speed

        # Showing the output Image
        if args.ui:
            if show_ui(frame):
                break

        if args.mqtt:
            frame_count = update_frame_count(frame_count)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Automatic control of a PTZ camera using image recognition"
    )
    parser.add_argument(
        "visca_ip",
        help="IP address of the VISCA interface for the camera to be controlled. Required.",
    )
    parser.add_argument(
        "-p",
        "--visca_port",
        default=52381,
        help="Port number of the VISCA interface for the camera to be controlled (default: %(default)s)",
    )
    parser.add_argument(
        "-m",
        "--mqtt",
        action="store_true",
        help="Enable remote control over MQTT (default: %(default)s)",
    )
    parser.add_argument(
        "--mqtt_host",
        default="127.0.0.1",
        help="Hostname or IP of the MQTT Broker (default: %(default)s)",
    )
    parser.add_argument(
        "-c",
        "--control",
        action="store_true",
        help="If provided and MQTT is enabled, will start controlling the camera on launch, otherwise program will wait for a control command over MQTT. (default: %(default)s)",
    )
    parser.add_argument(
        "-b",
        "--boundary",
        default=0.35,
        type=float,
        help="Width of the target box to keep the tracked person inside, as a percentage of screen width (default: %(default)s)",
    )
    parser.add_argument(
        "-s",
        "--speed_min",
        default=1,
        type=int,
        help="Minimum speed to move the camera at. Not more than speed_max, min 1. (default: %(default)s)",
    )
    parser.add_argument(
        "-S",
        "--speed_max",
        default=12,
        type=int,
        help="Maximum speed to move the camera at. Not less than speed_min, max 24. (default: %(default)s)",
    )
    parser.add_argument(
        "--net_resolution",
        default="-1x128",
        help="Argument for net_resolution passed directly to OpenCV (default: %(default)s)",
    )
    parser.add_argument(
        "-v",
        "--video_device",
        default=0,
        type=int,
        help="Video device number to read frames from (default: %(default)s)",
    )
    parser.add_argument(
        "--ui",
        action="store_true",
        help="If provided, display a UI interface visualising the processing (default: %(default)s)",
    )
    args = parser.parse_args()

    if args.mqtt:
        control_camera = args.control
    else:
        control_camera = True

    do_setup()
    main_loop()
    sigint_handler(None, None)  # force tidy exit
