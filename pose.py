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


class PTZTrack:
    control_camera = None

    def __init__(self, args):
        self.args = args
        self.move = ViscaControl(self.args)

        # OpenPose Setup
        params = dict()
        params["model_folder"] = "/openpose/models"
        params["net_resolution"] = self.args.net_resolution

        self.openpose = op.WrapperPython()
        self.openpose.configure(params)
        self.openpose.start()

        # Video Source Setup
        self.video_capture = cv2.VideoCapture(args.video_device)

        # MQTT Setup
        if self.args.mqtt:
            self.mqttc = mqtt.Client("PTZTrack")
            self.mqttc.connect(args.mqtt_host)
            self.mqttc.loop_start()
            self.mqttc.subscribe("PTZ_SETSTATE")
            self.mqttc.on_message = self.mqtt_message
            self.mqtt_publish_state()
            print("MQTT Connected")
        else:
            print("Skipping MQTT Connection")

        signal(SIGINT, self.sigint_handler)

    def mqtt_message(self, client, userdata, message):
        data = message.payload.decode("utf-8")
        print("Control message:", data)

        if data.startswith("control state"):
            self.mqtt_publish_state()

        elif data.startswith("control on"):
            self.control_camera = True
            self.mqtt_publish_state()

        elif data.startswith("control off"):
            self.control_camera = False
            self.mqtt_publish_state()

        elif data.startswith("control toggle"):
            self.control_camera = not self.control_camera
            self.mqtt_publish_state()

    def calculate_speed(self, smin: int, val: int, smax: int):
        speed_ratio = (val - smin) / smax
        speed = int(
            ((self.args.speed_max - self.args.speed_min) * speed_ratio)
            + self.args.speed_min
        )
        return speed

    def move_state_str(self):
        return "on" if self.control_camera else "off"

    def get_keypoints_rectangle(self, keypoints, threshold=0.2):
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

    def sigint_handler(self, signal_received, frame):
        print("Program exit requested... Exiting gracefully")
        self.control_camera = False
        if self.args.mqtt:
            self.mqtt_publish_state()
            time.sleep(0.1)
            self.mqttc.loop_stop()
        self.video_capture.release()
        cv2.destroyAllWindows()
        exit(0)

    def read_frame(self):
        ret, frame = self.video_capture.read()
        frame = cv2.resize(frame, (1280, 720))
        return ret, frame

    def mqtt_publish_state(self):
        self.mqttc.publish("PTZ_STATE", self.move_state_str())

    def show_ui(self, frame):
        if (frame.shape[0] != 720) and (frame.shape[1] != 1280):
            frame = cv2.resize(frame, (1280, 720))

        cv2.imshow("PTZTrack Frame", frame)

        if cv2.waitKey(25) & 0xFF == ord("q"):
            return True

    def calculate_edges(self, frame_shape):
        l_edge = int(frame_shape[1] * args.boundary)
        r_edge = frame_shape[1] - l_edge
        height = frame_shape[0]
        width = frame_shape[1]

        bounding = [frame_shape[1], height, 0, 0]

        return l_edge, r_edge, height, width, bounding

    def process_datum_keypoints(self, frame, datum):
        regions = []

        for i in range(0, datum.poseKeypoints.shape[0]):
            p = self.get_keypoints_rectangle(datum.poseKeypoints[i], 0.1)
            regions.append([p[0], p[1], p[2] - p[0], p[3] - p[1]])
            cv2.rectangle(frame, (p[0], p[1]), (p[2], p[3]), (0, 255, 255), 2)

        return frame, regions

    def calculate_boundaries(self, bounding, regions):
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

    def main_loop(self):
        bounding = []

        self.move.set_direction(Move.STOP)
        last_direction = self.move.direction

        self.move.set_speed(self.args.speed_min)
        last_speed = self.move.speed

        frame_count = 0

        while self.video_capture.isOpened():
            check, frame = self.read_frame()

            if not check:
                time.sleep(0.01)
                continue

            l_edge, r_edge, height, width, bounding = self.calculate_edges(frame.shape)

            openpose_datum = op.Datum()
            openpose_datum.cvInputData = frame
            self.openpose.emplaceAndPop(op.VectorDatum([openpose_datum]))
            frame = openpose_datum.cvOutputData

            frame, regions = self.process_datum_keypoints(frame, openpose_datum)

            if len(regions) > 0:
                bounding = self.calculate_boundaries(bounding, regions)

                lrmiddle = int(
                    ((bounding[Edge.RIGHT] - bounding[Edge.LEFT]) / 2)
                    + bounding[Edge.LEFT]
                )
                udmiddle = int(
                    ((bounding[Edge.BOTTOM] - bounding[Edge.TOP]) / 2)
                    + bounding[Edge.TOP]
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
                    self.move.set_speed(
                        self.calculate_speed(0, l_edge - lrmiddle, l_edge)
                    )
                    self.move.set_direction(Move.LEFT)
                elif lrmiddle > r_edge:
                    self.move.set_speed(self.calculate_speed(r_edge, lrmiddle, width))
                    self.move.set_direction(Move.RIGHT)
                else:
                    self.move.set_speed(args.speed_min)
                    self.move.set_direction(Move.STOP)
            else:
                self.move.set_direction(Move.STOP)

            if self.move.direction != last_direction or self.move.speed != last_speed:
                if self.control_camera:
                    self.move.do_move()
                last_direction = self.move.direction
                last_speed = self.move.speed

            # Showing the output Image
            if self.args.ui:
                if self.show_ui(frame):
                    break

            # Publish control state occasionally
            frame_count += 1
            if self.args.mqtt and frame_count == 20:
                self.mqtt_publish_state()
                frame_count = 0


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

    ptztrack = PTZTrack(args)

    if args.mqtt:
        ptztrack.control_camera = args.control
    else:
        ptztrack.control_camera = True

    ptztrack.main_loop()
    ptztrack.sigint_handler(None, None)  # force tidy exit
