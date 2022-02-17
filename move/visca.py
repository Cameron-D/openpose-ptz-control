from move import BaseMoveControl
from pose import Move
import socket


class ViscaMoveControl(BaseMoveControl):
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
        self.socket.sendto(message, (self.host, self.port))
