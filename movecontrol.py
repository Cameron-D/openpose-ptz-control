from argparse import Namespace
from enum import IntEnum


class Move(IntEnum):
    STOP = 0
    LEFT = 1
    RIGHT = 2


class BaseMoveControl:
    """A generic base class for implementing a camera movement controller"""

    direction = None
    speed = None

    def __init__(self, args: Namespace):
        raise NotImplementedError

    def do_move(self):
        """Actually sends the move command to the camera."""
        raise NotImplementedError

    def set_direction(self, direction: Move):
        """Update the internal state with the new movement direction

        :param direction: The direction the camera needs to move in.
        """
        raise NotImplementedError

    def set_speed(self, speed: int):
        """Update the internal state with the new speed.

        :param speed: The speed to move at. Currently 0 is min and 24 is max.
        """
        raise NotImplementedError


class ViscaMoveControl(BaseMoveControl):
    SEQUENCE_RESET = "02 00 00 01 00 00 00 01 01"
    MOVE_HEADER = "81 01 06 01 "
    LEFT = " 01 03 FF"
    RIGHT = " 02 03 FF"
    STOP = " 03 03 FF"

    def __init__(self, args: Namespace):
        import socket

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.host = args.visca_ip
        self.port = args.visca_port

        self.sequence_number = 1

        self.set_direction(Move.STOP)
        self.set_speed(args.speed_min)

    def set_direction(self, direction):
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
        """The VISCA protcol includes a sequence number along with each command to
        ensure that commands can't be repeated. As other devices will still be used
        to control the camera alongside this program we ask the camera to reset the
        sequence number each time we send a command. This works as any other controller
        will be unaware of the reset, and will send commands with a higher sequence number.
        """

        self.socket.sendto(
            bytearray.fromhex(self.SEQUENCE_RESET), (self.host, self.port)
        )
        self.sequence_number = 1

    def make_move_str(self, direction_str: str):
        """This method makes a hex string representing a full VISCA movement command.
        It takes a provided direction hex string and generates the speed hex value.

        :param direction_str: a string of hexadecimal values representing a VISCA movement direction
        """

        # returns a padded hex value without 0x
        spd_hex = "{0:0{1}x}".format(int(self.speed), 2)
        return self.MOVE_HEADER + spd_hex + spd_hex + direction_str

    def send_packet(self, command: str):
        """This method builds and sends a VISCA packet over the network. It calculates the necessary
        headers for the provided command string.

        :param command: a complete VISCA command as a hexadecimal string
        """

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
