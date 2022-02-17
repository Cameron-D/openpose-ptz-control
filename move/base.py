from pose import Move


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
