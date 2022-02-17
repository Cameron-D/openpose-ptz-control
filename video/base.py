class BaseVideoSource:
    def __init__(self, args):
        raise NotImplementedError

    def source_available(self):
        raise NotImplementedError

    def frame_read(self):
        raise NotImplementedError

    def close(self):
        raise NotImplementedError
