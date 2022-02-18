class BaseVideoSource:
    def __init__(self, args):
        raise NotImplementedError

    def source_available(self):
        raise NotImplementedError

    def frame_read(self):
        raise NotImplementedError

    def close(self):
        raise NotImplementedError


class CaptureVideoSource(BaseVideoSource):
    def __init__(self, args):
        import cv2

        self.capture_device = cv2.VideoCapture(args.video_device)

    def source_available(self):
        return self.capture_device.isOpened()

    def frame_read(self):
        return self.capture_device.read()

    def close(self):
        return self.capture_device.release()
