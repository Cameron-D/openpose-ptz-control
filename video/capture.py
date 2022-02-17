from video import BaseVideoSource
import cv2


class CaptureVideoSource(BaseVideoSource):
    def __init__(self, args):
        self.capture_device = cv2.VideoCapture(args.video_device)

    def source_available(self):
        return self.video_capture.isOpened()

    def frame_read(self):
        return self.video_capture.read()

    def close(self):
        return self.video_capture.release()
