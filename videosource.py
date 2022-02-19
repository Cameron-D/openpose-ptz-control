import cv2, time


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
        self.capture_device = cv2.VideoCapture(args.video_device)

    def source_available(self):
        return self.capture_device.isOpened()

    def frame_read(self):
        return self.capture_device.read()

    def close(self):
        return self.capture_device.release()


class NDIVideoSource(BaseVideoSource):
    def __init__(self, args):
        import NDIlib as ndi
        import numpy as np

        self.ndi = ndi
        self.np = np

        self.ready = True
        if not self.ndi.initialize():
            self.ready = False
            return

        find_create = self.ndi.FindCreate()
        if args.ndi_extra_ips:
            find_create.extra_ips = args.ndi_extra_ips

        ndi_finder = self.ndi.find_create_v2(find_create)
        if not ndi_finder:
            self.ready = False
            return

        self.ndi_source = None
        while self.ndi_source == None:
            ndi.find_wait_for_sources(ndi_finder, 1000)
            sources = ndi.find_get_current_sources(ndi_finder)

            for source in sources:
                print(source.ndi_name)
                if source.ndi_name == args.ndi_source:
                    self.ndi_source = source
                    break
            time.sleep(0.1)

        recv_create = self.ndi.RecvCreateV3()
        recv_create.color_format = self.ndi.RECV_COLOR_FORMAT_BGRX_BGRA

        self.ndi_recv = ndi.recv_create_v3(recv_create)
        if self.ndi_recv is None:
            self.ready = False
            return

        self.ndi.recv_connect(self.ndi_recv, self.ndi_source)
        self.ndi.find_destroy(ndi_finder)
        print("Connected to NDI")

    def source_available(self):
        return self.ready

    def frame_read(self):
        while True:
            t, v, _, _ = self.ndi.recv_capture_v2(self.ndi_recv, 1000)
            if t == self.ndi.FRAME_TYPE_VIDEO:
                print("Video data received (%dx%d)." % (v.xres, v.yres))
                frame = self.np.copy(v.data)
                self.ndi.recv_free_video_v2(self.ndi_recv, v)
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
                return True, frame

    def close(self):
        self.ndi.recv_destroy(self.ndi_recv)
        self.ndi.destroy()
