from .base import BaseVideoSource
import NDIlib as ndi


class NDIVideoSource(BaseVideoSource):
    def __init__(self, args):
        self.ready = True
        if not ndi.initialize():
            self.ready = False
            return

        ndi_recv_create = ndi.RecvCreateV3()
        ndi_recv_create.source_to_connect_to.p_ndi_name = args.ndi_source
        ndi_recv_create.color_format = ndi.RECV_COLOR_FORMAT_BGRX_BGRA

        self.ndi_recv = ndi.recv_create_v3(ndi_recv_create)

    def source_available(self):
        return self.ready

    def frame_read(self):
        return self.video_capture.read()

    def close(self):
        return self.video_capture.release()
