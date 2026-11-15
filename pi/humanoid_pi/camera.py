"""Head camera (Raspberry Pi camera, 5 MP, CSI) as an MJPEG stream for the web page.

Uses picamera2 (installed with apt on Raspberry Pi OS). Elsewhere, or without a camera,
`available` is False and the web page hides the video.
"""
import io
import logging
import threading

log = logging.getLogger("camera")


class _Frames(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.cond = threading.Condition()

    def write(self, buf):
        with self.cond:
            self.frame = bytes(buf)
            self.cond.notify_all()
        return len(buf)


class Camera:
    def __init__(self, enabled=True, size=(640, 480), rotate180=False):
        self.available = False
        self._out = _Frames()
        if not enabled:
            return
        try:
            from libcamera import Transform
            from picamera2 import Picamera2
            from picamera2.encoders import JpegEncoder
            from picamera2.outputs import FileOutput
        except ImportError:
            log.warning("picamera2 not available: no video")
            return
        try:
            cam = Picamera2()
            t = Transform(hflip=1, vflip=1) if rotate180 else Transform()
            cam.configure(cam.create_video_configuration(main={"size": size}, transform=t))
            cam.start_recording(JpegEncoder(), FileOutput(self._out))
            self._cam = cam
            self.available = True
        except Exception as e:  # ribbon unplugged, camera busy...
            log.warning("camera failed to start: %s", e)

    def frames(self):
        """Generator of JPEG frames (blocks until each new frame)."""
        while True:
            with self._out.cond:
                self._out.cond.wait(timeout=2.0)
                f = self._out.frame
            if f:
                yield f
