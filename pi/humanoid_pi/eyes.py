"""Two 1.3-inch I2C OLED eyes (SH1106, 128 x 64, both at address 0x3C).

They share an address, so each has its own bus: left eye on I2C bus 1 (GPIO 2/3), right eye on a
software bus 3 (GPIO 23/24, dtoverlay=i2c-gpio). Without the displays (or on a laptop) the eyes are
still drawn, and the web page shows them at /eyes.png.
"""
import logging
import random
import threading
import time

from PIL import Image, ImageDraw

log = logging.getLogger("eyes")
W, H = 128, 64

# expression -> (eye height factor, pupil dx, pupil dy, lid: None | "top" | "bottom" | "angry", mouth-less extras)
EXPRESSIONS = {
    "neutral": (1.0, 0, 0, None),
    "happy": (0.55, 0, 6, "bottom"),
    "surprised": (1.25, 0, 0, None),
    "sleepy": (0.25, 0, 4, "top"),
    "look_left": (1.0, -22, 0, None),
    "look_right": (1.0, 22, 0, None),
    "look_up": (1.0, 0, -10, None),
    "look_down": (1.0, 0, 10, None),
    "fault": (0, 0, 0, "x"),
}


class Eyes:
    def __init__(self, use_hardware=True, left_bus=1, right_bus=3, fps=15):
        self.devices = [None, None]
        if use_hardware:
            try:
                from luma.core.interface.serial import i2c
                from luma.oled.device import sh1106
                for k, bus in enumerate((left_bus, right_bus)):
                    try:
                        self.devices[k] = sh1106(i2c(port=bus, address=0x3C))
                    except Exception as e:  # one eye missing is not fatal
                        log.warning("OLED on bus %d not found: %s", bus, e)
            except ImportError:
                log.warning("luma.oled not installed: eyes drawn for the web page only")
        self.expression, self.until, self.after = "neutral", 0.0, "neutral"
        self.frame = Image.new("1", (2 * W + 8, H))
        self._blink_at = time.time() + 3
        self._fps = fps
        self._lock = threading.Lock()
        threading.Thread(target=self._run, name="eyes", daemon=True).start()

    def show(self, expression, seconds=0.0, then="neutral"):
        """Switch expression, optionally for a while before going to `then`."""
        if expression not in EXPRESSIONS:
            raise ValueError(expression)
        with self._lock:
            self.expression = expression
            self.until = time.time() + seconds if seconds else 0.0
            self.after = then

    def png(self):
        import io
        buf = io.BytesIO()
        with self._lock:
            img = self.frame.convert("L").resize((self.frame.width * 2, H * 2), Image.NEAREST)
        img.save(buf, "PNG")
        return buf.getvalue()

    # ---- drawing
    def _draw(self, side, expr, blink):
        hf, pdx, pdy, lid = EXPRESSIONS[expr]
        img = Image.new("1", (W, H))
        d = ImageDraw.Draw(img)
        cx, cy = W // 2, H // 2
        if lid == "x":
            for s in (-1, 1):
                d.line([cx - 18, cy - 18 * s, cx + 18, cy + 18 * s], fill=1, width=5)
            return img
        h = max(2, int(44 * hf * (0.1 if blink else 1.0)))
        w = 56
        d.rounded_rectangle([cx - w // 2 + pdx // 2, cy - h // 2, cx + w // 2 + pdx // 2, cy + h // 2],
                            radius=min(14, h // 2), fill=1)
        if h > 14:  # pupil
            px, py = cx + pdx, cy + pdy
            d.ellipse([px - 9, py - 9, px + 9, py + 9], fill=0)
            d.ellipse([px + 1, py - 6, px + 5, py - 2], fill=1)  # highlight
        if lid == "bottom":  # smiling eyes: cut the lower half with an arc
            d.ellipse([cx - 40, cy + 2, cx + 40, cy + 50], fill=0)
        if lid == "top":
            d.rectangle([0, 0, W, cy - 2], fill=0)
        return img

    def _run(self):
        period = 1.0 / self._fps
        while True:
            now = time.time()
            with self._lock:
                if self.until and now > self.until:
                    self.expression, self.until = self.after, 0.0
                expr = self.expression
            blink = False
            if now > self._blink_at:
                blink = now < self._blink_at + 0.12
                if not blink:
                    self._blink_at = now + random.uniform(2.5, 6.0)
            imgs = [self._draw(k, expr, blink) for k in (0, 1)]
            frame = Image.new("1", (2 * W + 8, H))
            frame.paste(imgs[0], (0, 0))
            frame.paste(imgs[1], (W + 8, 0))
            with self._lock:
                self.frame = frame
            for dev, img in zip(self.devices, imgs):
                if dev is not None:
                    try:
                        dev.display(img)
                    except OSError as e:  # a loose wire must not stop the service
                        log.warning("OLED write failed: %s", e)
            time.sleep(period)
