"""Start the Pi service.

On the robot:      python -m humanoid_pi                          (Mega on /dev/ttyACM0)
Against the twin:  python -m humanoid_pi --port socket://localhost:5555 --no-camera
Then open  http://<robot address>:8080
"""
import argparse
import logging

from .behaviour import Behaviour
from .camera import Camera
from .eyes import Eyes
from .link import MegaLink
from .web import create_app


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0", help="Mega serial port, or socket://host:port for the twin")
    ap.add_argument("--host", default="0.0.0.0", help="web server address")
    ap.add_argument("--web-port", type=int, default=8080)
    ap.add_argument("--token", default="", help="if set, commands need ?token=... in the page URL")
    ap.add_argument("--no-eyes", action="store_true", help="do not drive the OLEDs (still drawn for the page)")
    ap.add_argument("--no-camera", action="store_true")
    ap.add_argument("--camera-rotate", action="store_true", help="rotate the image 180 degrees")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    link = MegaLink(a.port).start()
    eyes = Eyes(use_hardware=not a.no_eyes)
    camera = Camera(enabled=not a.no_camera, rotate180=a.camera_rotate)
    behaviour = Behaviour(link, eyes)
    app = create_app(link, behaviour, eyes, camera, token=a.token)
    logging.getLogger("werkzeug").setLevel(logging.WARNING)
    app.run(host=a.host, port=a.web_port, threaded=True)


if __name__ == "__main__":
    main()
