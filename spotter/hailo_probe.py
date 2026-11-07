"""M3 check: show exactly what the Hailo model returns, and the person boxes we parse from it.

python3 -m spotter.hailo_probe [model.hef]      (stand in front of the camera)
"""
import sys
import time

import numpy as np


def describe(obj, indent=0, limit=3):
    pad = "  " * indent
    if isinstance(obj, dict):
        print(f"{pad}dict with {len(obj)} keys")
        for k, v in list(obj.items())[:limit]:
            print(f"{pad}  [{k!r}]:")
            describe(v, indent + 2, limit)
    elif isinstance(obj, (list, tuple)):
        print(f"{pad}{type(obj).__name__} len={len(obj)}")
        nonempty = [(i, x) for i, x in enumerate(obj) if not hasattr(x, "__len__") or len(x)]
        for i, x in (nonempty or list(enumerate(obj)))[:limit]:
            print(f"{pad}  [{i}]:")
            describe(x, indent + 2, limit)
    elif isinstance(obj, np.ndarray):
        print(f"{pad}ndarray shape={obj.shape} dtype={obj.dtype} first={np.round(obj.reshape(-1)[:6], 3).tolist()}")
    else:
        print(f"{pad}{type(obj).__name__}: {obj!r}")


def main():
    from picamera2 import Picamera2
    from picamera2.devices import Hailo

    from spotter.detector import parse_nms, pick_model

    path = sys.argv[1] if len(sys.argv) > 1 else pick_model()
    with Hailo(path) as hailo:
        h, w, _ = hailo.get_input_shape()
        print(f"model {path}  input {w}x{h}")
        picam2 = Picamera2()
        picam2.configure(picam2.create_preview_configuration(
            main={"size": (1280, 720), "format": "RGB888"}, lores={"size": (w, h), "format": "RGB888"}))
        picam2.start()
        time.sleep(1.0)                                   # let exposure settle
        frame = picam2.capture_array("lores")
        print(f"lores frame {frame.shape} {frame.dtype}")
        out = hailo.run(frame)
        print("---- raw output ----")
        describe(out)
        print("---- parsed person boxes ----")
        try:
            for d in parse_nms(out):
                print(f"  score {d.score:.2f}  x {d.x0:.2f}-{d.x1:.2f}  y {d.y0:.2f}-{d.y1:.2f}")
        except Exception as e:
            print(f"  parse_nms failed: {e!r}  <- adapt parse_nms to the raw output above")
        t0, n = time.monotonic(), 30
        for _ in range(n):
            hailo.run(picam2.capture_array("lores"))
        print(f"capture+inference: {n / (time.monotonic() - t0):.1f} FPS")
        picam2.stop()


if __name__ == "__main__":
    main()
