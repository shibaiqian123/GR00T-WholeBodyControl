"""Publish the bridge's latest right-eye frame for local data collection."""
import threading
import time
import cv2
import msgpack
import numpy as np
import zmq


def right_eye_recording_frame(frame):
    if frame.shape != (720, 2560, 3):
        raise ValueError("Expected 2560x720 USB stereo frame")
    # Preserve the complete 16:9 view inside the existing 640x480 dataset schema.
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    image[60:420] = cv2.resize(frame[:, 1280:], (640, 360), interpolation=cv2.INTER_AREA)
    return image


class RecordingPublisher:
    def __init__(self, source, port):
        self.source, self.port = source, port
        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self.run, daemon=True)

    def start(self):
        self.thread.start()
        if not self.ready.wait(5):
            raise RuntimeError("Recording publisher did not start")
        if self.error:
            raise RuntimeError("Recording publisher failed") from self.error

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=3)

    def run(self):
        context = zmq.Context()
        sock = context.socket(zmq.PUB)
        sock.setsockopt(zmq.SNDHWM, 2)
        sock.setsockopt(zmq.LINGER, 0)
        try:
            sock.bind(f"tcp://127.0.0.1:{self.port}")
            self.ready.set()
            seq = -1
            while not self.stop_event.wait(.005):
                frame, received, current = self.source.latest.get()
                if frame is None or current == seq or time.monotonic()-received > .5:
                    continue
                seq = current
                image = right_eye_recording_frame(frame)
                ok, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 95])
                if not ok:
                    raise RuntimeError("Could not encode recording image")
                # Desktop receive time, not hardware exposure time.
                stamp = time.time() - (time.monotonic()-received)
                payload = {"timestamps": {"ego_view": stamp},
                           "images": {"ego_view": jpeg.tobytes()}}
                sock.send(msgpack.packb(payload, use_bin_type=True))
        except Exception as exc:
            self.error = exc
            self.ready.set()
            self.source.log.emit("recording_publisher_error", error=str(exc))
        finally:
            sock.close()
            context.term()
