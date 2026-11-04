"""Read only UVC MJPEG source for the workstation PICO bridge."""
import argparse
import os
import signal
import socket
import struct
import subprocess
import threading
import time
import uuid

def read_trace_request(conn):
    opcode = conn.recv(1)
    if opcode == b'N':
        return None
    if opcode != b'T':
        raise RuntimeError('Unknown camera request')
    token = bytearray()
    while len(token) < 16:
        part = conn.recv(16-len(token))
        if not part:
            raise RuntimeError('Truncated trace request')
        token.extend(part)
    return uuid.UUID(bytes=bytes(token)).hex

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device', default='/dev/v4l/by-id/usb-USB2.0_Camera_RGB_USB2.0_Camera_RGB_01.00.00-video-index0')
    p.add_argument('--host', default='192.168.123.164')
    p.add_argument('--allow-peer', default='192.168.123.99')
    p.add_argument('--port', type=int, default=15556)
    p.add_argument('--latency-requests', action='store_true',
                   help='Opt-in T+UUID requests; N response bytes remain unchanged')
    a = p.parse_args()
    if a.latency_requests:
        from latency_trace import mark, stamp as trace_stamp
    else:
        def mark(*args, **kwargs): pass
        def trace_stamp(): return None
    stop = threading.Event()
    condition = threading.Condition()
    latest = [None, 0.0, 0, None]
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    command = ['v4l2-ctl', '-d', a.device,
               '--set-fmt-video=width=2560,height=720,pixelformat=MJPG',
               '--set-parm=30', '--stream-mmap=3', '--stream-to=-']
    proc = None
    def capture():
        buf = bytearray()
        try:
            while not stop.is_set():
                chunk = os.read(proc.stdout.fileno(), 65536)
                if not chunk:
                    raise RuntimeError('UVC capture exited')
                buf.extend(chunk)
                while True:
                    start = buf.find(b'\xff\xd8')
                    if start < 0:
                        if len(buf) > 1: del buf[:-1]
                        break
                    if start: del buf[:start]
                    end = buf.find(b'\xff\xd9', 2)
                    if end < 0: break
                    frame = bytes(buf[:end+2]); del buf[:end+2]
                    with condition:
                        latest[:] = [frame, time.monotonic(), latest[2]+1, trace_stamp()]
                        condition.notify_all()
                if len(buf) > 8*1024*1024:
                    raise RuntimeError('Invalid or oversized MJPEG frame')
        except Exception as exc:
            print('capture_failed:', exc, flush=True)
            stop.set()
            with condition: condition.notify_all()
    try:
        with socket.socket() as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((a.host, a.port)); server.listen(1); server.settimeout(.5)
            proc = subprocess.Popen(command, stdout=subprocess.PIPE, bufsize=0)
            worker = threading.Thread(target=capture, daemon=True); worker.start()
            print('listening', a.host, a.port, 'allowed', a.allow_peer, flush=True)
            while not stop.is_set():
                try: conn, peer = server.accept()
                except socket.timeout: continue
                with conn:
                    if peer[0] != a.allow_peer: continue
                    conn.settimeout(1)
                    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    seq = -1
                    try:
                        while not stop.is_set():
                            if a.latency_requests:
                                trace_id = read_trace_request(conn) or uuid.uuid4().hex
                                mark(trace_id, 'g1_camera_requested')
                            else:
                                if conn.recv(1) != b'N': break
                                trace_id = None
                            with condition:
                                condition.wait_for(lambda: stop.is_set() or latest[2] != seq, timeout=.5)
                                frame, stamp, seq, frame_ready = latest
                            if stop.is_set() or frame is None or time.monotonic()-stamp > .5:
                                raise RuntimeError('No fresh camera frame')
                            mark(trace_id, 'g1_mjpeg_ready', at=frame_ready, frame_seq=seq,
                                 timestamp_origin='userspace_complete_mjpeg', exposure_measured=False)
                            mark(trace_id, 'g1_camera_selected', frame_seq=seq)
                            mark(trace_id, 'g1_camera_send', frame_seq=seq)
                            conn.sendall(struct.pack('>I', len(frame)) + frame)
                            mark(trace_id, 'g1_camera_send_done', frame_seq=seq)
                    except (OSError, RuntimeError) as exc:
                        print('client_closed:', exc, flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        if proc:
            proc.terminate()
            try: proc.wait(timeout=3)
            except subprocess.TimeoutExpired: proc.kill(); proc.wait()
        print('usb_source_stopped', flush=True)

if __name__ == '__main__': main()
