import importlib.util
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest
import uuid

ROOT=Path(__file__).resolve().parents[2]
SOURCE=Path(__file__).with_name('robot_usb_stereo.py')

class CameraCompatibility(unittest.TestCase):
    def test_legacy_and_trace_wire(self):
        with tempfile.TemporaryDirectory() as td:
            td=Path(td)
            fake=td/'v4l2-ctl'
            fake.write_text('#!/usr/bin/python3\nimport os,time\nwhile True:\n os.write(1,b"\\xff\\xd8test-camera-frame\\xff\\xd9");time.sleep(.02)\n')
            fake.chmod(0o755)
            with socket.socket() as probe:
                probe.bind(('127.0.0.1',0));port=probe.getsockname()[1]
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as udp:
                udp.bind(('127.0.0.1',0));udp.settimeout(2)
                env=dict(os.environ,PATH=str(td)+':'+os.environ['PATH'],
                         LATENCY_TRACE_HOST='g1',LATENCY_TRACE_UDP='127.0.0.1:'+str(udp.getsockname()[1]))
                proc=subprocess.Popen([sys.executable,str(SOURCE),'--host','127.0.0.1',
                                       '--allow-peer','127.0.0.1','--port',str(port),'--latency-requests'],
                                      env=env,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
                try:
                    deadline=time.monotonic()+5
                    while True:
                        try:
                            conn=socket.create_connection(('127.0.0.1',port),timeout=1);break
                        except OSError:
                            if time.monotonic()>deadline:raise
                            time.sleep(.02)
                    def exact(n):
                        result=b''
                        while len(result)<n:
                            part=conn.recv(n-len(result))
                            if not part:raise RuntimeError('EOF')
                            result+=part
                        return result
                    with conn:
                        conn.sendall(b'N')
                        legacy=exact(struct.unpack('>I',exact(4))[0])
                        ident=uuid.uuid4()
                        # Fragment UUID to exercise exact read without assuming packet boundaries.
                        conn.sendall(b'T'+ident.bytes[:5]);conn.sendall(ident.bytes[5:])
                        traced=exact(struct.unpack('>I',exact(4))[0])
                        self.assertEqual(legacy,b'\xff\xd8test-camera-frame\xff\xd9')
                        self.assertEqual(traced,legacy)
                        stages=set()
                        deadline=time.monotonic()+3
                        while len(stages)<5 and time.monotonic()<deadline:
                            e=json.loads(udp.recv(8192))
                            if e['trace_id']==ident.hex:stages.add(e['stage'])
                        self.assertEqual(stages,{'g1_camera_requested','g1_mjpeg_ready','g1_camera_selected',
                                                 'g1_camera_send','g1_camera_send_done'})
                finally:
                    proc.terminate()
                    try:proc.communicate(timeout=3)
                    except subprocess.TimeoutExpired:proc.kill();proc.communicate()

    def test_default_does_not_import_trace_dependency(self):
        s=SOURCE.read_text()
        self.assertIn("if a.latency_requests:\n        from latency_trace import",s)

if __name__=='__main__':unittest.main()
