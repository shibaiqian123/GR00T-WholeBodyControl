"""Optional local-only NVENC tests. Run with PICO_NVENC_TESTS=1. No robot DDS."""
import os
import socket
import struct
import subprocess
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from gear_sonic.utils.video.pico_bridge import CameraRequest,EventLog,LatestFrame,VideoSession,test_frame

def exact(sock,n):
    result=bytearray()
    while len(result)<n:
        part=sock.recv(n-len(result))
        if not part:raise EOFError('video socket closed')
        result.extend(part)
    return bytes(result)

@unittest.skipUnless(os.environ.get('PICO_NVENC_TESTS')=='1','explicit opt-in to local GPU tests')
class VideoRuntimeTests(unittest.TestCase):
    def create(self,directory,layout="stereo"):
        listener=socket.socket();listener.bind(('127.0.0.1',0));listener.listen();listener.settimeout(5)
        log=EventLog(Path(directory)/'logs')
        latest=LatestFrame();latest.put(test_frame(0))
        source=SimpleNamespace(latest=latest,mode='test',fps=10)
        req=CameraRequest(1280,720,30,2000000,0,0,listener.getsockname()[1],'ZED','127.0.0.1')
        session=VideoSession(req,source,log,1,layout=layout);session.start()
        conn,_=listener.accept();conn.settimeout(5)
        return listener,log,latest,session,conn
    def test_expired_source_encodes_stale_slate_and_closes_cleanly(self):
        with tempfile.TemporaryDirectory() as directory:
            listener,log,latest,session,conn=self.create(directory)
            frames=[]
            try:
                for _ in range(65):
                    n=struct.unpack('>I',exact(conn,4))[0];frames.append(exact(conn,n))
                decoded=subprocess.run(['ffmpeg','-nostdin','-hide_banner','-loglevel','error','-f','h264','-i','pipe:0',
                    '-vf','select=eq(n\\,60)','-frames:v','1','-f','rawvideo','-pix_fmt','bgr24','pipe:1'],
                    input=b''.join(frames),capture_output=True,timeout=10)
                self.assertEqual(decoded.returncode,0,decoded.stderr.decode())
                image=np.frombuffer(decoded.stdout,np.uint8).reshape(720,1280,3)
                self.assertGreater(image[20,20,2],60)
                self.assertLess(image[20,20,2],90)
                self.assertGreater(session.stale_frames,0)
                self.assertLessEqual(session.queue_high,3)
            finally:
                session.stop();conn.close();listener.close();log.close()
            self.assertIsNotNone(session.proc.poll())
            self.assertFalse(session.worker.is_alive())
    def test_flat_layout_survives_real_encoding(self):
        with tempfile.TemporaryDirectory() as directory:
            listener,log,latest,session,conn=self.create(directory,layout='mono')
            frames=[]
            try:
                for _ in range(5):
                    n=struct.unpack('>I',exact(conn,4))[0];frames.append(exact(conn,n))
                decoded=subprocess.run(['ffmpeg','-nostdin','-hide_banner','-loglevel','error','-f','h264','-i','pipe:0',
                    '-vf','select=eq(n\\,1)','-frames:v','1','-f','rawvideo','-pix_fmt','bgr24','pipe:1'],
                    input=b''.join(frames),capture_output=True,timeout=10)
                self.assertEqual(decoded.returncode,0,decoded.stderr.decode())
                image=np.frombuffer(decoded.stdout,np.uint8).reshape(720,1280,3)
                self.assertGreater(image[20,20,2],130)
                self.assertGreater(image[20,1260,1],130)
            finally:session.stop();conn.close();listener.close();log.close()

    def test_receiver_disconnect_stops_session(self):
        with tempfile.TemporaryDirectory() as directory:
            listener,log,latest,session,conn=self.create(directory)
            try:
                n=struct.unpack('>I',exact(conn,4))[0];exact(conn,n)
                conn.shutdown(socket.SHUT_RDWR);conn.close()
                until=time.monotonic()+5
                while not session.stop_event.is_set() and time.monotonic()<until:time.sleep(.05)
                self.assertTrue(session.stop_event.is_set())
                self.assertTrue(session.failed)
            finally:session.stop();listener.close();log.close()
    def test_slow_receiver_causes_bounded_failure_not_encoded_frame_drops(self):
        with tempfile.TemporaryDirectory() as directory:
            listener,log,latest,session,conn=self.create(directory)
            conn.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,1024)
            stop=threading.Event()
            def noise():
                rng=np.random.default_rng(17)
                while not stop.is_set():
                    latest.put(rng.integers(0,256,(360,640,3),np.uint8));stop.wait(.05)
            worker=threading.Thread(target=noise);worker.start()
            try:
                until=time.monotonic()+15
                while not session.stop_event.is_set() and time.monotonic()<until:time.sleep(.05)
                self.assertTrue(session.stop_event.is_set())
                self.assertTrue(session.failed)
                self.assertLessEqual(session.queue_high,3)
            finally:
                stop.set();worker.join();session.stop();conn.close();listener.close();log.close()

if __name__=='__main__':unittest.main()
