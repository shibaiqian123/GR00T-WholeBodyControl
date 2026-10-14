import struct
import unittest
import numpy as np
from gear_sonic.utils.video.pico_bridge import (ControlFramer,CameraRequest,AccessUnitFramer,
    LatestFrame,stereo_frame,mono_frame,pack_control,encoder_command)

def camera_payload(width=2560,height=720,fps=60,bitrate=4000000,port=12345,camera='ZED',ip='192.168.1.199',mv=0):
    data=b'\xca\xfe\x01'+struct.pack('<7i',width,height,fps,bitrate,mv,0,port)
    for value in (camera,ip):
        b=value.encode();data+=bytes([len(b)])+b
    return data

class ProtocolTests(unittest.TestCase):
    def test_control_fragmentation_and_multiple_frames(self):
        raw=pack_control('OPEN_CAMERA',camera_payload())+pack_control('CLOSE_CAMERA')
        for chunk_size in (1,2,3,4,7,1024):
            f=ControlFramer();out=[]
            for i in range(0,len(raw),chunk_size):out+=f.feed(raw[i:i+chunk_size])
            self.assertEqual(out,[('OPEN_CAMERA',camera_payload()),('CLOSE_CAMERA',b'')])
    def test_control_rejects_huge_or_malformed(self):
        for raw in [struct.pack('>I',2),struct.pack('>I',1000000),pack_control('X')[:-1]+b'\xff']:
            with self.assertRaises(ValueError):ControlFramer().feed(raw)
    def test_request_defaults_and_destination(self):
        r=CameraRequest.parse(camera_payload(),'192.168.1.199')
        self.assertEqual((r.width,r.height,r.fps,r.port),(2560,720,60,12345))
        self.assertEqual(CameraRequest.parse(camera_payload(ip=''),'192.168.1.199').ip,'192.168.1.199')
    def test_request_rejects_redirect_and_unsupported_layout(self):
        for args in [dict(ip='192.168.123.164'),dict(camera='VR'),dict(mv=1),dict(width=2559),dict(width=8000),dict(port=80),dict(fps=0)]:
            with self.assertRaises(ValueError):CameraRequest.parse(camera_payload(**args),'192.168.1.199')
    def test_request_truncation(self):
        payload=camera_payload()
        for end in range(len(payload)):
            with self.assertRaises(ValueError):CameraRequest.parse(payload[:end],'192.168.1.199')
    def test_access_units_all_chunk_boundaries(self):
        au1=b'\0\0\0\1\x09\xf0\0\0\0\1\x67sps\0\0\1\x65frame1'
        au2=b'\0\0\1\x09\xf0\0\0\1\x41frame2'
        for n in range(1,20):
            parser=AccessUnitFramer();out=[];raw=au1+au2
            for i in range(0,len(raw),n):out+=parser.feed(raw[i:i+n])
            out+=parser.finish();self.assertEqual(out,[au1,au2])
    def test_latest_frame_overwrites_old_without_queue(self):
        f=LatestFrame()
        for i in range(100):f.put(i)
        self.assertEqual(f.get()[0],99);self.assertEqual(f.get()[2],100)
    def test_mono_stereo_contains_two_identical_complete_eyes(self):
        img=np.zeros((100,200,3),np.uint8);img[:,:100]=(255,0,0);img[:,100:]=(0,255,0)
        out=stereo_frame(img,800,300,'TEST')
        self.assertEqual(out.shape,(300,800,3))
        self.assertTrue(np.array_equal(out[:,:400],out[:,400:]))
        self.assertTrue(np.array_equal(out[140,20],(255,0,0)))
        self.assertTrue(np.array_equal(out[140,380],(0,255,0)))
        self.assertTrue(np.all(out[0]==0))
    def test_legacy_eye_offset_compensation_aligns_sampled_centers(self):
        frame=np.zeros((200,400,3),np.uint8)
        frame[:,:,0]=np.arange(400,dtype=np.uint16)[None,:]%256
        out=stereo_frame(frame,800,200,'',.08)
        # The legacy shader samples left at x+.08 and right at x-.08.
        for x in (80,160,200,260):
            self.assertTrue(np.array_equal(out[80,x+32],out[80,400+x-32]))
            self.assertTrue(np.array_equal(out[80,x+32],frame[80,x]))

    def test_flat_mono_is_one_complete_centered_image_not_two_eyes(self):
        frame=np.zeros((100,200,3),np.uint8);frame[:,:100]=(255,0,0);frame[:,100:]=(0,255,0)
        out=mono_frame(frame,800,200,'')
        self.assertEqual(out.shape,(200,800,3))
        self.assertTrue(np.array_equal(out[80,220],(255,0,0)))
        self.assertTrue(np.array_equal(out[80,580],(0,255,0)))
        self.assertTrue(np.all(out[80,:200]==0));self.assertTrue(np.all(out[80,600:]==0))

    def test_stale_frame_is_visibly_distinct(self):
        out=stereo_frame(None,1280,720,'VIDEO STALE')
        self.assertTrue(np.array_equal(out[:,:640],out[:,640:]))
        self.assertEqual(out[0,0,2],75)
    def test_encoder_disables_reordering_and_has_aud(self):
        cmd=encoder_command(2560,720,30,4000000)
        for flag,value in [('-bf','0'),('-aud','1'),('-rc-lookahead','0'),('-g','30')]:
            self.assertEqual(cmd[cmd.index(flag)+1],value)

if __name__=='__main__':unittest.main()
