import copy
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from gear_sonic.utils.teleop.dex1_recording import (
    SampleCache, make_packet, metadata, features, modalities, check_dataset)
from gear_sonic.utils.teleop.dex1_trigger import Limits
from gear_sonic.scripts.run_dex1_trigger_control import DdsGrippers


def packet(now=10., session=1, sequence=0, writes=True):
    c=SimpleNamespace(limits={s:Limits(0.,5.) for s in ('left','right')},
                      phase='READY', requested={'left':0.1,'right':2.5})
    return make_packet(session,sequence,now,{'left':(1.2,now-.01,.3),'right':(2.3,now-.02,-.4)},
                       {'valid':True,'stop':False,'sample_time':now-.01,'left':.2,'right':.8},
                       {'left':(.9,now),'right':(2.5,now)} if writes else {},c)


class RecordingTests(unittest.TestCase):
    def test_continuous_and_held_target(self):
        c=SampleCache(); self.assertTrue(c.accept(packet(),10.))
        f=c.frame(10.01)
        np.testing.assert_allclose(f['observation.dex1_position'],[1.2,2.3])
        np.testing.assert_allclose(f['observation.dex1_velocity'],[.3,-.4])
        np.testing.assert_allclose(f['action.dex1_target_position'],[.9,2.5])
        np.testing.assert_allclose(f['teleop.dex1_requested_position'],[.1,2.5])
        self.assertTrue(f['dex1.feedback_valid'].all())
        self.assertTrue(f['dex1.command_valid'].all())

    def test_stale_missing_restart_and_reordering(self):
        c=SampleCache()
        self.assertFalse(c.frame(10.)['dex1.feedback_valid'].any())
        self.assertTrue(c.accept(packet(),10.))
        self.assertFalse(c.accept(packet(),10.02))
        self.assertFalse(c.frame(10.2)['dex1.command_valid'].any())
        np.testing.assert_allclose(c.frame(10.2)['observation.dex1_position'],[1.2,2.3])
        self.assertTrue(c.accept(packet(10.3,session=2),10.3))
        self.assertEqual(c.frame(10.3)['dex1.session_id'][0],2)
        self.assertFalse(c.accept(packet(10.2,session=1,sequence=100),10.3))

    def test_source_ages_and_invalid_values(self):
        c=SampleCache();p=packet()
        p['sides']['left']['feedback_time']=9.
        p['sides']['right']['dq']=float('nan')
        p['sides']['left']['trigger']=2.
        c.accept(p,10.)
        f=c.frame(10.)
        self.assertFalse(f['dex1.feedback_valid'].any())
        self.assertFalse(f['dex1.trigger_valid'][0])
        self.assertTrue(all(np.isfinite(a).all() for a in f.values()))

    def test_dry_run_and_partial_failure(self):
        d=DdsGrippers.__new__(DdsGrippers)
        d.publishers={};d.kp=5.;d.kd=.05
        d.write({'left':1.})
        self.assertEqual(d.last_writes,{})
        d.motor_type=lambda **kw:kw;d.command_type=lambda **kw:kw
        d.publishers={'left':SimpleNamespace(Write=lambda *a,**kw:True),
                      'right':SimpleNamespace(Write=lambda *a,**kw:False)}
        with self.assertRaises(RuntimeError):d.write({'left':1.,'right':2.})
        self.assertEqual(set(d.last_writes),{'left'})
        c=SampleCache();c.accept(packet(writes=False),10.)
        self.assertFalse(c.frame(10.)['dex1.command_valid'].any())

    def test_preflight(self):
        with tempfile.TemporaryDirectory() as tmp:
            r=Path(tmp)/'dataset';check_dataset(r,{})
            (r/'meta').mkdir(parents=True)
            with self.assertRaises(ValueError):check_dataset(r,{})
            info={'script_config':{'dex1_recording':metadata({})},'features':features()}
            (r/'meta/info.json').write_text(json.dumps(info))
            (r/'meta/modality.json').write_text(json.dumps(modalities()))
            check_dataset(r,{})
            with self.assertRaises(ValueError):check_dataset(r,{'changed':True})
            info['features']['observation.dex1_position']['shape']=[7]
            (r/'meta/info.json').write_text(json.dumps(info))
            with self.assertRaises(ValueError):check_dataset(r,{})

    def test_local_transport(self):
        import socket,time
        from gear_sonic.utils.teleop.dex1_recording import Publisher,Subscriber
        with socket.socket() as s:s.bind(('127.0.0.1',0));port=s.getsockname()[1]
        pub=Publisher(port);sub=Subscriber(port)
        try:
            deadline=time.monotonic()+2
            while time.monotonic()<deadline and sub.packet is None:
                p=packet(time.monotonic())
                import msgpack,zmq
                from gear_sonic.utils.teleop.dex1_recording import TOPIC
                pub.socket.send(TOPIC+msgpack.packb(p),flags=zmq.NOBLOCK)
                time.sleep(.02);sub.poll()
            self.assertIsNotNone(sub.packet)
            self.assertTrue(sub.frame(time.monotonic())['dex1.feedback_valid'].all())
        finally:pub.close();sub.close()


if __name__=='__main__':unittest.main()
