"""Actual client loop with fake time, transport, sensors and publishers. NO hardware."""
import asyncio
import json
import unittest
from types import SimpleNamespace
from unittest.mock import patch, MagicMock
import numpy as np
import run_dit4dit_rtc_sonic_dex1 as client

class Finished(Exception): pass

class ClientSyncTests(unittest.TestCase):
    def run_client(self, execute, steps=50, startup_stream_on=False):
        now = [100.]
        published = []
        events = []
        requests = []
        real_sleep = asyncio.sleep
        info = dict(state_dim=31, action_shape=[50,66], token_shape=[50,64], control_hz=50,
                    rtc_mode='disabled', rtc_enabled=False, rtc_supported=True, execution_mode='sync',
                    action_exec_horizon_default=steps, inference_steps=4,
                    checkpoint_sha256='a'*64, config_sha256='b'*64, statistics_sha256='c'*64)
        client.validate_info(info)
        class FakeShared(client.Shared):
            def observation_now(self, t):
                return dict(observation_id=str(t), observed_at=t, source_times=[t]*4)
            def bottom(self, *args): return True
        s=FakeShared();s.grip_phase='READY';s.armed=not startup_stream_on
        async def sleep(dt):
            now[0] += dt
            initial_wait = startup_stream_on and now[0] < 100.1
            if initial_wait:
                self.assertFalse(s.armed)
                self.assertEqual(published, [])
                self.assertEqual(requests, [])
                self.assertIsNone(s.target)
            s.heartbeat = dict(state='CONTROL', stop=False, time=now[0],
                               seq=int(now[0]*50), token_ready=False,
                               external_stream=1 if initial_wait else int(len(published)>=3))
            await real_sleep(0)
        class Recorder:
            def add(self, name, data):
                if name is None:
                    if execute and data['kind']=='sync_wait_target':
                        data=dict(data, target_left=s.target['left'])
                    events.append(data)
                    if data['kind']=='request':
                        requests.append(data)
                        if len(requests)==3: raise Finished()
        class WS:
            def __init__(self):self.pending=None
            async def __aenter__(self):return self
            async def __aexit__(self,*args):return False
            async def send(self, wire):
                self.pending=(json.loads(wire), now[0]+.12)
            async def recv(self):
                while self.pending is None or now[0]<self.pending[1]: await real_sleep(0)
                r,_=self.pending;self.pending=None
                return json.dumps(dict(ok=True,session=r['session'], request_id=r['request_id'],
                    chunk_id='chunk-'+r['request_id'], window_offset=r['consumed'], resources=info,
                    observation_id=r['observation']['observation_id'], observed_at=r['observation']['observed_at'],
                    normalized=np.zeros((50,66)).tolist(),motion_tokens=np.zeros((50,64)).tolist(),
                    gripper_actions=np.repeat((np.arange(50)/100)[:,None],2,axis=1).tolist()))
        pub=MagicMock();pub.send.side_effect=lambda *a,**k:published.append(now[0])
        ctx=MagicMock();ctx.socket.return_value=pub
        a=SimpleNamespace(execute=execute, compact_replies=False,host='fake',port=1,zmq_pub_port=2,diagnostics='minimal')
        d={'target_to_trigger':lambda packet,*args:packet,'pack_pose_message':lambda *args,**kwargs:b'fake'}
        limits={side:SimpleNamespace(closed=0.,opened=1.) for side in ('left','right')}
        async def immediate(fn, *args): return fn(*args)
        async def run():
            with patch.object(client,'time',SimpleNamespace(monotonic=lambda:now[0])), \
                 patch('asyncio.sleep',sleep), patch('asyncio.to_thread',immediate), patch('websockets.connect',return_value=WS()), \
                 patch('zmq.Context',return_value=ctx), patch.object(client.select,'select',return_value=([],[],[])):
                with self.assertRaises(Finished):
                    await client.session_once(a,d,s,info,limits,Recorder(),0)
        asyncio.run(run())
        if startup_stream_on:
            self.assertEqual(len([e for e in events if e['kind']=='startup_wait_external_stream_off']),1)
            self.assertEqual(len([e for e in events if e['kind']=='startup_stream_off_observed']),1)
            self.assertGreaterEqual(requests[0]['time'],100.1)
        consumed=[e for e in events if e['kind']==('delivered' if execute else 'readonly_consumed')]
        self.assertEqual([e['index'] for e in consumed],list(range(steps))*2)
        self.assertEqual([e['consumed'] for e in requests],[0,steps,steps])
        self.assertTrue(all(e['skipped_prefix']==0 for e in events if e['kind']=='accepted'))
        done=[e for e in events if e['kind']=='sync_block_complete']
        for req,completed in zip(requests[1:],done):
            self.assertGreater(min(req['source_times']),completed['completed_after'])
        waits=[e for e in events if e['kind']=='sync_wait_target']
        self.assertTrue(waits)
        self.assertTrue(all(e['index']==steps-1 and e['gripper_raw']==[(steps-1)/100]*2 for e in waits))
        if execute:
            self.assertGreaterEqual(len([e for e in events if e['kind']=='sync_prefill']),3)
            self.assertTrue(all(e['target_left']==(steps-1)/100 for e in waits))
        else:self.assertEqual(published,[])

    def test_existing_stream_waits_unarmed_then_recovers(self):
        self.run_client(True, 16, startup_stream_on=True)

    def test_readonly_full_blocks(self):
        for steps in (1,25,30,50):
            with self.subTest(steps=steps): self.run_client(False, steps)
    def test_fake_publish_prefill_and_wait(self):
        for steps in (1,25,30,50):
            with self.subTest(steps=steps): self.run_client(True, steps)

if __name__=='__main__':unittest.main()
