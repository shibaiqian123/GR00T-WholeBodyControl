"""Subscribe to rt/lowstate only; publish diagnostic state on localhost, never motor commands."""
import argparse
import json
import threading
import time
import uuid
from gear_sonic.utils.inference.readonly_body import LowStateCache, TOPIC


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--interface',required=True)
    p.add_argument('--domain',type=int,default=0)
    p.add_argument('--port',type=int,default=5564)
    p.add_argument('--duration',type=float,default=0)
    a=p.parse_args()
    if not 0 <= a.domain <= 232 or not 1024 <= a.port <= 65535 or a.duration<0:
        p.error('Invalid domain, port or duration')
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    import msgpack,zmq
    cache=LowStateCache();lock=threading.Lock()
    def receive(msg):
        with lock:
            try:
                cache.accept(int(msg.tick),[float(msg.motor_state[i].q) for i in range(29)],time.monotonic())
            except (AttributeError,IndexError,TypeError,ValueError):
                cache.error='malformed_lowstate'
    ChannelFactoryInitialize(a.domain,a.interface)
    sub=ChannelSubscriber('rt/lowstate',LowState_)
    sub.Init(receive,1)
    ctx=zmq.Context();sock=ctx.socket(zmq.PUB);sock.setsockopt(zmq.LINGER,0)
    sock.setsockopt(zmq.SNDHWM,5);sock.bind(f'tcp://127.0.0.1:{a.port}')
    session=uuid.uuid4().hex;seq=0;start=time.monotonic();last=-1.
    print('READ ONLY: DDS subscriber only; no command publishers',flush=True)
    try:
        while not a.duration or time.monotonic()-start<a.duration:
            with lock:
                now=time.monotonic()
                packet=cache.packet(session,seq,now)
            sock.send(TOPIC+msgpack.packb(packet,use_bin_type=True));seq+=1
            if now-last>=1:
                print(json.dumps(dict(valid=packet['valid'],source_tick=packet['source_tick'],
                    age=None if packet['sample_time'] is None else now-packet['sample_time'],
                    error=packet['error'],duplicate_samples=packet['duplicate_samples'])),flush=True);last=now
            time.sleep(.02)
    except KeyboardInterrupt:
        pass
    finally:
        sub.Close();sock.close();ctx.term()

if __name__=='__main__':main()
