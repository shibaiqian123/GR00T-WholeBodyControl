"""Bridge the G1 stock camera to existing PICO XRoboToolkit Remote Vision."""
import argparse
import datetime
import os
import signal
from pathlib import Path
from gear_sonic.utils.video.pico_bridge import Bridge,CameraSource,EventLog

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--interface',default='enx6c1ff7b8d821')
    p.add_argument('--listen-host',default='192.168.1.103')
    p.add_argument('--listen-port',type=int,default=13579)
    p.add_argument('--headset',default='192.168.1.199')
    p.add_argument('--source',choices=['robot','test'],default='robot')
    p.add_argument('--source-fps',type=int,default=10)
    p.add_argument('--max-bitrate',type=int,default=4000000)
    p.add_argument('--layout',choices=['mono','stereo'],default='mono',help='mono: one image for flat screen; stereo: duplicated side-by-side eyes')
    p.add_argument('--eye-shift',type=float,default=0.0,help='Per-eye inverse horizontal shift; legacy ZEDMINI shader candidate: 0.08')
    p.add_argument('--duration',type=float,help='Optional bounded run for validation')
    p.add_argument('--log-dir',type=Path)
    a=p.parse_args()
    if a.layout=='mono' and a.eye_shift:p.error('eye-shift applies only to stereo layout')
    if not -.2<=a.eye_shift<=.2:p.error('eye-shift must be in [-0.2,0.2]')
    if not 1<=a.source_fps<=15:p.error('source-fps must be 1..15 for the verified image RPC path')
    if not 100000<=a.max_bitrate<=8000000:p.error('max-bitrate must be 100000..8000000')
    if not 1024<=a.listen_port<=65535:p.error('Invalid control port')
    if a.duration is not None and a.duration<=0:p.error('duration must be positive')
    directory=a.log_dir or Path.home()/'pico-video-runs'/(datetime.datetime.now().strftime('%Y%m%d-%H%M%S')+f'-{os.getpid()}')
    log=EventLog(directory);source=CameraSource(a.source,a.interface,a.source_fps,log)
    bridge=Bridge(a.listen_host,a.listen_port,a.headset,source,log,a.max_bitrate,a.eye_shift,a.layout)
    signal.signal(signal.SIGTERM, lambda signum, frame: bridge.stop_event.set())
    try:bridge.run(a.duration)
    except KeyboardInterrupt:print('Stopping video bridge only.',flush=True)
    finally:log.close()
if __name__=='__main__':main()
