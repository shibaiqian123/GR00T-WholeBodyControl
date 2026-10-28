"""Model-only observations: independent DDS and optional existing camera reuse."""
import argparse,signal,socket,subprocess,time
from pathlib import Path

def main():
    p=argparse.ArgumentParser();p.add_argument('--interface',default='enx6c1ff7b8d821');a=p.parse_args()
    root=Path(__file__).resolve().parents[2];python=str(root/'.venv_teleop/bin/python');children=[];source=None;publisher=None
    try:
        with socket.socket() as s:s.settimeout(1);camera_present=s.connect_ex(('127.0.0.1',5555))==0
        if camera_present:print('Reusing camera 5555; it will not be stopped by this process.',flush=True)
        else:
            from gear_sonic.scripts.run_pico_usb_stereo import USBSource
            from gear_sonic.utils.video.recording_publisher import RecordingPublisher
            class Log:
                def emit(self,event,**data):print(event,data,flush=True)
            source=USBSource('192.168.123.164',15556,Log())
            source.start();publisher=RecordingPublisher(source,5555);publisher.start()
            print('Right-eye camera publisher started; no headset/VR connection.',flush=True)
        children.append(subprocess.Popen([python,'-u','-m','gear_sonic.scripts.run_dit4dit_body_observer','--interface',a.interface],cwd=root))
        while all(x.poll() is None for x in children):time.sleep(.5)
    except KeyboardInterrupt:pass
    finally:
        if publisher is not None:publisher.stop()
        if source is not None:source.stop()
        for child in children:
            if child.poll() is None:child.send_signal(signal.SIGINT)
        for child in children:
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=5)
if __name__=='__main__':main()
