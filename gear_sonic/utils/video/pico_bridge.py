"""XRoboToolkit native Remote Vision bridge. Does not publish robot commands."""
import dataclasses
import json
import os
from pathlib import Path
import queue
import re
import socket
import struct
import subprocess
import threading
import time

MAX_CONTROL = 65536
MAX_AU = 8 * 1024 * 1024

class ControlFramer:
    """Outer uint32 BE length; inner LE command length, command, payload length, payload."""
    def __init__(self): self.buffer = bytearray()
    def feed(self, data):
        self.buffer.extend(data)
        result=[]
        while len(self.buffer)>=4:
            n=struct.unpack_from('>I',self.buffer)[0]
            if n<8 or n>MAX_CONTROL: raise ValueError('Invalid control frame length')
            if len(self.buffer)<4+n: break
            body=bytes(self.buffer[4:4+n]);del self.buffer[:4+n]
            cn=struct.unpack_from('<i',body)[0]
            if not 0<cn<=128 or 8+cn>len(body): raise ValueError('Invalid command length')
            cmd=body[4:4+cn].decode('ascii').rstrip('\0')
            dn=struct.unpack_from('<i',body,4+cn)[0]
            if dn<0 or 8+cn+dn!=len(body): raise ValueError('Invalid command payload length')
            result.append((cmd,body[8+cn:]))
        return result

def pack_control(command,payload=b''):
    cmd=command.encode('ascii');body=struct.pack('<i',len(cmd))+cmd+struct.pack('<i',len(payload))+payload
    return struct.pack('>I',len(body))+body

@dataclasses.dataclass(frozen=True)
class CameraRequest:
    width:int; height:int; fps:int; bitrate:int; mv_hevc:int; render_mode:int; port:int; camera:str; ip:str

    @classmethod
    def parse(cls,data,peer):
        if len(data)<33 or data[:3]!=b'\xca\xfe\x01': raise ValueError('Unsupported camera request magic/version')
        fields=struct.unpack_from('<7i',data,3);pos=31;strings=[]
        for _ in range(2):
            if pos>=len(data): raise ValueError('Missing camera string')
            n=data[pos];pos+=1
            if pos+n>len(data): raise ValueError('Truncated camera string')
            strings.append(data[pos:pos+n].decode('utf-8'));pos+=n
        if pos!=len(data): raise ValueError('Unexpected camera payload tail')
        camera,ip=strings
        if not ip or ip=='0.0.0.0': ip=peer
        r=cls(*fields,camera,ip)
        if r.camera!='ZED': raise ValueError('Select ZEDMINI in Remote Vision')
        if r.ip!=peer: raise ValueError('Video destination must match the headset control peer')
        if not 320<=r.width<=3840 or r.width%4 or not 180<=r.height<=2160 or r.height%2:
            raise ValueError('Unsupported side-by-side dimensions')
        if r.width*r.height>3840*1080: raise ValueError('Frame too large')
        if not 1<=r.fps<=120 or not 1<=r.bitrate<=100000000 or not 1024<=r.port<=65535:
            raise ValueError('Invalid fps/bitrate/video port')
        if r.mv_hevc: raise ValueError('MV-HEVC is unsupported; H.264 required')
        return r

class AccessUnitFramer:
    """Annex-B H.264 access units delimited by AUD NALs. At most one AU lookahead.

    Never drop individual encoded P frames: on transport backpressure restart
    the session/encoder with a new IDR instead of emitting a broken reference chain.
    """
    def __init__(self): self.buffer=bytearray()
    def feed(self,data):
        self.buffer.extend(data);out=[]
        while True:
            starts=[m.start() for m in re.finditer(b'\x00\x00(?:\x00)?\x01',self.buffer)
                    if m.end()<len(self.buffer) and self.buffer[m.end()]&31==9]
            if len(starts)<2: break
            end=starts[1];out.append(bytes(self.buffer[:end]));del self.buffer[:end]
        if len(self.buffer)>MAX_AU: raise ValueError('H.264 AU exceeded size limit or missing AUD')
        return out
    def finish(self):
        result=bytes(self.buffer);self.buffer.clear();return [result] if result else []

class LatestFrame:
    def __init__(self): self.lock=threading.Lock();self.frame=None;self.received=0.;self.sequence=0
    def put(self,frame):
        with self.lock:self.frame=frame;self.received=time.monotonic();self.sequence+=1
    def get(self):
        with self.lock:return self.frame,self.received,self.sequence

def stereo_frame(frame,width,height,label,eye_shift=0.0):
    """Both eyes see the full same mono image, letterboxed rather than cropped."""
    import cv2,numpy as np
    ew=width//2;eye=np.zeros((height,ew,3),dtype=np.uint8)
    if frame is not None:
        h,w=frame.shape[:2];scale=min(ew/w,height/h)
        nw=max(1,round(w*scale));nh=max(1,round(h*scale))
        small=cv2.resize(frame,(nw,nh),interpolation=cv2.INTER_AREA)
        x=(ew-nw)//2;y=(height-nh)//2;eye[y:y+nh,x:x+nw]=small
    else:
        eye[:]=(0,0,75)
    font=max(.35,min(ew/1000.,height/600.))
    cv2.putText(eye,label,(12,max(25,height-18)),cv2.FONT_HERSHEY_SIMPLEX,font,(255,255,255),1,cv2.LINE_AA)
    if eye_shift:
        # Inverse of legacy SampleRT.shader's +0.08/-0.08 eye texture offsets.
        # Keep this configurable because installed APKs may use different shaders.
        dx=round(ew*eye_shift)
        left=cv2.warpAffine(eye,np.float32([[1,0,dx],[0,1,0]]),(ew,height))
        right=cv2.warpAffine(eye,np.float32([[1,0,-dx],[0,1,0]]),(ew,height))
    else:left=right=eye
    return np.concatenate([left,right],axis=1)

def mono_frame(frame,width,height,label):
    """One complete image for the existing APK flat/screen view (no stereo split)."""
    import cv2,numpy as np
    canvas=np.zeros((height,width,3),np.uint8)
    if frame is not None:
        h,w=frame.shape[:2];scale=min(width/w,height/h)
        nw=max(1,round(w*scale));nh=max(1,round(h*scale))
        small=cv2.resize(frame,(nw,nh),interpolation=cv2.INTER_AREA)
        x=(width-nw)//2;y=(height-nh)//2;canvas[y:y+nh,x:x+nw]=small
    else:canvas[:]=(0,0,75)
    font=max(.35,min(width/1600.,height/600.))
    cv2.putText(canvas,label,(12,max(25,height-18)),cv2.FONT_HERSHEY_SIMPLEX,font,(255,255,255),1,cv2.LINE_AA)
    return canvas

def test_frame(t):
    import cv2,numpy as np
    f=np.zeros((360,640,3),np.uint8)
    f[:180,:320]=(0,0,160);f[:180,320:]=(0,150,0)
    f[180:,:320]=(160,0,0);f[180:,320:]=(60,60,60)
    cv2.putText(f,'TOP - MONO TEST',(150,45),cv2.FONT_HERSHEY_SIMPLEX,.8,(255,255,255),2)
    cv2.putText(f,'LEFT',(20,170),cv2.FONT_HERSHEY_SIMPLEX,.8,(255,255,255),2)
    cv2.putText(f,'RIGHT',(480,170),cv2.FONT_HERSHEY_SIMPLEX,.8,(255,255,255),2)
    cv2.circle(f,(int(t*80)%600+20,260),16,(0,255,255),-1)
    cv2.putText(f,f'{t:.2f}',(220,330),cv2.FONT_HERSHEY_SIMPLEX,.8,(255,255,255),2)
    return f

class EventLog:
    def __init__(self,directory):
        self.directory=Path(directory);self.directory.mkdir(parents=True,exist_ok=False)
        self.file=(self.directory/'events.jsonl').open('x',buffering=1);self.lock=threading.Lock()
    def emit(self,event,**fields):
        row={'wall_time':time.time(),'monotonic':time.monotonic(),'event':event,**fields}
        with self.lock:
            self.file.write(json.dumps(row,default=str)+'\n')
            print(json.dumps(row,default=str),flush=True)
    def close(self):self.file.close()

class CameraSource:
    def __init__(self,mode,interface,fps,log):
        self.mode=mode;self.interface=interface;self.fps=fps;self.log=log
        self.latest=LatestFrame();self.stop_event=threading.Event();self.frames=0;self.errors=0
        self.last_rpc_ms=None;self.last_error='';self.thread=threading.Thread(target=self.run,daemon=True)
    def start(self):self.thread.start()
    def run(self):
        client=None
        try:
            if self.mode=='robot':
                from unitree_sdk2py.core.channel import ChannelFactoryInitialize
                from unitree_sdk2py.go2.video.video_client import VideoClient
                # Read-only videohub API 1001, verified on this G1. No actuator interfaces.
                ChannelFactoryInitialize(0,self.interface)
                client=VideoClient();client.SetTimeout(.5);client.Init()
            import cv2,numpy as np
            while not self.stop_event.is_set():
                begin=time.monotonic()
                try:
                    if self.mode=='test':frame=test_frame(begin)
                    else:
                        code,data=client.GetImageSample();self.last_rpc_ms=(time.monotonic()-begin)*1000
                        if code!=0 or not data:raise RuntimeError(f'videohub return code {code}')
                        frame=cv2.imdecode(np.frombuffer(bytes(data),np.uint8),cv2.IMREAD_COLOR)
                        if frame is None:raise RuntimeError('JPEG decode failed')
                    self.latest.put(frame);self.frames+=1;self.last_error=''
                except Exception as exc:
                    self.errors+=1;self.last_error=str(exc)
                    if self.errors==1 or self.errors%20==0:self.log.emit('source_error',error=str(exc),count=self.errors)
                self.stop_event.wait(max(0.,1/self.fps-(time.monotonic()-begin)))
        except Exception as exc:
            self.last_error=str(exc);self.log.emit('source_failed',error=str(exc))
    def stop(self):
        self.stop_event.set()
        if self.thread.ident is not None:self.thread.join(timeout=3)

def encoder_command(width,height,fps,bitrate):
    return ['ffmpeg','-nostdin','-hide_banner','-loglevel','warning','-f','rawvideo','-pix_fmt','bgr24',
        '-video_size',f'{width}x{height}','-framerate',str(fps),'-i','pipe:0','-an',
        '-c:v','h264_nvenc','-preset','p1','-tune','ull','-zerolatency','1','-delay','0',
        '-rc-lookahead','0','-bf','0','-g',str(fps),'-b:v',str(bitrate),'-maxrate',str(bitrate),
        '-bufsize',str(max(bitrate//fps,100000)),'-pix_fmt','yuv420p','-aud','1',
        '-flush_packets','1','-f','h264','pipe:1']

class VideoSession:
    def __init__(self,request,source,log,index,max_bitrate=4000000,eye_shift=0.0,layout="stereo"):
        self.request=request;self.source=source;self.log=log;self.index=index;self.eye_shift=eye_shift;self.layout=layout
        self.fps=min(request.fps,30);self.bitrate=min(request.bitrate,max_bitrate)
        self.stop_event=threading.Event();self.failed='';self.proc=None;self.sock=None
        self.units=queue.Queue(3);self.worker=None;self.children=[]
        self.raw_frames=0;self.encoded_frames=0;self.sent_frames=0;self.sent_bytes=0;self.queue_high=0
        self.start_time=time.monotonic();self.end_time=None;self.last_source_sequence=0;self.stale_frames=0
    def start(self):
        self.worker=threading.Thread(target=self.run,daemon=True);self.worker.start()
    def fail(self,reason):
        if not self.failed:self.failed=reason;self.log.emit('session_failed',session=self.index,error=reason)
        self.stop_event.set()
    def encode_input(self):
        try:
            tick=time.monotonic()
            while not self.stop_event.is_set():
                now=time.monotonic();frame,received,seq=self.source.latest.get();age=now-received
                stale=frame is None or age>.75
                if stale:self.stale_frames+=1
                label=('VIDEO STALE - NO LIVE IMAGE' if stale else
                       f'{self.source.mode.upper()} RGB MONO  frame {seq}  RX age {age*1000:.0f} ms')
                if self.layout == "mono":
                    sbs=mono_frame(None if stale else frame,self.request.width,self.request.height,label)
                else:
                    sbs=stereo_frame(None if stale else frame,self.request.width,self.request.height,label,self.eye_shift)
                pending=memoryview(sbs.tobytes())
                while pending and not self.stop_event.is_set():
                    count=self.proc.stdin.write(pending)
                    if not count:raise BrokenPipeError('Encoder stdin closed')
                    pending=pending[count:]
                if self.stop_event.is_set():break
                self.raw_frames+=1;self.last_source_sequence=seq
                tick+=1/self.fps
                if now-tick>.1:tick=now
                self.stop_event.wait(max(0.,tick-time.monotonic()))
        except (BrokenPipeError,OSError,ValueError) as exc:
            if not self.stop_event.is_set():self.fail('encoder input: '+str(exc))
    def encoded_output(self):
        parser=AccessUnitFramer()
        try:
            while not self.stop_event.is_set():
                chunk=os.read(self.proc.stdout.fileno(),65536)
                if not chunk:
                    if not self.stop_event.is_set():self.fail('Encoder exited; inspect encoder stderr log')
                    break
                for au in parser.feed(chunk):
                    self.encoded_frames+=1
                    try:self.units.put_nowait(au);self.queue_high=max(self.queue_high,self.units.qsize())
                    except queue.Full:
                        self.fail('Video transport backpressure; reconnect Listen for a fresh IDR');return
        except Exception as exc:
            if not self.stop_event.is_set():self.fail('encoder output: '+str(exc))
    def run(self):
        stderr=None
        try:
            for attempt in range(10):
                if self.stop_event.is_set():return
                try:
                    self.sock=socket.create_connection((self.request.ip,self.request.port),timeout=.5);break
                except OSError:
                    if attempt==9:raise
                    self.stop_event.wait(.3)
            self.sock.settimeout(.5);self.sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,65536)
            path=self.log.directory/f'encoder-{self.index}.log';stderr=path.open('x')
            command=encoder_command(self.request.width,self.request.height,self.fps,self.bitrate)
            self.proc=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=stderr,bufsize=0)
            for target in (self.encode_input,self.encoded_output):
                thread=threading.Thread(target=target,daemon=True);self.children.append(thread);thread.start()
            self.log.emit('video_connected',session=self.index,peer=self.request.ip,port=self.request.port,
                          width=self.request.width,height=self.request.height,encoded_fps=self.fps,
                          source_fps_limit=self.source.fps,bitrate=self.bitrate,layout=self.layout,eye_shift=self.eye_shift)
            while not self.stop_event.is_set():
                try:au=self.units.get(timeout=.2)
                except queue.Empty:continue
                self.sock.sendall(struct.pack('>I',len(au))+au)
                self.sent_frames+=1;self.sent_bytes+=len(au)
        except Exception as exc:
            if not self.stop_event.is_set():self.fail(str(exc))
        finally:
            self.end_time=time.monotonic()
            self.stop_event.set()
            if self.sock:
                try:self.sock.shutdown(socket.SHUT_RDWR)
                except OSError:pass
                self.sock.close()
            if self.proc:
                try:self.proc.stdin.close()
                except (OSError,ValueError):pass
                if self.proc.poll() is None:
                    self.proc.terminate()
                    try:self.proc.wait(timeout=2)
                    except subprocess.TimeoutExpired:self.proc.kill();self.proc.wait(timeout=2)
                for pipe in (self.proc.stdin,self.proc.stdout):
                    try:pipe.close()
                    except OSError:pass
            for child in self.children:child.join(timeout=1)
            if stderr:stderr.close()
            self.log.emit('session_closed',session=self.index,**self.stats())
    def stats(self):
        elapsed=max(.001,(self.end_time or time.monotonic())-self.start_time)
        return {'raw_frames':self.raw_frames,'encoded_frames':self.encoded_frames,'sent_frames':self.sent_frames,
                'average_sent_fps':self.sent_frames/elapsed,'bytes':self.sent_bytes,'queue_high':self.queue_high,
                'queued_access_units':self.units.qsize(),'stale_frames':self.stale_frames,'error':self.failed}
    def stop(self):
        self.stop_event.set()
        if self.sock:
            try:self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:pass
        if self.worker:self.worker.join(timeout=7)

class Bridge:
    def __init__(self,host,port,headset,source,log,max_bitrate=4000000,eye_shift=0.0,layout="stereo"):
        self.host=host;self.port=port;self.headset=headset;self.source=source;self.log=log
        self.max_bitrate=max_bitrate;self.eye_shift=eye_shift;self.layout=layout;self.stop_event=threading.Event();self.server=None
        self.control=None;self.parser=None;self.peer=None;self.session=None;self.index=0
    def close_control(self):
        if self.session:self.session.stop();self.session=None
        if self.control:self.control.close();self.control=None
    def run(self,duration=None):
        import select
        start=time.monotonic();last_report=start;last_source_frames=0
        self.server=socket.socket();self.server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        try:
            self.server.bind((self.host,self.port));self.server.listen(2)
            self.log.emit('listening',host=self.host,port=self.port,allowed_headset=self.headset,source=self.source.mode)
            self.source.start()
            while not self.stop_event.is_set() and (duration is None or time.monotonic()-start<duration):
                watch=[self.server]+([self.control] if self.control else [])
                ready,_,_=select.select(watch,[],[],.2)
                for sock in ready:
                    if sock is self.server:
                        conn,addr=self.server.accept()
                        if addr[0]!=self.headset:
                            self.log.emit('peer_rejected',peer=addr[0]);conn.close();continue
                        self.close_control();self.control=conn;self.peer=addr[0];self.parser=ControlFramer()
                        self.log.emit('control_connected',peer=addr[0]);continue
                    if sock is not self.control:continue
                    try:
                        data=sock.recv(8192)
                        if not data:
                            self.log.emit('control_disconnected');self.close_control();continue
                        for command,payload in self.parser.feed(data):
                            if command=='OPEN_CAMERA':
                                r=CameraRequest.parse(payload,self.peer)
                                self.log.emit('camera_request',**dataclasses.asdict(r))
                                if self.session:self.session.stop()
                                self.index+=1;self.session=VideoSession(r,self.source,self.log,self.index,self.max_bitrate,self.eye_shift,self.layout);self.session.start()
                            elif command=='CLOSE_CAMERA':
                                self.log.emit('camera_close_request')
                                if self.session:self.session.stop();self.session=None
                            else:self.log.emit('control_command_ignored',command=command)
                    except (OSError,ValueError,UnicodeError) as exc:
                        self.log.emit('control_error',error=str(exc));self.close_control()
                now=time.monotonic()
                if now-last_report>=5:
                    self.log.emit('status',source_frames=self.source.frames,
                                  source_fps=(self.source.frames-last_source_frames)/(now-last_report),
                                  source_errors=self.source.errors,last_rpc_ms=self.source.last_rpc_ms,
                                  source_error=self.source.last_error,video=self.session.stats() if self.session else None)
                    last_report=now;last_source_frames=self.source.frames
        finally:
            self.close_control()
            if self.server:self.server.close()
            self.source.stop();self.log.emit('bridge_stopped')
