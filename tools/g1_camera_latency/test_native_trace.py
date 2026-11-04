import json
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import unittest

class NativeTraceTest(unittest.TestCase):
    def test_disabled_and_enabled(self):
        root = Path(__file__).resolve().parents[2]
        inc = root / "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include"
        with tempfile.TemporaryDirectory() as td:
            src = Path(td) / "test.cpp"
            exe = Path(td) / "test"
            src.write_text('#include "latency_trace.hpp"\nint main(){auto n=sonic_latency::tracer.cycle();sonic_latency::tracer.emit("cycle",n,"sonic_decode_start");return 0;}')
            subprocess.run(["g++", "-std=c++20", "-I", str(inc), str(src), "-o", str(exe)], check=True)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.bind(("127.0.0.1", 0))
                sock.settimeout(0.15)
                env = dict(os.environ, LATENCY_TRACE_HOST="229", LATENCY_TRACE_EVERY="1")
                env.pop("LATENCY_TRACE_UDP", None)
                subprocess.run([str(exe)], env=env, check=True)
                with self.assertRaises(socket.timeout):
                    sock.recv(8192)
                env["LATENCY_TRACE_UDP"] = "127.0.0.1:%d" % sock.getsockname()[1]
                subprocess.run([str(exe)], env=env, check=True)
                event = json.loads(sock.recv(8192))
                self.assertEqual(event["stage"], "sonic_decode_start")
                self.assertEqual(event["clock"]["host"], "229")
                self.assertFalse(event["meta"]["motor_execution_confirmed"])

if __name__ == "__main__":
    unittest.main()
