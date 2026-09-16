import importlib.util
import json
import queue
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('cartpole',ROOT/'cartpole.py')
console=importlib.util.module_from_spec(spec)
spec.loader.exec_module(console)

class SerialStub:
    def __init__(self):
        self.lines=queue.Queue()
        self.commands=[]
        self.closed=False
    def write(self, data): self.commands.append(data)
    def readline(self):
        try: return self.lines.get(timeout=0.01)
        except queue.Empty: return b''
    def close(self): self.closed=True

class ConsoleEventsTest(unittest.TestCase):
    def test_commands_parameters_faults_and_telemetry_survive_close(self):
        serial=SerialStub()
        with tempfile.TemporaryDirectory() as folder, patch.object(console.serial,'Serial',return_value=serial):
            link=console.Link('fake-port',logdir=folder)
            link.send('params')
            for line in [b'# firmware swingup-175mm-v3\n',b'= leff 0.16600\n',
                         b'! no pendulum response to cart command -> FAULT\n',
                         b'T 100 4 3.14 0 0.02 0 0 -1 0 0 90 500\n']:
                serial.lines.put(line)
            deadline=time.monotonic()+2
            while link.rx_count<1 and time.monotonic()<deadline:time.sleep(.01)
            self.assertEqual(link.rx_count,1)
            link.close()
            records=[json.loads(line) for line in Path(link.eventpath).read_text().splitlines()]
            text=[r['text'] for r in records]
            self.assertIn('params',text)
            self.assertIn('= leff 0.16600',text)
            self.assertIn('! no pendulum response to cart command -> FAULT',text)
            self.assertIn('stop',text)
            self.assertIn('100.0,4.0',Path(link.logpath).read_text())
            self.assertTrue(serial.closed)
            self.assertFalse(link.thread.is_alive())

if __name__=='__main__':unittest.main()
