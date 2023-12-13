import asyncio
import serial
import websockets
import numpy as np
from cryptography.fernet import Fernet
import subprocess

class Turret:
    def __init__(self):
        self.ser_opts = {
            'port': '/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0',
            'baudrate': 115200,
        }
        self.pos = np.array([None, None])
        self.ubx = np.array([90,   40])
        self.lbx = np.array([-90, -35])
        with open('/etc/shoot-key', 'r') as file:
            self.f = Fernet(eval(file.read()))
        self.relative = False
    def bound(self, point: np.array):
        pt = np.copy(point)
        umask = pt>self.ubx
        lmask = pt<self.lbx
        pt[umask] = self.ubx[umask]
        pt[lmask] = self.lbx[lmask]
        return pt
    def send(self, cmd: str):
        if cmd[-1] != '\n': cmd += '\n'
        cmd = cmd.upper().encode()

        while True:
            self.ser.write(cmd)
            if (res:=self.ser.readline().decode())[:2]=='OK': break
            else: print(res)
            sleep(0.05) # don't spam the nano
    def set_relative(self):
        self.send('G91')
    def set_absolute(self):
        self.send('G90')
    def point(self, point: np.array):
        self.pos = self.bound(point)
        self.send(f'G0 P{self.pos[0]} T{self.pos[1]}')

    async def handler(self, websocket):
        while True:
            message = await websocket.recv()
            message = self.f.decrypt(message)
            try:
                pt = eval(message)
            except:
                continue
            if not (isinstance(pt, tuple) and len(pt) == 3):
                pass
            else:
                self.point(np.array(pt[:2]))
                if pt[2]: self.send('M3')

    async def socket(self):
        async with websockets.serve(self.handler, "", 8001, ping_interval=None) as server:
            await server.serve_forever()
    def __enter__(self):
        subprocess.run(['startstream &>/dev/null &'])
        self.ser = serial.Serial(**self.ser_opts)
        self.set_relative()
        self.point(np.array([0,0]))
        return self
    def start(self):
        asyncio.run(self.socket(), debug=False)
    def __exit__(self, *args, **kwargs):
        self.send('M999')
        subprocess.run(['stopstream'])
        self.ser.close()


if __name__ == '__main__':
    from time import sleep
    with Turret() as t:
        t.start()
    print('exited')

