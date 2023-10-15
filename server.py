import asyncio
import serial
import websockets
import numpy as np
from cryptography.fernet import Fernet
import subprocess

class Turret:
    def __init__(self):
        self.ser_opts = {
            'port': '/dev/cu.usbserial-14330',
            'baudrade': 115200,
        }
        self.pos = np.array([None, None])
        self.ubx = np.array([90,   40])
        self.lbx = np.array([-90, -35])
        with open('/etc/shoot-key', 'r') as file:
            self.f = Fernet(eval(file.read()))
    def bound(self, point: np.array):
        pt = np.copy(point)
        umask = pt>self.ubx
        lmask = pt<self.lbx
        pt[umask] = self.ubx[umask]
        pt[lmask] = self.lbx[lmask]
        return pt
    
    def point(self, point: np.array):
        self.pos = self.bound(point)
        self.ser.write(f"G0 P{self.pos[0]} T{self.pos[1]}\n".encode())

    async def handle(self, websocket):
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
                if pt[2]: self.ser.write(b"M3\n")
                
    async def socket(self):
        async with websockets.serve(self.handler, "", 8001):
            await self.fut
    def __enter__(self):
        self.stream = subprocess.Popen(["startstream"])
        self.ser = serial.Serial(**self.ser_opts)
        self.ser.write(b"G90\n")
        self.fut = asyncio.Future()
        asyncio.run(self.socket)
    def __exit__(self):
        self.stream.terminate()
        self.ser.write(b"M999\n")
        self.ser.close()
        self.fut.set_exception(BaseException)

if __name__ == '__main__':
    t = Turret()

