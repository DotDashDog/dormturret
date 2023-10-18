import asyncio
import keyboard
import websockets
from time import sleep
from cryptography.fernet import Fernet

with open('/etc/shoot-key', 'r') as file:
    f = Fernet(eval(file.read()))
async def main():
    async with websockets.connect('ws://169.229.96.70:8001') as websocket:
        while True:
            state = [0, 0]
            fire = 0
            if keyboard.is_pressed("up"):
                state[1] = 5
            if keyboard.is_pressed("down"):
                state[1] = -5
            if keyboard.is_pressed("left"):
                state[0] = 5
            if keyboard.is_pressed("right"):
                state[0] = -5
            if keyboard.is_pressed("f"):
                fire = 1
            if state[0] != 0 or state[1] != 0 or fire != 0:
                print(tuple(state+[fire]))
                await websocket.send(f.encrypt(str(tuple(state+[fire])).encode()))
            if fire == 1:
                while keyboard.is_pressed("f"): sleep(0.1)

            sleep(0.1)

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())

