import asyncio
import keyboard
import websockets
from time import sleep
from cryptography.fernet import Fernet
import subprocess
import os
import requests
print('initializing remote...')
res, err = subprocess.Popen(['curl', 'https://www.ocf.berkeley.edu/~reiddye/turret_ip_status.txt'], stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()
ip = res.decode().strip('\n')

# try: 
#     requests.get(f'http://{ip}:8080')
#     subprocess.Popen(['ssh', f'pi@{ip}', 'stopstream'])
# except: pass
# subprocess.Popen(['ssh', f'pi@{ip}', 'startstream'])
# subprocess.Popen(['ssh', f'pi@{ip}', 'python ~/dormturret/server.py &>/dev/null &'])
sleep(4)
print('remote initialized!')

with open('/etc/shoot-key', 'r') as file:
    f = Fernet(eval(file.read()))
async def main():
    async with websockets.connect(f'ws://{ip}:8001') as websocket:
        state = [0, 0]
        d = 3
        while True:
            fire = 0
            if keyboard.is_pressed("up"): state[1] +=  d
            if keyboard.is_pressed("down"): state[1] -= d
            if keyboard.is_pressed("left"): state[0] +=  d
            if keyboard.is_pressed("right"): state[0] -= d
            if keyboard.is_pressed("f"): fire          = 1

            # if state[0] != 0 or state[1] != 0 or fire != 0: 
            if state[0] >  90: state[0] =  90
            if state[0] < -90: state[0] = -90
            if state[1] >  40: state[1] =  40
            if state[1] < -35: state[1] = -35

            print(tuple(state+[fire]))
            await websocket.send(f.encrypt(str(tuple(state+[fire])).encode()))

            sleep(0.1)

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())

