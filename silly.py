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
res = res.decode()

print(*['error: '+line for line in err.decode().split('\n')], sep='\n')
ip = res.split('inet ')[-1].split(' ')[0]

try: 
    requests.get(f'http://{ip}:8080')
    subprocess.Popen(['ssh', f'pi@{ip}', 'stopstream'])
except: pass
subprocess.Popen(['ssh', f'pi@{ip}', 'python ~/dormturret/server.py &>/dev/null &'])
sleep(2)
print('remote initialized!')

with open('/etc/shoot-key', 'r') as file:
    f = Fernet(eval(file.read()))
async def main():
    async with websockets.connect(f'ws://{ip}:8001') as websocket:
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

