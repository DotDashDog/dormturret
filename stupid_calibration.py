import numpy as np
import pickle
# from turret_helper import *
import cv2 as cv
from threadedCamera import threadedCamera
import asyncio
from time import sleep
import subprocess
import requests
from cryptography.fernet import Fernet
import websockets

print('initializing remote...')
res, err = subprocess.Popen(['curl', 'https://www.ocf.berkeley.edu/~reiddye/turret_ip_status.txt'], stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()
ip = res.decode().strip('\n')
# subprocess.Popen(['ssh', f'pi@{ip}', 'pkill python'])
# try: 
#     requests.get(f'http://{ip}:8080')
#     subprocess.Popen(['ssh', f'pi@{ip}', 'stopstream'])
# except: pass
# subprocess.Popen(['ssh', f'pi@{ip}', 'python ~/dormturret/server.py &>/dev/null &'])
# sleep(5)
# print('remote initialized!')

cam = threadedCamera(cam=f'http://{ip}:8080/?action=stream')
# cam = cv.VideoCapture(f'http://{ip}:8080/?action=stream')
cam.cam.set(cv.CAP_PROP_BUFFERSIZE, 3)
sleep(1)

with open('/etc/shoot-key', 'r') as file:
    f = Fernet(eval(file.read()))
async def main():
    async with websockets.connect(f'ws://{ip}:8001') as websocket:
        for pan in range(-70, 75, 5):
            for tilt in range(-40, 40, 5):
                await websocket.send(f.encrypt(str((pan, tilt, 0)).encode()))
                sleep(0.75)
                ret, image = cam.read()
                img_rgb = cv.cvtColor(image, cv.COLOR_BGR2HSV)
                coords = np.unravel_index(np.argmin(np.linalg.norm(img_rgb-np.array([337,57,40]), axis=2)), image.shape[:2])
                greydist = img_rgb@np.array([337,57,40])
                cv.imshow('image', (greydist/np.max(greydist))*255)
                # cv.imshow(f"image", cv.circle(image, coords, 10, (0, 255, 0), 2))
                cv.waitKey(1)
                cv.imwrite(f'images/(({pan},{tilt}),{coords}).jpg', image)
                # input('ok?')
            await websocket.send(f.encrypt(str((pan, -40, 0)).encode()))
            sleep(0.75)
            
if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
