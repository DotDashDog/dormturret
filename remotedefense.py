#%%
import face_recognition
import os
import numpy as np
import pickle
from turret_helper import *
import cv2 as cv
from threadedCamera import threadedCamera
import asyncio
from time import sleep
import subprocess
#%%
print('initializing remote...')
res = subprocess.Popen(['curl', 'https://www.ocf.berkeley.edu/~reiddye/turret_ip_status.txt'], stdout=subprocess.PIPE).communicate()[0].decode()
ip = res.split('inet ')[-1].split(' ')[0]
os.system(f'ssh pi@{ip} stopstream')
sleep(1)
os.system(f'ssh pi@{ip} "python ~/dormturret/server.py &>/dev/null &"')
sleep(2)
print('remote initialized!')
#* Load whitelist encodings, updating (with a few runs of the model) if necessary
whitelist_encs = latest_whitelist_encodings(whitelist_dir, state_file)

#* Load camera characteristics
with open(camera_file, "rb") as f:
    camera_state = pickle.load(f)

# bow = Arduino(115200)
# cam = cv.VideoCapture('http://169.229.96.70:8080/?action=stream')
# capture.set(cv.CAP_PROP_FOURCC ,cv2.VideoWriter_fourcc('M', 'J', 'P', 'G') )
cam = threadedCamera()

#* Set to 1080p
# cam.cam.set(3, 1920)
# cam.cam.set(4, 1080)
# cam.cam.set(5, 5)
cam.cam.set(cv.CAP_PROP_BUFFERSIZE, 3)

with open('/etc/shoot-key', 'r') as file:
    f = Fernet(eval(file.read()))
#%%
async def main():
    async with websockets.connect(f'ws://{ip}:8001') as websocket:
        while True:
            sleep(1/10)
            #* Load the image to evaluate
            #fov = face_recognition.load_image_file(os.path.join("test_images", "bidenpelosi1.jpeg"))
            return_value, image = cam.read()
            
            
            
            locations = face_recognition.api.face_locations(
                image,
                number_of_times_to_upsample=1, #* 1 is default, higher finds smaller faces. Tuning needed
                model="cnn", #* Options: "hog" (faster on cpu, less accurate), "cnn" (faster on gpu, more accurate)
            )

            #* Location is a tuple in the format (top, right, bottom, left) (bounding box)

            face_encodings = face_recognition.api.face_encodings(image, locations)
            # print("{} faces detected.".format(len(face_encodings)))

            # skip = True
            targets = []
            for i, face in enumerate(face_encodings):
                box = locations[i]
                if not isWhitelisted(face, whitelist_encs):
                    print("Target found. Face center at {}".format(faceCenter(box)))
                else:
                    # skip = False
                    print("Non-target found. Face center at {}".format(faceCenter(box)))
                    targets.append(box)
                    pass

            if len(targets) == 0:
                cv.imshow('img', image)
                cv.waitKey(1)
                continue

            box = targets[0]

            faceLoc = faceCenter(box)

            targetLoc = faceLoc + np.array([0, 2.5*np.abs(box[0]-box[2])])

            # import matplotlib.pyplot as plt
            # import matplotlib.patches as patches

            # ax = plt.gca()

            # ax.imshow(fov)

            # #* Add box around face
            # rect = patches.Rectangle((box[3], box[2]), box[1]-box[3], box[0]-box[2], linewidth=1, edgecolor='r', facecolor='none')
            # ax.plot(*targetLoc, marker="v", color="red")
            # ax.add_patch(rect)
            # print("Target found. Face center at {}".format(faceCenter(box))

            # plt.show()

            # target_angle = pixel_to_angle(np.array([targetLoc]), camera_state["mtx"], camera_state["dist"])[0]
            cv.imshow('img', cv.rectangle(cv.rectangle(image, (box[3], box[2]),(box[1], box[0]), color=(0, 255, 0), thickness=2), (0, 0), (int(faceLoc[0]), int(faceLoc[1])), color=(255, 0, 0), thickness=2))
            cv.waitKey(1)
            im_size = np.array(image.shape[:2])[::-1]
            pct = faceLoc/im_size
            print(faceLoc, im_size, pct)
            angular_size = np.array([150, 130])
            angular_size = np.array([-60, -60])
            pct = (pct-np.array([0.5, 0.5]))
            angles = pct*angular_size
            print(pct)
            await websocket.send(f.encrypt(str((angles[0], angles[1], 0)).encode()))
            target_angle = pixel_to_angle_stupid(np.array(targetLoc), im_size, angular_size/im_size)
            # print(f"Turning {target_angle}")
            # bow.point(np.array([0, 0]))
            # bow.turn(target_angle) 
            # bow.fire()
            # sleep(1/40)
            

asyncio.run(main())
        # %%
