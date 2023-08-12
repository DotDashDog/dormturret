#%%
import face_recognition
import os
import numpy as np
import pickle
from turret_helper import *


#%%
#* Load whitelist encodings, updating (with a few runs of the model) if necessary
whitelist_encs = latest_whitelist_encodings(whitelist_dir, state_file)

#* Load camera characteristics
with open(camera_file, "rb") as f:
    camera_state = pickle.load(f)

bow = Arduino(9600)

#%%

while True:
    #* Load the image to evaluate
    #fov = face_recognition.load_image_file(os.path.join("test_images", "bidenpelosi1.jpeg"))
    
    
    locations = face_recognition.api.face_locations(
        fov,
        number_of_times_to_upsample=1, #* 1 is default, higher finds smaller faces. Tuning needed
        model="hog", #* Options: "hog" (faster on cpu, less accurate), "cnn" (faster on gpu, more accurate)
    )

    #* Location is a tuple in the format (top, right, bottom, left) (bounding box)

    face_encodings = face_recognition.api.face_encodings(fov, locations)
    print(f"{len(face_encodings)} faces detected.")


    targets = []
    for i, face in enumerate(face_encodings):
        box = locations[i]
        if not isWhitelisted(face, whitelist_encs):
            print(f"Target found. Face center at {faceCenter(box)}")
            targets.append(box)

    box = targets[0]

    faceLoc = faceCenter(box)

    targetLoc = faceLoc + np.array([0, 2.5*height(box)])

    # import matplotlib.pyplot as plt
    # import matplotlib.patches as patches

    # ax = plt.gca()

    # ax.imshow(fov)

    # #* Add box around face
    # rect = patches.Rectangle((box[3], box[2]), box[1]-box[3], box[0]-box[2], linewidth=1, edgecolor='r', facecolor='none')
    # ax.plot(*targetLoc, marker="v", color="red")
    # ax.add_patch(rect)
    # print(f"Target found. Face center at {faceCenter(box)}")

    # plt.show()

    target_angle = pixel_to_angle(np.array([targetLoc]), camera_state["mtx"], camera_state["dist"])[0]
    bow.turn(target_angle)
    bow.fire()


# %%
