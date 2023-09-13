#%%
import face_recognition
import os
import numpy as np
import pickle
from turret_helper import *
import cv2 as cv


#%%
#* Load whitelist encodings, updating (with a few runs of the model) if necessary
whitelist_encs = latest_whitelist_encodings(whitelist_dir, state_file)

#* Load camera characteristics
with open(camera_file, "rb") as f:
    camera_state = pickle.load(f)

bow = Arduino(9600)
cam = cv.VideoCapture(0)
#%%

while True:
    #* Load the image to evaluate
    #fov = face_recognition.load_image_file(os.path.join("test_images", "bidenpelosi1.jpeg"))
    return_value, image = cam.read()
    
    
    locations = face_recognition.api.face_locations(
        image,
        number_of_times_to_upsample=1, #* 1 is default, higher finds smaller faces. Tuning needed
        model="hog", #* Options: "hog" (faster on cpu, less accurate), "cnn" (faster on gpu, more accurate)
    )

    #* Location is a tuple in the format (top, right, bottom, left) (bounding box)

    face_encodings = face_recognition.api.face_encodings(image, locations)
    print("{} faces detected.".format(len(face_encodings)))


    targets = []
    for i, face in enumerate(face_encodings):
        box = locations[i]
        if not isWhitelisted(face, whitelist_encs):
            print("Target found. Face center at {}".format(faceCenter(box)))
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
    # print("Target found. Face center at {}".format(faceCenter(box))

    # plt.show()

    target_angle = pixel_to_angle(np.array([targetLoc]), camera_state["mtx"], camera_state["dist"])[0]
    bow.turn(target_angle)
    bow.fire()


# %%
