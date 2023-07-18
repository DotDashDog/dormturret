#%%
import face_recognition
import os
import numpy as np
import pickle
from turret_helper import *

#%%
#* Load whitelist encodings, updating (with a few runs of the model) if necessary
whitelist_encs = latest_whitelist_encodings(whitelist_dir, state_file)

#%%
#* Load the image to evaluate
fov = face_recognition.load_image_file(os.path.join("test_images", "bidenpelosi1.jpeg"))

locations = face_recognition.api.face_locations(
    fov,
    number_of_times_to_upsample=3, #* 1 is default, higher finds smaller faces. Tuning needed
    model="hog", #* Options: "hog" (faster on cpu, less accurate), "cnn" (faster on gpu, more accurate)
)

#* Location is a tuple in the format (top, right, bottom, left) (bounding box)

face_encodings = face_recognition.api.face_encodings(fov, locations)
print(f"{len(face_encodings)} faces detected.")


# %%

import matplotlib.pyplot as plt
import matplotlib.patches as patches

ax = plt.gca()

ax.imshow(fov)

for i, face in enumerate(face_encodings):
    box = locations[i]
    if not isWhitelisted(face, whitelist_encs):
        #* Add box around face
        rect = patches.Rectangle((box[3], box[2]), box[1]-box[3], box[0]-box[2], linewidth=1, edgecolor='r', facecolor='none')
        print(f"Target found. Face center at {faceCenter(box)}")
    else:
        rect = patches.Rectangle((box[3], box[2]), box[1]-box[3], box[0]-box[2], linewidth=1, edgecolor='g', facecolor='none')
        print(f"Non-Target found. Face center at {faceCenter(box)}")
    ax.add_patch(rect)

plt.show()
# %%