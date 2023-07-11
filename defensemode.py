#%%
import face_recognition
import os
import numpy as np
import pickle

whitelist_dir = "face_whitelist"
state_file = "whitelist_state.pkl"

#%%

def listdir_nohidden(path):
    #* ignores all files beginning with "."
    a = []
    for f in os.listdir(path):
        if not f.startswith("."):
            a.append(f)
    return a

def whitelistupdated(whitelist_dir, prev_state):
    return os.path.getmtime(whitelist_dir) != prev_state["mtime"]

def update_encodings(whitelist_dir):
    encodings = []
    for f in listdir_nohidden(whitelist_dir):
        img = face_recognition.load_image_file(os.path.join(whitelist_dir, f))
        enc = face_recognition.api.face_encodings(img)
        if len(enc) != 1:
            raise ValueError(f"More than one face found in {f} while generating encodings")
        encodings.append(enc[0])

    new_state = {"encodings" : encodings,
                 "mtime" : os.path.getmtime(whitelist_dir)}
    
    with open(state_file, "wb") as f:
        pickle.dump(new_state, f)

    return new_state

def latest_whitelist_encodings(whitelist_dir, state_file):
    if os.path.exists(state_file):
        with open(state_file, "rb") as f:
            state = pickle.load(f)
        
        if whitelistupdated(whitelist_dir, state):
            print("Updating encodings")
            state = update_encodings(whitelist_dir)

    else:
        print("Creating new encoding file")
        state = update_encodings(whitelist_dir)
    
    return state["encodings"]


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

def isWhitelisted(encoding, whitelist_encodings):
    distances = face_recognition.api.face_distance(whitelist_encodings, encoding)
    return np.any(distances < 0.6)

def faceCenter(box):
    return (box[1] + box[3])/2, (box[0] + box[2])/2 

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