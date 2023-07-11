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
whitelist_encs = latest_whitelist_encodings(whitelist_dir, state_file)

#%%

image1 = face_recognition.load_image_file(os.path.join("test_images", "biden1.jpg"))
enc1 = face_recognition.api.face_encodings(image1)[0]


# %%

face_recognition.api.face_distance(whitelist_encs, enc1)

# %%
