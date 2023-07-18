import face_recognition
import os
import numpy as np
import pickle
import cv2 as cv

whitelist_dir = "face_whitelist"
state_file = "whitelist_state.pkl"
camera_file = "camera_characteristics.pkl"


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

def isWhitelisted(encoding, whitelist_encodings):
    distances = face_recognition.api.face_distance(whitelist_encodings, encoding)
    return np.any(distances < 0.6)

def faceCenter(box):
    return np.array([(box[1] + box[3])/2, (box[0] + box[2])/2])

#! These two may be negative
def height(box):
    return box[2] - box[0]

def width(box):
    return box[1] - box[3]

#* Camera, pixel, and angle utility functions
def unit_vecs(vecs):
    return vecs/np.concatenate(3*[[np.linalg.norm(vecs, axis=1)]], axis=0).T

def alt_az(vecs):
    rays = unit_vecs(vecs)
    out = []
    for i in range(len(rays)):
        x = rays[i, 0]
        y = rays[i, 1]
        z = rays[i, 2]

        out.append([np.arctan(x/z), np.arctan(y/z)])

    return np.array(out)

def pixel_to_angle(px_loc, mtx, dist):
    px_loc = np.reshape(px_loc, [px_loc.shape[0], 1, 2])

    xy_undistorted = np.reshape(cv.undistortPoints(px_loc, mtx, dist), [px_loc.shape[0], 2])
    homogeneous_points = np.append(xy_undistorted, 1*np.ones((xy_undistorted.shape[0], 1)), axis=1)
    out_pts = homogeneous_points

    return np.rad2deg(alt_az(out_pts))

class Arduino:
    def __init__(self, serial):
        self.direction = np.array([90, 90])
        self.minAngle = np.array([0, 50])
        self.maxAngle = np.array([180, 130])
        #* RasPi communication code here
        pass

    def point(self, newDirection):
        self.direction = newDirection
        #* RasPi communication code here
    
    def fire(self):
        #* RasPi communication code here
        pass