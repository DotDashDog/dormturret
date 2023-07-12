#%%
import cv2 as cv
import numpy as np
import glob

board_x = 7
board_y = 6

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = []

for y in range(board_y):
    for x in range(board_x):
        objp.append([x, y, 0])
objp = np.array(objp, dtype=np.float32)
#%%

objpoints = []
imgpoints = []

images = glob.glob('calibration_images/*.jpg')

for fname in images:
    print(fname)
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (7,6), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
    
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv.drawChessboardCorners(img, (7,6), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)

objpoints = np.array(objpoints)
imgpoints = np.array(imgpoints)

cv.destroyAllWindows()

#%%

#* ret : ???
#* mtx : camera matrix
#* dist : distortion coefficients
#* rvecs : rotation vectors
#* tvecs : translation vectors

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

#? There's an improved camera matrix I can calculate. IDK if I need it. See tutorial

#%%

#* Instead of undistorting the whole image, I can just undistort the points that I need

n_points = 10
testpoints = np.zeros((n_points, 1, 2), dtype=np.float32)
xy_undistorted = np.squeeze(cv.undistortPoints(testpoints, mtx, dist))

# %%
#! No clue if the W dimension should be 1 or not. Reddit says it should be 1
homogeneous_points = np.append(xy_undistorted, np.ones((xy_undistorted.shape[0], 1)), axis=1)

inv_mtx = np.linalg.inv(mtx)

out_pts = (inv_mtx @ homogeneous_points.T).T


# %%

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

alt_az(out_pts)
    

# %%
