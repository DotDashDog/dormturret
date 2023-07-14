#%%
import cv2 as cv
import numpy as np
import glob

image_x = 1920
image_y = 1080

center_x = image_x/2
center_y = image_y/2

board_x = 7
board_y = 7

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

images = glob.glob('calibration_images/*')
if ".DS_Store" in images:
    images.remove(".DS_Store")

for fname in images:
    print(fname)
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (board_x,board_y), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        print("Found Chessboard")
        objpoints.append(objp)
    
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv.drawChessboardCorners(img, (board_x,board_y), corners2, ret)
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

mtx = np.array([])

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

#* Characteristics obtained from calibration
fovx, fovy, focalLength, principalPoint, aspectRatio = cv.calibrationMatrixValues(mtx, (image_x, image_y), 6.058, 4.415)
fovdiag = np.sqrt(fovx**2, fovy**2)
#? There's an improved camera matrix I can calculate. IDK if I need it. See tutorial

#%%

#* Instead of undistorting the whole image, I can just undistort the points that I need

testpoints = np.array([[[0, 0]], [[center_x, center_y]], [[image_x, image_y]]], dtype=np.float32)
xy_undistorted = np.squeeze(cv.undistortPoints(testpoints, mtx, dist))

#! What should W be? I think it should be 1, but idk
homogeneous_points = np.append(xy_undistorted, 1*np.ones((xy_undistorted.shape[0], 1)), axis=1)

# inv_mtx = np.linalg.inv(mtx)

# out_pts = (inv_mtx @ homogeneous_points.T).T #! DON'T NEED THIS, INTERNET LIES!!! cv.undistortpoints already does this
out_pts = homogeneous_points

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
    


# img = cv.imread('calibration_images/CalibrationPhoto1.jpg')
# h, w = img.shape[:2]
# newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 0, (w,h))

# # undistort
# dst = cv.undistort(img, mtx, dist, None, newcameramtx)
# # crop the image
# x, y, w, h = roi
# dst = dst[y:y+h, x:x+w]
# cv.imwrite('calibresult.png', dst)

# %%
