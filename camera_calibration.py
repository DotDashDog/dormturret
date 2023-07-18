#%%
import cv2 as cv
import numpy as np
import glob
import pickle
from turret_helper import *

image_x = 1920
image_y = 1080

center_x = image_x/2
center_y = image_y/2

board_x = 7
board_y = 7

efl_mm =1.6 #1.95

sensor_w_mm = 6.058
sensor_h_mm = 4.415

pixel_size_mm = 2.9 * 10**(-3)
efl_px = efl_mm/pixel_size_mm

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = []

for y in range(board_y):
    for x in range(board_x):
        objp.append([x, y, 0])
objp = 2 * np.array(objp, dtype=np.float32)
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

spec_mtx = np.array([
    [efl_px,   0,      image_x/2],
    [0,        efl_px, image_y/2],
    [0,        0,      1        ]
])

#calib_flags = cv.CALIB_USE_INTRINSIC_GUESS + cv.CALIB_FIX_FOCAL_LENGTH + cv.CALIB_FIX_ASPECT_RATIO + cv.CALIB_FIX_PRINCIPAL_POINT

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], spec_mtx, None) #flags=calib_flags

#* Characteristics obtained from calibration
fovx, fovy, focalLength, principalPoint, aspectRatio = cv.calibrationMatrixValues(mtx, (image_x, image_y), sensor_w_mm, sensor_h_mm)
fovdiag = np.rad2deg(2*np.arccos(np.cos(np.deg2rad(fovx)/2) * np.cos(np.deg2rad(fovy)/2)))


print("X FOV:", fovx, ", Y FOV:", fovy, ", Diagonal FOV:", fovdiag)

with open(camera_file, "wb") as f:
    pickle.dump({"mtx": mtx, "dist": dist}, f)


#%%

with open(camera_file, "rb") as f:
    camera_state = pickle.load(f)

#* Instead of undistorting the whole image, I can just undistort the points that I need

testpoints = np.array([[0, 0], [center_x, center_y], [image_x, image_y]], dtype=np.float32)

print(pixel_to_angle(testpoints, camera_state["mtx"], camera_state["dist"]))

#! What should W be? I think it should be 1, but idk

# inv_mtx = np.linalg.inv(mtx)

# out_pts = (inv_mtx @ homogeneous_points.T).T #! DON'T NEED THIS, INTERNET LIES!!! cv.undistortpoints already does this
    


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
