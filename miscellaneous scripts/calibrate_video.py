import cv2
import numpy as np

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((7*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:7].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.


video = cv2.VideoCapture('vid.mp4')
success = True
count = 0
num_frames = int(video.get(int(cv2.CAP_PROP_FRAME_COUNT)))
while success:
    success, img = video.read()
    if success:
        img = cv2.resize(img, (640,480))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (9,7), None)
        count += 1
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners)
        print('\rRead frame: %3d of %3d' % (count, num_frames), end='')
    else:
        print('Failed to read image!')

print('\nChoosing 100 evenly spaced frames...')
objectpoints = []
imagepoints = []
num_found = len(imgpoints)
for i in range(0 , num_found, int(num_found/100)):
    objectpoints.append(objpoints[i])
    imagepoints.append(imgpoints[i])

print('Calibrating camera...')
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objectpoints, imagepoints, gray.shape[::-1], None, None)

print('Calibrated with RMSE: {:.2f}'.format(ret))
