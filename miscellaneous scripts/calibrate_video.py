import cv2
import numpy as np
import scipy.misc

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((7*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:7].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

width,height = 1280,960

# Collect an image that measures grid coverage of the field of view.
coverage_scale=0.25
coverage = np.zeros((int(height*coverage_scale),int(width*coverage_scale)),np.float32)

video = cv2.VideoCapture('vid.mp4')
success = True
count = 0
num_frames = int(video.get(int(cv2.CAP_PROP_FRAME_COUNT)))
while success:
    success, img = video.read()
    if success:
        img = cv2.resize(img, (width,height))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (9,7), None)
        count += 1
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners)            
            for c in corners:
                for cc in c:
                    # note: change to +=1 for a grayscale result
                    coverage[int(cc[1]*coverage_scale),int(cc[0]*coverage_scale)]=1
        print('Read frame: %3d of %3d (%d) Corners: %3d\r' % (count, num_frames, ret, len(corners if corners is not None else [])),end='')
    else:
        print('\nFailed to read image!')

print('Found corners in {} of {} frames'.format(len(imgpoints), num_frames))

print('Saving coverage map')
scipy.misc.imsave('coverage.jpg', coverage)

num_to_calibrate = 100
print('\nChoosing {} evenly spaced frames...'.format(num_to_calibrate))
objectpoints = []
imagepoints = []
num_found = len(imgpoints)
for i in range(0 , num_found, max(1,int(num_found/num_to_calibrate))):
    objectpoints.append(objpoints[i])
    imagepoints.append(imgpoints[i])

print('Calibrating camera...')
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objectpoints, imagepoints, gray.shape[::-1], None, None)

print('Calibrated with RMSE: {:.2f}'.format(ret))
