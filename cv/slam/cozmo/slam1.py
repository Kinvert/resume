import cozmo
import cv2
import numpy as np
import time

def good_features_method(img, min_dist):
    '''
    Using goodFeaturesToTrack to find keypoints
    '''
    num_features = 1000
    quality = 0.01
    new_features = cv2.goodFeaturesToTrack(img, num_features, quality, min_dist)
    keypoints = []
    for feature in new_features:
        x, y = feature.ravel()
        keypoint = cv2.KeyPoint(x, y, min_dist)
        keypoints.append(keypoint)
    return keypoints

def orb_method(img):
    '''
    Using ORB to find keypoints
    '''
    num_features = 1000
    orb = cv2.ORB_create(nfeatures=num_features)
    keypoints, descriptors = orb.detectAndCompute(img, None)
    return keypoints

def sift_method(img):
    '''
    Using SIFT to find keypoints
    LICENSING FEE IF COMMERCIAL
    '''
    sift = cv2.xfeatures2d.SIFT_create()
    keypoints, descriptors = sift.detectAndCompute(img, None)
    return keypoints
    
def surf_method(img):
    '''
    Using SURF to find keypoints
    LICENSING FEE IF COMMERCIAL
    THIS WILL NOT WORK WITHOUT CMAKE ETC
    '''
    surf = cv2.xfeatures2d.SURF_create()
    keypoints, descriptors = surf.detectAndCompute(img, None)
    return keypoints

def slam(robot: cozmo.robot.Robot):

    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    
    min_dist = 5
    runs = 500
    i = 0
    start_time = time.time()
    while i < runs:
        latest_image = robot.world.latest_image
        if latest_image is not None:
            raw_img = latest_image.raw_image # 320 X 240
            cv_img = np.array(raw_img).astype(np.uint8)
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            
            #keypoints = good_features_method(cv_img, min_dist) # 3.5s benchmark
            #keypoints = sift_method(cv_img) # 12.75s benchmark
            keypoints = orb_method(cv_img) # 2.94s benchmark
            
            #cv2.circle(cv_img, (int(keypoint.pt[0]),int(keypoint.pt[1])), 2, 255, -1) # cv2.drawKeypoints is slightly slower maybe 10% or so
            cv_img = cv2.drawKeypoints(cv_img, keypoints, None)
            cv2.imshow('main', cv_img)
            cv2.waitKey(1)
            i += 1
    print('TOOK {}s'.format(time.time() - start_time))
    cv2.destroyAllWindows()
        
cozmo.robot.Robot.drive_off_charger_on_connect = False
cozmo.run_program(slam)