import cozmo
import cv2
import numpy as np

def slam(robot: cozmo.robot.Robot):

    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    runs = 1000
    i = 0
    num_features = 25
    quality = 0.1
    min_dist = 10
    while i < runs:
        latest_image = robot.world.latest_image
        if latest_image is not None:
            raw_img = latest_image.raw_image
            cv_img = np.array(raw_img).astype(np.uint8)
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            
            new_features = cv2.goodFeaturesToTrack(cv_img, num_features, quality, min_dist)
            
            keypoints = []
            for feature in new_features:
                x, y = feature.ravel()
                keypoint = cv2.KeyPoint(x, y, min_dist)
                keypoints.append(keypoint)
                cv2.circle(cv_img, (int(keypoint.pt[0]),int(keypoint.pt[1])), 2, 255, -1)
                # drawKeypoints doesn't seem to be working well for me
            cv2.imshow('main', cv_img)
            cv2.waitKey(1)
            i += 1
            old_features = new_features
    cv2.destroyAllWindows()
        
cozmo.robot.Robot.drive_off_charger_on_connect = False
cozmo.run_program(slam)