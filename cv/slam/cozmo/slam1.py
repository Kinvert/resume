import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps, Pose, radians
import cv2
import numpy as np
import skimage
from skimage.measure import ransac
from skimage.transform import FundamentalMatrixTransform, EssentialMatrixTransform
import time



def matplotlib_point_plotting():
    '''
    Looks like I can use Matplotlib to plot points once I can take camera imgages and turn that in to 3d data
    '''
    from mpl_toolkits import mplot3d
    import matplotlib.pyplot as plt
     
    fig = plt.figure()
     
    # syntax for 3-D projection
    ax = plt.axes(projection ='3d')
     
    # defining all 3 axes
    x = np.random.randn(28, 28)
    y = np.random.randn(28, 28)
    z = np.random.randn(28, 28)
     
    # plotting
    #ax.plot3D(x, y, z, 'green')
    ax.scatter(x, y, z)
    ax.set_title('3D line plot geeks for geeks')
    plt.show()


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

def tut_optical_flow(img1, img2):
    '''
    https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html
    '''
    mask = np.zeros((240,320))
    feature_params = dict( maxCorners = 100,
               qualityLevel = 0.3,
               minDistance = 7,
               blockSize = 7 )
    lk_params = dict( winSize  = (15, 15),
          maxLevel = 2,
          criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
    
    p0 = cv2.goodFeaturesToTrack(img1, mask=None, **feature_params)
    p1, st, err = cv2.calcOpticalFlowPyrLK(img1, img2, p0, None, **lk_params)
    # Select good points
    if p1 is not None:
        good_new = p1[st==1]
        good_old = p0[st==1]
    # draw the tracks
    for i, (new, old) in enumerate(zip(good_new, good_old)):
        a, b = new.ravel()
        c, d = old.ravel()
        mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), 255, 2)
        #img2 = cv2.circle(img2, (int(a), int(b)), 5, 255, -1)
    #img = img2 + mask
    return mask

def ransac():
    '''
    https://scikit-image.org/docs/stable/auto_examples/transform/plot_ransac.html
    RANSAC https://www.youtube.com/watch?v=9D5rrtCC_E0&ab_channel=CyrillStachniss
    https://www.youtube.com/watch?v=Cu1f6vpEilg&ab_channel=CyrillStachniss
    https://www.youtube.com/watch?v=EkYXjmiolBg&ab_channel=FirstPrinciplesofComputerVision
    https://www.youtube.com/watch?v=QDiheqzZv4s&ab_channel=MATLAB
    '''
    # model, inliers = ransac(data, LineModelND, min_samples, residual_threshold, max_trials)
    pass
    
def fund_mat():
    '''
    https://scikit-image.org/docs/dev/auto_examples/transform/plot_fundamental_matrix.html
    '''
    pass

def draw():
    '''
    OpenGL
    '''
#    from OpenGL.GL import *
#    from OpenGL.GLU import *
#    from OpenGL.GLUT import *
#    import sys
    # OpenGL
#    glutInit(sys.argv)
#    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB)
#    glutInitWindowSize(500,500)
#    glutInitWindowPosition(100,100)
#    glutCreateWindow(b'Cube')
#    glutDisplayFunc(draw)
#    glutMainLoop()
    pass
#    xrot = 45
#    yrot = 45
#    glClear(GL_COLOR_BUFFER_BIT)
#    glRotatef(xrot, 1, 0, 0)
#    glRotatef(yrot, 0, 1, 0)
#    glutWireCube(0.7)
#    glFlush()

def slam(robot: cozmo.robot.Robot):
    '''
    SLAM
    
    https://scikit-image.org/docs/dev/auto_examples/transform/plot_fundamental_matrix.html
    ransac ret, ret, fundamentalmatrixtransform, min samples, residual_thresh=1, max_trials = 100)
    Extended Kalman Filter?
    ICP https://nbviewer.org/github/niosus/notebooks/blob/master/icp.ipynb This guy has a lot of stuff https://nbviewer.org/github/niosus/
    
    '''
    
    # Initialize Robot
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    robot.set_head_angle(degrees(0.00)).wait_for_completed()
    robot.set_lift_height(0.1).wait_for_completed()
    
    # Set some params
    min_dist = 5
    runs = 500
    start_time = time.time()
    des1 = None
    img1 = None
    while True:
        latest_image = robot.world.latest_image
        if latest_image is not None:
            robot.drive_straight(distance_mm(5), speed_mmps(200)).wait_for_completed()
            time.sleep(0.25)
            img2 = latest_image.raw_image # 320 X 240
            img2 = np.array(img2).astype(np.uint8)
            img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
            if img1 is None:
                img1 = img2.copy()
            
            orb = cv2.ORB_create()
            kp2, des2 = orb.detectAndCompute(img2, None)
            mask = tut_optical_flow(img1, img2)
            
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            if des1 is not None:
                matches = bf.match(des1, des2)
                matches = sorted(matches, key=lambda x:x.distance)
                for match in matches[:10]:
                    print(match.distance)
        
                result = cv2.drawMatches(img1, kp1, img2, kp2, matches[:50], None, flags=2)

            #cv2.circle(cv_img, (int(keypoint.pt[0]),int(keypoint.pt[1])), 2, 255, -1) # cv2.drawKeypoints is slightly slower maybe 10% or so
            #cv_img = cv2.drawKeypoints(cv_img, keypoints, None)
            
            # SFM Observation Matrix
            # https://youtu.be/oIvg7sbJRIA?t=114
            # Apparently important that is has a very low rank
            
            kp1 = kp2
            img1 = img2.copy()
            des1 = des2
            
            try:
                cv2.imshow('main', result)
                cv2.imshow('frame', mask)
            except:
                pass
            c = cv2.waitKey(0)
            if c == ord('q') or c == 27:
                break
            cv2.destroyAllWindows()
    cv2.destroyAllWindows()
        
cozmo.robot.Robot.drive_off_charger_on_connect = False
cozmo.run_program(slam)