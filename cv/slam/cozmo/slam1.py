import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps, Pose, radians
import cv2
import matplotlib.pyplot as plt
import numpy as np
import skimage
from skimage import data
from skimage.color import rgb2gray
from skimage.feature import match_descriptors, ORB, plot_matches
from skimage.measure import ransac
from skimage.transform import FundamentalMatrixTransform, EssentialMatrixTransform
import time



def tut_optical_flow(imgold, imgnew):
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
    
    p0 = cv2.goodFeaturesToTrack(imgold, mask=None, **feature_params)
    p1, st, err = cv2.calcOpticalFlowPyrLK(imgold, imgnew, p0, None, **lk_params)
    # Select good points
    if p1 is not None:
        good_new = p1[st==1]
        good_old = p0[st==1]
    # draw the tracks
    for i, (new, old) in enumerate(zip(good_new, good_old)):
        a, b = new.ravel()
        c, d = old.ravel()
        mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), 255, 2)
        #imgnew = cv2.circle(imgnew, (int(a), int(b)), 5, 255, -1)
    #img = imgnew + mask
    return mask

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

def denormalize(Kinv, pt):
    res = np.dot(Kinv, np.array([pt[0], pt[1], 1]))
    return res[0], res[1]

def normalize(Kinv, pt):
    res = np.dot(Kinv, np.array([pt[0], pt[1], 1]).T).T
    return res[0], res[1]

def normalize2(x, y):
    return x/319, y/239

def test_E(yns, yos, E):
    '''
    https://en.wikipedia.org/wiki/Eight-point_algorithm
    yn is y_new
    yo is y_old
    NOT WORKING YET PROBABLY NEEDS TO NORMALIZE THE DATA PROPERLY
    '''
    error = 0
    runs = 0
    for yn, yo in zip(yns, yos):
        yn = yn[0] # Get it out of extra square brackets
        yo = yo[0] # Get it out of extra square brackets
        yp = np.array([[yn[0]],
                       [yn[1]],
                       [1]])
        y = np.array([[yo[0]],
                       [yo[1]],
                       [1]])
        '''
        Old Methods
        e = yp[0]*y[0]*E[0][0] + yp[0]*y[1]*E[0][1] + yp[0]*E[0][2] + \
            yp[1]*y[0]*E[1][0] + yp[1]*y[1]*E[1][1] + yp[1]*E[1][2] + \
            yp[2]*y[0]*E[2][0] + yp[2]*y[1]*E[2][1] + yp[2]*E[2][2]
        e2 = np.dot(np.dot(yp.T, E),y)
        runs += 1
        error += e + e2
        '''
        kron_error = np.dot(np.kron(yp, y).T, np.ndarray.flatten(E)) # https://www.youtube.com/watch?v=zX5NeY-GTO0&ab_channel=CyrillStachniss
        error += kron_error
        runs += 1
    try:
        error = error / runs
    except:
        error = -1
    return error
        
def test_F(xns, xos, F):
    '''
    https://en.wikipedia.org/wiki/Eight-point_algorithm
    xn is x_new
    xo is x_old
    Look up Kronecker Product
    '''
    error = 0
    runs = 0
    for xn, xo in zip(xns, xos):
        xn = normalize2(xn[0], xn[1])
        xo = normalize2(xo[0], xo[1])
        xp = np.array([[xn[0]],
                       [xn[1]],
                       [1]])
        x = np.array([[xo[0]],
                       [xo[1]],
                       [1]])
        '''
        Old Methods
        f = xp[0]*x[0]*F[0][0] + xp[0]*x[1]*F[0][1] + xp[0]*F[0][2] + \
            xp[1]*x[0]*F[1][0] + xp[1]*x[1]*F[1][1] + xp[1]*F[1][2] + \
            xp[2]*x[0]*F[2][0] + xp[2]*x[1]*F[2][1] + xp[2]*F[2][2]
        f2 = np.dot(np.dot(xp.T, F),x)
        runs += 1
        error += f + f2
        '''
        kron_error = np.dot(np.kron(xp, x).T, np.ndarray.flatten(F)) # https://www.youtube.com/watch?v=zX5NeY-GTO0&ab_channel=CyrillStachniss
        error += kron_error
        runs += 1
    try:
        error = error / runs
    except:
        error = -1
    return error

def linear_LS_triangulation(u1, P1, u2, P2):
    """
    https://github.com/Eliasvan/Multiple-Quadrotor-SLAM/blob/master/Work/python_libs/triangulation.py
    Linear Least Squares based triangulation.
    Relative speed: 0.1
    
    (u1, P1) is the reference pair containing normalized image coordinates (x, y) and the corresponding camera matrix.
    (u2, P2) is the second pair.
    
    u1 and u2 are matrices: amount of points equals #rows and should be equal for u1 and u2.
    
    The status-vector will be True for all points.
    """
    linear_LS_triangulation_C = -np.eye(2, 3)
    A = np.zeros((4, 3))
    b = np.zeros((4, 1))
    
    # Create array of triangulated points
    x = np.zeros((3, len(u1)))
    
    # Initialize C matrices
    C1 = np.array(linear_LS_triangulation_C)
    C2 = np.array(linear_LS_triangulation_C)
    
    for i in range(len(u1)):
        # Derivation of matrices A and b:
        # for each camera following equations hold in case of perfect point matches:
        #     u.x * (P[2,:] * x)     =     P[0,:] * x
        #     u.y * (P[2,:] * x)     =     P[1,:] * x
        # and imposing the constraint:
        #     x = [x.x, x.y, x.z, 1]^T
        # yields:
        #     (u.x * P[2, 0:3] - P[0, 0:3]) * [x.x, x.y, x.z]^T     +     (u.x * P[2, 3] - P[0, 3]) * 1     =     0
        #     (u.y * P[2, 0:3] - P[1, 0:3]) * [x.x, x.y, x.z]^T     +     (u.y * P[2, 3] - P[1, 3]) * 1     =     0
        # and since we have to do this for 2 cameras, and since we imposed the constraint,
        # we have to solve 4 equations in 3 unknowns (in LS sense).

        # Build C matrices, to construct A and b in a concise way
        C1[:, 2] = u1[i, :]
        C2[:, 2] = u2[i, :]
        
        # Build A matrix:
        # [
        #     [ u1.x * P1[2,0] - P1[0,0],    u1.x * P1[2,1] - P1[0,1],    u1.x * P1[2,2] - P1[0,2] ],
        #     [ u1.y * P1[2,0] - P1[1,0],    u1.y * P1[2,1] - P1[1,1],    u1.y * P1[2,2] - P1[1,2] ],
        #     [ u2.x * P2[2,0] - P2[0,0],    u2.x * P2[2,1] - P2[0,1],    u2.x * P2[2,2] - P2[0,2] ],
        #     [ u2.y * P2[2,0] - P2[1,0],    u2.y * P2[2,1] - P2[1,1],    u2.y * P2[2,2] - P2[1,2] ]
        # ]
        A[0:2, :] = C1.dot(P1[0:3, 0:3])    # C1 * R1
        A[2:4, :] = C2.dot(P2[0:3, 0:3])    # C2 * R2
        
        # Build b vector:
        # [
        #     [ -(u1.x * P1[2,3] - P1[0,3]) ],
        #     [ -(u1.y * P1[2,3] - P1[1,3]) ],
        #     [ -(u2.x * P2[2,3] - P2[0,3]) ],
        #     [ -(u2.y * P2[2,3] - P2[1,3]) ]
        # ]
        b[0:2, :] = C1.dot(P1[0:3, 3:4])    # C1 * t1
        b[2:4, :] = C2.dot(P2[0:3, 3:4])    # C2 * t2
        b *= -1
        
        # Solve for x vector
        cv2.solve(A, b, x[:, i:i+1], cv2.DECOMP_SVD)
    
    return x.T.astype(dtype=float), np.ones(len(u1), dtype=bool)

def slam(robot: cozmo.robot.Robot):
    '''
    SLAM
    
    https://scikit-image.org/docs/dev/auto_examples/transform/plot_fundamental_matrix.html
    ransac ret, ret, fundamentalmatrixtransform, min samples, residual_thresh=1, max_trials = 100)
    Extended Kalman Filter?
    ICP https://nbviewer.org/github/niosus/notebooks/blob/master/icp.ipynb This guy has a lot of stuff https://nbviewer.org/github/niosus/
    '''
    
    # Initialize Robot
    # http://cozmosdk.anki.com/docs/search.html?q=head&check_keywords=yes&area=default
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    robot.set_head_angle(degrees(0.00)).wait_for_completed()
    robot.set_lift_height(0.1).wait_for_completed()
    # https://web.stanford.edu/class/cs231a/course_notes/01-camera-models.pdf
    K = np.array([[292.54517539,   0,         157.75284395],
                  [  0,         292.86320684, 125.99736712],
                  [  0,           0,            1         ]])
    Kbig = np.array([[292.54517539,   0,         157.75284395, 0],
                     [  0,         292.86320684, 125.99736712, 0],
                     [  0,           0,            1,          0]])
    f = (K[0][0] + K[1][1]) / 2
    Kinv = np.linalg.inv(K)
    distCoeffs = np.array([[ .0391500220, -.258324807, .00212615449, -.000973903455, .978481250]])
    
    # Set some params
    des_new = None
    des_old = None
    img_old = None
    pose_old = None
    runs = 0
    while True:
        print()
        print('=====================New Loop=====================')
        print()
        latest_image = robot.world.latest_image
        if latest_image is not None:
            robot.drive_straight(distance_mm(5), speed_mmps(50)).wait_for_completed()
            time.sleep(0.25) # Wait a bit to avoid a blurry image
            img_new = latest_image.raw_image # 320 X 240
            img_new = np.array(img_new).astype(np.uint8)
            img_new = cv2.cvtColor(img_new, cv2.COLOR_BGR2GRAY)
            img_new = cv2.undistort(img_new, K, distCoeffs)
            if img_old is None: # Avoid hiccups first run
                img_old = img_new.copy()
                
            #Landmark detection
            orb = cv2.ORB_create()
            kp_new, des_new = orb.detectAndCompute(img_new, None)
            mask = tut_optical_flow(img_old, img_new)
            
            #Data association
            # Alternative Method https://docs.opencv.org/3.4/da/de9/tutorial_py_epipolar_geometry.html
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            if des_old is not None:
                matches = bf.match(des_new, des_old)
                matches = sorted(matches, key=lambda x:x.distance)
                prep_new = []
                prep_old = []
                for match in matches[:10]:
                    p_new = kp_new[match.queryIdx].pt
                    p_old = kp_old[match.trainIdx].pt
                    if match.distance < 30:
                        prep_new.append(kp_new[match.queryIdx].pt) # Is trainIdx or queryIdx from first or 2nd image? idk
                        prep_old.append(kp_old[match.trainIdx].pt)
                points_new = np.array([[pt[0], pt[1]] for pt in prep_new])
                points_old = np.array([[pt[0], pt[1]] for pt in prep_old])
        
                result = cv2.drawMatches(img_new, kp_new, img_old, kp_old, matches[:10], None, flags=2)

            #==============================================
            # FIND FUNDAMENTAL MATRIX
            #==============================================
            # https://en.wikipedia.org/wiki/Fundamental_matrix_(computer_vision)
            # https://stackoverflow.com/questions/59014376/what-do-i-do-with-the-fundamental-matrix
            # https://github.com/anki/cozmo-python-sdk/blob/master/src/cozmo/camera.py
            # https://stackoverflow.com/questions/25251676/opencv-findfundamentalmat-very-unstable-and-sensitive
            
            
            # SFM Observation Matrix
            # https://youtu.be/oIvg7sbJRIA?t=114
            # Apparently important that is has a very low rank
            
            # EXTENDED KALMAN FILTER
            
#            State Estimation
#            State Update
#            Landmark Update
            
            # Real Time Pose Estimation of a Textured Object
            # https://docs.opencv.org/3.4/dc/d2c/tutorial_real_time_pose.html
            
            # Camera Calibratio and 3D Reconstruction
            # https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
            
            # reprojectImageTo3D
            
            
            
            
            
            # Make SLAM
            
                # Process Frame
                
                    # FRAME
                    
                        # Extract Features
                            # Returns kps.pt, des
                            
                    # Match Frames
                        # get matches
                        # filtering etc
                        # RANSAC
                        # fundamentalToRt(model.params)
                        # Returns idx1[inlies] and idx2[inliers] and the result of fundamentalToRt(model.params)
            
                    # Add Observation
                        
                    # Pose Optimization
                        #pose_opt = self.mapp.optimize()
                        
                    # IMPORTANT https://github.com/geohot/twitchslam/blob/master/slam.py#L68
                    
            
            
            
            
            # Essential Matrix https://github.com/scikit-image/scikit-image/blob/master/skimage/transform/_geometric.py
            # https://github.com/scikit-image/scikit-image/blob/main/skimage/transform/_geometric.py#L488
            # https://github.com/scikit-image/scikit-image/blob/4cd351ab6560fc77512cea9d5f781423b41987fd/skimage/transform/_geometric.py#L309
            
            try:
                cv2.imshow('main', result)
                cv2.imshow('frame', mask)
                #cv2.imshow('fundmask', fundmask)
            except:
                pass
            c = cv2.waitKey(0)
            if c == ord('q') or c == 27:
                break
            elif c == ord('c'):
                method = cv2.FM_RANSAC # 
                ransacReprojThreshold = 3
                confidence = 0.75
                maxIters = 10
                # https://www.youtube.com/watch?v=zX5NeY-GTO0&ab_channel=CyrillStachniss
                F, Fmask = cv2.findFundamentalMat(points_old, points_new, method, ransacReprojThreshold, confidence, maxIters=maxIters)
                F_error = test_F(points_new, points_old, F)
                
                # https://stackoverflow.com/questions/33906111/how-do-i-estimate-positions-of-two-cameras-in-opencv
                # https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#gad245d60e64d0c1270dbfd0520847bb87
                # https://www.cs.cmu.edu/~16385/s17/Slides/12.2_Essential_Matrix.pdf
                y_news = cv2.undistortPoints(points_new, cameraMatrix=K, distCoeffs=distCoeffs)
                y_olds = cv2.undistortPoints(points_old, cameraMatrix=K, distCoeffs=distCoeffs)
                E, Emask = cv2.findEssentialMat(y_olds, y_news, K)
                E_error = test_E(y_news, y_olds, E)
                
                # SVD
                # https://stackoverflow.com/questions/38600871/incorrect-camera-pose-from-two-view-sfm
                # https://stackoverflow.com/questions/20614062/pose-from-fundamental-matrix-and-vice-versa
                # https://en.wikipedia.org/wiki/Singular_value_decomposition
                # https://en.wikipedia.org/wiki/Essential_matrix#Extracting_rotation_and_translation
                # fundamentalToRt
                W = np.mat([[0,-1,0],[1,0,0],[0,0,1]],dtype=float)          # Pg 258 Multiple View Geometry       
                Z = np.mat([[0,1,0],[-1,0,0],[0,0,0]])
                U, Sigma, Vt = np.linalg.svd(E)
                if np.linalg.det(U) < 0:
                    U *= -1.0
                if np.linalg.det(Vt) < 0:
                    Vt *= -1.0
                R = np.dot(np.dot(U, W), Vt)
                if np.sum(R.diagonal()) < 0:
                    R = np.dot(np.dot(U, W.T), Vt)
                t = U[:, 2]
                if t[2] < 0:
                    t *= -1
                # https://github.com/geohot/twitchslam/blob/c52a14fe1034426b6dfc1e6222984a9a06e40b6a/helpers.py#L66
                # poseRt(R, t) https://github.com/geohot/twitchslam/blob/c52a14fe1034426b6dfc1e6222984a9a06e40b6a/helpers.py#L40
                ret = np.eye(4)
                ret[:3, :3] = R
                ret[:3, 3] = t
                fundToRt = np.linalg.inv(ret)
                Rt = fundToRt
                # END fundamentalToRt
            
                #idx1, idx2, Rt = match_frames(f1, f2) = idx1[inliers], idx2[inliers], fundToRt # https://github.com/geohot/twitchslam/blob/master/frame.py#L64
                
                idx_new = points_new
                idx_old = points_old
                
                if pose_old is None:
                    pose_old = np.eye(4)
                pose_new = np.dot(Rt, pose_old) # http://cozmosdk.anki.com/docs/generated/cozmo.util.html#cozmo.util.Pose
                
                #pts4d = triangulate(f1.pose, f2.pose, f1.kps[idx1], f2.kps[idx2])
                #def TRIANGULATE(pose1, pose2, pts_new, pts_old):
                pts_old = points_old
                pts_new = points_new
                ret = np.zeros((pts_new.shape[0], 4))
                for i, p in enumerate(zip(pts_new, pts_old)):
                    A = np.zeros((4,4))
                    A[0] = p[0][0] * pose_new[2] - pose_new[0]
                    A[1] = p[0][1] * pose_new[2] - pose_new[1]
                    A[2] = p[1][0] * pose_old[2] - pose_old[0]
                    A[3] = p[1][1] * pose_old[2] - pose_old[1]
                    _, _, vt = np.linalg.svd(A)
                    ret[i] = vt[3]
                # END TRIANGULATE
                     
                good_pts4d = points_new # np.array([pts_new[i] is None for i in idx_new])
                pts4d = ret
                #good_pts4d &= np.abs(pts4d[:, 3]) != 0
                pts4d /= pts4d[:, 3:]
                
                for i, p in enumerate(pts4d):
#                    print('POINT = {}'.format(p))
                    pass
                
#                print('pose_old = {}'.format(pose_old))
#                print('pose_new = {}'.format(pose_new))
                    
                # https://answers.opencv.org/question/38131/converting-a-2d-image-point-to-a-3d-world-point/
                # https://stackoverflow.com/questions/24944266/calculating-3d-world-point-from-2d-image-point-using-opencv
                # https://stackoverflow.com/questions/62941410/calculating-3d-point-from-essential-matrix-backprojection-2d-to-3d
                # reprojectImageTo3D https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga1bc1152bd57d63bc524204f21fde6e02
                # triangulatePoints https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#gad3fc9a0c82b08df034234979960b778c
                points4d = cv2.triangulatePoints(Kbig, Kbig, y_news.T, y_olds.T) # Seems to be bad results
#                print()
#                print()
#                print('points4d = {}'.format(points4d))
#                print('points4d.shape = {}'.format(points4d.shape))
#                print()
#                for i in range(points4d.shape[2]):
#                    pt = points4d[:,0,i].reshape(-1,1)
#                    print(pt)
#                print()

                pts_new = []
                pts_old = []
                for pt_new, pt_old in zip(points_new, points_old): # These points have to be of same length for recoverPose
                    #print(pt_new)
                    pts_new.append(pt_new)
                    pts_old.append(pt_old)
                pts_new = np.array([[pt[0], pt[1]] for pt in pts_new])
                pts_old = np.array([[pt[0], pt[1]] for pt in pts_old])
                pts_new_l = np.array([[pt[0], pt[1], 1] for pt in pts_new])
                pts_old_l = np.array([[pt[0], pt[1], 1] for pt in pts_old])
                pts_new_l = np.array([[pt[0], pt[1], 1] for pt in pts_new])
                pts_new_norm = np.array([[normalize2(pt[0],pt[1])[0], normalize2(pt[0],pt[1])[1]] for pt in pts_new])
                pts_old_norm = np.array([[normalize2(pt[0],pt[1])[0], normalize2(pt[0],pt[1])[1]] for pt in pts_old])
                
                
#                val1, val2 = linear_LS_triangulation(tup_news, Kbig, tup_olds, Kbig)
#                print('val1 = {}'.format(val1))
#                print('val2 = {}'.format(val2))
#                print()
#                print()
#                print()
#                print()
#                print()
                
                pts_new = []
                pts_old = []
                for pt_new, pt_old in zip(y_news, y_olds): # These points have to be of same length for recoverPose
                    print(pt_new)
                    pts_new.append(pt_new[0])
                    pts_old.append(pt_old[0])
                pts_new = np.array([[pt[0], pt[1]] for pt in pts_new])
                pts_old = np.array([[pt[0], pt[1]] for pt in pts_old])
                pts_new_l = np.array([[pt[0], pt[1], 1] for pt in pts_new])
                pts_old_l = np.array([[pt[0], pt[1], 1] for pt in pts_old])
                pts_new_l = np.array([[pt[0], pt[1], 1] for pt in pts_new])
                pts_new_norm = np.array([[normalize2(pt[0],pt[1])[0], normalize2(pt[0],pt[1])[1]] for pt in pts_new])
                pts_old_norm = np.array([[normalize2(pt[0],pt[1])[0], normalize2(pt[0],pt[1])[1]] for pt in pts_old])
                print('pts_new_norm = {}'.format(pts_new_norm))
                
                
                val1, val2 = linear_LS_triangulation(pts_new_norm, Kbig, pts_old_norm, Kbig)
                print('val1 = {}'.format(val1))
                print('val2 = {}'.format(val2))
                print()
                print()
                print()
                print()
                print()
                
                print('recoverPose')
                points, R_est, t_est, mask_pose = cv2.recoverPose(E=E, points1=pts_new, points2=pts_old)
                print('points = {}'.format(points))
                print()
                print('R_est = {}'.format(R_est))
                print()
                print('t_est = {}'.format(t_est))
                print()
                
                # initUndistortRectifyMap https://docs.opencv.org/3.4/da/d54/group__imgproc__transform.html#ga7dfb72c9cf9780a347fbe3d1c47e5d5a
                
                
                
            kp_old = kp_new
            img_old = img_new.copy()
            des_old = des_new
            try:
                pose_old = pose_new.copy()
            except:
                pass
    cv2.destroyAllWindows()
        
cozmo.robot.Robot.drive_off_charger_on_connect = False
cozmo.run_program(slam)