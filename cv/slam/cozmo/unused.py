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

def fund_mat(imgold, imgnew):
    '''
    https://scikit-image.org/docs/dev/auto_examples/transform/plot_fundamental_matrix.html
    '''
    #img_left, img_right, groundtruth_disp = data.stereo_motorcycle()
    img_left, img_right = map(rgb2gray, (imgold, imgnew))
    
    # Find sparse feature correspondences between left and right image.
    
    descriptor_extractor = ORB()
    
    descriptor_extractor.detect_and_extract(imgold)
    keypoints_left = descriptor_extractor.keypoints
    descriptors_left = descriptor_extractor.descriptors
    
    descriptor_extractor.detect_and_extract(imgnew)
    keypoints_right = descriptor_extractor.keypoints
    descriptors_right = descriptor_extractor.descriptors
    
    matches = match_descriptors(descriptors_left, descriptors_right,
                                cross_check=True)
    
    # Estimate the epipolar geometry between the left and right image.
    
#    model, inliers = ransac((keypoints_left[matches[:, 0]],
#                             keypoints_right[matches[:, 1]]),
#                            FundamentalMatrixTransform, min_samples=8,
#                            residual_threshold=1, max_trials=5000)
    
    inlier_keypoints_left = keypoints_left[matches[:, 0]]
    inlier_keypoints_right = keypoints_right[matches[:, 1]]
    
    print(f'Number of matches: {matches.shape[0]}')
    #print(f'Number of inliers: {inliers.sum()}')
    
    # Compare estimated sparse disparities to the dense ground-truth disparities.
    
    disp = inlier_keypoints_left[:, 1] - inlier_keypoints_right[:, 1]
    disp_coords = np.round(inlier_keypoints_left).astype(np.int64)
    #  disp_idxs = np.ravel_multi_index(disp_coords.T, groundtruth_disp.shape)
    #disp_error = np.abs(groundtruth_disp.ravel()[disp_idxs] - disp)
    #disp_error = disp_error[np.isfinite(disp_error)]
    
    # Visualize the results.
    
    fig, ax = plt.subplots(nrows=2, ncols=1)
    
    plt.gray()
    
    plot_matches(ax[0], imgnew, imgnew, keypoints_left, keypoints_right,
                 matches[:], only_matches=True)
    ax[0].axis("off")
    ax[0].set_title("Inlier correspondences")
    
    #ax[1].hist(disp_error)
    #ax[1].set_title("Histogram of disparity errors")
    
    plt.show()


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