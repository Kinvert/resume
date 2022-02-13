import cv2
import time

def draw_matches(img1, img2):
    '''
    https://youtu.be/Fe-KWKPk9Zc?t=946
    '''
    start = time.time()
    orb = cv2.ORB_create()
    kp1, des1 = orb.detectAndCompute(img1, None)
    kp2, des2 = orb.detectAndCompute(img2, None)
    
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)
    matches = sorted(matches, key=lambda x:x.distance)
    for match in matches[:10]:
        print(match.distance)
    
    result = cv2.drawMatches(img1, kp1, img2, kp2, matches[:10], None, flags=2)
    
    cv2.imshow('img1', img1)
    cv2.imshow('img2', img2)
    cv2.imshow('result', result)
    print('took {}s'.format(time.time() - start))
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
img1 = cv2.imread('img/cube1-000000.jpg')
img2 = cv2.imread('img/cube1-000001.jpg')
draw_matches(img1, img2)