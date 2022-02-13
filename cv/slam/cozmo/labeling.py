import cozmo
import cv2
import numpy as np
import os
import time

from utils import add_zeros

p1 = ()
p2 = ()
drawing = False

def label_draw(event, x, y, flags, params):
    global p1, p2, drawing
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        p1 = (x, y)
        p2 = ()
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        p2 = (x, y)
        
def img_label(robot: cozmo.robot.Robot):
    '''
    Watch values labeled as important
    
    NAMING CONVENTION Name all categories pictured with dashes between
    
    Examples
        cube1                              will become cube1-000397
        cube1-legoperson-road-car          will become cube1-legoperson-road-car-000018
        
    Reason
        When training, a search should be possible to find images that contain
            the object your are looking for
    
    space = check next frame without saving
    s = save
    q = quit
    esc = quit
    '''
    robot.camera.image_stream_enabled = True        # LEAVE AS True
    digits = 6                                      # IMPORTANT - PROBABLY LEAVE AT 6
    robot.camera.color_image_enabled = False        # IMPORTANT - Generally leave as False
    
    #======================================================================
    #======================================================================
    short_fname = 'cube1'                           # IMPORTANT
    #======================================================================
    #======================================================================
    
    cv2.namedWindow('label')
    cv2.setMouseCallback('label', label_draw)

    while True:
        latest_image = robot.world.latest_image
        if latest_image is not None:
            raw_img = latest_image.raw_image # 320 X 240
            cv_img = np.array(raw_img).astype(np.uint8)
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            drawn = cv_img.copy()
            cv2.imshow('label', cv_img)
            c = cv2.waitKey(1)
            while c == -1:
                if p1 and p2 and drawing is False:
                    drawn = cv2.rectangle(drawn, p1, p2, 255)
                    cv2.imshow('label', drawn)
                c = cv2.waitKey(10)
                print(c)
            if c == ord('s'):
                i = 0
                i_string = add_zeros(i, digits)
                while os.path.exists('img/%s-%s.jpg' % (short_fname, i_string)):
                    i += 1
                    i_string = add_zeros(i, digits)
                num = add_zeros(i, digits)
                full_name = 'img/{}-{}.jpg'.format(short_fname, num)
                cv2.imwrite(full_name, cv_img)
                print('saved as {}'.format(full_name))
                time.sleep(0.1)
            elif c == ord('q') or c == 27:
                print('quit')
                cv2.destroyAllWindows()
                break
    cv2.destroyAllWindows()
        
cozmo.robot.Robot.drive_off_charger_on_connect = False
cozmo.run_program(img_label)