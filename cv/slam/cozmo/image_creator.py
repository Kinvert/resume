import cozmo
import cv2
import numpy as np
import os
import time

def add_zeros(num_string, digits):
    num_string = str(num_string)
    while len(num_string) < digits:
        num_string = '0' + num_string
    return num_string

def img_create(robot: cozmo.robot.Robot):
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

    while True:
        latest_image = robot.world.latest_image
        if latest_image is not None:
            raw_img = latest_image.raw_image # 320 X 240
            cv_img = np.array(raw_img).astype(np.uint8)
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            
            cv2.imshow('main', cv_img)
            c = cv2.waitKey(0)
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
        
cozmo.robot.Robot.drive_off_charger_on_connect = False
cozmo.run_program(img_create)