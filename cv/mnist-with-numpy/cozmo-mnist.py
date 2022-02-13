import numpy as np
import random
import time
import cv2

import cozmo

# Setup
start = time.time()
epochs = 3
lr = 0.01
batch = 1
debug_data_aug = False

def add_noise(x):
    amt = 150
    x = (x + amt*np.random.rand(x.shape[0], x.shape[1]))
    x = x - np.min(x)
    x = x * (255 / np.max(x))
    return x

def rot(x):
    ang = np.random.rand()*30-15 # Degrees
    h, w = x.shape # Numpy puts out the image axes in the wrong order
    cx = w / 2
    cy = h / 2
    theta = ang * 3.14 / 180
    ang = -theta
    rotmat = np.array([[np.cos(ang), -np.sin(ang)],
                    [np.sin(ang), np.cos(ang)]])

    new_im = np.zeros((h, w))
    for i in range(w):
        for j in range(h):
            newx = i - cx
            newy = (h-j) - cy
            vec = np.matmul(rotmat, np.array([[newx],[newy]]))
            oldx = round(vec[0][0] + cx)
            oldy = round(cy-vec[1][0])
            if oldx >= 0 and oldx < w and oldy >= 0 and oldy < h:
                new_im[j][i] = x[oldy][oldx]
    return new_im

def shift_hor(x):
    if np.random.rand() >= 0.999:
        for i in range(x.shape[1]-1): # Left
            x[:,i] = x[:,i+1]
    else:
        for i in range(x.shape[1]): # Right
            i = x.shape[0] - i
            if i <27:
                x[:,i+1] = x[:,i]
    return x

def shift_vert(x):
    if np.random.rand() >= 0.5:
        for i in range(x.shape[0]-1): # Up
            x[i] = x[i+1]
    else:
        for i in range(x.shape[0]): # Down
            i = x.shape[0] - i
            if i <27:
                x[i] = x[i-1]
    return x

def smooth_blur(x):
    size = 3
    w, h = x.shape
    kernel = np.ones((size,size)) # Smooth Kernel
    for a in range(w-size+1):
        for b in range(h-size+1):
            x[a][b] = (x[a][b] + 0.25*np.sum( x[a:a+size, b:b+size]*kernel ) / (size**2) ) / 1.25
    return x

def data_aug(x):
    '''
    Calls functions to augment data
    '''
    if np.random.rand() >= 100: # Adds more compute time than it removes
        x = smooth_blur(x)
    if np.random.rand() >= 0.01:
        x = shift_vert(x)
    if np.random.rand() >= 0.5:
        x = shift_hor(x)
    if np.random.rand() >= 0.00001: # This probably helps less than shifting since test data has no noise like this
        x = add_noise(x)
    if np.random.rand() >= 100: # Adds more compute time than it removes
        x = rot(x)
    if debug_data_aug:
        cv2.imshow('qwerty', x.copy()/255)
        c = cv2.waitKey(0)
        if c == ord('q') or c == 27:
            cv2.destroyAllWindows()
            raise SystemExit(0)
    return x

# Data
print('Importing MNIST Data')
from keras.datasets import mnist
(X_train, Y_train), (X_test, Y_test) = mnist.load_data()
X_train = X_train #.reshape(-1, 784)/255
X_test = X_test.reshape(-1, 784)/255
from keras.utils.np_utils import to_categorical
Y_train = to_categorical(Y_train)
Y_test = to_categorical(Y_test)

# Layers
print('Setup')
w0 = np.random.randn(64, 784)*np.sqrt(1/(64+784)) # Xavier Initialization
w1 = np.random.randn(32, 64)*np.sqrt(1/(32+64))
out = np.random.randn(10, 32)*np.sqrt(1/(10+32))

def shuffl3(x, y):
    '''
    Shuffle the order of incoming images
    '''
    assert len(x) == len(y)
    ids = np.random.permutation(len(x))
    return x[ids], y[ids]

def for_back_pass(x, y, backpass=True):
    '''
    x is the incoming singular image
    y is the label such as [0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
    backpass is True by default. Set as true if you want to correct weights. False if you want to leave weights alone.
    '''
    # Forward pass
    res_w0 = np.dot(w0, x)
    res_rel0 = np.maximum(res_w0, 0)
    res_w1 = np.dot(w1, res_rel0)
    res_rel1 = np.maximum(res_w1, 0)
    res_out = np.dot(out, res_rel1)
    #https://www.youtube.com/watch?v=mlaLLQofmR8 softmax video
    guess = np.exp(res_out - res_out.max()) / np.sum(np.exp(res_out - res_out.max()), axis=0) # Softmax eqn I found somewhere
    loss = abs((guess - y)).mean(axis=0)
    correct = (np.argmax(y) == np.argmax(guess))
    error = (guess - y)
    
    # Backward Prop
    if backpass:
        dd = guess*(1-guess)
        error = error * dd
        dx_out = np.outer(error, res_rel1)
        error = np.dot(out.T, error) * (res_rel1 > 0)
        dx_w1 = np.outer(error, res_rel0)
        error = np.dot(w1.T, error) * (res_rel0 > 0)
        dx_w0 = np.outer(error, x)
    else:
        dx_out, dx_w0, dx_w1 = 0, 0, 0
    
    return dx_out, dx_w0, dx_w1, guess, loss, correct

# Loop
loss_list = []
print('Running {} epochs'.format(epochs))
vold_dx_out = 0
vold_dx_w0 = 0
vold_dx_w1 = 0
old_dx_out = 0
old_dx_w0 = 0
old_dx_w1 = 0
backpass = True
validate = True
for epoch in range(epochs):
    if epoch == 5:
        lr = lr / 2
    if epoch == 8:
        lr = lr / 2
    if epoch == 10:
        lr = lr / 2
    temp_loss = []
    correct = []
    solver = 'my_momentum_v2'
    if batch == 1:
        X = X_train
        Y = Y_train
        X, Y = shuffl3(X, Y)
        for x, y in zip(X, Y):
            x = data_aug(x) # This doesn't help much for accuracy 95-97% and it slows things down a bit. Use >97%
            x = x.reshape(-1, 784)/255 # It will be faster to do this before the epochs but data_aug easier with square img
            x = x[0]
            dx_out, dx_w0, dx_w1, guess, loss, correcti = for_back_pass(x, y, backpass=backpass)
            if backpass:
                if solver == 'my_momentum_v2':
                    out = out - lr*dx_out - 0.5*lr*old_dx_out - 0.25*lr*vold_dx_out
                    w0 = w0 - lr*dx_w0 - 0.5*lr*old_dx_w0 - 0.25*lr*vold_dx_w0
                    w1 = w1 - lr*dx_w1 - 0.5*lr*old_dx_w1 - 0.25*lr*vold_dx_w1
                    # Trying Momentum
                    vold_dx_out = old_dx_out
                    vold_dx_w0 = old_dx_w0
                    vold_dx_w1 = old_dx_w1
                    old_dx_out = dx_out
                    old_dx_w0 = dx_w0
                    old_dx_w1 = dx_w1
                elif solver == 'adam':
                    pass

            correct.append(correcti)
            
    else: # batching will require more epochs
        ids = [random.randint(0, X_train.shape[0]) for i in range(batch)]
        X = X_train[ids]
        Y = Y_train[ids]
        dx_out_l = np.zeros_like(out)
        dx_w0_l = np.zeros_like(w0)
        dx_w1_l = np.zeros_like(w1)
        loss_l = []
        correcti_l = []
        for x, y in zip(X, Y):
            x = x.reshape(-1, 784)/255 # It will be faster to do this before the epochs but data_aug easier with square img
            x = x[0]
            dx_out, dx_w0, dx_w1, guess, loss, correcti = for_back_pass(x, y, backpass=backpass)
            dx_out_l += dx_out
            dx_w0_l += dx_w0
            dx_w1_l += dx_w1
            loss_l.append(loss)
            correcti_l.append(correcti)
        dx_out = dx_out_l / batch
        dx_w0 = dx_w0_l /batch
        dx_w1 = dx_w1_l /batch
        loss = sum(loss_l)/batch
        correcti = sum(correcti_l)/batch
        if backpass:
            if solver == 'my_momentum_v2':
                out = out - lr*dx_out - 0.5*lr*old_dx_out - 0.25*lr*vold_dx_out
                w0 = w0 - lr*dx_w0 - 0.5*lr*old_dx_w0 - 0.25*lr*vold_dx_w0
                w1 = w1 - lr*dx_w1 - 0.5*lr*old_dx_w1 - 0.25*lr*vold_dx_w1
                # Trying Momentum
                vold_dx_out = old_dx_out
                vold_dx_w0 = old_dx_w0
                vold_dx_w1 = old_dx_w1
                old_dx_out = dx_out
                old_dx_w0 = dx_w0
                old_dx_w1 = dx_w1
            elif solver == 'adam':
                pass

        correct.append(correcti)
        
    correct_percent = sum(correct) / len(correct)
    loss_list.append(loss)
    if epochs > 10000:
        if epoch % 10000 == 0:
            print('Epoch{} Time = {}s loss={} accuracy = {}'.format(epoch, time.time() - start, loss, correct_percent))
    elif epochs > 1000:
        if epoch % 1000 == 0:
            print('Epoch{} Time = {}s loss={} accuracy = {}'.format(epoch, time.time() - start, loss, correct_percent))
    elif epochs > 100:
        if epoch % 100 == 0:
            print('Epoch{} Time = {}s loss={} accuracy = {}'.format(epoch, time.time() - start, loss, correct_percent))
    elif epochs < 100:
        print('Epoch{} Time = {}s loss={} accuracy = {}'.format(epoch, time.time() - start, loss, correct_percent))
print('Final Epoch Result')
print('Epoch{} Time = {}s loss={} accuracy = {}'.format(epoch, time.time() - start, loss, correct_percent))
        
def mnist(robot: cozmo.robot.Robot):
    '''
    Cozmo MNIST
    '''
    robot.camera.image_stream_enabled = True        # LEAVE AS True
    robot.camera.color_image_enabled = False        # IMPORTANT - Generally leave as False

    while True:
        latest_image = robot.world.latest_image
        if latest_image is not None:
            raw_img = latest_image.raw_image # 320 X 240
            cv_img = np.array(raw_img).astype(np.uint8)
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            
            cv2.rectangle(cv_img, (145, 105), (175, 135), 255)
            cv2.imshow('asdf', cv_img)
            cv_img_crop = cv_img[106:134, 146:174].copy()
            cv_img_crop = 255 - cv_img_crop
            cv_img_crop = cv2.normalize(cv_img_crop, np.zeros_like(cv_img_crop), 0, 255, cv2.NORM_MINMAX)
            cv2.imshow('qwerty', cv_img_crop)
            c = cv2.waitKey(0)
            if c == ord('s'):
                x = cv_img_crop
                x = x.reshape(-1, 784)/255 # It will be faster to do this before the epochs but data_aug easier with square img
                x = x[0]
                y = [1,0,0,0,0,0,0,0,0,0]
                dx_out, dx_w0, dx_w1, guess, loss, correcti = for_back_pass(x, y, backpass=False)
                max_guess = np.max(guess)
                idx = np.where(guess == np.amax(max_guess))[0][0]
                robot.say_text(str(idx)).wait_for_completed()
                print(guess)
                print(max_guess)
                print(idx)
                print(guess)
                print(max_guess)
                print(idx)
                print(guess)
                print(max_guess)
                print(idx)
                print(guess)
                print(max_guess)
                print(idx)
                
            elif c == ord('q') or c == 27:
                print('quit')
                cv2.destroyAllWindows()
                break
        
cozmo.robot.Robot.drive_off_charger_on_connect = False
cozmo.run_program(mnist)