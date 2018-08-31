#-*- encoding:utf8 -*-
import cv2
import threading
import Queue as que
import time
import numpy as np

from polyfitter import Polyfitter
from warper import Warper

warper = Warper()
polyfitter = Polyfitter()

q1 = que.Queue()

error_sum = 0
error_old = 0
wfPath = "/home/kimsunggon/바탕화면/p2"
p = [0.0035, 0.000005, 0.005] # optimized kp,ki,kd
dp = [p[0]/10, p[1]/10, p[2]/10] # to twiddle kp, ki, kd

def main():  

    wp = open(wfPath, 'w')
    wp.write(str(0.000))
    wp.close()

    t1 = threading.Thread(target = generate_imshow)
    t2 = threading.Thread(target = time_queue)    

    t1.start()
    t2.start()

def cal_error(x_current, setpoint=318):
    return setpoint - x_current

# twiddle is for optimize the kp,ki,kd
def twiddle(x_current, setpoint=318):

    global p,dp

    best_err = cal_error(x_current) 
    #threshold = 0.001
    #threshold = 1e-09 
    threshold = 0.0000000000000000000000000000001 

    # searching by move 1.1x to the target and if go more through the target comeback to -2x
    while sum(dp) > threshold:
        for i in range(len(p)):
            p[i] += dp[i]
            err = cal_error(x_current) 

            if err < best_err:  # There was some improvement
                best_err = err
                dp[i] *= 1.1
            else:  # There was no improvement
                p[i] -= 2*dp[i]  # Go into the other direction
                err = cal_error(x_current)

                if err < best_err:  # There was an improvement
                    best_err = err
                    dp[i] *= 1.05
                else:  # There was no improvement
                    p[i] += dp[i]
                    # As there was no improvement, the step size in either
                    # direction, the step size might simply be too big.
                    dp[i] *= 0.95

    print(p)
   
# setpoint is the center and the x_current is where the car is
# width = 640, so 320 is the center but 318 is more accurate in real
def pid_control(x_current, setpoint=318):
   
    global error_sum
    global error_old

    twiddle(x_current)

    error = setpoint - x_current
    p1 = round(p[0] * error, 9)
    error_sum += error
    i1 = round(p[1] * error_sum, 9)
    d1 = round(p[2] * (error -  error_old), 9)
    error_old = error
    pid = p1 + i1 + d1    
    #print("p : " ,p)
    #print("i : " ,i)
    #print("d : " ,d)
    return pid

# to redece real sight and camera sight
def time_queue(): 

    time.sleep(2.5)
    while True:
        x_current = q1.get()
        pid = round(pid_control(int(x_current)), 6)
        pid = '{0:.6f}'.format(pid)

        # sending the xlocation to ros by namedpipe
        global wfPath
        wp = open(wfPath, 'w')
        wp.write(str(pid))
        wp.close()
        print("pid",pid)   

def generate_imshow():

    max_y = None
    x_location = None
    # cap = cv2.VideoCapture(2) # for webcam
    cap = cv2.VideoCapture("new1.avi") # for videofile
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)

    # record the origin
    out = cv2.VideoWriter('/home/nvidia/Desktop/outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 30, (640,480))

    # record the processed
    out2 = cv2.VideoWriter('/home/nvidia/Desktop/oripy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 30, (640,480))

    while(cap.isOpened()):

        ret, frame = cap.read()
        img1, max_y, x_location = process_image(frame, max_y)
        cv2.imshow('result', img1)
        #time.sleep(tmp)
        if x_location != None:
            q1.put(x_location)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
	cv2.imshow("origin", frame)
        out.write(img1)
        out2.write(frame)
 
    cap.release()
    out.release()
    out2.release()
    cv2.destroyAllWindows()

def process_image(frame, max_y):
    
    # grayscle
    gray = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY)
    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)
    # canny edge
    low_threshold = 40
    high_threshold = 50
    edges_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)
    # warper
    img = warper.warp(edges_img)
    # polyfit(window sliding and polyfit)
    img1, max_y, x_location = polyfitter.polyfit(img, max_y)
    
    return img1, max_y, x_location

if __name__ == '__main__':
    main()
