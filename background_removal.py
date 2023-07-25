# import cv2 to capture videofeed
import cv2

import numpy as np

# attach camera indexed as 0
camera = cv2.VideoCapture(0)

# setting framewidth and frameheight as 640 X 480
camera.set(3 , 640)
camera.set(4 , 480)

# loading the image
image = cv2.imread('eiffel_tower.jpg')

# resizing the image as 640 X 480
image = cv2.resize('eiffel_tower.jpg' , (640,480))

while True:

    # read a frame from the attached camera
    status , frame = camera.read()

    # if we got the frame successfully
    if status:

        # flip it
        frame = cv2.flip(frame , 1)

        # converting the image to RGB for easy processing
        frame_rgb = cv2.cvtColor(frame , cv2.COLOR_BGR2RGB)

        # creating thresholds
        lower_bound = np.array([0 ,0 ,0])
        upper_bound = np.array([150,150,150])

        # thresholding image
        mask = cv2.inRange(frame_rgb,lower_bound,upper_bound)
        
        cv2.imshow("mask" , mask)
        # inverting the mask

        mask_2 = cv2.bitwise_not(mask)

        # bitwise and operation to extract foreground / person
        mask = cv2.morphologyEx(mask , cv2.MORPH_OPEN , np.ones((3,3) , np.uint8 ))

        mask = cv2.morphologyEx(mask , cv2.MORPH_DILATE , np.ones((3,3) , np.uint8 ))


        # final image

        bg_final = cv2.bitwise_and(image,image,mask=mask_2)
        person_final = cv2.bitwise_and(frame , frame ,mask= mask)

        # show it
        cv2.imshow('frame' , frame)

        # wait of 1ms before displaying another frame
        code = cv2.waitKey(1)
        if code  ==  32:
            break

# release the camera and close all opened windows
camera.release()
cv2.destroyAllWindows()
