import cv2
import numpy as np
import math
import pyautogui

def identify_hand(crop_image,box):
     # Apply Gaussian blur
    blur = cv2.GaussianBlur(crop_image, (3,3), cv2.BORDER_DEFAULT)
    
    # Change color-space from BGR -> HSV
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
           
    #extract skin colur image
    mask = cv2.inRange(hsv, (0,20,70), (20,255,255))
    dilate = cv2.dilate(mask,np.ones((5,5)),iterations = 3)
    blur = cv2.blur(dilate,(5,5),100) 
    contours,hierarchy= cv2.findContours(blur,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
   
    try:
        # Find contour with maximum area
        contour = max(contours, key = lambda x: cv2.contourArea(x))
    
        # Fi convexity defects
        hull = cv2.convexHull(contour, returnPoints=False)
        defects = cv2.convexityDefects(contour,hull)
        
        # Use cosine rule to find angle of the far point from the start and end point i.e. the convex points (the finger 
        # tips) for all defects
        count_defects = 0
        for i in range(defects.shape[0]):
            s,e,f,d = defects[i,0]
            start = tuple(contour[s][0])
            end = tuple(contour[e][0])
            far = tuple(contour[f][0])

            a = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
            b = math.sqrt((far[0] - start[0])**2 + (far[1] - start[1])**2)
            c = math.sqrt((end[0] - far[0])**2 + (end[1] - far[1])**2)
            angle = (math.acos((b**2 + c**2 - a**2)/(2*b*c))*180)/math.pi
            
            # if angle >= 90 draw a circle at the far point
            if angle <= 90:
                count_defects += 1
                cv2.circle(crop_image,far,3,[0,0,255],-1)

            cv2.line(crop_image,start,end,[0,255,0],2)
        
        # Press SPACE if condition is match
        if count_defects >=3:
            if box ==1:
                pyautogui.press('right')
                cv2.putText(frame,"RIGHT", (300,100), cv2.FONT_HERSHEY_SIMPLEX,  1, (255, 255, 255) , 2 , cv2.LINE_AA)
            else:
                pyautogui.press('left')
                cv2.putText(frame,"LEFT", (300,100), cv2.FONT_HERSHEY_SIMPLEX,  1, (0, 0, 0) , 2 , cv2.LINE_AA)
                

    except:
        pass

    
capture = cv2.VideoCapture(0)

while capture.isOpened():
    
    # Capture frames from the camera
    ret, frame = capture.read()
    frame=cv2.flip(frame,1)
     
    cv2.rectangle(frame,(400,100),(600,300),(0,255,0),0)
    crop_image1 = frame[100:300,400:600]
    cv2.rectangle(frame,(0,100),(200,300),(0,255,0),0)  
    crop_image2=frame[100:300, 0:200]
        
    right=1
    left=-1
    
    identify_hand(crop_image1,right)
    identify_hand(crop_image2,left)
    #frame = cv2.resize(frame, (960, 540)) 
    
    # Show required images
    cv2.imshow("Gesture", frame)            
     
    # Close the camera if 'q' is pressed
    if cv2.waitKey(1) == ord('q'):
        break       

capture.release()
cv2.destroyAllWindows()