import numpy as np
import cv2
import matplotlib.pyplot as plt
cap = cv2.VideoCapture(0)
#blueUpper = (113+10, 255, 255)
#blueLower = (113-10, 100, 100) 
blueUpper = (250, 255, 255)
blueLower = (10, 100, 100) 



while(cap.isOpened()):
    ret, frame = cap.read()
    if ret==True:
        
        x1 = int(np.size(frame,0)/2)
        y1 = int(np.size(frame,1)/2)
        blur = cv2.GaussianBlur(frame,(9,9),0)
        img1=cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)
        ma = cv2.inRange(img1,blueLower,blueUpper)

    
    
        mas = cv2.erode(ma, None, iterations=2)
        mask = cv2.dilate(mas, None, iterations=2)

        img,cnts,hie = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        center = []
        center2=(x1,y1)

        if len(cnts) > 0:
            for i in range(len(cnts)):
                c = cnts[i]
                M = cv2.moments(c)
                cv2.drawContours(frame,c,-1,(0,255,255),2)
                center.append((int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])))
                
                
                cv2.circle(frame, center[i], 5, (0, 0, 0), -1)
                cv2.putText(frame,'Center:'+str(center[i]),(20,50+(i*30)),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255, 255, 255), lineType=cv2.LINE_AA)
                print(center2)
        
        

       
        

        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

# Release everything if job is finished
cap.release()

cv2.destroyAllWindows()