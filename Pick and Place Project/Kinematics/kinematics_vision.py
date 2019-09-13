from mpmath import *
from sympy import *
import numpy as np

import cv2
import matplotlib.pyplot as plt

import serial
import time

cap = cv2.VideoCapture(0)

ser = serial.Serial('/dev/ttyACM0',9600)
time.sleep(2) 

blackUpper = (180, 255, 50)
BlackLower = (0, 0, 0) 

# Define source and destination points
src = np.float32([[ 353, 9 ], [ 545, 179 ], [ 244, 456 ], [ 58, 236 ] ])
dst = np.float32([[ 150, 0 ], [ 490, 0 ], [ 490, 480 ], [ 150, 480 ]]) 

pi = 3.1415
Z = 1
distX = 10.4
distY = 4
ref_pt = 2.4 # 140 degree
ref_pt4= 1.57 # 90 degree

### Define functions for homogeneous transform matrices of rotations about x, y, and z given specific angle.
def rot_x(q):
    R_x = Matrix([[1, 0,      0       ],
                  [0, cos(q), -sin(q) ],
                  [0, sin(q), cos(q) ]])

    return R_x

def rot_y(q):
    R_y = Matrix([[cos(q),  0, sin(q)],
                  [0,       1,      0],
                  [-sin(q), 0, cos(q)]])
    return R_y

def rot_z(q):
    R_z = Matrix([[cos(q), -sin(q), 0],
                  [sin(q), cos(q),  0],
                  [0,      0,       1]])
    return R_z


### Creates Homogeneous Transform Matrix
def tf_matrix(q, d, a, alpha):
    T = Matrix([[            cos(q),           -sin(q),           0,             a],
                [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                 0,                 0,           0,             1]])
    return T

# Joint angle (variable for revolute)
q1, q2, q3, q4, q5 = symbols('q1:6')
# Link offset (variable for prismatic)
d1, d2, d3, d4, d5 = symbols('d1:6')
# Link length
a0, a1, a2, a3, a4 = symbols('a0:5')
# Twist angles
alpha0,alpha1, alpha2, alpha3, alpha4 = symbols('alpha0:5')

# Create Modified DH parameters
DH = {alpha0:     0.0,  a0:    0.0,  d1:      4,  q1:        q1,
      alpha1: -pi/2.0,  a1:    3.8,  d2:    0.0,  q2: 	q2-pi/2,
      alpha2:     0.0,  a2:   17.6,  d3:    0.0,  q3:        q3,
      alpha3:     0.0,  a3:   17.3,  d4:    0.0,  q4:        q4,
      alpha4: -pi/2.0,  a4:    0.0,  d5:      6,  q5:       0.0}

# Create individual transformation matrices
T0_1  = tf_matrix(q1, d1, a0, alpha0).subs(DH)
T1_2  = tf_matrix(q2, d2, a1, alpha1).subs(DH)
T2_3  = tf_matrix(q3, d3, a2, alpha2).subs(DH)
T3_4  = tf_matrix(q4, d4, a3, alpha3).subs(DH)
T4_EE = tf_matrix(q5, d5, a4, alpha4).subs(DH)

# Generalized homogeneous transform
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_EE

def coor():

	while(cap.isOpened()):

	    ret, frame = cap.read()

	    # Get transform matrix using cv2.getPerspectivTransform()
	    M = cv2.getPerspectiveTransform(src, dst)
	    # Warp image using cv2.warpPerspective()
	    # keep same size as input image
	    warped = cv2.warpPerspective(frame, M, (640, 480))
	    warped = warped[:, 150:490, :]

	    if ret==True:
		
		x1 = int(np.size(warped,0)/2)
		y1 = int(np.size(warped,1)/2)
		blur = cv2.GaussianBlur(warped,(9,9),0)
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
		        cv2.drawContours(warped,c,-1,(0,255,255),2)
		        center.append((int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])))

			convX = (29.3 * center[i][0] / 340) + distX
			convY = (42 * center[i][1] / 480) + distY
		        coor = (convX, convY)

			cv2.circle(warped, center[i], 5, (0, 0, 0), -1)
		        cv2.putText(warped,'Center:'+str(coor),(20,50+(i*30)),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255, 255, 255), lineType=cv2.LINE_AA)

			# Theta 1 from above 
			theta1 = (atan2(convY, convX)).evalf(subs={'pi':3.14})
			

			# Theta 2 from side
			r1 = sqrt(convX**2+ convY**2) - DH[a1] - DH[d5]
			r2 = Z - DH[d1]
			r3 = sqrt(r1**2+ r2**2)
			phi2 = atan2(r2, r1)
			phi1 = acos( (DH[a3]**2 - DH[a2]**2 - r3**2)/ (-2 * DH[a2] * r3) )

			theta2 =  ref_pt - ( phi1 + phi2 ).evalf(subs={'pi':3.14}) 

			# Theta 3 from side
			phi3 = acos( (r3**2 - DH[a2]**2 - DH[a3]**2)/ (-2 * DH[a2] * DH[a3]) )

			theta3 = phi3.evalf(subs={'pi':3.14}) 
			

			# Calculating Euler angles from orientation
			Rrpy = np.array([[ 0.0, -1.0, 0.0],
					[ 0.0, 0.0, 1.0],
					[ -1.0, 0.0, 0.0]])

			R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
			R0_3 = R0_3.evalf(subs={'q1':theta1, 'q2':theta2, 'q3':theta3})
			R3_4 = R0_3.T * Rrpy
	
			theta4 = (acos(R3_4[0, 0])).evalf(subs={'pi':3.14})
			theta4 = ref_pt4 + theta4
			
			print('-----')
			print(theta1 * 180/pi)
			print(theta2 * 180/pi)
			print(theta3 * 180/pi)
			print(theta4 * 180/pi)

		cv2.imshow('Perspective Transform', warped)
		if cv2.waitKey(1) & 0xFF == ord('k'):
		    
				angle1 = str(theta1 * 180/pi) 
				angle2 = str(theta2 * 180/pi) 
				angle3 = str(theta3 * 180/pi) 
				angle4 = str(theta4 * 180/pi) 
				totAngle = '<' + angle1 + ',' + angle2 + ',' + angle3 + ',' + angle4 + '>'

				ser.write(totAngle.encode('utf-8'))
				time.sleep(2) 
				ser.close()
		
	    else:
		break

	# Release everything if job is finished
	cap.release()
	cv2.destroyAllWindows()		        

if __name__ == "__main__":
	coor()


