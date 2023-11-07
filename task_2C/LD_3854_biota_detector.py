#!/usr/bin/env python3

'''

This python file runs a ROS-node of name drone_control which holds the position of Swift-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''
'''
# Team ID:          3854
# Theme:            LD
# Author List:      Pushkar Gupta , Nibesh sahu , Nobel Das
# Filename:         	LD_3854_waypoint_navigation.py
# Functions:        < Comma separated list of functions in this file >
# Global variables: < List of global variables defined in this file, None if no global variables >
'''


# Importing the required libraries


from swift_msgs.msg import *
from typing import *
from swift_msgs.msg import swift_msgs
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
from luminosity_drone.msg import Biolocation
from std_msgs.msg import String
import rospy
import time
from imutils import contours
from skimage import measure
import numpy as np
import imutils
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge

class swift():

	"""docstring for swift"""
	def __init__(self):

		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	
		
		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [0,0,25] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
		self.a=0
		self.find=0;
		self.count=0
		self.organism_type = "nothing"
		self.controll=0
		self.b=0;
		self.publish=0;
		self.biolocation_msg = Biolocation()
		#Declaring a cmd of message type swift_msgs and initializing values
		self.cmd = swift_msgs()
		self.cmd.rcRoll = 0
		self.cmd.rcPitch = 0
		self.cmd.rcYaw = 0
		self.cmd.rcThrottle = 0
		self.cmd.rcAUX1 = 0
		self.cmd.rcAUX2 = 0
		self.cmd.rcAUX3 = 0
		self.cmd.rcAUX4 = 0


		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters

		self.Kp = [0, 0, 0]
		self.Ki = [0, 0, 0]
		self.Kd = [0, 0, 0]
		self.error = [0, 0, 0]
		
   
		#-----------------------Add other required variables for pid here ----------------------------------------------
		self.alt_error = [0.0,0.0,0.0]
		self.prev_alt_error = [0.0,0.0,0.0]
		self.sum_alt_error = [0.0,0.0,0.0]
		self.min_throttle = 1000
		self.max_throttle = 2000
     #error value for PID parameters








		# Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		# self.sample_time = 0.033 # in seconds


		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		print("pub")
		self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)

		self.alt_error_pub = rospy.Publisher('/alt_error',Float64, queue_size=1)
		self.image_pub = rospy.Publisher('/drone_display/image_view/output', Image, queue_size=1)
		self.astro_pub = rospy.Publisher('astrobiolocation', Biolocation, queue_size=10)
		self.bridge = CvBridge()



    



        
		#------------------------Add other ROS Publishers here-----------------------------------------------------






	#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		
		self.camera_sub = rospy.Subscriber('/swift/camera_rgb/image_raw', Image, self.displayimage)
		#self.upper_sub =  rospy.Subscriber('/whycon/image_out', Image, self.whyconimage)
		
		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE
		
	
	def displayimage(self, msg):
		   
		    
		    img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		    dim = (1200, 1200)
		    image = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
		    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		    blurred = cv2.GaussianBlur(gray, (11, 11), 0)
		    thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)[1]
		    thresh = cv2.erode(thresh, None, iterations=1)
		    thresh = cv2.dilate(thresh, None, iterations=2)
		    #cv2.imshow("myimg", thresh)
		    _, binary = cv2.threshold(thresh, 128, 255, cv2.THRESH_BINARY)

		    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		    min_object_area = 1
		    
		    object_positions = []
		    object_centroids = []
		    area=0
		    self.count=0
		    for contour in contours:
		    	area = cv2.contourArea(contour)
		    	#print(area)
		 	
		    	if min_object_area <= area < 2000:
		    		self.count += 1
		    		M = cv2.moments(contour)
		    		if M["m00"] != 0:
		    			centroid_x = int(M["m10"] / M["m00"])
		    			centroid_y = int(M["m01"] / M["m00"])
		    			object_positions.append((centroid_x, centroid_y))
		    			object_centroids.append((centroid_x, centroid_y))

				    

		    if object_centroids and self.controll==0 and self.publish==0:
		    	
		    	self.find=1;
		    	common_centroid_x = int(np.mean([centroid[0] for centroid in object_centroids]))
		    	common_centroid_y = int(np.mean([centroid[1] for centroid in object_centroids]))
		    	x_pos = common_centroid_x - 600
		    	y_pos = common_centroid_y - 600
		    	if self.controll==0:
			    	self.setpoint[0] = self.drone_position[0] +x_pos/500
			    	self.setpoint[1] = self.drone_position[1] +y_pos/500
			    	self.setpoint[2] = 25
			    	
			    	#print(self.drone_position)
		    	if self.find==1 and self.controll==0 and abs(self.alt_error[2])<0.5 and abs(self.alt_error[0])<0.3 and abs(self.alt_error[1])<0.3:
		    		self.setpoint[0] = self.drone_position[0] +x_pos/500
			    	self.setpoint[1] = self.drone_position[1] +y_pos/500
			    	self.setpoint[2] = 25
			    	self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			    	self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			    	self.alt_error[1] = self.setpoint[1] - self.drone_position[1]

		    	print(f"Number of small objects: {self.count}")
		    	#print(self.setpoint)
		    	print(x_pos / 200, y_pos / 200)
		    	if abs(x_pos/200) <0.35 and abs(y_pos/200)<0.35:
		    		print("published")
		    		self.find=0
		    		
		    		self.b=self.a
		    		self.publish=1
		    		print(self.biolocation_msg)
		    		self.astro_pub.publish(self.biolocation_msg)
		    
		    cv2.drawContours(image, contours, -1, (0, 255, 0), 2)
		    
		    #cv2.imshow('Image with Contours', image)
		    cv2.waitKey(1)

	# Disarming condition of the drone
	
	
	
	
	
	
	
	
	
	
	def disarm(self):
	
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
	
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z

		num_leds = self.count
		if num_leds == 2:
			self.organism_type = "alien_a"
		elif num_leds == 3:
			self.organism_type = "alien_b"
		elif num_leds == 4:
			self.organism_type = "alien_c"
		
		self.biolocation_msg.organism_type = self.organism_type
		self.biolocation_msg.whycon_x = msg.poses[0].position.x
		self.biolocation_msg.whycon_y = msg.poses[0].position.y
		self.biolocation_msg.whycon_z = 37.5


	
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		print("lauda")
		#self.Kp[2] = alt.Kp *0.06
		self.Kp[2] =120.42
		
		#* 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = 0
		self.Kd[2] = 896.7
		
	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------
	def roll_set_pid(self,alt):
		print("lauda")
		self.Kp[0] = 61.62
		#* 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[0] = 0.024
		self.Kd[0] = 524.4
	def pitch_set_pid(self,alt):
		print("lauda")
		self.Kp[1] = 39.18
		#* 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[1] = 0.008
		self.Kd[1] = 406.2











	#----------------------------------------------------------------------------------------------------------------------

	def pid(self):
	 #-----------------------------Write the PID algorithm here--------------------------------------------------------------

	 # Steps:
	 # 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	 #	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	 #	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	 #	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	 #	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	 #	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	 #																														self.cmd.rcPitch = self.max_values[1]
	 #	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	 #	8. Add error_sum
	 
	 
		
		
		
		self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
		self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
		self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
		
		
		err2=1.0
		err0=1.0
		err1=1.0
		print(self.setpoint)
		print(self.drone_position)
		if self.controll==0:
			self.Kp[1] = 35.18
			self.Ki[1] = 0.008
			self.Kd[1] = 506.2
			self.Kp[0] = 41.62
			self.Ki[0] = 0.024
			self.Kd[0] = 624.4
			self.Kp[2] =120.42
			self.Ki[2] = 0
			self.Kd[2] = 1976.7
		if self.a>self.b+3:
			self.publish=0
		if self.controll==1:
			self.Kp[1] = 39.18
			self.Ki[1] = 0.008
			self.Kd[1] = 406.2
			self.Kp[0] = 31.62
			self.Ki[0] = 0.024
			self.Kd[0] = 324.4
			self.Kp[2] =40.42
			self.Ki[2] = 0
			self.Kd[2] = 476.7
		if self.controll<2:
			self.cmd.rcThrottle = int(1590 - self.Kp[2] * self.alt_error[2] - self.Ki[2] * self.sum_alt_error[2] - self.Kd[2] * (self.alt_error[2] - self.prev_alt_error[2]))
			self.cmd.rcPitch = int(1500 - self.Kp[1] * self.alt_error[1] - self.Ki[1] * self.sum_alt_error[1] - self.Kd[1] * (self.alt_error[1] - self.prev_alt_error[1]))
			self.cmd.rcRoll = int(1500 + self.Kp[0] * self.alt_error[0] + self.Ki[0] * self.sum_alt_error[0] + self.Kd[0] * (self.alt_error[0] - self.prev_alt_error[0]))
			
			
		if self.find==0 and self.a==0 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [3,0,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
			
		if self.find==0 and self.a==1 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [3,3,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		
		if self.find==0 and self.a==2 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [0,3,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
			
		if self.find==0 and self.a==3 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-3,3,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==4 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-3,0,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==5 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-3,-3,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==6 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [0,-3,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==7 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [3,-3,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==8 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [7,-3,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==9 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [7,0,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		
		if self.find==0 and self.a==10 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [7,3,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			self.controll=0
			print(self.a)
			
		if self.find==0 and self.a==11 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [7,7,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			self.controll=0
			print(self.a)
		if self.find==0 and self.a==12 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [3,7,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			self.controll=0
			print(self.a)
		if self.find==0 and self.a==13 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [0,7,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			self.controll=0
			print(self.a)
		if self.find==0 and self.a==14 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-3,7,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			self.controll=0
			print(self.a)
		if self.find==0 and self.a==15 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-7,7,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			self.controll=0
			print(self.a)
		if self.find==0 and self.a==16 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-7,7,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			self.controll=0
			print(self.a)
		if self.find==0 and self.a==17 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-7,0,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			self.controll=0
			print(self.a)
			
		if self.find==0 and self.a==18 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-7,-3,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			self.controll=0
			print(self.a)
		
		if self.find==0 and self.a==19 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-7,-7,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			self.controll=0
			print(self.a)
			
		if self.find==0 and self.a==20 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-3,-7,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			self.controll=0
			print(self.a)
		if self.find==0 and self.a==21 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [0,-7,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			self.controll=0
			print(self.a)
		if self.find==0 and self.a==22 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [3,-7,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			self.controll=0
			print(self.a)
		if self.find==0 and self.a==23 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [7,-7,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			self.controll==0
			print(self.a)
		if self.a==24 and abs(self.alt_error[2])<1 and abs(self.alt_error[0])<0.5 and abs(self.alt_error[1])<0.5:
			self.controll=1
			self.setpoint[2]=35
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			
		if self.a==25 and abs(self.alt_error[2])<0.5 and abs(self.alt_error[0])<0.2 and abs(self.alt_error[1])<0.2:
			self.setpoint[0] = 8
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			
		if self.a==26 and abs(self.alt_error[2])<0.5 and abs(self.alt_error[0])<0.2 and abs(self.alt_error[1])<0.2:
			self.setpoint = [11,11,35]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
		if self.a==27 and abs(self.alt_error[2])<0.5 and abs(self.alt_error[0])<0.2 and abs(self.alt_error[1])<0.2:
			self.setpoint = [11,11,37.5]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
		if self.a==28 and abs(self.alt_error[2])<0.55 and abs(self.alt_error[0])<0.19 and abs(self.alt_error[1])<0.19:
			print("Yeah......")
			self.controll=2
			self.cmd.rcThrottle=0
			self.cmd.rcPitch=0
			self.cmd.rcRoll=0
			






		#self.cmd.rcThrottle = int(self.Kp[2])
		if self.cmd.rcThrottle > 2000:
			self.cmd.rcThrottle = 2000
		if self.cmd.rcThrottle <1000:
			self.cmd.rcThrottle = 1000
		if self.cmd.rcRoll > 2000:
			self.cmd.rcRoll = 2000
		if self.cmd.rcRoll <1000:
			self.cmd.rcRoll = 1000
		if self.cmd.rcPitch > 2000:
			self.cmd.rcPitch = 2000
		if self.cmd.rcPitch <1000:
			self.cmd.rcPitch = 1000
		# print(self.cmd.rcRoll)
		# print(self.cmd.rcPitch)
		# print(self.cmd.rcThrottle)
		#print(self.alt_error[2])
		# print(self.prev_alt_error[2])
		# print(self.drone_position[2])
		
		
		self.prev_alt_error[0] = self.alt_error[0]
		self.prev_alt_error[1] = self.alt_error[1]
		self.prev_alt_error[2] = self.alt_error[2]


		self.sum_alt_error[2] = self.sum_alt_error[2] + self.alt_error[2]
		self.sum_alt_error[0] = self.sum_alt_error[0] + self.alt_error[0]
		self.sum_alt_error[1] = self.sum_alt_error[1] + self.alt_error[1]















	 #------------------------------------------------------------------------------------------------------------------------
		self.command_pub.publish(self.cmd)
		self.alt_error_pub.publish(self.alt_error[2])
		







if __name__ == '__main__':

	swift_drone = swift()
	
	r = rospy.Rate(30) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		swift_drone.pid()

		r.sleep()
