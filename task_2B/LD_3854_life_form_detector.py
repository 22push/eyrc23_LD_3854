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
# Filename:         	LD_3854_life_form_detector.py
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

		
		rospy.init_node('drone_control')	

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
		    cv2.imshow("myimg", thresh)
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

				    

		    if object_centroids and self.controll==0:
		    	self.find=1;
		    	common_centroid_x = int(np.mean([centroid[0] for centroid in object_centroids]))
		    	common_centroid_y = int(np.mean([centroid[1] for centroid in object_centroids]))
		    	x_pos = common_centroid_x - 600
		    	y_pos = common_centroid_y - 600
		    	if self.controll==0:
			    	self.setpoint[0] = self.drone_position[0] +x_pos/500
			    	self.setpoint[1] = self.drone_position[1] +y_pos/500
			    	self.setpoint[2] = 25
			    	
			    	print(self.drone_position)
		    	if self.find==1 and self.controll==0 and abs(self.alt_error[2])<1 and abs(self.alt_error[0])<1 and abs(self.alt_error[1])<1:
		    		self.setpoint[0] = self.drone_position[0] +x_pos/500
			    	self.setpoint[1] = self.drone_position[1] +y_pos/500
			    	self.setpoint[2] = 25
			    	self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			    	self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			    	self.alt_error[1] = self.setpoint[1] - self.drone_position[1]

		    	print(f"Number of small objects: {self.count}")
		    	print(self.setpoint)
		    	for i, position in enumerate(object_positions):
		    		print(f"Position of object {i + 1}: {position}")
		    		print(x_pos / 200, y_pos / 200)
		    	if abs(x_pos/200) <0.3 and abs(y_pos/200)<0.3:
		    		print("published")
		    		self.find=2
		    		self.controll=1
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
		self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
		self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
		self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
		
		
		err2=1.5
		err0=1.5
		err1=1.5
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
			self.setpoint = [2,0,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
			
		if self.find==0 and self.a==1 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [2,2,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		
		if self.find==0 and self.a==2 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [0,2,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
			
		if self.find==0 and self.a==3 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-2,2,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==4 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-2,0,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==5 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-2,-2,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==6 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [0,-2,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==7 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [2,-2,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==8 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [4,-2,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==9 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [4,0,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		
		if self.find==0 and self.a==10 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [4,2,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
			
		if self.find==0 and self.a==11 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [4,4,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==12 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [2,4,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==13 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [0,4,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==14 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-2,4,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==15 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-4,4,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==16 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-4,2,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==17 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-4,0,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
			
		if self.find==0 and self.a==18 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-4,-2,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		
		if self.find==0 and self.a==19 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-4,-4,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
			
		if self.find==0 and self.a==20 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-2,-4,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==21 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [0,-4,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==22 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [2,-4,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==23 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [4,-4,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==24 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [6,-4,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==25 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [6,-2,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==26 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [6,0,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		
		if self.find==0 and self.a==27 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [6,2,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
			
		if self.find==0 and self.a==28 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [6,4,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==29 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [6,6,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==30 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [4,6,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==31 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [2,6,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==32 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [0,6,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==33 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-2,6,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==34 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-4,6,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==35 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-6,6,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==36 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-6,4,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		
		if self.find==0 and self.a==37 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-6,2,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
			
		if self.find==0 and self.a==38 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-6,0,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==39 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-6,-2,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==40 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-6,-4,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==41 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-6,-6,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==42 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-4,-6,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==43 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-2,-6,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		
		if self.find==0 and self.a==44 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [0,-6,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==45 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [2,-6,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==46 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [2,-6,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		
		if  self.find==0 and self.a==47 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [4,-6,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if  self.find==0 and self.a==48 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [6,-6,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==49 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [7.5,-6,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==50 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [7.5,-6,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==51 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [7.5,-4,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==52 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [7.5,-2,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==53 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [7.5,0,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==54 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [7.5,2,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==55 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [7.5,4,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==56 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [7.5,6,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==57 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [7.5,7.5,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		
		if self.find==0 and self.a==58 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [6,7.5,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==59 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [4,7.5,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==60 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [2,7.5,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==61 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [0,7.5,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==62 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-2,7.5,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==63 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-4,7.5,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==64 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-6,7.5,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==65 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-7.5,7.5,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==66 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-7.5,6,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==67 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-7.5,4,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==68 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-7.5,2,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==69 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-7.5,0,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==70 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-7.5,-2,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==71 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-7.5,-4,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==72 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-7.5,-6,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==73 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-7.5,-7.5,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==74 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-6,-7.5,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==75 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-4,-7.5,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==76 and abs(self.alt_error[2])<err2 and abs(self.alt_error[0])<err0 and abs(self.alt_error[1])<err1:
			self.setpoint = [-2,-7.5,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==77 and abs(self.alt_error[2])<0.8 and abs(self.alt_error[0])<0.6 and abs(self.alt_error[1])<0.6:
			self.setpoint = [0,-7.5,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==78 and abs(self.alt_error[2])<0.8 and abs(self.alt_error[0])<0.6 and abs(self.alt_error[1])<0.6:
			self.setpoint = [2,-7.5,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==79 and abs(self.alt_error[2])<0.8 and abs(self.alt_error[0])<0.6 and abs(self.alt_error[1])<0.6:
			self.setpoint = [4,-7.5,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==80 and abs(self.alt_error[2])<0.8 and abs(self.alt_error[0])<0.6 and abs(self.alt_error[1])<0.6:
			self.setpoint = [6,-7.5,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==0 and self.a==81 and abs(self.alt_error[2])<0.8 and abs(self.alt_error[0])<0.6 and abs(self.alt_error[1])<0.6:
			self.setpoint = [7.5,-7.5,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.find==2 and abs(self.alt_error[2])<0.5 and abs(self.alt_error[0])<0.2 and abs(self.alt_error[1])<0.2:
			self.setpoint[2]=35
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.find=self.find+1
			
		if self.find==3 and abs(self.alt_error[2])<0.5 and abs(self.alt_error[0])<0.2 and abs(self.alt_error[1])<0.2:
			self.setpoint[0] = 11
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.find=self.find+1
			
		if self.find==4 and abs(self.alt_error[2])<0.5 and abs(self.alt_error[0])<0.2 and abs(self.alt_error[1])<0.2:
			self.setpoint = [11,11,35]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.find=self.find+1
		if self.find==5 and abs(self.alt_error[2])<0.5 and abs(self.alt_error[0])<0.2 and abs(self.alt_error[1])<0.2:
			self.setpoint = [11,11,37.5]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.find=self.find+1
		if self.find == 6 and abs(self.alt_error[2])<0.5 and abs(self.alt_error[0])<0.15 and abs(self.alt_error[1])<0.15:
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
