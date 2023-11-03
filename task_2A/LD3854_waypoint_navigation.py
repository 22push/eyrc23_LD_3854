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
import rospy
import time


class swift():

	"""docstring for swift"""
	def __init__(self):

		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [0,0,23] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
		self.a=0

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



    



        
		#------------------------Add other ROS Publishers here-----------------------------------------------------






	#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)


		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z


	
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
	 
	 
		self.Kp[1] = 39.18
		self.Ki[1] = 0.008
		self.Kd[1] = 406.2
		self.Kp[0] = 61.62
		self.Ki[0] = 0.024
		self.Kd[0] = 524.4
		self.Kp[2] =120.42
		self.Ki[2] = 0
		self.Kd[2] = 1976.7
		
		
		self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
		self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
		self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
		
		self.cmd.rcThrottle = int(1590 - self.Kp[2] * self.alt_error[2] - self.Ki[2] * self.sum_alt_error[2] - self.Kd[2] * (self.alt_error[2] - self.prev_alt_error[2]))
		self.cmd.rcPitch = int(1500 - self.Kp[1] * self.alt_error[1] - self.Ki[1] * self.sum_alt_error[1] - self.Kd[1] * (self.alt_error[1] - self.prev_alt_error[1]))
		self.cmd.rcRoll = int(1500 + self.Kp[0] * self.alt_error[0] + self.Ki[0] * self.sum_alt_error[0] + self.Kd[0] * (self.alt_error[0] - self.prev_alt_error[0]))
		if self.a==0 and abs(self.alt_error[2])<0.1 and abs(self.alt_error[0])<0.2 and abs(self.alt_error[1])<0.2:
			self.setpoint = [2,0,23]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
			
		if self.a==1 and abs(self.alt_error[2])<0.1 and abs(self.alt_error[0])<0.2 and abs(self.alt_error[1])<0.2:
			self.setpoint = [2,2,23]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		
		if self.a==2 and abs(self.alt_error[2])<0.1 and abs(self.alt_error[0])<0.2 and abs(self.alt_error[1])<0.2:
			self.setpoint = [2,2,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
			
		if self.a==3 and abs(self.alt_error[2])<0.1 and abs(self.alt_error[0])<0.2 and abs(self.alt_error[1])<0.2:
			self.setpoint = [-5,2,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.a==4 and abs(self.alt_error[2])<0.1 and abs(self.alt_error[0])<0.2 and abs(self.alt_error[1])<0.2:
			self.setpoint = [-5,-3,25]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.a==5 and abs(self.alt_error[2])<0.1 and abs(self.alt_error[0])<0.2 and abs(self.alt_error[1])<0.2:
			self.setpoint = [-5,-3,21]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.a==6 and abs(self.alt_error[2])<0.1 and abs(self.alt_error[0])<0.2 and abs(self.alt_error[1])<0.2:
			self.setpoint = [7,-3,21]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.a==7 and abs(self.alt_error[2])<0.1 and abs(self.alt_error[0])<0.2 and abs(self.alt_error[1])<0.2:
			self.setpoint = [7,0,21]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)
		if self.a==8 and abs(self.alt_error[2])<0.1 and abs(self.alt_error[0])<0.2 and abs(self.alt_error[1])<0.2:
			self.setpoint = [0,0,19]
			self.alt_error[2] = self.setpoint[2] - self.drone_position[2]
			self.alt_error[0] = self.setpoint[0] - self.drone_position[0]
			self.alt_error[1] = self.setpoint[1] - self.drone_position[1]
			self.a=self.a+1
			print(self.a)









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
