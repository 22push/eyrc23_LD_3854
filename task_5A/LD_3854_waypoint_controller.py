#!/usr/bin/env python3

"""
Controller for the drone
"""

# standard imports
import copy
import time

# third-party imports
import scipy.signal
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PidTune
from swift_msgs.msg import PIDError, RCMessage
from swift_msgs.srv import CommandBool


print("-------------------------------------------------------------------------------")
MIN_ROLL = 1350
BASE_ROLL = 1500
MAX_ROLL = 1600
SUM_ERROR_ROLL_LIMIT = 10000
MAX_THROTAL =  1800;
MIN_ThROTAL = 1000;

MAX_PITCH =  1600;
MIN_PITCH = 1350;
DRONE_WHYCON_POSE = [[], [], []]

# Similarly, create upper and lower limits, base value, and max sum error values for roll and pitch

class DroneController():
    def __init__(self,node):
        self.node= node
        self.rate=0.005
        self.decthrottle=10
        self.duration=15
        self.time= time.time()
        self.rc_message = RCMessage()
        self.drone_whycon_pose_array = PoseArray()
        self.last_whycon_pose_received_at = 0
        self.commandbool = CommandBool.Request()
        service_endpoint = "/swift/cmd/arming"
        self.count=0
        self.level = 0
        self.arming_service_client = self.node.create_client(CommandBool,service_endpoint)
        self.set_points = [0, 0, 24]         # Setpoints for x, y, z respectively      
        self.set_des_points = [[0,0,23],[2,3,23],[-1,2,25],[-3,-3,25],[0,0,27]]
        self.error = [0, 0, 0]         # Error for roll, pitch and throttle        

        # Create variables for integral and differential error
        self.integral = [0,0,0]
        self.derivative = [0,0,0]

        # Create variables for previous error and sum_error
        self.previous_error=[0,0,0]
        self.sum_error=[0,0,0]

        self.Kp = [407 * 0.01, 373 * 0.01, 250 * 0.01]
        self.Ki = [57 * 0.0002, 57 * 0.0002, 401 * 0.0001]
        self.Kd = [200, 211.9, 168.1]
        # Similarly create variables for Kd and Ki

        # Create subscriber for WhyCon 
        
        self.whycon_sub = node.create_subscription(PoseArray,"/whycon/poses",self.whycon_poses_callback,1)
        
        # Similarly create subscribers for pid_tuning_altitude, pid_tuning_roll, pid_tuning_pitch and any other subscriber if required
       
        self.pid_alt = node.create_subscription(PidTune,"/pid_tuning_altitude",self.pid_tune_throttle_callback,1)

        # Create publisher for sending commands to drone 

        self.rc_pub = node.create_publisher(RCMessage, "/swift/rc_command",1)

        # Create publisher for publishing errors for plotting in plotjuggler 
        
        self.pid_error_pub = node.create_publisher(PIDError, "/luminosity_drone/pid_error",1)        


    def whycon_poses_callback(self, msg):
        self.last_whycon_pose_received_at=0.1
        #self.last_whycon_pose_received_at = self.node.get_clock().now().seconds_nanoseconds()[0]
        self.drone_whycon_pose_array = msg


    def pid_tune_throttle_callback(self, msg):
        self.Kp[2] = msg.kp * 0.05
        self.Ki[2] = msg.ki * 0.0001
        self.Kd[2] = msg.kd * 0.1

    # Similarly add callbacks for other subscribers


    def pid(self):          # PID algorithm

        # 0 : calculating Error, Derivative, Integral for Roll error : x axis
        
        try:
            if(abs(self.drone_whycon_pose_array.poses[0].position.z-self.set_des_points[self.count][2])<0.6 and abs(self.drone_whycon_pose_array.poses[0].position.y-self.set_des_points[self.count][1])<0.6 and abs(self.drone_whycon_pose_array.poses[0].position.x-self.set_des_points[self.count][0])<0.6 and self.count<4):
                self.count+=1
                self.node.get_logger().info(str("---------------------change-------------------------------"))
            self.set_points[2]=self.set_points[2] + (self.set_des_points[self.count][2]-self.set_points[2])*self.rate
            self.set_points[1]=self.set_points[1] + (self.set_des_points[self.count][1]-self.set_points[1])*self.rate
            self.set_points[0]=self.set_points[0] + (self.set_des_points[self.count][0]-self.set_points[0])*self.rate
            
            self.node.get_logger().info(str(self.set_des_points[self.count]))
            self.error[2] = self.drone_whycon_pose_array.poses[0].position.z - self.set_points[2]
            print(self.drone_whycon_pose_array.poses[0].position.x,self.drone_whycon_pose_array.poses[0].position.y,self.drone_whycon_pose_array.poses[0].position.z)
            self.derivative[2] = self.error[2]-self.previous_error[2]
            self.integral[2]= self.sum_error[2]
            self.error[1] = self.drone_whycon_pose_array.poses[0].position.y - self.set_points[1]
            self.derivative[1] = self.error[1]-self.previous_error[1]
            self.integral[1]= self.sum_error[1]
            self.error[0] = self.drone_whycon_pose_array.poses[0].position.x - self.set_points[0]
            self.derivative[0] = self.error[0]-self.previous_error[0]
            self.integral[0]= self.sum_error[0]
        # Similarly calculate error for y and z axes 
        
        except:
            pass

        # Calculate derivative and intergral errors. Apply anti windup on integral error (You can use your own method for anti windup, an example is shown here)

        # self.integral[0] = (self.integral[0] + self.error[0])
        # if self.integral[0] > SUM_ERROR_ROLL_LIMIT:
        #     self.integral[0] = SUM_ERROR_ROLL_LIMIT
        # if self.integral[0] < -SUM_ERROR_ROLL_LIMIT:
        #     self.integral[0] = -SUM_ERROR_ROLL_LIMIT

        # Save current error in previous error

        # 1 : calculating Error, Derivative, Integral for Pitch error : y axis

        # 2 : calculating Error, Derivative, Integral for Alt error : z axis


        # Write the PID equations and calculate the self.rc_message.rc_throttle, self.rc_message.rc_roll, self.rc_message.rc_pitch

        
    #------------------------------------------------------------------------------------------------------------------------


        self.previous_error[2]=self.error[2]
        self.sum_error[2]+=self.error[2]
        self.previous_error[1]=self.error[1]
        self.sum_error[1]+=self.error[1]
        self.previous_error[0]=self.error[0]
        self.sum_error[0]+=self.error[0]
        #self.rc_message.rc_throttle = int(self.Kp[2])
        self.rc_message.rc_throttle = int(
            1470
            + self.Kp[2] * self.error[2]
            + self.Kd[2] * self.derivative[2]
            + self.Ki[2] * self.integral[2]
        )
        # self.rc_message.rc_throttle = 1200
        self.rc_message.rc_pitch = int(
            1485
            + self.Kp[1] * self.error[1]
            + self.Kd[1] * self.derivative[1]
            + self.Ki[1] * self.integral[1]
        )
        self.rc_message.rc_roll = int(
            1505
            - self.Kp[0] * self.error[0]
            - self.Kd[0] * self.derivative[0]
            - self.Ki[0] * self.integral[0]
        )
        if(self.level == 1 or (self.count==4 and abs(self.drone_whycon_pose_array.poses[0].position.z-self.set_des_points[self.count][2])<0.7 and abs(self.drone_whycon_pose_array.poses[0].position.y-self.set_des_points[self.count][1])<0.7 and abs(self.drone_whycon_pose_array.poses[0].position.x-self.set_des_points[self.count][0])<0.7)):
            self.level=1
            self.rc_message.rc_throttle=1100
        # self.rc_message.rc_throttle=1300
        self.publish_data_to_rpi( roll = self.rc_message.rc_roll , pitch =self.rc_message.rc_pitch , throttle = self.rc_message.rc_throttle)
        

        #Replace the roll pitch and throttle values as calculated by PID 
        
        
        # Publish alt error, roll error, pitch error for plotjuggler debugging

        self.pid_error_pub.publish(
            PIDError(
                roll_error=float(self.error[0]),
                pitch_error=float(self.error[1]),
                throttle_error=float(self.error[2]),
                yaw_error=-0.0,
                zero_error=0.0,
            )
        )
        print("jyf")
        


    def publish_data_to_rpi(self, roll, pitch, throttle):

        self.rc_message.rc_throttle = int(throttle)
        self.rc_message.rc_roll = int(roll)
        self.rc_message.rc_pitch = int(pitch)

        # Send constant 1500 to rc_message.rc_yaw
        self.rc_message.rc_yaw = int(1500)
        
        # BUTTERWORTH FILTER
        span = 15
        for index, val in enumerate([roll, pitch, throttle]):
             DRONE_WHYCON_POSE[index].append(val)
             if len(DRONE_WHYCON_POSE[index]) == span:
                 DRONE_WHYCON_POSE[index].pop(0)
             if len(DRONE_WHYCON_POSE[index]) != span-1:
                 return
             order = 3
             fs = 60
             fc = 5
             nyq = 0.5 * fs
             wc = fc / nyq
             b, a = scipy.signal.butter(N=order, Wn=wc, btype='lowpass', analog=False, output='ba')
             filtered_signal = scipy.signal.lfilter(b, a, DRONE_WHYCON_POSE[index])
             if index == 0:
                 self.rc_message.rc_roll = int(filtered_signal[-1])
             elif index == 1:
                 self.rc_message.rc_pitch = int(filtered_signal[-1])
             elif index == 2:
                 self.rc_message.rc_throttle = int(filtered_signal[-1])

        if self.rc_message.rc_roll > MAX_ROLL:     #checking range i.e. bet 1000 and 2000
            self.rc_message.rc_roll = MAX_ROLL
        elif self.rc_message.rc_roll < MIN_ROLL:
            self.rc_message.rc_pitch = MIN_ROLL
        if self.rc_message.rc_pitch > MAX_PITCH:     #checking range i.e. bet 1000 and 2000
            self.rc_message.rc_pitch = MAX_PITCH
        elif self.rc_message.rc_pitch < MIN_PITCH:
            self.rc_message.rc_roll = MIN_PITCH
        if self.rc_message.rc_throttle > MAX_THROTAL:     #checking range i.e. bet 1000 and 2000
            self.rc_message.rc_throttle = MAX_THROTAL
        elif self.rc_message.rc_throttle < MIN_ThROTAL:
            self.rc_message.rc_throttle = MIN_ThROTAL
            
	
        # Similarly add bounds for pitch yaw and throttle 
        self.node.get_logger().info(str(self.rc_message))
        self.node.get_logger().info(str(self.set_points))
        self.rc_pub.publish(self.rc_message)


    # This function will be called as soon as this rosnode is terminated. So we disarm the drone as soon as we press CTRL + C. 
    # If anything goes wrong with the drone, immediately press CTRL + C so that the drone disamrs and motors stop 

    def shutdown_hook(self):
        self.node.get_logger().info("Calling shutdown hook")
        self.disarm()

    # Function to arm the drone 

    def arm(self):
        self.node.get_logger().info("Calling arm service")
        self.commandbool.value = True
        self.future = self.arming_service_client.call_async(self.commandbool)

    # Function to disarm the drone 

    def disarm(self):
        self.node.get_logger().info("Calling disarm service")
        self.commandbool.value = False
        self.future = self.arming_service_client.call_async(self.commandbool)


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('controller')
    node.get_logger().info(f"Node Stahrted")
    node.get_logger().info("Entering PID controller loop")

    controller = DroneController(node)
    controller.arm()
    controller.arm()
    node.get_logger().info("Armed")

    try:
        while rclpy.ok():
            controller.pid()
            if node.get_clock().now().to_msg().sec - controller.last_whycon_pose_received_at > 1:
                node.get_logger().error("Unable to detect WHYCON poses")
            rclpy.spin_once(node)
            print("hf") # Sleep for 1/30 secs
        

    except Exception as err:
        print(err)

    finally:
        controller.shutdown_hook()
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()