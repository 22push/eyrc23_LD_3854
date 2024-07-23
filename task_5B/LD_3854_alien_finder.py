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
# import rospy
import threading
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PidTune
from swift_msgs.msg import PIDError, RCMessage
from swift_msgs.srv import CommandBool
from sensor_msgs.msg import Image
from loc_msg.msg import Biolocation
import cv2
import matplotlib.pyplot as plt
from imutils import contours
from skimage import measure
import imutils
import asyncio
from rclpy.callback_groups import ReentrantCallbackGroup
print("-------------------------------------------------------------------------------")
MIN_ROLL = 1350
BASE_ROLL = 1500
MAX_ROLL = 1600
SUM_ERROR_ROLL_LIMIT = 10000
MAX_THROTAL = 1800
MIN_ThROTAL = 1000

MAX_PITCH = 1600
MIN_PITCH = 1350
DRONE_WHYCON_POSE = [[], [], []]

# Similarly, create upper and lower limits, base value, and max sum error values for roll and pitch
aliens = {
    2: 'alien_a',
    3: 'alien_b',
    4: 'alien_c',
    5: 'alien_d'
}

class DroneController:
    def __init__(self, node):
        
        self.node = node
        self.rate = 0.01
        self.decthrottle = 10
        self.duration = 10.1
        self.start_time = time.time()
        self.rc_message = RCMessage()
        self.drone_whycon_pose_array = PoseArray()
        self.last_whycon_pose_received_at = 0
        self.commandbool = CommandBool.Request()
        service_endpoint = "/swift/cmd/arming"
        self.count = 0
        self.level = 0
        self.ss=0
        self.alien=0
        self.down=0
        self.find=0
        self.back=0
        self.off =0
        self.approuch=0
        self.alien_pose=[]
        self.done=0
        self.clock = Clock()
        self.set_land_points=[9.3, 9.00, 25]
        self.arming_service_client = self.node.create_client(
            CommandBool, service_endpoint
        )
        self.set_points = [7.629, 7.594,26]
        self.error = [0, 0, 0]
        x=23
        self.set_des_points =[[7.486, 7.575, x], [3.846, 7.6, x], [-0.066, 7.58, x], [-3.843, 7.544, x], [-7.589, 7.538, x], [-7.657, 3.686, x], [-3.789, 3.659, x], [-0.021, 3.647, x], [3.704, 3.454, x], [7.776, 3.819, x], [7.783, -0.278, x], [3.708, -0.186, x], [-0.008, -0.158, x], [-3.853, -0.194, x], [-7.673, -0.113, x], [-7.876, -4.074, x], [-3.884, -4.041, x], [0.021, -4.007, x], [3.802, -3.969, x], [7.668, -3.963, x], [7.835, -7.902, x], [3.501, -7.832, x], [0.007, -8.034, x], [-3.854, -7.956, x], [-7.788, -7.978, x]]
        self.integral = [0, 0, 0]

        self.derivative = [0, 0, 0]
        self.previous_error = [0, 0, 0]
        self.sum_error = [0, 0, 0]

        self.Kp = [407 * 0.01, 373 * 0.01, 250 * 0.01]
        self.Ki = [57 * 0.0002, 57 * 0.0002, 401 * 0.0001]
        self.Kd = [200, 211.9, 168.1]

        self.whycon_sub = node.create_subscription(
            PoseArray, "/whycon/poses", self.whycon_poses_callback, 1
        )

        self.pid_alt = node.create_subscription(
            PidTune, "/pid_tuning_altitude", self.pid_tune_throttle_callback, 1
        )
        self.callback_group = ReentrantCallbackGroup()
        self.image = node.create_subscription(Image, "/video_frames", self.image_callback, 1, callback_group=self.callback_group)

        # self.image = node.create_subscription(Image,"/video_frames",self.image_callback,1)

        self.rc_pub = node.create_publisher(RCMessage, "/swift/rc_command", 1)
        self.alien_pub = node.create_publisher(Biolocation,'/astrobiolocation',10)

        # Create publisher for publishing errors for plotting in plotjuggler
        self.biolocation_msg = Biolocation()
        self.pid_error_pub = node.create_publisher(
            PIDError, "/luminosity_drone/pid_error", 1
        )

    def leds_cropped(self,image, mask):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (15,15), 0)
        thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.erode(thresh, None, iterations=2)
        thresh = cv2.dilate(thresh, None, iterations=4)
        labels = measure.label(thresh, connectivity=2, background=0)

        for label in np.unique(labels):
            if label == 0:
                continue
            labelMask = np.zeros(thresh.shape, dtype="uint8")
            labelMask[labels == label] = 255
            numPixels = cv2.countNonZero(labelMask)
            if numPixels > 10:
                mask = cv2.add(mask, labelMask)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        cv2.drawContours(image, cnts, -1, (0, 0, 255), 2)
        cv2.imshow("df", image)

        centroids = []
        areas = []
        cx=0
        cy=0
        for (i, c) in enumerate(cnts):
            x,y = c[0][0]
            cv2.putText(image, str(i + 1), (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            area = cv2.contourArea(c)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cX = M["m10"] / M["m00"]
                cY = M["m01"] / M["m00"]
            else:
                cX, cY = 0, 0
            centroids.append((cX, cY))
            cx = cx+cX
            cy=cy+cY
            areas.append(area)
        
        return len(np.unique(labels)),cx/(len(np.unique(labels))-1),cy/(len(np.unique(labels))-1)
    def count_leds_in_mask(self,mask, original_image):
        image_h, image_w, _ = original_image.shape

        countours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        countours = imutils.grab_contours(countours)

        led_count = len(countours)

        # print(f"Number of LEDs in the given mask: {led_count}")

        for i,led_contour in enumerate(countours):
            mask_led = np.zeros((image_h, image_w), dtype=np.uint8)
            cv2.drawContours(mask_led, [led_contour], 0, (255), thickness=cv2.FILLED)

            cropped_region = cv2.bitwise_and(original_image, original_image, mask=mask_led)
            numled_onep,centroid_x,centroid_y=(self.leds_cropped(cropped_region, np.zeros(mask.shape, dtype="uint8")))
            print((centroid_x-320)/2.76,(centroid_y-240)/2.76,i,"alien",numled_onep-1)
            self.alien=numled_onep-1
            if(self.back==0):
                self.approuch=1
                self.set_points[0]=self.set_points[0]+(centroid_x-320)/300
                self.set_points[1]=self.set_points[1]+(centroid_y-240)/300
                # self.node.get_logger().info(str(self.set_points[1]))
            if(abs(centroid_x-320)<100 and abs(centroid_y-240)<30):
                try:
                    self.alien_pose=[self.drone_whycon_pose_array.poses[0].position.x,self.drone_whycon_pose_array.poses[0].position.y,self.drone_whycon_pose_array.poses[0].position.z]
                    self.back=1
                except:
                    pass
            
                
            # write_to_txt_file('abcd.txt',aliens[numled_onep-1],centroid_x,centroid_y)
            cv2.imshow("Cropped LED Region", cropped_region)
            # cv2.waitKey(0)

        return led_count

    async def image_callback(self,image):
        print(type(image))
        width = image.width
        height = image.height
        print(image.width,image.height)
        data = image.data
        image_array = np.array(data, dtype=np.uint8)
        image_array = image_array.reshape((height, width,3))
        cv2.imshow("frame",image_array)
        cv2.waitKey(1)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        gray = cv2.cvtColor(image_array, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (15, 15), 0)
        thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.erode(thresh, None, iterations=2)
        thresh = cv2.dilate(thresh, kernel, iterations=25)

        labels = measure.label(thresh, connectivity=1.5, background=0)
        mask = np.zeros(thresh.shape, dtype="uint8")

        for label in np.unique(labels):
            if label == 0:
                continue
            labelMask = np.zeros(thresh.shape, dtype="uint8")
            labelMask[labels == label] = 255
            numPixels = cv2.countNonZero(labelMask)
            try:
                if numPixels > 100 and self.drone_whycon_pose_array.poses[0].position.z<24:
                    self.find=1
                    try:
                        self.set_points=[self.drone_whycon_pose_array.poses[0].position.x,self.drone_whycon_pose_array.poses[0].position.y,self.drone_whycon_pose_array.poses[0].position.z]
                    except:
                        pass
                    print("numpix",numPixels)
                    mask = cv2.add(mask, labelMask)
            except:
                pass

        led_count= self.count_leds_in_mask(mask, image_array)
        print(led_count)
        
        if(self.back==1 and self.done==0):
                self.biolocation_msg.organism_type = aliens[self.alien] 
                self.biolocation_msg.whycon_x = self.alien_pose[0]  
                self.biolocation_msg.whycon_y = self.alien_pose[1] 
                self.biolocation_msg.whycon_z = 30.5 
                self.alien_pub.publish(self.biolocation_msg)
                print(self.biolocation_msg)
                self.beep(self.alien)
                self.blink(self.alien)
                self.done=1


    def blink(self, num_blinks):
        def _blink():
            for _ in range(num_blinks):
                # Turn LED to Red
                self.rc_message.aux4 = 2000
                self.rc_pub.publish(self.rc_message)
                time.sleep(0.2)

                # Turn LED off
                self.rc_message.aux4 = 1500
                self.rc_pub.publish(self.rc_message)
                time.sleep(0.2)

        threading.Thread(target=_blink).start()


    def beep(self, num_beeps):
        def _beep():
            for _ in range(num_beeps):
                # Activate beeping sound
                self.rc_message.aux3 = 2000
                self.rc_pub.publish(self.rc_message)
                time.sleep(0.2)
                self.rc_message.aux3 = 1000
                self.rc_pub.publish(self.rc_message)
                # time.sleep(0.2)

        threading.Thread(target=_beep).start()

    def whycon_poses_callback(self, msg):
        self.last_whycon_pose_received_at = 0.1
        self.last_whycon_pose_received_at = self.node.get_clock().now().seconds_nanoseconds()[0]
        self.drone_whycon_pose_array = msg

    def pid_tune_throttle_callback(self, msg):
        self.Kp[1] = msg.kp * 0.01
        self.Ki[1] = msg.ki * 0.0002
        self.Kd[1] = msg.kd * 0.1

    # Similarly add callbacks for other subscribers

    def pid(self):  # PID algorithm

        # 0 : calculating Error, Derivative, Integral for Roll error : x axis

        try:
            if(abs(self.drone_whycon_pose_array.poses[0].position.z-self.set_des_points[self.count][2])<0.8 and abs(self.drone_whycon_pose_array.poses[0].position.y-self.set_des_points[self.count][1])<0.8 and abs(self.drone_whycon_pose_array.poses[0].position.x-self.set_des_points[self.count][0])<0.8 and  self.find == 0):
                self.count+=1
                # self.set_points=self.set_des_points[self.count]
                self.node.get_logger().info(str("---------------------change-------------------------------"))
            if(self.approuch==0):
                self.set_points[2]=self.set_points[2] + (self.set_des_points[self.count][2]-self.set_points[2])*self.rate
                self.set_points[1]=self.set_points[1] + (self.set_des_points[self.count][1]-self.set_points[1])*self.rate
                self.set_points[0]=self.set_points[0] + (self.set_des_points[self.count][0]-self.set_points[0])*self.rate
            if(self.back==1):
                self.set_points[2]=self.set_points[2] + (self.set_land_points[2]-self.set_points[2])*self.rate
                self.set_points[1]=self.set_points[1] + (self.set_land_points[1]-self.set_points[1])*self.rate
                self.set_points[0]=self.set_points[0] + (self.set_land_points[0]-self.set_points[0])*self.rate

            self.error[2] = (
                self.drone_whycon_pose_array.poses[0].position.z - self.set_points[2]
            )
            print(
                self.drone_whycon_pose_array.poses[0].position.x,
                self.drone_whycon_pose_array.poses[0].position.y,
                self.drone_whycon_pose_array.poses[0].position.z,
            )
            self.derivative[2] = self.error[2] - self.previous_error[2]
            self.integral[2] = self.sum_error[2]
            self.error[1] = (
                self.drone_whycon_pose_array.poses[0].position.y - self.set_points[1]
            )
            self.derivative[1] = self.error[1] - self.previous_error[1]
            self.integral[1] = self.sum_error[1]
            self.error[0] = (
                self.drone_whycon_pose_array.poses[0].position.x - self.set_points[0]
            )
            self.derivative[0] = self.error[0] - self.previous_error[0]
            self.integral[0] = self.sum_error[0]
            self.ss=abs(self.drone_whycon_pose_array.poses[0].position.z-27)
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

        # ------------------------------------------------------------------------------------------------------------------------

        self.previous_error[2] = self.error[2]
        self.sum_error[2] += self.error[2]
        self.previous_error[1] = self.error[1]
        self.sum_error[1] += self.error[1]
        self.previous_error[0] = self.error[0]
        self.sum_error[0] += self.error[0]
        self.rc_message.rc_throttle = int(self.Kp[2])
        self.rc_message.rc_throttle = int(
            1460
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
        if( self.ss< 0.75 and self.down==1):
            self.rc_message.rc_throttle=1100
          
        if(self.back==1 and abs(self.drone_whycon_pose_array.poses[0].position.y-self.set_land_points[1])<0.6 and abs(self.drone_whycon_pose_array.poses[0].position.x-self.set_land_points[0])<0.6):
            self.set_points[2]=29
            self.off=1
            self.back=2
        if(self.off==1 and abs(self.drone_whycon_pose_array.poses[0].position.z-self.set_points[2])<0.6 and abs(self.drone_whycon_pose_array.poses[0].position.y-self.set_points[1])<0.6 and abs(self.drone_whycon_pose_array.poses[0].position.x-self.set_points[0])<0.6 ):
            self.rc_message.rc_throttle = 1100
        self.publish_data_to_rpi(
            roll=self.rc_message.rc_roll,
            pitch=self.rc_message.rc_pitch,
            throttle=self.rc_message.rc_throttle,
        )

        # Replace the roll pitch and throttle values as calculated by PID

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
            if len(DRONE_WHYCON_POSE[index]) != span - 1:
                return
            order = 3
            fs = 60
            fc = 5
            nyq = 0.5 * fs
            wc = fc / nyq
            b, a = scipy.signal.butter(
                N=order, Wn=wc, btype="lowpass", analog=False, output="ba"
            )
            filtered_signal = scipy.signal.lfilter(b, a, DRONE_WHYCON_POSE[index])
            if index == 0:
                self.rc_message.rc_roll = int(filtered_signal[-1])
            elif index == 1:
                self.rc_message.rc_pitch = int(filtered_signal[-1])
            elif index == 2:
                self.rc_message.rc_throttle = int(filtered_signal[-1])

        if self.rc_message.rc_roll > MAX_ROLL:  # checking range i.e. bet 1000 and 2000
            self.rc_message.rc_roll = MAX_ROLL
        elif self.rc_message.rc_roll < MIN_ROLL:
            self.rc_message.rc_pitch = MIN_ROLL
        if (
            self.rc_message.rc_pitch > MAX_PITCH
        ):  # checking range i.e. bet 1000 and 2000
            self.rc_message.rc_pitch = MAX_PITCH
        elif self.rc_message.rc_pitch < MIN_PITCH:
            self.rc_message.rc_roll = MIN_PITCH
        if (
            self.rc_message.rc_throttle > MAX_THROTAL
        ):  # checking range i.e. bet 1000 and 2000
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
        cv2.destroyAllWindows()
        self.node.get_logger().info("Calling disarm service")
        self.commandbool.value = False
        self.future = self.arming_service_client.call_async(self.commandbool)


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("controller")
    node.get_logger().info(f"Node Stahrted")
    node.get_logger().info("Entering PID controller loop")

    controller = DroneController(node)
    controller.arm()
    controller.arm()
    node.get_logger().info("Armed")

    try:
        while rclpy.ok():
            controller.pid()
            if (
                node.get_clock().now().to_msg().sec
                - controller.last_whycon_pose_received_at
                > 1
            ):
                node.get_logger().error("Unable to detect WHYCON poses")
            rclpy.spin_once(node)
            print("hf")  # Sleep for 1/30 secs

    except Exception as err:
        print(err)

    finally:
        controller.shutdown_hook()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()