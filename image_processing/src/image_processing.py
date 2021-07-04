#!/usr/bin/env python2
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray
import Jetson.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)

#GPIO.output(12, GPIO.HIGH)
#GPIO.output(16, GPIO.HIGH)
#GPIO.output(18, GPIO.HIGH)
GPIO.output(12, GPIO.LOW)
GPIO.output(16, GPIO.LOW)
GPIO.output(18, GPIO.LOW)

class processor():
    def __init__(self):
        self.selecting_sub_image = "compressed" # you can choose image type "compressed", "raw"
 
        if self.selecting_sub_image == "compressed":
            self._sub0 = rospy.Subscriber('/csi_cam_0/image_raw/compressed', CompressedImage, self.callback0, queue_size=1)
            self._sub1 = rospy.Subscriber('/csi_cam_1/image_raw/compressed', CompressedImage, self.callback1, queue_size=1)
        else:
            self._sub0 = rospy.Subscriber('/csi_cam_0/image_raw', Image, self.callback0, queue_size=1)
            self._sub1 = rospy.Subscriber('/csi_cam_1/image_raw', Image, self.callback1, queue_size=1)
        
        led_array = Int16MultiArray()
        led_array.data = [0,0,0]
        led_sub = rospy.Subscriber('/led/control', Int16MultiArray, self.led_callback, queue_size=1)
        
        self.bridge = CvBridge()
        self.target_id = 0
        self.mid_point_x = 160 
        self.mid_point_y = 120 

        self.target_error_front = Float32MultiArray()
        self.target_error_lower = Float32MultiArray()
        self.target_error_front.data = [0,0,0]
        self.target_error_lower.data = [0,0,0]
        self.x = self.mid_point_x
        self.y = self.mid_point_y
        self.detect_flag = 0
        

    
    def find_red_circle(self, cv_image):
        imgHSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        imgThreshRed = cv2.inRange(imgHSV, (0, 0, 0), (10, 255, 255))

        #gaussian = cv2.GaussianBlur(imgThreshRed, (3, 3), 2)                 # blur
        median_blur_red = cv2.medianBlur(imgThreshRed, 15)
        #median_blur_red = cv2.dilate(median_blur_red, np.ones((5,5),np.uint8))        # close image (dilate, then erode)
        #median_blur_red = cv2.erode(median_blur_red, np.ones((5,5),np.uint8))         # closing "closes" (i.e. fills in) foreground gaps

        intRows, intColumns = median_blur_red.shape        # break out number of rows and columns in the image, rows is used for minimum distance between circles in call to Hough Circles

        circles = cv2.HoughCircles(median_blur_red, cv2.HOUGH_GRADIENT, 2, 50, param1 = 200, param2 = 50, minRadius = 10)      # fill variable circles with all circles in the processed image
        x = 0
        y = 0
        detect_flag = 0
        if circles is not None and len(circles[0]) < 3:                     # this line is necessary to keep program from crashing on next line if no circles were found
            #print(len(circles[0]))
            for circle in circles[0, :]:                           # for each circle
                x, y, radius = circle  
                radius = int(radius)                                                                     # break out x, y, and radius
                #print "ball position x = " + str(x) + ", y = " + str(y) + ", radius = " + str(radius)       # print ball position and radius
                cv2.circle(cv_image, (x, y), 3, (0, 255, 0), cv2.FILLED)           # draw small green circle at center of detected object
                cv2.circle(cv_image, (x, y), radius, (0, 0, 255), 3)                     # draw red circle around the detected object
                detect_flag = 1
        return cv_image, x, y, detect_flag 

    def find_blue_rect(self, cv_image):
        imgHSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        imgThreshBlue = cv2.inRange(imgHSV, (100, 10, 10), (120, 255, 255))
        kernel1 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))

        median_blur_blue = cv2.medianBlur(imgThreshBlue, 11)


        contours, _ = cv2.findContours(median_blur_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        target_x = 0
        target_y = 0
        detect_flag = 0
        if (len(contours) > 0):
            for cont in contours: 
                approx = cv2.approxPolyDP(cont, cv2.arcLength(cont, True) * 0.02, True) 
                vtc = len(approx) 
                if vtc == 4:
                    (x, y, w, h) = cv2.boundingRect(cont) 
                    cv2.rectangle(cv_image, (x,y), (x+w, y+h), (0, 0, 255), 3)
                    target_x = x + (w / 2)
                    target_y = y + (h / 2)
                    cv2.circle(cv_image, (target_x, target_y), 3, (0, 255, 0), cv2.FILLED)  
                    detect_flag = 1
        return cv_image, target_x, target_y, detect_flag
        
    def callback0(self, image_msg):
 
        if self.selecting_sub_image == "raw":
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        elif self.selecting_sub_image == "compressed":
            #converting compressed image to opencv image
            np_arr = np.fromstring(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            #cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
 
            cv_image = cv2.resize(cv_image, dsize=(320, 240), interpolation=cv2.INTER_AREA)
            #print(cv_image.shape)
            
            if(self.target_id == 0):
                cv_image, x, y, detect_flag = self.find_red_circle(cv_image)  # Detect Red circle
                self.x = self.mid_point_x - x
                self.y = self.mid_point_y -y
                #print(self.y)
                self.detect_flag = detect_flag
            else:
                cv_image, x, y, detect_flag = self.find_blue_rect(cv_image)  # Detect Blue Rect
                self.x = self.mid_point_x - x
                self.y = self.mid_point_y -y
                self.detect_flag = detect_flag

            self.target_error_front.data[0] = self.x
            self.target_error_front.data[1] = self.y
            self.target_error_front.data[2] = self.detect_flag
            self.target_error_front_pub.publish(self.target_error_front)
            self.img_pub_front.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

            #cv2.imshow("imgThreshRed0", imgThreshRed)
            #cv2.imshow("imgThreshBlue0", imgThreshBlue)
            #cv2.imshow('median_blur_blue0', median_blur_blue)
            #cv2.imshow('median_blur_red0', median_blur_red)
 
            #cv2.imshow('cv_image0', cv_image), cv2.waitKey(1)
            #self.img_pub_front.publish(cv_image)
    
    def callback1(self, image_msg):
 
        if self.selecting_sub_image == "raw":
            #converting compressed image to opencv image
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        elif self.selecting_sub_image == "compressed":
            np_arr = np.fromstring(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv_image = cv2.resize(cv_image, dsize=(320, 240), interpolation=cv2.INTER_AREA)
        #cv_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
           
            if(self.target_id == 0):
                cv_image, x, y, detect_flag = self.find_red_circle(cv_image)  # Detect Red circle
                self.x = self.mid_point_x - x
                self.y = self.mid_point_y -y
                self.detect_flag = detect_flag
            else:
                cv_image, x, y, detect_flag = self.find_blue_rect(cv_image)  # Detect Blue Rect
                self.x = self.mid_point_x - x
                self.y = self.mid_point_y -y
                self.detect_flag = detect_flag
            self.target_error_lower.data[0] = self.x
            self.target_error_lower.data[1] = self.y
            self.target_error_lower.data[2] = self.detect_flag
            self.target_error_lower_pub.publish(self.target_error_lower)
            self.img_pub_lower.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            #cv2.imshow("imgThreshRed1", imgThreshRed)
            #cv2.imshow("imgThreshBlue1", imgThreshBlue)
            #cv2.imshow('median_blur_blue1', median_blur_blue)
            #cv2.imshow('median_blur_red1', median_blur_red)
            #cv2.imshow('cv_image1', cv_image) 
            #self.img_pub_lower.publish(cv_image)
    def led_callback(self, led_msg):
        self.led_array.data = led_msg.data
        i = 0
        while i < 3:
            if self.led_array.data[i] == 0:
                if i == 0:
                    GPIO.output(12, GPIO.LOW)
                elif i == 1:
                    GPIO.output(16, GPIO.LOW)
                else:
                    GPIO.output(18, GPIO.LOW)
            else:
                if i == 0:
                    GPIO.output(12, GPIO.HIGH)
                elif i == 1:
                    GPIO.output(16, GPIO.HIGH)
                else:
                    GPIO.output(18, GPIO.HIGH)             
            i = i + 1


    def main(self):
        self.img_pub_front = rospy.Publisher('/result_img/front', Image , queue_size=1)
        self.img_pub_lower = rospy.Publisher('/result_img/lower', Image , queue_size=1)
        self.target_error_front_pub = rospy.Publisher('/target_error/front', Float32MultiArray, queue_size=1)
        self.target_error_lower_pub = rospy.Publisher('/target_error/lower', Float32MultiArray, queue_size=1)
        
        rospy.spin()
 
if __name__ == '__main__':
    rospy.init_node('image_processing')
    node = processor()
    node.main()


