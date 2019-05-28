#!/usr/bin/env python
# encoding:utf-8
import rospy
import cv2
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist


class FaceDetector():
    '''face detect, sending angle'''

    def __init__(self):
        rospy.init_node('FaceDetector')  # define node-name
        self.esk_key = 27
        self.interval = 33
        self.fps = 30
        # f / width, f / height
        self.fx = 579.322572
        self.fy = 572.417313
        # 分類器の指定
        self.cascade = cv2.CascadeClassifier("./front2.xml")

        # ros
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.got_image)
        self.angle_pub = rospy.Publisher('servo_angle', Twist, queue_size=10)
        self.bridge = CvBridge()

    def got_image(self, data):
        '''convert ros-message to cv-image'''
        rospy.loginfo("got image!!")

        try:
            # ros = bgr8
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e
            rospy.loginfo("imge convert error occured ...")

        self.face_detection(cv_image)

    def face_detection(self, img):
        '''face detection'''
        rospy.loginfo("start detectioning")

        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        face_list = self.cascade.detectMultiScale(img_gray, minSize=(100, 100))
        rospy.loginfo("finish detectioning")

        # 検出した顔に印を付ける
        angle = Twist()
        for (x, y, w, h) in face_list:
            rospy.loginfo("detect!!")
            color = (0, 0, 255)
            blue = (255, 0, 0)
            x_mid = int(x + (w / 2.0))
            y_mid = int(y + (h / 2.0))
            cv2.circle(img, (x_mid, y_mid), 25, color, -1)
            font = cv2.FONT_HERSHEY_PLAIN
            cv2.putText(img, "x:" + str(x_mid), (x, y + 20), font, 2, color, 3)
            cv2.putText(img, "y:" + str(y_mid), (x, y + 60), font, 2, color, 3)

            x_angle, y_angle = self.calc_angle(x_mid, y_mid)
            # publish angle
            angle.linear.x = math.degrees(x_angle)
            angle.linear.y = math.degrees(y_angle)

            cv2.putText(img, "x_angle:" + str(angle.linear.x), (0, 30), font, 2, blue, 3)
            cv2.putText(img, "y_angle:" + str(angle.linear.y), (0, 60), font, 2, blue, 3)
            break

        self.angle_pub.publish(angle)
        cv2.imshow("webcam", img)
        cv2.waitKey(3)
        rospy.loginfo("showed image")

    def calc_angle(self, x_mid, y_mid):
        '''calc angle radian'''
        x = (x_mid - 320.0)
        y = (y_mid - 240.0)

        x_angle = math.atan2(x, self.fx)
        y_angle = math.atan2(y, self.fy)

        return x_angle, y_angle


if __name__ == '__main__':
    '''main'''
    rospy.loginfo("start")
    facedetector = FaceDetector()
    rospy.loginfo("generated instance")
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        rospy.loginfo("looppppp")
        rate.sleep()

    # 終了処理
    cv2.destroyAllWindows()
