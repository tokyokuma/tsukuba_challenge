#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import time
from darknet_ros_msgs.msg import BoundingBoxes
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

#initilanze
xmax, xmin, ymax, ymin = 0, 0, 0, 0
judge = 'none'
pub = rospy.Publisher('/darknet_ros/judge_traffic_light', String, queue_size=3)

def High_contrast(min, max, img):
    #re_contrast table
    min_table = min
    max_table = max
    diff_table = max_table - min_table

    LUT_HC = np.arange(256, dtype = 'uint8' )
    LUT_LC = np.arange(256, dtype = 'uint8' )

    # create high-contrast LUT
    for a in range(0, min_table):
        LUT_HC[a] = 0
    for a in range(min_table, max_table):
        LUT_HC[a] = 255 * (a - min_table) / diff_table
    for a in range(max_table, 255):
        LUT_HC[a] = 255

    return cv2.LUT(img, LUT_HC)

def Recolor_hsv(hsv_img, v):
    recolor_img = hsv_img.copy()
    #cond_p = (hsv_img[..., 2] >= h)
    cond_p = (hsv_img[..., 2] <= v)
    cond_f = np.logical_not(cond_p)
    recolor_img[cond_p] = [0, 255, 0]
    recolor_img[cond_f] = [255, 255, 255]

    return recolor_img

def Open_Close(img):
    img_tmp = img
    element8 = np.array([[1,1,1], [1,1,1], [1,1,1]], np.uint8)
    iteration = 1
    while iteration <= 20:
        img_tmp = cv2.morphologyEx(img_tmp, cv2.MORPH_OPEN, element8)
        img_tmp = cv2.morphologyEx(img_tmp, cv2.MORPH_CLOSE, element8)
        iteration = iteration + 1
    return img

def Judge(img):
    count_up = 0
    count_bottom = 0
    height, width = img.shape
    up = img[0:height/2,0:width]
    bottom = img[height/2:height,0:width]

    height_up, width_up = up.shape
    height_bottom, width_bottom = bottom.shape

    for y in range(0, height_up):
        for x in range(0, width_up):
            pixel = up.item(y,x)
            if pixel == 0:
                count_up = count_up + 1
            else:
                pass
    black_ratio_up = float(count_up) / float(height_up * width_up) * 100
    print black_ratio_up

    for y in range(0, height_bottom):
        for x in range(0, width_bottom):
            pixel = bottom.item(y,x)
            if pixel == 0:
                count_bottom = count_bottom + 1
            else:
                pass
    black_ratio_bottom = float(count_bottom) / float(height_bottom * width_bottom) * 100
    print black_ratio_bottom

    if black_ratio_up < 2 and black_ratio_bottom < 2:
        judge = 'off'
        return judge
    elif black_ratio_up < black_ratio_bottom:
        judge = 'green'
        return judge
    elif black_ratio_up > black_ratio_bottom:
        judge = 'red'
        return judge
    else:
        judge = 'miss'
        return judge

def Get_box(data, id):
    global xmax
    global xmin
    global ymax
    global ymin
    xmax, xmin, ymax, ymin = 0, 0, 0, 0
    #initilaze box coordination
    class_name = data.bounding_boxes
    num_of_object = len(class_name)
    for i in range(0,num_of_object):
        if class_name[i].Class == 'traffic light':
            xmax = class_name[i].xmax
            xmin = class_name[i].xmin
            ymax = class_name[i].ymax
            ymin = class_name[i].ymin
        else:
            pass

def Get_rgb(data, id):

    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print e

    if xmax != 0 and ymax != 0:
        start = time.time()
        print 'xmax : ' + str(xmax) + 'xmin : ' + str(xmin) + 'ymax : ' + str(ymax) + 'ymin : ' + str(ymin)
        trimming_img = cv_image[ymin:ymax, xmin:xmax]
        hsv = cv2.cvtColor(trimming_img, cv2.COLOR_RGB2HSV)
        test_hsv = hsv.copy()

        recolor_hsv = Recolor_hsv(hsv, 40)
        gray = cv2.cvtColor(recolor_hsv, cv2.COLOR_RGB2GRAY)
        ret, th = cv2.threshold(gray, 151, 255, cv2.THRESH_BINARY)
        th = Open_Close(th)
        judge = Judge(th)
        pub.publish(String(judge))
        print 'judge : ' + judge
        elapsed_time = time.time() - start
        print elapsed_time
        cv2.imshow('RGB', trimming_img)

        key = cv2.waitKey(delay=1)
    else:
        #print 'no data'
        pub.publish(String('None'))
        pass

def listener():
    rospy.init_node('judge_traffic_light', anonymous=True)
    box_sub_yolo = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, Get_box, callback_args=0, queue_size = 1)
    image_sub_rgb = rospy.Subscriber("/camera_forward/color/image_raw", Image, Get_rgb, callback_args=1, queue_size = 1)
    pub.publish(String(judge))
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
