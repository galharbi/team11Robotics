#!/usr/bin/env python
import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3, Twist
from time import sleep
import exp_quat_func as eqf
import kin_func_skeleton as kfs

# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

import scipy
import scipy.misc as misc
import matplotlib.pyplot as plt
import sklearn
from sklearn import svm
from sklearn.utils import shuffle
import scipy.interpolate as interpol
import move_fruit
import image_classifier

import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
from baxter_interface import gripper as baxter_gripper


listener = None
robot = None
scene = None
left_arm = None
left_gripper = None
init = False


def follow_ar_tag(ar_tag):
    listener = tf.TransformListener()
    rate = rospy.Rate(10)
    rospy.sleep(2.0)

    # i = 0
    # turn to face the AR tag
    while not rospy.is_shutdown():
        # if i >= 4000:
        #     return None
        # i+=1
        try:
            listener.waitForTransform('/base',ar_tag, rospy.Time(0),rospy.Duration(6.0))
            (trans, rot) = listener.lookupTransform('/base',ar_tag, rospy.Time(0))
        except:
            print('No AR Tag found')
            continue
        # YOUR CODE HERE
        [omega, theta] = eqf.quaternion_to_exp(rot)
        R = kfs.skew_3d(-omega)
        TransMatrix = np.array([trans]).T
        cordTrans = np.dot(R,TransMatrix)
        #  orient the Zumy to face the tag
        return trans[0],trans[1],trans[2]

def get_current_pos():
    # rospy.init_node('gripper_pos')
    listener = tf.TransformListener()
    rospy.sleep(2.0)
    rate = rospy.Rate(10)
    pos_orient = listener.lookupTransform('/base', '/left_gripper', rospy.Time(0))
    position = pos_orient[0]
    position = np.array([position[0],position[1],position[2]])
    return position

def mimic_gripper(args):
    listener = tf.TransformListener()
    rate = rospy.Rate(10)
    vals = {}


    i = 0
    j = 0
    # turn to face the AR tag
    while not rospy.is_shutdown():
        i+=1
        raw_input('Press enter to get Position %s' %args[j])
        try:
            (trans, rot) = listener.lookupTransform('/base', '/left_hand_gripper', rospy.Time(0))

        except:
            print('Position not found at iter %i' %i)

            continue
        # YOUR CODE HERE
        #  orient the Zumy to face the tag
        vals[args[j]] = (trans,rot)
        j+=1
    return vals



def move(current_Pos, coordTrans, increment_ct=10, grip=False, orient=[0.0,-1.0,0.0,0.0],let_go=False):
    global robot,scene,left_arm,left_gripper
    if grip:
        left_gripper.close(block=False, timeout=100)
    if not grip:
        left_gripper.open(block=False, timeout=100)
    if let_go:
        left_gripper.open(block=False, timeout=100)
        return
    x_diff = np.round(coordTrans[0],5)
    y_diff = np.round(coordTrans[1],5)
    z_diff = np.round(coordTrans[2],5)



    #First goal pose ------------------------------------------------------
    goal_1 = PoseStamped()
    goal_1.header.frame_id = "base"

    #x, y, and z position
    goal_1.pose.position.x = current_Pos[0]
    goal_1.pose.position.y = current_Pos[1]
    goal_1.pose.position.z = current_Pos[2]
    n = float(increment_ct)
    #print(goal_1.pose)
    for i in range(0,int(n)):
        goal_1.pose.position.x = goal_1.pose.position.x + x_diff/n
        goal_1.pose.position.y = goal_1.pose.position.y + y_diff/n
        goal_1.pose.position.z = goal_1.pose.position.z + z_diff/n


        goal_1.pose.position.x = np.round(goal_1.pose.position.x,5)
        goal_1.pose.position.y = np.round(goal_1.pose.position.y,5)
        goal_1.pose.position.z = np.round(goal_1.pose.position.z,5)


        goal_1.pose.orientation.x = orient[0]
        goal_1.pose.orientation.y = orient[1]
        goal_1.pose.orientation.z = orient[2]
        goal_1.pose.orientation.w = orient[3]

        left_arm.set_pose_target(goal_1)

        #Set the start state for the right arm
        left_arm.set_start_state_to_current_state()

        left_plan = left_arm.plan()
        left_arm.execute(left_plan)
    rospy.sleep(1.0)

def init_control(args):
    #Initialize moveit_commander
    global robot,scene,left_arm,left_gripper
    moveit_commander.roscpp_initialize(args)

    #Start a node
    rospy.init_node('moveit_node3')

    #Initialize both arms
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    left_arm = moveit_commander.MoveGroupCommander('left_arm')

    left_arm.set_planner_id('RRTConnectkConfigDefault')
    left_arm.set_planning_time(10)

    #Set up the left gripper
    left_gripper = baxter_gripper.Gripper('left')

    #Calibrate the gripper (other commands won't work unless you do this first)
    print('Calibrating...')
    left_gripper.calibrate()
    init = True
    rospy.sleep(1.0)



if __name__=='__main__':
    heights = [.068,0.035,0.025]
    init_control(sys.argv)
    #move to initial find fruit position
    #take picture and save transformation values
    #classify picture
    #move to initial find bin1,...bin3 position
    #save transformation values
    #call move_fruit on the appropriate values
    ar_tags = []
    # r = raw_input('First AR Tag number, corresponding to fruit: ')
    ar_tags.append('ar_marker_3')

    # r = raw_input('Second AR Tag number, corresponding to Pineapple Bin: ')
    ar_tags.append('ar_marker_14')

    # r = raw_input('Third AR Tag number, corresponding to Lemon Bin: ')
    ar_tags.append('ar_marker_12')

    # r = raw_input('Fourth AR Tag number, corresponding to Tomato Bin: ')
    ar_tags.append('ar_marker_4')

    fruit_vals = ['pineapple','lemon','tomato']

    pos_orients = np.load('pos_orients.npy')

    fruit_pos = np.array([pos_orients[0][0][0],pos_orients[0][0][1],pos_orients[0][0][2]])
    bins = [pos_orients[1][0][0],pos_orients[1][0][1],pos_orients[1][0][2]]
    # offset = np.array([pos_orients[4][0][0],pos_orients[4][0][1],pos_orients[4][0][2]])
    current_pos = get_current_pos()
    #move to default fruit position
    print('default position for fruit scan')
    move(current_pos,fruit_pos-current_pos, increment_ct=3)

    #get translation from default fruit position to ar_tag
    ar_tag_fruit_loc = follow_ar_tag(ar_tags[0])
    ar_tag_fruit_loc = [ar_tag_fruit_loc[0],ar_tag_fruit_loc[1],ar_tag_fruit_loc[2]]
    hover_pos = [ar_tag_fruit_loc[0]+.13,ar_tag_fruit_loc[1]-.115,ar_tag_fruit_loc[2]+.2]
    print('default position for hover')
    current_pos = get_current_pos()
    move(current_pos,hover_pos-current_pos, increment_ct=3)
    #classify fruit
    classified_val = np.array([image_classifier.classify() for i in range(5)])
    classified_val_p = np.sum(classified_val == 0)
    classified_val_l = np.sum(classified_val == 1)
    classified_val_t = np.sum(classified_val == 2)
    print(classified_val)
    print(classified_val_p,classified_val_l,classified_val_t)
    classified_val = np.argmax([classified_val_p,classified_val_l,classified_val_t])
    print(classified_val)
    if classified_val == 0:
	print('Pineapple found')
    if classified_val == 1:
        print('Lemon found')
    if classified_val == 2:
        print('Tomato found')
    ar_tag_fruit_loc = [ar_tag_fruit_loc[0],ar_tag_fruit_loc[1]-.115,ar_tag_fruit_loc[2]+heights[classified_val]]
    #move to bin corresponding to fruit
    print('going to bin')
    current_pos = get_current_pos()
    move(current_pos,bins+np.array([0,0,.01])-current_pos,increment_ct=3)

    #find translatiom from default bin position to ar_tag2
    bin_location = follow_ar_tag(ar_tags[classified_val+1])
    if bin_location == None:
        for i in range(3):
            rx = np.random.uniform(-0.05,.05)
            ry = np.random.uniform(-0.05,.05)
            rz = np.random.uniform(-0.01,.01)
            perturb = np.array([rx,ry,rz])
            move(bins,perturb,increment_ct=1)
            bin_location = follow_ar_tag(ar_tags[classified_val+1])
            if bin_location != None:
                break
        if classified_val == 0:
            bin_location = [.05,-.05,0]
        if classified_val == 1:
            bin_location = [-.05,-.05,0]
        else:
            bin_location = [0,.1,0]

    bin_location = [bin_location[0],bin_location[1],bin_location[2]]

    print('going to fruit')
    #move back to default fruit position
    move(bins,fruit_pos-bins,increment_ct=5)

    print('grabbing fruit')
    #move to fruit and grab
    move(fruit_pos,ar_tag_fruit_loc-fruit_pos,grip=True,increment_ct=3)
    rospy.sleep(1)
    print('moving up')
    #move back up to default fruit position
    move(ar_tag_fruit_loc,fruit_pos-ar_tag_fruit_loc,grip=True,increment_ct=3)


    print('move to bin x,y')
    #move from default bin position to ar_tag2 position
    move(fruit_pos, np.array([bin_location[0],bin_location[1],fruit_pos[2]])-fruit_pos,grip=True,increment_ct=6)


    print('drop fruit')
    move(bin_location,bin_location,grip=False,let_go=True)
