#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
from baxter_interface import gripper as baxter_gripper
import numpy as np


robot = None
scene = None
left_arm = None
left_gripper = None
init = False

#def tf_callback(msg):
    #do some stuff to return the postition of the gripper?
def init_control(args):
    #Initialize moveit_commander
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

def move(current_Pos, coordTrans, increment_ct=6, grip=False, orient=[[0.0,-1.0,0.0,0.0]]):
    if not init:
        print('Error: Not Initialized')
        return None
    if grip:
        left_gripper.close(block=False, timeout=100)
    if not grip:
        left_gripper.open(block=False, timeout=100)
    x_diff = np.round(coordTrans[0][0],5)
    y_diff = np.round(coordTrans[1][0],5)
    z_diff = np.round(coordTrans[2][0],5)
    
    

    #First goal pose ------------------------------------------------------
    goal_1 = PoseStamped()
    goal_1.header.frame_id = "base"

    #x, y, and z position
    goal_1.pose.position.x = current_Pos[0][0]
    goal_1.pose.position.y = current_Pos[1][0]
    goal_1.pose.position.z = current_Pos[2][0]
    n = float(increment_ct)
    #print(goal_1.pose)
    for i in range(0,int(n)):
        goal_1.pose.position.x = goal_1.pose.position.x + x_diff/n
        goal_1.pose.position.y = goal_1.pose.position.y + y_diff/n
        goal_1.pose.position.z = goal_1.pose.position.z + z_diff/n

    goal_1.pose.position.x = np.round(goal_1.pose.position.x,5)
    goal_1.pose.position.y = np.round(goal_1.pose.position.y,5)
    goal_1.pose.position.z = np.round(goal_1.pose.position.z,5)
    #Orientation as a quaternion
    goal_1.pose.orientation.x = orient[0]
    goal_1.pose.orientation.y = orient[1]
    goal_1.pose.orientation.z = orient[2]
    goal_1.pose.orientation.w = orient[3]
    #Set the goal state to the pose you just defined

    left_arm.set_pose_target(goal_1)

    #Set the start state for the right arm
    left_arm.set_start_state_to_current_state()

    #Plan a path
    left_plan = left_arm.plan()
    left_arm.execute(left_plan)
   
    #Close the right gripper
    print('Closing...')
    left_gripper.close(block=True)
    rospy.sleep(1.0)


# def move(pos):
#     moveit_commander.roscpp_initialize(sys.argv)

#     #Start a node
#     rospy.init_node('moveit_node')

#     #Initialize both arms
#     robot = moveit_commander.RobotCommander()
#     scene = moveit_commander.PlanningSceneInterface()
#     left_arm = moveit_commander.MoveGroupCommander('left_arm')
#     left_arm.set_planner_id('RRTConnectkConfigDefault')
#     left_arm.set_planning_time(20)
#     left_gripper = baxter_gripper.Gripper('left')
#     left_gripper.calibrate()

#     goal_target = PoseStamped()
#     goal_target.header.frame_id = "base"
#     #x, y, and z position of goal position
#     goal_target.pose.position.x = pos[0]#0.800
#     goal_target.pose.position.y = pos[1]#0.048
#     goal_target.pose.position.z = pos[2]#0.116
    
#     #Goal Orientation as a quaternion
#     goal_target.pose.orientation.x = 0.0#0.984
#     goal_target.pose.orientation.y = -1.0#-0.172
#     goal_target.pose.orientation.z = 0.0#0.035
#     goal_target.pose.orientation.w = 0.0#-0.012

#     targetPosX = goal_target.pose.position.x 
#     targetPosY = goal_target.pose.position.y

#     #Set the goal state to the pose you just defined
#     left_arm.set_pose_target(goal_1)

#     #Set the start state for the left arm
#     left_arm.set_start_state_to_current_state()

#     #Plan a path
#     left_plan = left_arm.plan()

#     #Execute the plan
#     left_arm.execute(left_plan)
#     return

# def follow_ar_tag(ar_tags):

#     listener = tf.TransformListener()
#     rate = rospy.Rate(10)
    
    
    
#     # turn to face the AR tag
#     while not rospy.is_shutdown():
#         try:
#             (trans, rot) = listener.lookupTransform('/left_gripper', ar_tags['ar1'], rospy.Time(0))
#         except:
#             continue
#         # YOUR CODE HERE
#         [omega, theta] = eqf.quaternion_to_exp(rot)
#         R = kfs.skew_3d(-omega)
#         TransMatrix = np.array([trans]).T
#         coordPos = np.dot(R,TransMatrix)
#         #  orient the Zumy to face the tag
        
#         return coordPos



# def move_it(pos_g,pos_t,vert_offset=.1,n=6):
#     #Initialize moveit_commander
#     moveit_commander.roscpp_initialize(sys.argv)

#     #Start a node
#     rospy.init_node('moveit_node')

#     #Initialize both arms
#     robot = moveit_commander.RobotCommander()
#     scene = moveit_commander.PlanningSceneInterface()
#     left_arm = moveit_commander.MoveGroupCommander('left_arm')
#     left_arm.set_planner_id('RRTConnectkConfigDefault')
#     left_arm.set_planning_time(20)
#     left_gripper = baxter_gripper.Gripper('left')
#     left_gripper.calibrate()

#     goal_target = PoseStamped()
#     goal_target.header.frame_id = "base"
#     #x, y, and z position of goal position
#     goal_target.pose.position.x = pos_g[0]#0.800
#     goal_target.pose.position.y = pos_g[1]#0.048
#     goal_target.pose.position.z = pos_g[2]#0.116
    
#     #Goal Orientation as a quaternion
#     goal_target.pose.orientation.x = 0.0#0.984
#     goal_target.pose.orientation.y = -1.0#-0.172
#     goal_target.pose.orientation.z = 0.0#0.035
#     goal_target.pose.orientation.w = 0.0#-0.012

#     targetPosX = goal_target.pose.position.x 
#     targetPosY = goal_target.pose.position.y




#     #First goal pose ------------------------------------------------------
#     goal_1 = PoseStamped()
#     goal_1.header.frame_id = "base"

#     #x, y, and z position
#     goal_1.pose.position.x = pos_t[0] #0.802
#     goal_1.pose.position.y = pos_t[1] #0.400
#     goal_1.pose.position.z = pos_t[2] #-0.024 + vert_offset
    
#     #Orientation as a quaternion
#     goal_1.pose.orientation.x = 0.0 #0.984
#     goal_1.pose.orientation.y = -1.0 #-0.172
#     goal_1.pose.orientation.z = 0.0 #0.035
#     goal_1.pose.orientation.w = 0.0 #-0.012

#     fruitPosX = goal_1.pose.position.x 
#     fruitPosY = goal_1.pose.position.y

#     #Set the goal state to the pose you just defined
#     left_arm.set_pose_target(goal_1)

#     #Set the start state for the left arm
#     left_arm.set_start_state_to_current_state()

#     #Plan a path
#     left_plan = left_arm.plan()

#     #Execute the plan
#     raw_input('Press <Enter> to move the left arm to goal pose 1 (path constraints are never enforced during this motion): ')
#     left_arm.execute(left_plan)
#     #left_gripper.close(block=True)
#     rospy.sleep(1.0)
#     #left_gripper.open(block=True)

#     #Second goal pose -----------------------------------------------------
#     rospy.sleep(2.0)
    
#     goal_2 = PoseStamped()

#     goal_2.header.frame_id = "base"

#     #x, y, and z position
#     goal_2.pose.position.x = goal_1.pose.position.x 
#     goal_2.pose.position.y = goal_1.pose.position.y 
#     goal_2.pose.position.z = goal_1.pose.position.z - vert_offset
    
#     #Orientation as a quaternion
#     goal_2.pose.orientation = goal_1.pose.orientation
        

#     #Set the goal state to the pose you just defined
#     left_arm.set_pose_target(goal_2)

#     #Set the start state for the left arm
#     left_arm.set_start_state_to_current_state()

#     #Plan a path
#     left_plan = left_arm.plan()

#     #Execute the plan
#     raw_input('Press <Enter> to move the left arm to goal pose 2: ')
#     left_arm.execute(left_plan)
#     #left_gripper.close(block=True)
#     rospy.sleep(1.0)
#     #left_gripper.open(block=True)

#     #Third goal pose -----------------------------------------------------
    
#     goal_3 = PoseStamped()
#     goal_3 = goal_1

#     #Set the goal state to the pose you just defined
#     left_arm.set_pose_target(goal_3)

#     #Set the start state for the left arm
#     left_arm.set_start_state_to_current_state()

#     #Plan a path
#     left_plan = left_arm.plan()
    
#     #Execute the plan
#     raw_input('Press <Enter> to move the left arm to goal pose 3: ')
#     left_gripper.close(block=False, timeout=100)
#     rospy.sleep(1.0)
#     left_arm.execute(left_plan)
#     #left_gripper.close(block=True)
#     rospy.sleep(1.0)
#     #left_gripper.open(block=True)

#     #interpolate linearly in between the pickup location and the goal position
#     # n = 6;
#     xPosInterpolation = np.linspace(fruitPosX, targetPosX, n)
#     yPosInterpolation = np.linspace(fruitPosY, targetPosY, n)

#     for i in range (1,n):
#         goal_interpolation = PoseStamped()
#         goal_interpolation.header.frame_id = "base"
#         goal_interpolation.pose.position.x = round(xPosInterpolation[i],5)
#         goal_interpolation.pose.position.y = round(yPosInterpolation[i],5)
#         goal_interpolation.pose.position.z = goal_target.pose.position.z
#         goal_interpolation.pose.orientation = goal_1.pose.orientation

        
        
#         left_arm.set_pose_target(goal_interpolation)
#         print("incrementatal target set")
#         print(goal_interpolation.pose.position.x, goal_interpolation.pose.position.y)

#         left_arm.set_start_state_to_current_state()

#         left_plan = left_arm.plan()

#         #raw_input('Press <Enter> to move the left arm to goal pose increment: ')
#         left_arm.execute(left_plan)
        
#         #rospy.sleep(1.0)

#     #place the fruit down in the appropriate box

#     goal_place = PoseStamped()
#     goal_place.header.frame_id = "base"

#     goal_place.pose.position.x = goal_target.pose.position.x
#     goal_place.pose.position.y = goal_target.pose.position.y
#     goal_place.pose.position.z = goal_target.pose.position.z - vert_offset
    


#     goal_place.pose.orientation = goal_target.pose.orientation
#     #Goal Orientation as a quaternion

#     left_arm.set_pose_target(goal_place)
#     print("place target set")

#     left_arm.set_start_state_to_current_state()

#     left_plan = left_arm.plan()

#     #raw_input('Press <Enter> to move the left arm to place the fruit: ')
#     left_arm.execute(left_plan)

#     rospy.sleep(1.0)

#     #open the gripper to drop the fruit    
#     left_gripper.open(block=False)


# if __name__ == '__main__':
#     main()




    