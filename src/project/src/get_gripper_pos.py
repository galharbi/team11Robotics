#!/usr/bin/env python
import rospy
import tf
import numpy as np



def main():
    rospy.init_node('gripper_pos')
    listener = tf.TransformListener()
    rospy.sleep(2.0)
    rate = rospy.Rate(10)
    pos_orients = []

    while not rospy.is_shutdown():
        r = raw_input('Please press Enter to record current gripper position:')
        if r == 'b':
            break
        pos_orient = listener.lookupTransform('/base', '/left_gripper', rospy.Time(0.0))
        pos_orients.append(pos_orient)
        print('Saving Positions')
        np.save('pos_orients',pos_orients)
    return None


if __name__ == '__main__':
    main()
    