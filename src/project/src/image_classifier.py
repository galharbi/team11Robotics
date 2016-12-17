# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import numpy as np
import scipy
import scipy.misc as misc
import matplotlib.pyplot as plt
import sklearn
from sklearn import svm
from sklearn.utils import shuffle
import scipy.interpolate as interpol


# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    global increment
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    cv2.imwrite('test_image.jpg',cv2_img)
    print("Received an image!")
    rospy.sleep(5)

def extract_multiple_images(path,fruit_type,ct):
    for j in range(1,ct+1):
        im2 = misc.imread('sources/Identifier/identifier2.jpg')
        im1 = misc.imread('%s/%s%s.jpg' %(path,fruit_type,int(j)))
        orb = cv2.ORB(3000)
        pts1,dev1 = orb.detectAndCompute(im1,None)
        pts2,dev2 = orb.detectAndCompute(im2,None)
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(dev1,dev2,k=2)
        best = []
        for m,n in matches:
            if m.distance < .85*n.distance:
                best.append([pts1[m.queryIdx],pts2[m.trainIdx]])
        best = np.array(best)
        best = np.array([np.array([[best[i,0].pt[0],best[i,0].pt[1]] for i in range(len(best))]),
                np.array([[best[i,1].pt[0],best[i,1].pt[1]] for i in range(len(best))])])
        h = cv2.findHomography(best[0],best[1],cv2.RANSAC,5.0)[0]
        im3 = cv2.warpPerspective(im1,h,(im2.shape[1],im2.shape[0]))
        misc.imsave('results/%s/%s%i.jpg'%(fruit_type,fruit_type,j),im3)
    print('finished: %s' % fruit_type)

def extract(path):
    im2 = misc.imread('identifier2.jpg')
    im1 = misc.imread(path)
    orb = cv2.ORB(2500)
    pts1,dev1 = orb.detectAndCompute(im1,None)
    pts2,dev2 = orb.detectAndCompute(im2,None)
    bf = cv2.BFMatcher()
    matches = bf.knnMatch(dev1,dev2,k=2)
    best = []
    for m,n in matches:
        if m.distance < .85*n.distance:
            best.append([pts1[m.queryIdx],pts2[m.trainIdx]])
    best = np.array(best)
    best = np.array([np.array([[best[i,0].pt[0],best[i,0].pt[1]] for i in range(len(best))]),
            np.array([[best[i,1].pt[0],best[i,1].pt[1]] for i in range(len(best))])])
    h = cv2.findHomography(best[0],best[1],cv2.RANSAC,5.0)[0]
    im3 = cv2.warpPerspective(im1,h,(im2.shape[1],im2.shape[0]))
    misc.imsave('test.jpg',im3)
    return im3


def learn_set(source,ct):
    x = []
    for i in range(ct):
        x.append(misc.imread('%s%s.jpg'%(source,i+1)))
    return np.array(x)

def train_fruit_classifiers(data,labels):
    X,tru_y = shuffle(data,labels,random_state=0)
    clfb = svm.SVR(kernel='rbf',tol=1e-3)
    clfb.fit(X,tru_y)
    return clfb

#predicts 1 image at a time
def svm_predict(clfb,im):
    #seperate into 3-channels to predict on
    # assert im.shape[0] == 3
    pred = clfb.predict(im)
    print(np.abs(pred))
    pred = np.round(np.average(pred)).astype(int)
    return np.abs(pred-1)

def classify():
    clfb = np.load('trained_svm_v3.npy')[0]

    # Define your image topic
    image_topic = "/cameras/left_hand_camera/image"
    # Set up your subscriber and define its callback
    sub = rospy.Subscriber(image_topic, Image, image_callback)
    rospy.sleep(2)
    sub.unregister()

    # extract object on paper from iamge
    f = extract('test_image.jpg').astype(float)
    g = (f-np.min(f))/(np.max(f)-np.min(f))
    h1 = (g[:,:,0]-np.average(g[:,:,0]))/np.std(g[:,:,0])
    h2 = (g[:,:,1]-np.average(g[:,:,1]))/np.std(g[:,:,1])
    h3 = (g[:,:,2]-np.average(g[:,:,2]))/np.std(g[:,:,2])
    ex_mage_array = np.array([g[:,:,0],g[:,:,1],g[:,:,2]])
    ex_mage_array = ex_mage_array.reshape((ex_mage_array.shape[0],ex_mage_array.shape[1]*ex_mage_array.shape[2]))
    predict_fruit = svm_predict(clfb,ex_mage_array)
    return predict_fruit
