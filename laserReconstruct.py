#!/usr/bin/python

# Author: Andrew Cunningham
# Description: RPI computer vision class final project, obtaining 3D coordinates
# by using a laser and a camera to scan the environment
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import Image as Picture
import time
import tf

from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

# Used to find the red laser line on an image
# sample rate is the number of pixels to skip over in selecting a new pixel
def findLine(imageIn, sampleRate = 1, plotStuff = False):
    #print np.size(imageIn,0)
    im=Picture.fromarray(imageIn)
    threshold = 50
    r, g, b = im.split()
    mask = r.point(lambda i: i>threshold and 255)
    
    # Optional plot
    if plotStuff:    
        plt.figure()
        plt.imshow(im, cmap=plt.cm.gray)
        plt.figure()
        plt.imshow(mask, cmap=plt.cm.gray)
        
    numCols,numRows = mask.size
    data = np.asarray(mask)
    
    resultsList = []
    for i in range(0,numRows-1):
        for j in range(0,numCols-1):
            if data[i,j] > 250:
                resultsList.append((j,i))

    # Down sample for less points to plot
    downSampledResults = []
    for i in range(len(resultsList)):         
        if i%sampleRate == 0:
            downSampledResults.append(resultsList[i])

    return downSampledResults

# image_converter handles getting an image from a ROS topic and converting it
# into an array type that find_line can use
class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cameras/left_hand_camera/image",Image,self.callback)
        self.newImage = False        
        
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
            print(e)

        self.npImage = np.asarray(cv_image)
        self.newImage = True
                
        # if we want to display
        #cv2.imshow("Image window", cv_image)
        #cv2.waitKey(3)
                
    def getImage(self):
        self.newImage = False
        return self.npImage
        
# Given the inrinsic parameters of a camera and the arm it is attached to
# find the projcetion matrix
# default W is for left camera frame
def getPFull(R,T, W = np.array([[406.00431,0,590.6386],[0,406.0743,420.9455],[0,0,1]])):    
    M = np.append(R,T.reshape(3,1),1)
    P = np.dot(W,M)
    return P
    
def activeStereo3Dfrom2D( p2D, P, planeA, planeB, planeC, planeD):
    #activeStereo3Dfrom2D reconstruct 3D points from 2D image point generated
    # Pixel Coordinates
    c = p2D[0]
    r = p2D[1]
      
    # Form system of equations Ax = b where x is [X,Y,lambda]
    A = np.zeros([3,3])
    A[0]= np.array([-P[0,0]+P[0,2]*planeA/planeC, -P[0,1]+P[0,2]*planeB/planeC, c])
    A[1]= np.array([-P[1,0]+P[1,2]*planeA/planeC, -P[1,1]+P[1,2]*planeB/planeC, r])
    A[2]= np.array([-P[2,0]+P[2,2]*planeA/planeC, -P[2,1]+P[2,2]*planeB/planeC, 1])
    b = np.array([[P[0,2]*planeD/planeC+P[0,3]],[P[1,2]*planeD/planeC+P[1,3]],[P[2,2]*planeD/planeC+P[2,3]]])
    b = b.reshape(3,1)    
    # Solve for X,Y and lambda
    x = np.dot(np.linalg.inv(A),b)   

    pX = x[0] # X COORD
    pY = x[1] # Y COORD
    pZ = (planeD+planeA*pX + planeB*pY)/float(-planeC)
    
    #MANUAL ROTATION 
   
    point = np.array([[pX],[pY],[pZ]])
    point=point.reshape(3,1)
#    rot = np.array([[0,1,0],[0,0,1],[1,0,0]])
#    rot= np.transpose(rot)
    #point = np.dot(rot,point)   
    
    return point
    #rotPoint = np.dot(rot,point)
    #return rotPoint
    

# Get plane parameters for a given orientation of end effector and translation
# to end effector
def getPlaneParams(R,T):
    # we know that the x direction in the manipulator frame is the direction
    # perpendicular to the laser plane through inspection of the baxter model
    # and the transform tree
    ex = np.array([[1],[0],[0]])
    normal = np.dot(R, ex)
    planeA = normal[0]
    planeB = normal[1]
    planeC= normal[2]
    planeD = -np.dot(normal.reshape(1,3),T)
    return (planeA,planeB,planeC,planeD)

# plot 3D results in scatter plot, this function will update the plot
# each time it is called
def plot3DResults(ax, reconstructed3D):
    x = []
    y = []
    z = []
    for point in reconstructed3D:
        x.append(point[0])
        y.append(point[1])
        z.append(point[2])
        
    ax.clear()
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.scatter(x, y, z, c='r', marker='o')
    plt.draw()
    plt.pause(0.01)   

def getTransform(listener, frame1, frame2):
    try:
        position, quaternion= listener.lookupTransform(frame1, frame2, rospy.Time(0))
        homogenousMatrix= listener.fromTranslationRotation(position, quaternion)
    except:
        print "couldnt transform"
        time.sleep(.5)
        return (-1,-1)
    rotation = homogenousMatrix[0:3,0:3]
    translation = homogenousMatrix[0:3,3]
    return rotation, translation

def transformPoint(target,source,point):
   R,T=getTransform(target,source)
   return np.dot(R,point)+T
   
def testPlanePoint(planeA,planeB,planeC,planeD,point):
    return planeA*point[0]+planeB*point[1]+planeC*point[2]+planeD
    
def main():
    rospy.init_node('baxter_laser_reconstruction')
    ic = image_converter()
    pointcloud_publisher = rospy.Publisher("/my_pcl_topic", PointCloud2)    
    
    start = time.time()    

    listener= tf.TransformListener()
    
    # get pipeline filled with transforms (first couple of transforms wont work)
    while time.time() - start < 1:
       getTransform(listener,'base','right_hand')
       
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')  
    
    while time.time() - start < 800:
        reconstructed3D = []
        if ic.newImage: # wait till we have a new image to do anything
            detected2DPoints = findLine(ic.getImage(), sampleRate = 1)
            
            Rleft,Tleft = getTransform(listener,'base','left_hand_camera')
            Rtest, Ttest = getTransform(listener,'left_hand_camera','base')
            Rright,Tright = getTransform(listener,'base','right_hand')
            #Rright = np.transpose(Rright)
            planeA,planeB,planeC,planeD = getPlaneParams(Rright,Tright)
            P = getPFull(Rtest,Ttest)           
            
            Tright = Tright.reshape(3,1) 
            Tleft = Tleft.reshape(3,1)
            
            for detected2DPoint in detected2DPoints: 
                new3D = activeStereo3Dfrom2D(detected2DPoint,P,planeA,planeB,planeC,planeD)
                reconstructed3D.append(new3D)

            # Get ready to publish point cloud
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'base'
            
            # ADD LASER PLANE POINT
            offsetR = np.array([[.1],[0],[0]])
            offsetL = np.array([[.1],[0],[0]])
            testPointR = np.dot(Rright,offsetR)+Tright
            testPointL = np.dot(Rleft,offsetL)+Tleft
            reconstructed3D.append(testPointR)
            reconstructed3D.append(testPointL)
            
            testPoint1 = np.array([[0],[.1],[.1]])
            testPoint2 = np.array([[0],[-.1],[.1]])
            testPoint3 = np.array([[0],[0],[.2]])
            
            # Transform to base coords
            testPoint1 = np.dot(Rright,testPoint1)+Tright
            testPoint2 = np.dot(Rright,testPoint2)+Tright
            testPoint3 = np.dot(Rright,testPoint3)+Tright            
            
            # Solve for the X coord of the plane
            testPoint1[0] = (planeD + planeB*testPoint1[1] + planeC*testPoint1[2])/float(-planeA)
            testPoint2[0] = (planeD + planeB*testPoint2[1] + planeC*testPoint2[2])/float(-planeA)
            testPoint3[0] = (planeD + planeB*testPoint3[1] + planeC*testPoint3[2])/float(-planeA)
            
            reconstructed3D.append(testPoint1)
            reconstructed3D.append(testPoint2)
            reconstructed3D.append(testPoint3)
            
            scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, reconstructed3D)          
            pointcloud_publisher.publish(scaled_polygon_pcl)
            
#            plot3DResults(ax, reconstructed3D)

if __name__ == "__main__":
    main()
