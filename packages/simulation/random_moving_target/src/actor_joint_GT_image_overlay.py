#!/usr/bin/env python
import numpy as np
import rospy
import sys
import cv2
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from alphapose_node.msg import AlphaRes
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import tf
import message_filters
from uav_msgs.msg import uav_pose

#               'actor::actor_pose', 'actor::Hips', 'actor::LHipJoint', 'actor::LeftUpLeg', 'actor::LeftLeg', 'actor::LeftFoot', 'actor::LeftToeBase', 'actor::RHipJoint', 'actor::RightUpLeg',
#               'actor::RightLeg', 'actor::RightFoot', 'actor::RightToeBase', 'actor::LowerBack', 'actor::Spine', 'actor::Spine1', 'actor::Neck', 'actor::Neck1', 'actor::Head', 'actor::LeftShoulder',
#               'actor::LeftArm', 'actor::LeftForeArm', 'actor::LeftHand', 'actor::LeftFingerBase','actor::LeftHandIndex1', 'actor::LThumb', 'actor::RightShoulder', 'actor::RightArm','actor::RightForeArm',
#               'actor::RightHand', 'actor::RightFingerBase', 'actor::RightHandIndex1', 'actor::RThumb'

dict_joints={'actor::actor_pose':0,
             'actor::Head':0,
             'actor::LeftArm':0,
             'actor::RightArm':0,
             'actor::LeftForeArm':0,
             'actor::RightForeArm':0,
             'actor::LeftFingerBase':0,
             'actor::RightFingerBase':0,
             'actor::LeftUpLeg':0,
             'actor::RightUpLeg':0,
             'actor::LeftLeg':0,
             'actor::RightLeg':0,
             'actor::LeftFoot':0,
             'actor::RightFoot':0,
             }
joint_names  = ['actor::actor_pose',
             'actor::Head',
             'actor::LeftArm',
             'actor::RightArm',
             'actor::LeftForeArm',
             'actor::RightForeArm',
             'actor::LeftFingerBase',
             'actor::RightFingerBase',
             'actor::LeftUpLeg',
             'actor::RightUpLeg',
             'actor::LeftLeg',
             'actor::RightLeg',
             'actor::LeftFoot',
             'actor::RightFoot']
n_joints = len(joint_names)


robID = sys.argv[1]



class links_tf():
    def __init__(self):

        ''' FOR  publishes image with ground truth joint overlays '''
        self.res_img_pub = rospy.Publisher('machine_'+str(robID)+'/keypoints_gt',Image,queue_size=1)


        # bounding box publisher
        self.boxes_pub = rospy.Publisher('machine_'+str(robID)+'/noisy_bbox',AlphaRes,queue_size=1)

        '''groundtruth, noisy detection and noisy pose detection publishers'''
        self.res_pub = rospy.Publisher('machine_'+str(robID)+'/groundtruth_joints',AlphaRes,queue_size=1)
        self.res_noisy_pub = rospy.Publisher('machine_'+str(robID)+'/noisy_joints',AlphaRes,queue_size=1)
        self.detection_result_pub = rospy.Publisher('machine_'+str(robID)+'/noisy_detection',Int16,queue_size=1)

        '''Initialize groundtruth'''
        self.res_msg = AlphaRes();self.res_noisy_msg = AlphaRes();self.bbox_msg = AlphaRes();self.detection_msg = Int16()

        self.links = LinkStates()
        self.cam_info = CameraInfo()
        self.actorpose = Odometry()
        self.bridge=CvBridge()

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.Subscriber("/firefly_"+str(robID)+'/firefly_'+str(robID)+"/xtion/rgb/camera_info", CameraInfo, self.camera_info_callback)
        # rospy.Subscriber("/actorpose", PoseStamped, self.actor_callback)
        actor_sub = message_filters.Subscriber("/actorpose", Odometry)
        self.actor_cache = message_filters.Cache(actor_sub, 100)

        machine_pose_sub = message_filters.Subscriber("/machine_"+str(robID)+'/pose/groundtruth', uav_pose)
        self.machine_cache = message_filters.Cache(machine_pose_sub, 100)
        self.machine_cache.registerCallback(self.machine_callback)
        image_sub = rospy.Subscriber('/firefly_'+str(robID)+'/firefly_'+str(robID)+'/xtion/rgb/image_raw',Image,self.img_callback)
        # ts = message_filters.TimeSynchronizer(image_sub, 10)
        # ts.registerCallback(self.img_callback)
        # rospy.Subscriber('/firefly_'+str(robID)+'/xtion/rgb/image_raw',Image,self.img_callback)
        self.rate = rospy.Rate(10)  # 10hz
        self.prev = [0,0,0,0]
        self.joints = np.zeros((n_joints,4))

    def camera_info_callback(self,msg):
        self.cam_info = msg
        self.P = np.array([self.cam_info.P[0:4],self.cam_info.P[4:8],self.cam_info.P[8:12]])

    def machine_callback(self,msg):
        self.machine_data = msg

    def actor_callback(self,msg):
        self.actorpose = msg
        # for k in range(n_joints):
        #     try:
        #         trans = self.tfBuffer.lookup_transform('firefly_'+str(robID)+'/xtion_rgb_optical_frame',joint_names[k], rospy.Time.now())
        #         self.joints[k,:] = np.array([trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z,1])
        #     except:
        #         continue


    def img_callback(self,msg):
        self.stamp = msg.header.stamp
        self.img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        new_img= self.img
        image_projection_list = []
        image_projection_noisy_list = []
        bbox_min_x = 100000;bbox_max_x = 0
        bbox_min_y = 100000;bbox_max_y = 0
        detection = 0


        for k in range(n_joints):
            trans = self.tfBuffer.lookup_transform('firefly_'+str(robID)+'/xtion_rgb_optical_frame',joint_names[k], self.stamp)
            self.joints[k,:] = np.array([trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z,1])
            try:
                if np.sum(self.joints[k,:]) > 0.01:
                    projected_point = np.dot(self.P,self.joints[k,:])
                    self.prev = projected_point
                else:
                    projected_point = self.prev
                    continue
            except:
                print('still not received')


            image_projection = [projected_point[0]/(projected_point[2]+0.001),projected_point[1]/(projected_point[2]+0.001)]
            # print(image_projection)
            image_projection_list.extend(image_projection)
            '''ADD NOISE TO JOINTS'''

            actor = self.actor_cache.getElemBeforeTime(self.stamp)

            d = np.linalg.norm(np.array([actor.pose.pose.position.x,actor.pose.pose.position.y])-\
                            np.array([self.machine_data.position.x,self.machine_data.position.y]))
            if d>10:
                mu, sigma = 0, 50
            elif d<5:
                mu, sigma = 0, 50
            else:
                mu, sigma = 0, 0
             # mean and standard deviation
            s = np.random.normal(mu, sigma, 2)
            image_projection_noisy = [image_projection[0]+s[0],image_projection[1]+s[1]]

            if k ==0:
                '''Check if the root joint is inside or outside the image'''
                if image_projection_noisy[0]<=0 or image_projection_noisy[1]<=0 or image_projection_noisy[0]>=np.shape(self.img)[0] or \
                image_projection_noisy[1]>=np.shape(self.img)[1]:
                    detection+=0
                    image_projection_noisy_list.extend([0.0,0.0])
                else:
                    image_projection_noisy_list.extend(image_projection_noisy)
                    detection+=1
                bbox_min_x = image_projection_noisy[0];bbox_min_y = image_projection_noisy[1];
                self.detection_result_pub.publish(self.detection_msg)

                new_img = cv2.circle(self.img,(int(image_projection_noisy[0]),int(image_projection_noisy[1])),10, (0,0,255), -1)
            else:
                if image_projection_noisy[0]<=0 or image_projection_noisy[1]<=0 or image_projection_noisy[0]>=np.shape(self.img)[0] or image_projection_noisy[1]>=np.shape(self.img)[1]:
                    detection+=0
                    image_projection_noisy_list.extend([0.0,0.0])
                else:
                    detection+=1
                    image_projection_noisy_list.extend(image_projection_noisy)
                new_img = cv2.circle(new_img,(int(image_projection_noisy[0]),int(image_projection_noisy[1])),10, (255,100,0), -1)


            '''Find bounding box coordinates'''
            if detection > 0:
                if image_projection_noisy[0]<bbox_min_x:
                    bbox_min_x = image_projection[0]
                if image_projection_noisy[1]<bbox_min_y:
                    bbox_min_y = image_projection[1]

                if image_projection_noisy[0]>bbox_max_x :
                    bbox_max_x = image_projection[0]
                if image_projection_noisy[1]>bbox_max_y:
                    bbox_max_y = image_projection[1]



        if detection > 0:
            self.detection_msg.data = 1
        else:
            self.detection_msg.data = 0

        '''Plot Bounding Box'''
        if self.detection_msg.data == 1:
            self.bbox_msg.bbox = [bbox_min_x,bbox_min_y,bbox_max_x,bbox_max_y]
            cv2.rectangle(new_img, (int(self.bbox_msg.bbox[0]), int(self.bbox_msg.bbox[1])), \
                (int(self.bbox_msg.bbox[2]), int(self.bbox_msg.bbox[3])), (255,0,0), 1)
        else:
            self.bbox_msg.bbox = [0,0,0,0]

        '''Publish groundtruth and noisy_joints'''
        self.res_msg.header.stamp = self.stamp
        self.res_msg.res = image_projection_list
        self.res_noisy_msg.header.stamp = self.stamp
        self.bbox_msg.header.stamp = self.res_noisy_msg.header.stamp
        if len(image_projection_noisy_list)==28:
            self.res_noisy_msg.res = image_projection_noisy_list
            self.res_noisy_pub.publish(self.res_noisy_msg)
        # else:
        #     self.res_noisy_msg.res = list(np.zeros((1,28),dtype="float32"))


        self.res_pub.publish(self.res_msg)


        self.boxes_pub.publish(self.bbox_msg)

        img_msg = self.bridge.cv2_to_imgmsg(new_img, 'bgr8')
        '''Only if someone subscribes to this image'''
        if self.res_img_pub.get_num_connections()>0:
            rospy.loginfo_once('Subscriber to gt joints visualization in image plane');
            self.res_img_pub.publish(img_msg)



if __name__ == '__main__':
    rospy.init_node('machine_'+str(robID)+'_actor_joint_GT_image_overlay')
    a = links_tf()
    rospy.spin()
