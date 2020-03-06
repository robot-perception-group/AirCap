#!/usr/bin/env python
import numpy as np
import rospy
import sys
import cv2
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import CameraInfo, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseArray, Pose, PoseWithCovarianceStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import tf
import message_filters
from threading import Lock, Event, Condition
import time

#               'actor::actor_pose', 'actor::Hips', 'actor::LHipJoint', 'actor::LeftUpLeg', 'actor::LeftLeg', 'actor::LeftFoot', 'actor::LeftToeBase', 'actor::RHipJoint', 'actor::RightUpLeg',
#               'actor::RightLeg', 'actor::RightFoot', 'actor::RightToeBase', 'actor::LowerBack', 'actor::Spine', 'actor::Spine1', 'actor::Neck', 'actor::Neck1', 'actor::Head', 'actor::LeftShoulder',
#               'actor::LeftArm', 'actor::LeftForeArm', 'actor::LeftHand', 'actor::LeftFingerBase','actor::LeftHandIndex1', 'actor::LThumb', 'actor::RightShoulder', 'actor::RightArm','actor::RightForeArm',
#               'actor::RightHand', 'actor::RightFingerBase', 'actor::RightHandIndex1', 'actor::RThumb'

dict_joints={'actor::actor_pose':None,
             'actor::Head':None,
             'actor::LeftArm':None,
             'actor::RightArm':None,
             'actor::LeftForeArm':None,
             'actor::RightForeArm':None,
             'actor::LeftFingerBase':None,
             'actor::RightFingerBase':None,
             'actor::LeftUpLeg':None,
             'actor::RightUpLeg':None,
             'actor::LeftLeg':None,
             'actor::RightLeg':None,
             'actor::LeftFoot':None,
             'actor::RightFoot':None,
             }

joint_names  = dict_joints.keys()

n_joints = len(dict_joints)

robID = sys.argv[1]

class callback_links:
    def __init__(self,id):
        self.id = id

    def links_callback_list(self,msg):
        self.links = msg
        names = self.links.name
        n_links = len(names)
        time_stamp = rospy.Time.now()
        self.tf_stamp=time_stamp
        if dict_joints[joint_names[self.id]] is None:
                for k in range(n_links):
                    if names[k] == joint_names[self.id]:
                        dict_joints[joint_names[self.id]] = k
                        print('Joint '+joint_names[self.id]+' has ID:' +str(k))
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        if joint_names[self.id] != 'actor::actor_pose':
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = 'actor::actor_pose'
            t.child_frame_id = joint_names[self.id]
            t.transform.translation.x = self.links.pose[dict_joints[joint_names[self.id]]].position.x
            t.transform.translation.y =  self.links.pose[dict_joints[joint_names[self.id]]].position.y
            t.transform.translation.z =  self.links.pose[dict_joints[joint_names[self.id]]].position.z
            t.transform.rotation.x = 0
            t.transform.rotation.y = 0
            t.transform.rotation.z = 0
            t.transform.rotation.w = 1
            br.sendTransform(t)
        else:
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "world"
            t.child_frame_id = joint_names[self.id]
            t.transform.translation.x = self.links.pose[dict_joints[joint_names[self.id]]].position.x
            t.transform.translation.y = self.links.pose[dict_joints[joint_names[self.id]]].position.y
            t.transform.translation.z = self.links.pose[dict_joints[joint_names[self.id]]].position.z
            t.transform.rotation.x = 0
            t.transform.rotation.y = 0
            t.transform.rotation.z = 0
            t.transform.rotation.w = 1
            br.sendTransform(t)



class links_tf:
    def __init__(self):
        link_sub = [];
        _cb = []
        for k in range(n_joints):
            _cb.append(callback_links(k))
            link_sub.append(rospy.Subscriber("/gazebo/link_states", LinkStates,_cb[k].links_callback_list))
        # self.cache = message_filters.Cache(link_sub, 100, allow_headerless=True)
        # self.cache.registerCallback(self.links_callback)
            # self.rate = rospy.Rate(40)
        ''' FOR  publishes image with ground truth joint overlays '''
        # #DEBUG
        # self.res_img_pub = rospy.Publisher('machine_'+str(robID)+'/keypoints_gt_debug',Image,queue_size=1)
        # self.proj_pub = rospy.Publisher('machine_'+str(robID)+'/gt_joint_observations',PoseArray,queue_size=1)
        #
        # self.links = LinkStates()
        # self.cam_info = CameraInfo()
        # self.actorpose = Odometry()
        # self.bridge=CvBridge()
        # self.projection_msg = PoseArray()
        # for k in range(n_joints):
        #     self.projection_msg.poses.append(Pose())
        #
        # #DEBUG
        # self.tfBuffer = tf2_ros.Buffer()
        # self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # rospy.Subscriber("/firefly_"+str(robID)+"/firefly_"+str(robID)+"/xtion/rgb/camera_info" ,CameraInfo,self.camera_info_callback)
        # img_sub = message_filters.Subscriber('/firefly_'+str(robID)+'/firefly_'+str(robID)+'/xtion/rgb/image_raw', Image)
        # proj_sub = message_filters.Subscriber('machine_'+str(robID)+'/gt_joint_observations',PoseArray)
        # self.obs_cache = message_filters.Cache(proj_sub, 100)
        # # ts = message_filters.TimeSynchronizer([img_sub,proj_sub],1000)
        # # self.obs_cache.registerCallback(self.img_callback)
        # img_sub.registerCallback(self.img_callback)
        # actor_sub = message_filters.Subscriber("/actorpose", Odometry)
        # self.actor_cache = message_filters.Cache(actor_sub, 100)
        # self.actor_cache.registerCallback(self.actor_callback)
        #
        # pose_sub = message_filters.Subscriber('/firefly_'+str(robID)+'/ground_truth/pose_with_covariance', PoseWithCovarianceStamped)
        # self.pose_cache = message_filters.Cache(pose_sub, 100)
        # self.pose_cache.registerCallback(self.pose_callback)
        #
        #
        # '''Define Transformation Matrices'''
        # # Robot in Xtion_link
        # self.T_R_Xl = tf.transformations.euler_matrix(0,-0.785,0,axes='sxyz')
        # self.T_R_Xl[0:4,3] = [-0.078, 0.000, -0.177, 1]
        #
        # # Xtion_link in Xtion_rgb_frame
        # self.T_Xl_Xrf = tf.transformations.euler_matrix(0,0,0,axes='sxyz')
        # self.T_Xl_Xrf[0:4,3] = [0.0, -0.022, 0.0, 1]
        #
        # # Xtion_rgb_frame in Xtion_rgb_optical_frame
        # self.T_Xrf_Xrof = tf.transformations.euler_matrix(1.571, -1.571, 0.0,axes='sxyz')
        # self.T_Xrf_Xrof[0:4,3] = [0.0, 0.0 , 0.0, 1]
        #
        # self.T_R_Xrof = np.dot(self.T_Xrf_Xrof,np.dot(self.T_Xl_Xrf,self.T_R_Xl))
        # print(self.T_R_Xl)
        # self.counter = 0



    def links_callback(self,msg):
        self.links = msg
        names = self.links.name
        n_links = len(names)
        time_stamp = rospy.Time.now()
        self.tf_stamp=time_stamp

        counter = 0
        for k in range(n_links):
            if names[k][0:5]=='actor' and names[k] in dict_joints:
                dict_joints[names[k]] = k
                br = tf2_ros.TransformBroadcaster()
                t = TransformStamped()

                if names[k] == 'actor::actor_pose':
                    t.header.stamp = rospy.Time.now()
                    t.header.frame_id = "world"
                    t.child_frame_id = names[k]
                    t.transform.translation.x = self.links.pose[k].position.x
                    t.transform.translation.y = self.links.pose[k].position.y
                    t.transform.translation.z = self.links.pose[k].position.z
                    t.transform.rotation.x = 0
                    t.transform.rotation.y = 0
                    t.transform.rotation.z = 0
                    t.transform.rotation.w = 1
                    br.sendTransform(t)
                else :
                    t.header.stamp = rospy.Time.now()
                    t.header.frame_id = 'actor::actor_pose'
                    t.child_frame_id = names[k]
                    t.transform.translation.x = self.links.pose[k].position.x
                    t.transform.translation.y =  self.links.pose[k].position.y
                    t.transform.translation.z =  self.links.pose[k].position.z
                    t.transform.rotation.x = 0
                    t.transform.rotation.y = 0
                    t.transform.rotation.z = 0
                    t.transform.rotation.w = 1
                    br.sendTransform(t)








    def camera_info_callback(self,msg):
        self.cam_info = msg




    def actor_callback(self,msg):
        self.actorpose = msg


    def img_callback(self,msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        new_img = self.img
        dict_ind = {v:k for k,v in dict_joints.items()}
        for k in range(n_joints):
            # This publishes the ground truth joint positions on the image plane
            trans = self.tfBuffer.lookup_transform("firefly_"+str(robID)+'/xtion_rgb_optical_frame',joint_names[k], msg.header.stamp,rospy.Duration(1.0))
            P = np.array([self.cam_info.P[0:4],self.cam_info.P[4:8],self.cam_info.P[8:12]])
            projected_point = np.dot(P,np.array([trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z,1]))
            image_projection = [projected_point[0]/projected_point[2],projected_point[1]/projected_point[2]]

            self.projection_msg.poses[k].position.x = int(image_projection[0]);
            self.projection_msg.poses[k].position.y = int(image_projection[1]);
            self.projection_msg.poses[k].position.z = dict_joints[joint_names[k]];

            '''Only if someone subscribes to this image'''
            if self.res_img_pub.get_num_connections()>0:
                if joint_names[k]=='actor::actor_pose':
                    new_img = cv2.circle(self.img,(int(image_projection[0]),int(image_projection[1])),10, (0,0,255), -1)
                else:
                    new_img = cv2.circle(new_img,(int(image_projection[0]),int(image_projection[1])),10, (255,0,0), -1)

        self.projection_msg.header.stamp = msg.header.stamp
        self.proj_pub.publish(self.projection_msg)

        '''Only if someone subscribes to this image'''
        if self.res_img_pub.get_num_connections()>0:
            rospy.loginfo_once('Subscriber to gt joints visualization in image plane')
            img_msg = self.bridge.cv2_to_imgmsg(new_img, 'bgr8')
            self.res_img_pub.publish(img_msg)

        # rospy.logdebug('image:'+str(msg.header.stamp)+' tf:'+str(self.stamp)+' diff:'+str((self.stamp-msg.header.stamp).to_sec()))

    def pose_callback(self, msg):
        # projection_msg = self.cache.getElemBeforeTime(msg.header.stamp)
        (r,p,y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        T_R_W = tf.transformations.euler_matrix(r,p,y,axes='sxyz')
        T_R_W[0:4,3] = [msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z,1]
        T_W_R = np.linalg.inv(T_R_W)
        T_W_Xrof = np.dot(self.T_R_Xrof,T_W_R)
        actor_pos = np.array([self.links.pose[1].position.x,self.links.pose[1].position.y,self.links.pose[1].position.z,1])
        p_A_Xrof = np.dot(T_W_Xrof,actor_pos)
        #DEBUG
        # '''Only if someone subscribes to this image'''
        # if self.res_img_pub.get_num_connections()>0:
        #     P = np.array([self.cam_info.P[0:4],self.cam_info.P[4:8],self.cam_info.P[8:12]])
        #     projected_point = np.dot(P,p_A_Xrof)
        #     image_projection = [projected_point[0]/projected_point[2],projected_point[1]/projected_point[2]]
        #     new_img = cv2.circle(self.img,(int(image_projection[0]),int(image_projection[1])),10, (0,0,255), -1)
        #
        #     rospy.loginfo_once('Subscriber to gt joints visualization in image plane')
        #     img_msg = self.bridge.cv2_to_imgmsg(new_img, 'bgr8')
        #     self.res_img_pub.publish(img_msg)




if __name__ == '__main__':
    rospy.init_node('machine_'+str(robID)+'_actor_world2cam')
    a = links_tf()
    rospy.spin()
