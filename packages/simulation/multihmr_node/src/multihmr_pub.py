#!/usr/bin/env python

PKG = 'multihmr_node'
import roslib; roslib.load_manifest(PKG)
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import message_filters
import sys,os

from std_msgs.msg import Int16

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose,PoseArray
import numpy as np
import time

from multihmr import main
from alphapose_node.msg import AlphaRes
from nav_msgs.msg import Odometry
import tf
import tensorflow
from src_ortho.RunModel import RunModel
from src_ortho.tf_smpl.batch_smpl import SMPL
import src_ortho.config
from absl import flags
tens_shape = tensorflow.placeholder(tensorflow.float32, shape=[1, 10])
tens_pose = tensorflow.zeros([1, 72])

sess = tensorflow.Session(config=tensorflow.ConfigProto(
    device_count = {'GPU': 1},gpu_options = tensorflow.GPUOptions(per_process_gpu_memory_fraction=0.333)
))
sess.run(tensorflow.global_variables_initializer())
sess.run(tensorflow.local_variables_initializer())
num_views = 2
config = flags.FLAGS
aircaprl_path = os.environ.get('AIRCAPDIR')
config.load_path = aircaprl_path+'/git/HumanMultiView/models/model.ckpt-55124'
config.batch_size = 1
model = RunModel(config, 4, num_views, sess=sess)
smpl = SMPL(config.smpl_model_path)


# sess = tensorflow.Session(config=tensorflow.ConfigProto(
#     device_count = {'GPU': 1}
# ))


RENDER = 0
J_names = {0:'R_Ankle',1:'R_Knee',2:'R_Hip',3:'L_Hip',4:'L_Knee',5:'L_Ankle',6: 'R_Wrist',7: 'R_Elbow',8:'R_Shoulder',9:'L_Shoulder',10: 'L_Elbow',11:'L_Wrist',12:'Spine',13:'Head'}


robotID = sys.argv[1]


bridge=CvBridge()
# bounding box publisher array
boxes_pub = []
res_pub = []
res_img_pub = []
detection_result_pub = []
res_msg = []
boxes_msg = []

detection_msg = Int16()
detection = 0

numRobots = 1

for k in range(int(numRobots)):
    machine = k+1

    # alpha pose detections publisher
    res_pub.append(rospy.Publisher('/multihmr_joints',PoseArray,queue_size=1))
    # image with projected detections publisher
    res_img_pub.append(rospy.Publisher('machine_'+str(robotID)+'/result_img_multihmr',Image,queue_size=1))
    # message for joint detections and probabilities
    res_msg.append(PoseArray())



def callback(img_msg, machine):

    img = bridge.imgmsg_to_cv2(img_msg, 'bgr8')
    img_neighbor = bridge.imgmsg_to_cv2(neighbor_image_cache.getElemBeforeTime(img_msg.header.stamp), 'bgr8')

    crop_box = box_cache.getElemBeforeTime(img_msg.header.stamp)
    crop_box_neighbor = box_neighbor_cache.getElemBeforeTime(img_msg.header.stamp)


    delta = 50
    res_msg[machine] = PoseArray()

    #try:
    crop_area = crop_box.bbox + np.array([-delta,-delta,delta,delta])
    crop_area_neighbor = crop_box_neighbor.bbox + np.array([-delta,-delta,delta,delta])
    if np.sum(crop_area) == 0 and np.sum(crop_area_neighbor) == 0:
        for k in J_names.keys():
            p = Pose()
            p.position.x = 0
            p.position.y = 0
            p.position.z = 0
            res_msg[machine].poses.append(p)
    else:
        image_dims = np.shape(img)
        crop_area =  np.maximum(np.minimum(crop_area,np.array([image_dims[1],image_dims[0],image_dims[1],image_dims[0]])),np.array([0,0,0,0]))
        cropped_image = img[crop_area[1]:crop_area[3],crop_area[0]:crop_area[2]]

        image_dims = np.shape(img_neighbor)
        crop_area_neighbor =  np.maximum(np.minimum(crop_area_neighbor,np.array([image_dims[1],image_dims[0],image_dims[1],image_dims[0]])),np.array([0,0,0,0]))
        cropped_image_neighbor = img_neighbor[crop_area_neighbor[1]:crop_area_neighbor[3],crop_area_neighbor[0]:crop_area_neighbor[2]]

        (trans_w,rot) = listener.lookupTransform('world','actor::actor_pose', img_msg.header.stamp)
        (trans,rot_w) = listener.lookupTransform('world','actor::Head', img_msg.header.stamp)
        (r,p,y) = tf.transformations.euler_from_quaternion(rot_w)
        rot_w = tf.transformations.quaternion_from_euler(1.57,0,y)

        joints3D,img_shape  = main(cropped_image,cropped_image_neighbor,sess,model,smpl,tens_pose,tens_shape,trans_w,rot_w)

        for k in J_names.keys():
                p = Pose()
                p.position.x = joints3D[0,k,0]+ trans_w[0]
                p.position.y = joints3D[0,k,1]+ trans_w[1]+0.2
                p.position.z = joints3D[0,k,2]+ trans_w[2]-0.3
                res_msg[machine].poses.append(p)
        if RENDER:
            if res_img_pub[machine].get_num_connections()>0:
                img_msg = bridge.cv2_to_imgmsg(img_shape,'rgb8')
                res_img_pub[machine].publish(img_msg)

    res_msg[machine].header.stamp = img_msg.header.stamp
    res_msg[machine].header.frame_id = 'world'
    res_pub[machine].publish(res_msg[machine])


if __name__ == '__main__':
    if int(robotID)==1:
        neighborID=2
    else:
        neighborID=1
    rospy.init_node('multihmr'+str(robotID))
    listener = tf.TransformListener()
    img_sub = [];ch_img = []

    '''Image Subscribers'''
    img_sub.append(rospy.Subscriber('firefly_'+str(robotID)+'/firefly_'+str(robotID)+'/xtion/rgb/image_raw',Image,callback, callback_args=0,queue_size=1))
    neighbor_image = message_filters.Subscriber('firefly_'+str(neighborID)+'/firefly_'+str(neighborID)+'/xtion/rgb/image_raw',Image)
    neighbor_image_cache = message_filters.Cache(neighbor_image,100)


    '''GT Bounding Box Subscribers'''
    boxes_sub = message_filters.Subscriber('machine_'+str(robotID)+'/noisy_bbox',AlphaRes);
    box_cache = message_filters.Cache(boxes_sub,500)
    boxes_neighbor_sub = message_filters.Subscriber('machine_'+str(neighborID)+'/noisy_bbox',AlphaRes);
    box_neighbor_cache = message_filters.Cache(boxes_neighbor_sub,500)

    rospy.spin()
