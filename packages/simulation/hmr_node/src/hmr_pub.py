#!/usr/bin/env python

PKG = 'hmr_node'
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

import torch
from torchvision.transforms import Normalize
from models import hmr, SMPL
from spin import forward
from alphapose_node.msg import AlphaRes
import config
from nav_msgs.msg import Odometry
from utils.renderer import Renderer
import tf



Actual_J_names = {0:'Neck',1:'Head'}
RENDER = 0
J_names = {
    39: 'Pelvis',
    43: 'Head',
    34: 'L_Shoulder',
    33: 'R_Shoulder',    
    35: 'L_Elbow',
    32: 'R_Elbow',
    36: 'L_Wrist',
    31: 'R_Wrist',
    28: 'L_Hip',
    27: 'R_Hip',
    29: 'L_Knee',
    26: 'R_Knee',
    30: 'L_Ankle',        
    25: 'R_Ankle'                          
}


gt_joints={  'actor::actor_pose':0,
             'actor::Head':7,
             'actor::LeftArm':8,
             'actor::RightArm':11,
             'actor::LeftForeArm':9,
             'actor::RightForeArm':12,
             'actor::LeftFingerBase':10,
             'actor::RightFingerBase':13,
             'actor::LeftUpLeg':1,
             'actor::RightUpLeg':4,
             'actor::LeftLeg':2,
             'actor::RightLeg':5,
             'actor::LeftFoot':3,
             'actor::RightFoot':6,
             }

numRobots = sys.argv[1]


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

print('Loading SPIN and HMR params')
device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
model = hmr(config.SMPL_MEAN_PARAMS).to(device)
aircaprl_path = os.environ.get('AIRCAPDIR')
checkpoint = torch.load(aircaprl_path+'/git/SPIN/data/model_checkpoint.pt',map_location=device)
model.load_state_dict(checkpoint['model'], strict=False)    
smpl = SMPL(config.SMPL_MODEL_DIR,
            batch_size=1,
            create_transl=False).to(device)
model.eval()



for k in range(int(numRobots)):
    machine = k+1

    # alpha pose detections publisher
    res_pub.append(rospy.Publisher('machine_'+str(machine)+'/multihmr_joints',PoseArray,queue_size=1))
    # image with projected detections publisher
    res_img_pub.append(rospy.Publisher('machine_'+str(machine)+'/result_img_multihmr',Image,queue_size=1))
    # message for joint detections and probabilities
    res_msg.append(PoseArray())



def callback(img_msg, machine):
    img = bridge.imgmsg_to_cv2(img_msg, 'bgr8')

    crop_box = box_cache.getElemBeforeTime(img_msg.header.stamp)
    delta = 50
    res_msg[machine] = PoseArray()
    try:
        crop_area = crop_box.bbox + np.array([-delta,-delta,delta,delta])        
        if np.sum(crop_area) == 0:
            for k in J_names.keys():
                p = Pose()
                p.position.x = 0
                p.position.y = 0
                p.position.z = 0
                res_msg[machine].poses.append(p)              
        else:
            image_dims = np.shape(img)
            crop_area =  np.maximum(np.minimum(crop_area,np.array([image_dims[1],image_dims[0],image_dims[1],image_dims[0]])),np.array([0,0,0,0]))

            initTime = time.time()
            cropped_image = img[crop_area[1]:crop_area[3],crop_area[0]:crop_area[2]]

            #try:
            (trans_w,rot) = listener.lookupTransform('world','actor::actor_pose', img_msg.header.stamp)
            #except:
                #(trans_w,rot) = listener.lookupTransform('world','actor::actor_pose', rospy.Time(0))   

            #try:
            (trans,rot_w) = listener.lookupTransform('world','actor::Head', img_msg.header.stamp)
            #except:
                #(trans,rot_w) = listener.lookupTransform('world','actor::Head', rospy.Time(0))
            (r,p,y) = tf.transformations.euler_from_quaternion(rot_w)
            rot_w = tf.transformations.quaternion_from_euler(1.57,0,y)
            
            joints3D,img_shape  = forward(cropped_image,model,checkpoint,smpl,trans_w,rot_w,device,RENDER)
            
            for k in J_names.keys():
                    p = Pose()
                    p.position.x = joints3D[0,k,0]
                    p.position.y = joints3D[0,k,1]
                    p.position.z = joints3D[0,k,2]
                    res_msg[machine].poses.append(p)
            if RENDER:
                if res_img_pub[machine].get_num_connections()>0:
                    img_msg = bridge.cv2_to_imgmsg(img_shape,'rgb8')
                    res_img_pub[machine].publish(img_msg)
            
    except:        
        res_msg[machine] = PoseArray()
        for k in range(len(J_names)):
            p = Pose()
            p.position.x = 0
            p.position.y = 0
            p.position.z = 0
            res_msg[machine].poses.append(p)             
        
    res_msg[machine].header.stamp = img_msg.header.stamp
    res_msg[machine].header.frame_id = 'world'
    res_pub[machine].publish(res_msg[machine])
    

if __name__ == '__main__':
    # rospy.init_node('alphapose_'+str(machine))
    rospy.init_node('hmr')
    listener = tf.TransformListener()
    img_sub = [];ch_img = []
    for k in range(int(numRobots)):
        img_sub.append(rospy.Subscriber('firefly_'+str(k+1)+'/firefly_'+str(k+1)+'/xtion/rgb/image_raw',Image,callback, callback_args=k,queue_size=1))
        boxes_sub = message_filters.Subscriber('machine_'+str(k+1)+'/noisy_bbox',AlphaRes);
        box_cache = message_filters.Cache(boxes_sub,500)
        #target_sub = message_filters.Subscriber('/actorpose',Odometry);
        #target_cache = message_filters.Cache(target_sub,500)

    rospy.spin()
