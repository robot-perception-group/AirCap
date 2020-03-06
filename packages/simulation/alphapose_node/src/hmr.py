#!/usr/bin/env python

PKG = 'alphapose_node'
import roslib; roslib.load_manifest(PKG)
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import message_filters
import sys
from AlphaPose.dataloader1 import DetectionLoader, DetectionProcessor, Mscoco
from AlphaPose.SPPE.src.main_fast_inference import *
from AlphaPose.opt import opt
from AlphaPose.yolo.preprocess import prep_image
from AlphaPose.SPPE.src.utils.eval import getPrediction
from AlphaPose.pPose_nms import pose_nms
from AlphaPose.fn import vis_frame, vis_frame_fast
from std_msgs.msg import Int16
import torch
from alphapose_node.msg import AlphaRes
from sensor_msgs.msg import Image
import numpy as np
import time

import torch
from torchvision.transforms import Normalize
from SPIN.models import hmr, SMPL

# machine = opt.machine
num_machines = opt.num_machines

det_loader = DetectionLoader()
det_processor = DetectionProcessor(det_loader)
pose_dataset = Mscoco()
pose_model = InferenNet_fast(4 * 1 + 1, pose_dataset)
pose_model.cuda()
pose_model.eval()

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


for k in range(int(num_machines)):
    machine = k+1

    detection_result_pub.append(rospy.Publisher('machine_'+str(machine)+'/detection',Int16,queue_size=1))
    # bounding box publisher
    boxes_pub.append(rospy.Publisher('machine_'+str(machine)+'/result_bbox',AlphaRes,queue_size=1))
    # alpha pose detections publisher
    res_pub.append(rospy.Publisher('machine_'+str(machine)+'/result_alpha',AlphaRes,queue_size=1))
    # image with projected detections publisher
    res_img_pub.append(rospy.Publisher('machine_'+str(machine)+'/result_img_alpha',Image,queue_size=1))
    # message for joint detections and probabilities
    res_msg.append(AlphaRes())

    # message for bounding box
    boxes_msg.append(AlphaRes())



def camera_info_callback(self,msg):
    self.cam_info = msg
    self.P = np.array([self.cam_info.P[0:4],self.cam_info.P[4:8],self.cam_info.P[8:12]])

def callback(img_msg, machine):
    rospy.loginfo(img_msg.header.stamp)
    rospy.loginfo(machine)

    try:
        img = bridge.imgmsg_to_cv2(img_msg, 'bgr8')
    except:
        import ipdb;ipdb.set_trace()

    crop_box = box_cache.getElemBeforeTime(img_msg.header.stamp)
    delta = 50
    crop_area = crop_box.bbox + np.array([-delta,-delta,delta,delta])
    image_dims = np.shape(img)
    crop_area =  np.maximum(np.minimum(crop_area,np.array([image_dims[1],image_dims[0],image_dims[1],image_dims[0]])),np.array([0,0,0,0]))

    initTime = time.time()

    img_k, orig_img_k, im_dim_list_k = prep_image(img[crop_area[1]:crop_area[3],crop_area[0]:crop_area[2]], int(opt.inp_dim))
    #img_k, orig_img_k, im_dim_list_k = prep_image(img, int(opt.inp_dim))
    im_dim_list = [im_dim_list_k]
    im_dim_list = torch.FloatTensor(im_dim_list).repeat(1,2)

    (inps, orig_img, boxes, scores, pt1, pt2) = det_processor.forward(img_k, orig_img_k, im_dim_list)

    # print(time.time()-initTime)

    if boxes is not None:

        boxes_numpy = boxes.numpy().astype(np.int32)
        datalen = inps.size(0)
        inps_j = inps.cuda()

        hm_j = pose_model(inps_j).cpu()

        preds_hm, preds_img, preds_scores = getPrediction(hm_j.detach(), pt1, pt2, opt.inputResH, opt.inputResW, opt.outputResH, opt.outputResW)
        result = pose_nms(
                    boxes, scores, preds_img, preds_scores)
        try:
            res = np.concatenate([result[0]['keypoints'].data.cpu().numpy()+np.array([crop_area[0],crop_area[1]]),
                                result[0]['kp_score'].data.cpu().numpy()],1)
            for k in range(0,17):
                if res[k,2] <= 0.05:
                    res[k,0:3]=np.array([0,0,0])
        except:
            res = np.zeros([17,3])
            print(result)

        boxes_msg[machine].header = img_msg.header
        try:
            boxes_msg[machine].bbox = list(boxes_numpy.reshape(-1)+np.array([crop_area[0],crop_area[1],crop_area[0],crop_area[1]])) #900 , 500 , 1000, 800
            boxes_pub[machine].publish(boxes_msg[machine])
        except:
            boxes_msg[machine].bbox = list(boxes_numpy.reshape(-1)[0:4]+np.array([crop_area[0],crop_area[1],crop_area[0],crop_area[1]])) #900 , 500 , 1000, 800
            boxes_pub[machine].publish(boxes_msg[machine])
        detection = 1
    else:
        res = np.zeros([17,3])
        detection = 0
        boxes_numpy = np.zeros([1,4]).astype(np.int32)
        boxes_msg[machine].header = img_msg.header
        boxes_msg[machine].bbox = boxes_numpy[0]
        boxes_pub[machine].publish(boxes_msg[machine])

    res_msg[machine].header = img_msg.header
    res_msg[machine].res = list(res.reshape(-1))
    res_pub[machine].publish(res_msg[machine])
    detection_msg.data = detection
    detection_result_pub[machine].publish(detection_msg)






    if boxes is not None:
        img_vis = vis_frame(orig_img, result, boxes_numpy)
        img_msg = bridge.cv2_to_imgmsg(img_vis,'bgr8')
    else:
        img_msg = bridge.cv2_to_imgmsg(orig_img,'bgr8')

    # try:
    if res_img_pub[machine].get_num_connections()>0:
        rospy.loginfo_once('Subscriber to alpha pose visualization');
        res_img_pub[machine].publish(img_msg)
    # except:
    #     import ipdb; ipdb.set_trace()


if __name__ == '__main__':
    # rospy.init_node('alphapose_'+str(machine))
    rospy.init_node('alphapose')
    img_sub = [];ch_img = []
    for k in range(int(num_machines)):
        img_sub.append(rospy.Subscriber('firefly_'+str(k+1)+'/firefly_'+str(k+1)+'/xtion/rgb/image_raw',Image,callback, callback_args=k,queue_size=1))
        boxes_sub = message_filters.Subscriber('machine_'+str(k+1)+'/noisy_bbox',AlphaRes);
        box_cache = message_filters.Cache(boxes_sub,100)

    device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
    model = hmr(config.SMPL_MEAN_PARAMS).to(device)
    checkpoint = torch.load(args.checkpoint,map_location=device)
    model.load_state_dict(checkpoint['model'], strict=False)    
    smpl = SMPL(config.SMPL_MODEL_DIR,
                batch_size=1,
                create_transl=False).to(device)
    model.eval()


    rospy.spin()
