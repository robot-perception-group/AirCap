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
import torch
from alphapose_node.msg import AlphaRes
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
import numpy as np
import time

machine = opt.machine
num_machines = opt.num_machines

det_loader = DetectionLoader()
det_processor = DetectionProcessor(det_loader)
pose_dataset = Mscoco()
pose_model = InferenNet_fast(4 * 1 + 1, pose_dataset)
pose_model.cuda()
pose_model.eval()

bridge=CvBridge()

# bounding box publisher
boxes_pub = rospy.Publisher('machine_'+str(machine)+'/result_bbox',AlphaRes,queue_size=1)
# alpha pose detections publisher
res_pub = rospy.Publisher('machine_'+str(machine)+'/result_alpha',AlphaRes,queue_size=1)
# image with projected detections publisher
res_img_pub = rospy.Publisher('machine_'+str(machine)+'/result_img_alpha',Image,queue_size=1)

detection_result_pub = rospy.Publisher('machine_'+str(machine)+'/detection',Int16,queue_size=1)
# message for joint detections and probabilities
res_msg = AlphaRes()

# message for bounding box
boxes_msg = AlphaRes()

detection_msg = Int16()
detection = 0

def listener():
    rospy.init_node('alphapose_'+str(machine))
    img_sub = message_filters.Subscriber('machine_'+str(machine)+'/v2alpha',Image)
    ch_img = message_filters.Cache(img_sub,1)

    print('aircap image subscriber spawned')
    img_sub.registerCallback(callback)
    detection_msg.data = detection
    detection_result_pub.publish(detection_msg)
    rospy.spin()


def callback(img_msg):
    rospy.loginfo(img_msg.header.stamp)
    # import ipdb;ipdb.set_trace()
    img = bridge.imgmsg_to_cv2(img_msg, 'bgr8')

    initTime = time.time()

    img_k, orig_img_k, im_dim_list_k = prep_image(img, int(opt.inp_dim))
    im_dim_list = [im_dim_list_k]
    im_dim_list = torch.FloatTensor(im_dim_list).repeat(1,2)

    (inps, orig_img, boxes, scores, pt1, pt2) = det_processor.forward(img_k, orig_img_k, im_dim_list)

    # print(time.time()-initTime)

    if boxes is not None:

        boxes_numpy = boxes.numpy().astype(np.int32)
        datalen = inps.size(0)
        inps_j = inps.cuda()

        hm_j = pose_model(inps_j).cpu()

        preds_hm, preds_img, preds_scores = getPrediction(
                            hm_j.detach(), pt1, pt2, opt.inputResH, opt.inputResW, opt.outputResH, opt.outputResW)
        result = pose_nms(
                    boxes, scores, preds_img, preds_scores)
        try:
            res = np.concatenate([result[0]['keypoints'].data.cpu().numpy(),
                                result[0]['kp_score'].data.cpu().numpy()],1)
            for k in range(0,17):
                if res[k,2] <= 0.05:
                    res[k,0:3]=np.array([0,0,0])
        except:
            res = np.zeros([17,3])
            print(result)

        boxes_msg.header = img_msg.header
        boxes_msg.bbox = list(boxes_numpy.reshape(-1))
        res_pub.publish(res_msg)
        boxes_pub.publish(boxes_msg)
        detection = 1
    else:
        res = np.zeros([17,3])
        detection = 0
        # boxes_numpy = np.zeros([1,4]).astype(np.int32)

    res_msg.header = img_msg.header
    res_msg.res = list(res.reshape(-1))

    if boxes is not None:
        img_vis = vis_frame(orig_img, result, boxes_numpy)
        img_msg = bridge.cv2_to_imgmsg(img_vis,'bgr8')
    else:
        img_msg = bridge.cv2_to_imgmsg(orig_img,'bgr8')

    # try:
    if res_img_pub.get_num_connections()>0:
        rospy.loginfo_once('Subscriber to alpha pose visualization');
        res_img_pub.publish(img_msg)
    # except:
    #     import ipdb; ipdb.set_trace()



if __name__ == '__main__':
    listener()
