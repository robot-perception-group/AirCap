#!/usr/bin/python3 -u

import argparse
import numpy as np
from collections import OrderedDict
import torch
import torch.utils.data
import re
import airpose as modelfactory
import select
import socket
import sys
import binary_structs

def fix_state_dict(old_state_dict):
    #state dicts created with DataParallel or DistributedDataParallel have a prefix to all keys
    new_state_dict = OrderedDict()
    for k,v in old_state_dict.items():
        k2 = re.sub('^model\.','',k) 
        new_state_dict[k2] = v
    return new_state_dict


use_cuda = torch.cuda.is_available()
device = torch.device("cuda" if use_cuda else "cpu")

parser = argparse.ArgumentParser(description='Evaluate Network')
parser.add_argument('-m', '--model', action='store', default='/is/ps3/nsaini/projects/copenet_real/copenet_logs/copenet_twoview/version_5_cont_limbwght/checkpoints/epoch=761.ckpt' )
parser.add_argument('-p', '--port', type=int, action='store', default='9900' )
parser.add_argument('-t', '--threshold', type=float, action='store', default=0.01 )
parser.add_argument('-s', '--speedup', type=int, action='store', default='0' )
args = parser.parse_args()
MODEL = args.model
PORT = args.port
THRESHOLD = float(args.threshold)
SPEEDUP = args.speedup
SIZE=224
BUFFERSIZE = 1 + 3*4 + SIZE*SIZE*3 #BGR data, 8 bit per channel
BUFFERSIZE_STAGES = 1 + (10+21*6)*4

print("Loading Model %s"%MODEL)
model = modelfactory.getmodel(smpl_mean_params_path="smpl_mean_params.npz")

if (SPEEDUP==1):
    print("Tracing model for speedup before deviceloading")
    demoframe = torch.zeros((1,3,SIZE,SIZE),dtype=torch.float32)
    tracedmodel=torch.jit.trace(model.forward,demoframe)
    model=tracedmodel

model = model.to(device)

if (SPEEDUP==2):
    print("Tracing model for speedup on device")
    demoframe = torch.zeros((1,3,SIZE,SIZE),dtype=torch.float32).to(device)
    tracedmodel=torch.jit.trace(model.forward,demoframe)
    model=tracedmodel

try:
    model.load_state_dict( fix_state_dict(torch.load(MODEL,map_location=device)),strict=False)
except:
    print("!!!!                  !!!!!!!!!!!!!!!!                                           !!!!");
    print("!!!! Training snapshot failed to load, starting uninitialized for time benchmark !!!!");
    print("!!!!                  !!!!!!!!!!!!!!!!                                           !!!!");

# set model into evaluation mode
model.eval()

xf = torch.zeros(1,2048).to(device).float()
bb = torch.zeros(1,3).to(device).float()
curr_pose = model.init_pose.clone().to(device)
curr_shape = model.init_shape.clone().to(device)
curr_position = torch.Tensor([[0,0,0.5]]).to(device).float()

mean = torch.tensor([0.485,0.456,0.406]).to(device)
std = torch.tensor([0.229,0.224,0.225]).to(device)

def process(data,metainfo,stage):
    # convert data to tensor, via numpy, and send to GPU
    global bb
    global xf
    global curr_pose
    global curr_shape
    global mean
    global std
    global curr_position

    
    if stage==0:
        print("Received data for stage {}".format(stage)) 
        bb=torch.from_numpy(np.frombuffer(data,dtype=np.float32,count=3,offset=1)).unsqueeze(0).float().to(device)
        npimg=np.frombuffer(data,dtype=np.uint8,count=(SIZE*SIZE*3),offset=1+3)

        # normalization used for torchvision pretrained models
        frame = (torch.from_numpy(npimg).view((SIZE,SIZE,3)).to(device).permute(2,0,1).unsqueeze(0).float() * (1.0/255))
        frame.sub_(mean[:,None,None]).div_(std[:,None,None])
        
        pose, shape = model.forward_reg(xf0 = model.forward_feat_ext(frame),
                                    bb0 = bb,
                                    pred_position0 = torch.Tensor([[0,0,0.5]]).to(device).float(),
                                    pred_orient0 = model.init_pose[:,:6],
                                    pred_art_pose0 = model.init_pose[:,6:22*6],
                                    pred_art_pose1 = model.init_pose[:,6:22*6],
                                    pred_shape0 = model.init_shape,
                                    pred_shape1 = model.init_shape)

        result_shape_pose = memoryview(torch.cat([shape.squeeze(0),pose.squeeze(0)[9:]]).detach().cpu().numpy())

    elif stage==1:
        print("Received data for stage {}".format(stage))
        curr_shape_view2 = torch.from_numpy(np.frombuffer(data,dtype=np.float32,count=10,offset=1)).unsqueeze(0).float().to(device)
        curr_art_pose_view2 = torch.from_numpy(np.frombuffer(data,dtype=np.float32,count=126,offset=41)).unsqueeze(0).float().to(device)
        pose,shape = model.forward_reg(xf0 = xf,
                                    bb0 = bb,
                                    pred_position0 = curr_position,
                                    pred_orient0 = curr_pose[:,:6],
                                    pred_art_pose0 = curr_pose[:,6:22*6],
                                    pred_art_pose1 = curr_art_pose_view2,
                                    pred_shape0 = curr_shape,
                                    pred_shape1 = curr_shape_view2)

        result_shape_pose = memoryview(torch.cat([shape.squeeze(0),pose.squeeze(0)[9:]]).detach().cpu().numpy())

    elif stage==2:
        print("Received data for stage {}".format(stage))
        curr_shape_view2 = torch.from_numpy(np.frombuffer(data,dtype=np.float32,count=10,offset=1)).unsqueeze(0).float().to(device)
        curr_art_pose_view2 = torch.from_numpy(np.frombuffer(data,dtype=np.float32,count=126,offset=41)).unsqueeze(0).float().to(device)
        pose,shape = model.forward_reg(xf0 = xf,
                                    bb0 = bb,
                                    pred_position0 = curr_position,
                                    pred_orient0 = curr_pose[:,:6],
                                    pred_art_pose0 = curr_pose[:,6:22*6],
                                    pred_art_pose1 = curr_art_pose_view2,
                                    pred_shape0 = curr_shape,
                                    pred_shape1 = curr_shape_view2)

        result_shape_pose = memoryview(torch.cat([shape.squeeze(0),pose.squeeze(0)]).detach().cpu().numpy())

    else:
        print("Invalid stage number {}".format(stage))

    curr_position = pose[:,:3]
    curr_pose = pose[:,3:]
    curr_shape = shape

    
    return result_shape_pose

print("Setting up TCP server on port %i"%PORT)

server = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
server.setblocking(0)
server.bind(('',PORT))
server.listen()

#for select call:
inputs=[server]
outputs=[]
output_queues={}
input_queues={}
conninfo={}

stage = -1

while inputs:
    readable, writeable, other = select.select( inputs, outputs, inputs) # waits for incoming connections OR data

    # flush data first so it doesn't accumulate
    for fd in writeable:
        if len(output_queues[fd]):
            print('.')
            try:
                fd.send(output_queues[fd].pop())
            except:
                print("Write failed to %s"%str(conninfo[fd]['address']))
        else:
            outputs.remove(fd)

    # read new data
    for fd in readable:
        if fd is server:
            conn,address = fd.accept()
            conn.setblocking(0)
            inputs.append(conn)
            output_queues[conn] = []
            input_queues[conn] = bytearray()
            conninfo[conn] = {'address':address,'Hx':None}
            print("Accepted connection from %s"%(str(address)))
        else:
            try:
                data = fd.recv(BUFFERSIZE)
            except:
                data = False
            
            if data:
                input_queues[fd] += data
                if stage == -1:
                    stage=np.frombuffer(input_queues[fd],dtype=np.uint8,count=1)[0]
                print("stage:{}".format(stage))
                if (len(input_queues[fd])>=BUFFERSIZE_STAGES and stage!=0):
                    output_queues[fd].insert(0,process(input_queues[fd],conninfo[fd],stage))
                    input_queues[fd]=input_queues[fd][BUFFERSIZE_STAGES:]
                    stage = -1
                    if fd not in outputs:
                        outputs.append(fd)
                elif(len(input_queues[fd])>=BUFFERSIZE):
                    output_queues[fd].insert(0,process(input_queues[fd],conninfo[fd],stage))
                    input_queues[fd]=input_queues[fd][BUFFERSIZE:]
                    stage = -1
                    if fd not in outputs:
                        outputs.append(fd)

            else:
                print("Closed connection to %s"%str(conninfo[fd]['address']))
                if fd in outputs:
                    outputs.remove(fd)
                inputs.remove(fd)
                fd.close()
                del output_queues[fd]
                del input_queues[fd]
                del conninfo[fd]

    # close broken fds
    for fd in other:
        inputs.remove(fd)
        if fd in outputs:
            outputs.remove(fd)
        fd.close()
        if output_queues[fd]:
            del output_queues[fd]
        if input_queues[fd]:
            del input_queues[fd]
        if conninfo[fd]:
            del conninfo[fd]



exit(1)

