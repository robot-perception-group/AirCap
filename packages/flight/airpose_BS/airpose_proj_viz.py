import os
os.environ["PYOPENGL_PLATFORM"] = "osmesa"
import rospy
import sys
import torch
import numpy as np
from airpose_client.msg import AirposeNetworkResult
# from std_msgs.msg import Float32MultiArray
import torch.nn.functional as F
from smplx import SMPLX
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from torchvision.utils import make_grid
import trimesh
import pyrender

poseTopic = "/"+sys.argv[1]+"/step3_pub"
imageTopic = "/"+sys.argv[1]+"/video"

##############################################

def transform_smpl(trans_mat,smplvertices=None,smpljoints=None, orientation=None, smpltrans=None):
    verts =  torch.bmm(trans_mat[:,:3,:3],smplvertices.permute(0,2,1)).permute(0,2,1) +\
                    trans_mat[:,:3,3].unsqueeze(1)
    if smpljoints is not None:
        joints = torch.bmm(trans_mat[:,:3,:3],smpljoints.permute(0,2,1)).permute(0,2,1) +\
                         trans_mat[:,:3,3].unsqueeze(1)
    else:
        joints = None
    
    if smpltrans is not None:
        trans = torch.bmm(trans_mat[:,:3,:3],smpltrans.unsqueeze(2)).squeeze(2) +\
                         trans_mat[:,:3,3]
    else:
        trans = None

    if orientation is not None:
        orient = torch.bmm(trans_mat[:,:3,:3],orientation)
    else:
        orient = None    
    return verts, joints, orient, trans

def rot6d_to_rotmat(x):
    """Convert 6D rotation representation to 3x3 rotation matrix.
    Based on Zhou et al., "On the Continuity of Rotation Representations in Neural Networks", CVPR 2019
    Input:
        (B,6) Batch of 6-D rotation representations
    Output:
        (B,3,3) Batch of corresponding rotation matrices
    """
    x = x.reshape(-1,3,2)
    a1 = x[:, :, 0]
    a2 = x[:, :, 1]
    b1 = F.normalize(a1)
    b2 = F.normalize(a2 - torch.einsum('bi,bi->b', b1, a2).unsqueeze(-1) * b1)
    b3 = torch.cross(b1, b2)
    return torch.stack((b1, b2, b3), dim=-1)

class Renderer:
    """
    Renderer used for visualizing the SMPL model
    Code adapted from https://github.com/vchoutas/smplify-x
    """
    def __init__(self, focal_length=[1475,1475], img_res=[224,224], center = [112,112], faces=None):
        self.renderer = pyrender.OffscreenRenderer(viewport_width=img_res[0],
                                       viewport_height=img_res[1],
                                       point_size=1.0)
        self.focal_length = focal_length
        self.camera_center = center
        self.faces = faces

    def visualize_tb(self, vertices, camera_translation,camera_rotation, images,nrow=5,color=(0.3, 0.3, 0.8, 1.0)):
        
        vertices = vertices.cpu().numpy()
        camera_translation = camera_translation.cpu().numpy()
        camera_rotation = camera_rotation.cpu().numpy()
        images = images.cpu()
        images_np = np.transpose(images.numpy(), (0,2,3,1))
        rend_imgs = []
        for i in range(vertices.shape[0]):
            rend_img = torch.from_numpy(np.transpose(self.__call__(vertices[i], camera_translation[i], camera_rotation[i],images_np[i],color=color), (2,0,1))).float()
            # rend_imgs.append(images[i])
            rend_imgs.append(rend_img)
        rend_imgs = make_grid(rend_imgs, nrow,padding=0)
        return rend_imgs

    def __call__(self, vertices, camera_translation, camera_rotation, image, color=(0.3, 0.3, 0.8, 1.0)):
        material = pyrender.MetallicRoughnessMaterial(
            metallicFactor=0.2,
            alphaMode='OPAQUE',
            baseColorFactor=color)
        # import ipdb; ipdb.set_trace()
        mesh = trimesh.Trimesh(vertices, self.faces)
        # perp = np.cross(np.array([0,0,1]),camera_translation)
        rot = trimesh.transformations.rotation_matrix(
            np.radians(180), [1,0,0])
        # import ipdb; ipdb.set_trace()
        camera_pose = np.eye(4)
        camera_pose[:3,:3] = camera_rotation
        camera_pose[:3, 3] = camera_translation
        # mesh.apply_transform(np.linalg.inv(camera_pose))
        mesh.apply_transform(camera_pose)
        mesh.apply_transform(rot)
        mesh = pyrender.Mesh.from_trimesh(mesh, material=material)

        scene = pyrender.Scene(ambient_light=(0.5, 0.5, 0.5))
        scene.add(mesh, 'mesh')

        # camera_pose = np.eye(4)
        # camera_pose[:3,:3] = camera_rotation
        # camera_pose = np.matmul(rot,camera_pose)
        # camera_pose[:3, 3] = camera_translation
        
        self.camera = pyrender.IntrinsicsCamera(fx=self.focal_length[0], fy=self.focal_length[1],
                                           cx=self.camera_center[0], cy=self.camera_center[1])
        scene.add(self.camera, pose=np.eye(4))

        light = pyrender.DirectionalLight(color=[1.0, 1.0, 1.0], intensity=1)
        light_pose = np.eye(4)

        light_pose[:3, 3] = np.array([0, -1, 1])
        scene.add(light, pose=light_pose)

        light_pose[:3, 3] = np.array([0, 1, 1])
        scene.add(light, pose=light_pose)

        light_pose[:3, 3] = np.array([1, 1, 2])
        scene.add(light, pose=light_pose)


        color, rend_depth = self.renderer.render(scene, flags=pyrender.RenderFlags.RGBA)

        color = color.astype(np.float32) / 255.0
        valid_mask = (rend_depth > 0)[:,:,None]
        output_img = (color[:, :, :3] * valid_mask +
                  (1 - valid_mask) * image)
        return output_img

#################################################

rospy.init_node("AirPoseImViz", anonymous=True)

device = "cuda" if torch.cuda.is_available() else "cpu"



smplx = SMPLX(sys.argv[2],
                         batch_size=1,
                         create_transl=False).to(device)
smplx.eval()

renderer = Renderer([3429,3430],[2448,2048],[1274,1045],faces=smplx.faces)

proj_pub = rospy.Publisher("/"+sys.argv[1]+"/airposeMeshProj", Image, queue_size=10)
br = CvBridge()

def callback(data,image):
    betas = torch.from_numpy(np.array(data.data[:10])).to(device).float().unsqueeze(0)
    trans = torch.from_numpy(np.array(data.data[10:13])).to(device).float().unsqueeze(0)*20
    pose = rot6d_to_rotmat(torch.from_numpy(np.array(data.data[13:])).to(device).float()).unsqueeze(0)

    smplx_out = smplx.forward(betas=betas, 
                                body_pose=pose[:,1:],
                                global_orient=torch.eye(3,device=device).float().unsqueeze(0).unsqueeze(1),
                                transl = torch.zeros(1,3).float().type_as(betas),
                                pose2rot=False)
    transf_mat0 = torch.cat([pose[:,:1].squeeze(1),
                                trans.unsqueeze(2)],dim=2)
    verts,joints,_,_ = transform_smpl(transf_mat0,
                                                smplx_out.vertices.squeeze(1),
                                                smplx_out.joints.squeeze(1))
    # import ipdb;ipdb.set_trace()
    proj_img = renderer(verts[0].cpu().detach().numpy(),
                np.zeros([1,3]),
                np.eye(3).reshape(1,3,3),
                br.imgmsg_to_cv2(image))
    proj_img = proj_img.astype(np.uint8)
    proj_pub.publish(br.cv2_to_imgmsg(proj_img, "rgb8"))


image_sub = message_filters.Subscriber(imageTopic, Image)
pose_sub = message_filters.Subscriber(poseTopic, AirposeNetworkResult)
ts = message_filters.TimeSynchronizer([pose_sub, image_sub], 10000)
ts.registerCallback(callback)

rospy.spin()
