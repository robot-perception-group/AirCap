#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseArray,Pose
from gazebo_msgs.msg import LinkStates
import message_filters

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
link_sub = message_filters.Subscriber("/gazebo/link_states", LinkStates)
pub = rospy.Publisher("/gt_joints", PoseArray, queue_size=1)

def links_callback(msg):
    links = msg
    names = links.name
    n_links = len(names)
    time_stamp = rospy.Time.now()
    links_publish = PoseArray()
    counter = 0
    for k in range(n_links):
        if names[k][0:5]=='actor' and names[k] in dict_joints:
            dict_joints[names[k]] = k
            if names[k] == 'actor::actor_pose':
                links_publish.poses.append(links.pose[k])
            else:
                links.pose[k].position.x = links.pose[k].position.x + links.pose[1].position.x
                links.pose[k].position.y = links.pose[k].position.y + links.pose[1].position.y
                links.pose[k].position.z = links.pose[k].position.z + links.pose[1].position.z
                links_publish.poses.append(links.pose[k])

    links_publish.header.stamp = rospy.Time.now()
    links_publish.header.frame_id = "world"
    pub.publish(links_publish)
    print(links_publish)


if __name__=='__main__':
    rospy.init_node('actor_joint_pub_node')
    cache = message_filters.Cache(link_sub, 100, allow_headerless=True)
    cache.registerCallback(links_callback)
    rate = rospy.Rate(40)
    rospy.spin()
