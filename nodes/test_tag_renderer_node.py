#! /usr/bin/env python

import rospy
import rospkg
from tag_renderer.msg import TagPose
from tag_renderer.srv import SetSceneViewport, SetTagSource, SetSceneViewportRequest, SetTagSourceRequest
import tf
import math


class TagRendererNodeTester:
  def __init__(self):
    rospy.init_node('tester')

    rospy.loginfo('Waiting for /tag_renderer_node/set_scene_viewport')
    rospy.wait_for_service('/tag_renderer_node/set_scene_viewport')
    rospy.loginfo('Waiting for /tag_renderer_node/set_tag_source')
    rospy.wait_for_service('/tag_renderer_node/set_tag_source')
    
    self.cln_set_scene_viewport = rospy.ServiceProxy('/tag_renderer_node/set_scene_viewport', SetSceneViewport)
    self.cln_set_tag_source = rospy.ServiceProxy('/tag_renderer_node/set_tag_source', SetTagSource)

    self.pub_tag_pose = rospy.Publisher('/tag_renderer_node/tag_pose', TagPose, queue_size=10)
    
    rospy.loginfo('%s initialized' % rospy.get_name())
    
    rospack = rospkg.RosPack()
    tag_renderer_dir = rospack.get_path('tag_renderer')

    rospy.loginfo('Resetting original tag')
    req = SetTagSourceRequest()
    req.filename = '%s/nodes/ftag2_6s2f22b_20_00_03_13_30_21.png' % (tag_renderer_dir)
    self.cln_set_tag_source(req)
    rospy.sleep(2.0)

    rospy.loginfo('Moving tag in quad')
    rospy.sleep(1.0)
    x=0.1; y=0.0; z=1.0; w=0.1; r=0.0; p=0.0; a=0.0;
    self.pubPose(x,y,z,w,r,p,a)
    rospy.sleep(1.0)

    x=0.1; y=0.1; z=1.0; w=0.1; r=0.0; p=0.0; a=0.0;
    self.pubPose(x,y,z,w,r,p,a)
    rospy.sleep(1.0)

    x=0.0; y=0.1; z=1.0; w=0.1; r=0.0; p=0.0; a=0.0;
    self.pubPose(x,y,z,w,r,p,a)
    rospy.sleep(1.0)

    x=0.0; y=0.0; z=1.0; w=0.1; r=0.0; p=0.0; a=0.0;
    self.pubPose(x,y,z,w,r,p,a)
    rospy.sleep(2.0)
    
    rospy.loginfo('Sweeping roll angle from -90deg to 90deg')
    for i in xrange(-10, 11):
      x=0.0; y=0.0; z=1.0; w=0.1; r=i*7.5; p=0.0; a=0.0;
      self.pubPose(x,y,z,w,r,p,a)
      rospy.sleep(0.2)
    
    rospy.loginfo('Loading new tag')
    rospy.sleep(2.0)
    req = SetTagSourceRequest()
    req.filename = '%s/nodes/ftag2_6s5f33322b_40024_05244_07424_37762_66560_67520.png' % (tag_renderer_dir)
    self.cln_set_tag_source(req)
    rospy.sleep(2.0)

    rospy.loginfo('Sweeping pitch angle from -90deg to 90deg')
    for i in xrange(-10, 11):
      x=0.0; y=0.0; z=1.0; w=0.1; r=0.0; p=i*7.5; a=0.0;
      self.pubPose(x,y,z,w,r,p,a)
      rospy.sleep(0.2)
    
    rospy.loginfo('Sweeping yaw angle from -90deg to 90deg')
    for i in xrange(-10, 11):
      x=0.0; y=0.0; z=1.0; w=0.1; r=0.0; p=20.0; a=i*7.5;
      self.pubPose(x,y,z,w,r,p,a)
      rospy.sleep(0.2)
    
    rospy.loginfo('Resetting original pose')
    x=0.0; y=0.0; z=1.0; w=0.1; r=0.0; p=0.0; a=0.0;
    self.pubPose(x,y,z,w,r,p,a)
    
    rospy.loginfo('All done')
    
    
  def pubPose(self, x, y, z, w, r, p, a):
    quat=tf.transformations.quaternion_from_euler(math.radians(r), math.radians(p), math.radians(a))
    pose_msg = TagPose()
    pose_msg.pose.position.x=x; pose_msg.pose.position.y=y; pose_msg.pose.position.z=z;
    pose_msg.width=w;
    pose_msg.pose.orientation.x=quat[0]
    pose_msg.pose.orientation.y=quat[1]
    pose_msg.pose.orientation.z=quat[2]
    pose_msg.pose.orientation.w=quat[3]
    self.pub_tag_pose.publish(pose_msg)


if __name__ == "__main__":
  try:
    node = TagRendererNodeTester()
    #node.spin()
  except rospy.ROSInterruptException:
    pass
