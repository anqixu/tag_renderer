#! /usr/bin/env python

import rospy
import rospkg
from std_msgs.msg import String
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

    self.pub_set_tag_pose = rospy.Publisher('/tag_renderer_node/set_tag_pose', TagPose, queue_size=10)
    self.pub_set_tag_source = rospy.Publisher('/tag_renderer_node/set_tag_source', String, queue_size=10)
    
    rospy.loginfo('%s initialized' % rospy.get_name())
    
    rospack = rospkg.RosPack()
    tag_renderer_dir = rospack.get_path('tag_renderer')

    rospy.loginfo('Resetting original tag')
    msg = String()
    msg.data = '%s/nodes/ftag2_6s2f22b_33_00_23_00_33_10.png' % (tag_renderer_dir)
    self.pub_set_tag_source.publish(msg)
    rospy.sleep(2.0)

    rospy.loginfo('Moving tag in quad')

    w=0.125; tx=0.0; ty=0.0; tz=1.0; rx=0.0; ry=0.0; rz=0.0;
    self.pubPose(w,tx,ty,tz,rx,ry,rz)
    rospy.sleep(1.0)

    w=0.125; tx=0.1; ty=0.0; tz=1.0; rx=0.0; ry=0.0; rz=0.0;
    self.pubPose(w,tx,ty,tz,rx,ry,rz)
    rospy.sleep(1.0)

    w=0.125; tx=0.1; ty=0.1; tz=1.0; rx=0.0; ry=0.0; rz=0.0;
    self.pubPose(w,tx,ty,tz,rx,ry,rz)
    rospy.sleep(1.0)

    w=0.125; tx=0.0; ty=0.1; tz=1.0; rx=0.0; ry=0.0; rz=0.0;
    self.pubPose(w,tx,ty,tz,rx,ry,rz)
    rospy.sleep(1.0)

    w=0.125; tx=0.0; ty=0.0; tz=1.0; rx=0.0; ry=0.0; rz=0.0;
    self.pubPose(w,tx,ty,tz,rx,ry,rz)
    rospy.sleep(2.0)
    
    rospy.loginfo('Sweeping rx angle (tag pitch) from -75deg to 75deg')
    for i in xrange(-10, 11):
      w=0.125; tx=0.0; ty=0.0; tz=1.0; rx=i*7.5; ry=0.0; rz=0.0;
      self.pubPose(w,tx,ty,tz,rx,ry,rz)
      rospy.sleep(0.2)
    
    rospy.loginfo('Loading new tag')
    rospy.sleep(2.0)
    req = SetTagSourceRequest()
    req.filename = '%s/nodes/ftag2_6s2f22b_30_00_20_22_31_12.png' % (tag_renderer_dir)
    self.cln_set_tag_source(req)
    rospy.sleep(2.0)

    rospy.loginfo('Sweeping ry angle (tag yaw) from -75deg to 75deg')
    for i in xrange(-10, 11):
      w=0.125; tx=0.0; ty=0.0; tz=1.0; rx=0.0; ry=i*7.5; rz=0.0;
      self.pubPose(w,tx,ty,tz,rx,ry,rz)
      rospy.sleep(0.2)
    
    rospy.loginfo('Sweeping rz angle (tag roll) from -75deg to 75deg')
    for i in xrange(-10, 11):
      w=0.125; tx=0.0; ty=0.0; tz=1.0; rx=0.0; ry=0.0; rz=i*7.5;
      self.pubPose(w,tx,ty,tz,rx,ry,rz)
      rospy.sleep(0.2)
    
    rospy.loginfo('Resetting original pose')
    w=0.125; tx=0.0; ty=0.0; tz=1.0; rx=0.0; ry=0.0; rz=0.0;
    self.pubPose(w,tx,ty,tz,rx,ry,rz)
    
    rospy.loginfo('All done')
    
    
  def pubPose(self, w, tx, ty, tz, rx, ry, rz):
    quat=tf.transformations.quaternion_from_euler(math.radians(rx), math.radians(ry), math.radians(rz))
    pose_msg = TagPose()
    pose_msg.pose.position.x=tx; pose_msg.pose.position.y=ty; pose_msg.pose.position.z=tz;
    pose_msg.width=w;
    pose_msg.pose.orientation.x=quat[0]
    pose_msg.pose.orientation.y=quat[1]
    pose_msg.pose.orientation.z=quat[2]
    pose_msg.pose.orientation.w=quat[3]
    self.pub_set_tag_pose.publish(pose_msg)


if __name__ == "__main__":
  try:
    node = TagRendererNodeTester()
    #node.spin()
  except rospy.ROSInterruptException:
    pass
