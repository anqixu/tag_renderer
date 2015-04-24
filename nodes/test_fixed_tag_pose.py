#! /usr/bin/env python

import rospy
import rospkg
from std_msgs.msg import String
from tag_renderer.msg import TagPose
from tag_renderer.srv import SetSceneViewport, SetTagSource, SetSceneViewportRequest, SetTagSourceRequest
import tf
import math

tag_source = "/home/thalassa/anqixu/indigo_ws/1_ftag/src/ftag2test/ftag2_datasets/6S2F22B_random_set/ftag2_6s2f22b_33_02_23_23_33_13.png"
tag_width_m = 0.125
if True:
  tag_tx_m = -0.01786992
  tag_ty_m = -0.19101908
  tag_tz_m = 1.31202207
  tag_rx_deg = -56.23339403
  tag_ry_deg = -44.49052721
  tag_rz_deg = 285.05987728
else:
  tag_tx_m = -0.01800997
  tag_ty_m = -0.18946870
  tag_tz_m = 1.30566856
  tag_rx_deg = 69.65410439
  tag_ry_deg = 59.29046935
  tag_rz_deg = -63.88953316


class TagRendererNodeTester:
  def __init__(self):
    rospy.init_node('tester')

    rospy.loginfo('Waiting for /tag_renderer_node/set_tag_source')
    rospy.wait_for_service('/tag_renderer_node/set_tag_source')

    self.pub_set_tag_pose = rospy.Publisher('/tag_renderer_node/set_tag_pose', TagPose, queue_size=10)
    self.cln_set_tag_source = rospy.ServiceProxy('/tag_renderer_node/set_tag_source', SetTagSource)

    '''
    msg = String()
    msg.data = tag_source
    self.pub_set_tag_source.publish(msg)
    '''
    req = SetTagSourceRequest()
    req.filename = tag_source
    self.cln_set_tag_source(req)
    rospy.sleep(0.1)
    
    self.pubPose(tag_width_m, tag_tx_m, tag_ty_m, tag_tz_m, tag_rx_deg, tag_ry_deg, tag_rz_deg)
    rospy.sleep(0.1)
    
    #rospy.sleep(1.0)
    
    rospy.loginfo('All done')
    #rospy.spin()
    
    
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
