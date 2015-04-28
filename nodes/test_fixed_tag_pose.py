#! /usr/bin/env python

import rospy
import rospkg
from std_msgs.msg import String
from tag_renderer.msg import TagPose
from tag_renderer.srv import SetSceneViewport, SetTagSource, SetSceneViewportRequest, SetTagSourceRequest
import tf
import math

# sprintf('tag_source = "%s"\ntag_width_m = 0.125\nif True:\n  tag_tx_m = %.8f\n  tag_ty_m = %.8f\n  tag_tz_m = %.8f\n  tag_rx_deg = %.8f\n  tag_ry_deg = %.8f\n  tag_rz_deg = %.8f\nelse:\n  tag_tx_m = %.8f\n  tag_ty_m = %.8f\n  tag_tz_m = %.8f\n  tag_rx_deg = %.8f\n  tag_ry_deg = %.8f\n  tag_rz_deg = %.8f\n', ps.tag_source, ps.tag_tx_m, ps.tag_ty_m, ps.tag_tz_m, ps.tag_rx_deg, ps.tag_ry_deg, ps.tag_rz_deg, ps.ftag2_tx_m, ps.ftag2_ty_m, ps.ftag2_tz_m, ps.ftag2_rx_deg, ps.ftag2_ry_deg, ps.ftag2_rz_deg)

tag_source = "/home/thalassa/anqixu/indigo_ws/1_ftag/src/ftag2test/ftag2_datasets/6S2F22B_random_set/ftag2_6s2f22b_12_22_21_22_10_33.png"
tag_width_m = 0.125
if False:
  tag_tx_m = 0.01945736
  tag_ty_m = -0.16459644
  tag_tz_m = 1.32217826
  tag_rx_deg = 6.74830730
  tag_ry_deg = -11.36325886
  tag_rz_deg = 125.71200784
else:
  tag_tx_m = 0.01867741
  tag_ty_m = -0.16445858
  tag_tz_m = 1.31425976
  tag_rx_deg = 8.85127890
  tag_ry_deg = -4.17989221
  tag_rz_deg = 125.67042507


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

    # Duplicate since sometimes node does not seem to receive 1st pub
    self.pubPose(tag_width_m, tag_tx_m, tag_ty_m, tag_tz_m, tag_rx_deg, tag_ry_deg, tag_rz_deg)
    rospy.sleep(0.1)
    
    #rospy.sleep(1.0)
    
    rospy.loginfo('All done')
    rospy.Timer(rospy.Duration(0.2), self.shutdown)
    rospy.spin()
    
    
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
    
  def shutdown(self, event):
    rospy.signal_shutdown('done')


if __name__ == "__main__":
  try:
    node = TagRendererNodeTester()
    #node.spin()
  except rospy.ROSInterruptException:
    pass
