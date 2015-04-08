#! /usr/bin/env python

from TagRenderer import *
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from tag_renderer.msg import TagPose
from tag_renderer.srv import SetSceneViewport, SetTagSource, SetSceneViewportResponse, SetTagSourceResponse
from cv_bridge import CvBridge
import tf


class TagRendererNode(TagRenderer):
  def __init__(self):
    rospy.init_node('tag_renderer_node')
    
    scene_width_px = rospy.get_param('~scene_width_px', 640)
    scene_height_px = rospy.get_param('~scene_height_px', 480)
    scene_fovy_deg = rospy.get_param('~scene_fovy_deg', 60.0)
    self.tag_filename = rospy.get_param('~tag_filename', '')
    self.default_tag_x_m = rospy.get_param('~default_tag_x_m', 0.0)
    self.default_tag_y_m = rospy.get_param('~default_tag_y_m', 0.0)
    self.default_tag_z_m = rospy.get_param('~default_tag_z_m', 1.0)
    self.default_tag_width_m = rospy.get_param('~default_tag_width_m', 0.1)
    self.default_tag_roll_deg = rospy.get_param('~default_tag_roll_deg', 0.0)
    self.default_tag_pitch_deg = rospy.get_param('~default_tag_pitch_deg', 0.0)
    self.default_tag_yaw_deg = rospy.get_param('~default_tag_yaw_deg', 0.0)
    TagRenderer.__init__(self, scene_width_px, scene_height_px, scene_fovy_deg, self.tag_filename)
    if len(self.tag_filename) > 0:
      rospy.loginfo('loaded tag source: %s' % self.tag_filename)
    else:
      rospy.logwarn('no default tag source specified')
    
    self.bridge = CvBridge()
    self.frustum_changed = True
    self.t_first_pub = None
    self.t_latest_pub = None
    self.update_viewport_req = None
    self.update_tag_source_req = None
    
    self.republish_delay_sec = rospy.get_param('~republish_delay_sec', -1) # < 0: do not republish; else: republish (at least) every N secs if possible
    
    self.enable_key_ctrls = rospy.get_param('~enable_key_ctrls', False) # If true, can translate/rotate/debug scene with various keys (see TagRenderer.handleKeyCB)

    self.postDrawCB = self.publishBuffer
    self.pub_image_raw = rospy.Publisher('~image_raw', Image, queue_size=10)
    self.pub_camera_info = rospy.Publisher('~camera_info', CameraInfo, queue_size=10)
    self.sub_tag_pose = rospy.Subscriber('~set_tag_pose', TagPose, self.handleTagPose)
    self.sub_tag_source = rospy.Subscriber('~set_tag_source', String, self.handleSetTagSourceMsg)
    self.srv_set_scene_viewport = rospy.Service('~set_scene_viewport', SetSceneViewport, self.handleSetSceneViewport)
    self.srv_set_tag_source = rospy.Service('~set_tag_source', SetTagSource, self.handleSetTagSource)
    
    rospy.loginfo('%s initialized' % rospy.get_name())


  def shutdown(self):
    rospy.signal_shutdown('Shutdown requested by user')
    glutLeaveMainLoop()

    
  def resetTagPose(self):
    self.tag_x_m = self.default_tag_x_m
    self.tag_y_m = self.default_tag_y_m
    self.tag_z_m = -self.default_tag_z_m # note negative z direction
    self.tag_width_m = self.default_tag_width_m
    self.tag_pitch_deg = self.default_tag_pitch_deg # i.e. rotation about x axis
    self.tag_yaw_deg = self.default_tag_yaw_deg     # i.e. rotation about y axis
    self.tag_roll_deg = self.default_tag_roll_deg   # i.e. rotation about z axis
    self.t_first_pub = None # Force immediate redisplay+publish


  def handleKeyCB(self, key, x, y):
    if self.enable_key_ctrls:
      TagRenderer.handleKeyCB(self, key, x, y)
    else:
      if key == '\033' or key == 'x': # ESC or x
        self.shutdown()


  def handleTagPose(self, msg):
    self.tag_x_m = msg.pose.position.x
    self.tag_y_m = msg.pose.position.y
    if self.tag_z_m != -msg.pose.position.z:
      self.tag_z_m = -msg.pose.position.z
      self.frustum_changed = True
    self.tag_width_m = msg.width
    
    quaternion = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    self.tag_roll_deg = math.degrees(euler[0]) # TODO: rpy order correct?
    self.tag_pitch_deg = math.degrees(euler[1])
    self.tag_yaw_deg = math.degrees(euler[2])
    
    self.t_first_pub = None # Force immediate redisplay+publish on next spinOnce
    
    #print 'Received tag pose request: w=%.2f, xyz=(%.2f, %.2f, %.2f), rpy_deg=(%.2f, %.2f, %.2f)' % (self.tag_width_m, self.tag_x_m, self.tag_y_m, self.tag_z_m, self.tag_roll_deg, self.tag_pitch_deg, self.tag_roll_deg) # TODO: remove
    
    glutPostRedisplay()
  
  
  def handleSetSceneViewport(self, req):
    self.update_viewport_req = req # Postpone resize till on main (GL) thread's render context
    return SetSceneViewportResponse()

    
  def handleSetTagSource(self, req):
    #print 'Received tag source request:', req.filename # TODO: remove
    self.update_tag_source_req = req # Postpone loading texture till on main (GL) thread's render context
    return SetTagSourceResponse()


  def handleSetTagSourceMsg(self, msg):
    self.update_tag_source_req = SetTagSource() # Postpone loading texture till on main (GL) thread's render context
    self.update_tag_source_req.filename = msg.data
    #print 'Received tag source msg request:', msg.data # TODO: remove


  def publishBuffer(self):
    buf = glReadPixels(0, 0, self.scene_width_px, self.scene_height_px, GL_RGB, GL_UNSIGNED_BYTE)
    im_rgb_flipped = numpy.reshape(numpy.fromstring(buf, dtype=numpy.uint8), (self.scene_height_px, self.scene_width_px, 3))
    im_rgb = cv2.flip(im_rgb_flipped, 0) # flip vertically
    im_bgr = cv2.cvtColor(im_rgb, cv2.cv.CV_RGB2BGR)
    
    im_msg = self.bridge.cv2_to_imgmsg(im_bgr, 'bgr8')
    self.pub_image_raw.publish(im_msg)
    
    info = CameraInfo()
    calibParams = GenCalibParams(self.scene_fovy_deg, self.scene_width_px, self.scene_height_px)
    info.header = im_msg.header
    info.height = self.scene_height_px
    info.width = self.scene_width_px
    info.distortion_model = "plumb_bob"
    info.K = calibParams[0].flatten().tolist()
    info.D = calibParams[1].flatten().tolist()
    info.R = calibParams[2].flatten().tolist()
    info.P = calibParams[3].flatten().tolist()
    self.pub_camera_info.publish(info)
    
    now = rospy.Time.now()
    self.t_latest_pub = now
    if self.t_first_pub is None:
      self.t_first_pub = now


  def spinOnce(self):
    if self.update_viewport_req is not None:
      req = self.update_viewport_req
      self.scene_width_px = req.scene_width_px
      self.scene_height_px = req.scene_height_px
      self.scene_fovy_deg = req.scene_fovy_deg
      self.handleResizeGLScene(-1, -1) # Force-resize to new scene_width_px and scene_height_px
      self.update_viewport_req = None
      self.t_first_pub = None # force immediate redisplay
      
    if self.update_tag_source_req is not None:
      req = self.update_tag_source_req
      self.tag_filename = req.filename
      self.loadTexture(self.tag_filename)
      rospy.loginfo('updated tag source: %s' % self.tag_filename)
      self.update_tag_source_req = None
      self.t_first_pub = None # force immediate redisplay
    
    now = rospy.Time.now()
    t_first_pub = self.t_first_pub
    t_latest_pub = self.t_latest_pub
    if t_first_pub is None:
      glutPostRedisplay()
    elif self.republish_delay_sec == 0:
      glutPostRedisplay()
    elif self.republish_delay_sec > 0:
      if math.floor((now - t_first_pub).to_sec()/self.republish_delay_sec) > math.floor((t_latest_pub - t_first_pub).to_sec()/self.republish_delay_sec):
        glutPostRedisplay()
    
    
  def spin(self):
    glutMainLoop()


if __name__ == "__main__":
  glutInit(sys.argv)
  try:
    node = TagRendererNode()
    node.spin()
  except rospy.ROSInterruptException:
    pass
