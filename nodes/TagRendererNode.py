#! /usr/bin/env python

from TagRenderer import *
import rospy
from tag_renderer.msg import TagPose
from tag_renderer.srv import SetSceneViewport, SetTagSource
from sensor_msgs.msg import Image, CameraInfo


class TagRenderer:
  def __init__(self):
    rospy.init_node('tag_renderer_node')
    
    self.default_tag_x_m = rospy.get_param('~default_tag_x_m', 0.0)
    self.default_tag_y_m = rospy.get_param('~default_tag_y_m', 0.0)
    self.default_tag_z_m = rospy.get_param('~default_tag_z_m', 1.0)
    self.default_tag_width_m = rospy.get_param('~default_tag_width_m', 0.1)
    self.default_tag_roll_deg = rospy.get_param('~default_tag_roll_deg', 0.0)
    self.default_tag_pitch_deg = rospy.get_param('~default_tag_pitch_deg', 0.0)
    self.default_tag_yaw_deg = rospy.get_param('~default_tag_yaw_deg', 0.0)
    self.resetTagPose()
    
    self.scene_width_px = rospy.get_param('~scene_width_px', self.scene_width_px)
    self.scene_height_px = rospy.get_param('~scene_height_px', self.scene_height_px)
    self.scene_fovy_deg = rospy.get_param('~scene_fovy_deg', self.scene_fovy_deg)
    self.frustum_changed = True
    
    self.tag_filename = rospy.get_param('~tag_filename', '')
    if len(self.tag_filename) > 0:
      self.loadTexture(self.tag_filename)
      rospy.loginfo('loaded tag source: %s' % self.tag_filename)
    else:
      rospy.logwarn('no default tag source specified')
    
    self.republish_delay_sec = rospy.get_param('~republish_delay_sec', -1) # < 0: do not republish; else: republish (at least) every N secs if possible
    
    self.enable_key_ctrls = rospy.get_param('~enable_key_ctrls', False) # If true, can translate/rotate/debug scene with various keys (see TagRenderer.handleKeyCB)

    self.postDrawCB = self.publishBuffer
    self.pub_image_raw = rospy.Publisher('image_raw', Image, queue_size=10)
    self.pub_camera_info = rospy.Publisher('camera_info', CameraInfo, queue_size=10)
    self.sub_tag_pose = rospy.Subscriber('tag_pose', TagPose, self.handleTagPose)
    self.srv_set_scene_viewport = rospy.Service('set_scene_viewport', SetSceneViewport, self.handleSetSceneViewport)
    self.srv_set_tag_source = rospy.Service('set_tag_source', SetTagSource, self.handleSetTagSource)
    
    rospy.loginfo('%s initialized' % rospy.get_name())
    
    
  def resetTagPose(self):
    self.tag_x_m = self.default_tag_x_m
    self.tag_y_m = self.default_tag_y_m
    self.tag_z_m = -self.default_tag_z_m # note negative z direction
    self.tag_width_m = self.default_tag_width_m
    self.tag_pitch_deg = self.default_tag_pitch_deg # i.e. rotation about x axis
    self.tag_yaw_deg = self.default_tag_yaw_deg     # i.e. rotation about y axis
    self.tag_roll_deg = self.default_tag_roll_deg   # i.e. rotation about z axis
    
    
        
    - SetViewport service
      - scene y fov in degrees
      - scene width and height
    
    - LoadImage service
      - source image path
    
    - SetPose topic
      - tag size
      - translation
      - orientation
      (what's the relationship between x y z rotations and quaternion pose?)
    
    - publish image, camera_info


  def handleKeyCB(self, key, x, y):
    if self.enable_key_ctrls:
      self.parent.handleKeyCB(key, x, y) # TODO: properly shut down ros?
    else:
      if key == '\033' or key == 'x': # ESC or x
        # TODO: properly shut down ros?
        sys.exit()


  def handleTagPose(self, msg):
    # TODO: retrieve contents from msg -or- use default, then glutPostRedisplay()
    # also if z changed, then self.frustum_changed = True
  
  
  def handleSetSceneViewport(self, ...):
    # TODO: re-parse follow vars
    self.scene_width_px
    self.scene_height_px
    self.scene_fovy_deg
    self.handleResizeGLScene(-1, -1) # Force-resize to new scene_width_px and scene_height_px

    
  def handleSetTagSource(self, ...):
    # TODO: LoadTexture


  def publishBuffer(self):
    '''
    buf = glReadPixels(0, 0, self.scene_width_px, self.scene_height_px, GL_RGB, GL_UNSIGNED_BYTE)
    im_rgb_flipped = numpy.reshape(numpy.fromstring(buf, dtype=numpy.uint8), (self.scene_height_px, self.scene_width_px, 3))
    im_rgb = cv2.flip(im_rgb_flipped, 0) # flip vertically
    im_bgr = cv2.cvtColor(im_rgb, cv2.cv.CV_RGB2BGR)
    '''
    # TODO


  def spinOnce(self):
    # TODO: only redisplay every so often
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
