#! /usr/bin/env python

import string

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import sys
import math
import numpy
import Image
import cv2
import time


# Generate camera intrinsics matrix, plumb-bob distortion vector, rectification matrix, and projection matrix, for given OpenGL perspective-projected scene
def GenCalibParams(fovy_deg, width_px, height_px, alpha=0.0):
  # OpenGL projection matrix (x/y/homogeneous components only, from eye frame to normalized near plane)
  # J = [fx  0  0
  #       0 fy  0
  #       0  0  1]
  focal_length_y = 1.0/math.tan(math.radians(float(fovy_deg))/2.0)
  #focal_length_x = focal_length_y/width_px*height_px
  
  # Viewport matrix and camera intrinsic matrix
  # V = [w/2   0 w/2
  #        0 h/2 h/2
  #        0   0   1]
  #
  # K = V * J = [  v  0 w/2
  #                0  v h/2
  #                0  0   1]
  # where v = fy*h/2
  focal_length_y_px = focal_length_y*height_px/2.
  intrinsics_mat = numpy.zeros((3, 3), numpy.float64)
  intrinsics_mat[0, 0] = focal_length_y_px
  intrinsics_mat[1, 1] = focal_length_y_px
  intrinsics_mat[0, 2] = float(width_px)/2.
  intrinsics_mat[1, 2] = float(height_px)/2.
  
  # Zero-distortion coefficients for plumb bob model
  distortion_vec = numpy.zeros((1, 5), numpy.float64)
  
  # Default rectification matrix for monocular camera = identify
  rectification_mat = numpy.eye(3, dtype=numpy.float64)

  # Projection matrix (from homogeneous 3D world coordinate to distortion-non-corrected 2D pixel coordinate)
  # alpha = 0.: after correcting for distortion, all pixels in resulting image are valid
  # alpha = 1.: all pixels in source, distortion-non-corrected image are visible, but so might be some black borders
  projection_mat = numpy.zeros((3, 4), numpy.float64)
  new_intrinsics_mat, _ = cv2.getOptimalNewCameraMatrix(intrinsics_mat, distortion_vec, (width_px, height_px), alpha)
  for j in range(3):
    for i in range(3):
      projection_mat[j, i] = new_intrinsics_mat[j, i]
  
  return (intrinsics_mat, distortion_vec, rectification_mat, projection_mat)
  

class TagRenderer:
  def __init__(self, scene_width_px, scene_height_px, scene_fovy_deg, tag_filename=None):
    # Set internal variables
    self.scene_width_px = scene_width_px
    self.scene_height_px = scene_height_px
    self.scene_fovy_deg = scene_fovy_deg
    self.tag_texture = None
    self.postDrawCB = None
    self.resetTagPose()
    self.frustum_changed = False
    
    self.temp_save_t = None # TODO: remove
    self.temp_save_i = 0

    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH)
  
    # Create GLUT display window
    glutInitWindowSize(self.scene_width_px, self.scene_height_px)
    glutInitWindowPosition(0, 0)
    self.window = glutCreateWindow("Tag Renderer")

    # Initialize OpenGL renderer
    glutDisplayFunc(self.handleDrawGLScene)
    glutIdleFunc(self.spinOnce)
    glutReshapeFunc(self.handleResizeGLScene)
    glutKeyboardFunc(self.handleKeyCB)

    if tag_filename is not None:
      self.loadTexture(tag_filename)
    self.initGL()


  def resetTagPose(self):
    self.tag_x_m = 0.0
    self.tag_y_m = 0.0
    self.tag_z_m = -1.0 # note negative z direction
    self.tag_width_m = 0.1
    self.tag_pitch_deg = 0.0 # i.e. rotation about x axis
    self.tag_yaw_deg = 0.0   # i.e. rotation about y axis
    self.tag_roll_deg = 0.0  # i.e. rotation about z axis


  def loadTexture(self, filename):
    image = Image.open(filename)
    image_width_px = image.size[0]
    image_height_px = image.size[1]
    image_str = image.convert("RGBX").tostring("raw", "RGBX", 0, -1)
  
    texture = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, texture)
    #glPixelStorei(GL_UNPACK_ALIGNMENT, 1) # Read pixels in byte-aligned manner (vs default of word-aligned manner)
    glTexImage2D(GL_TEXTURE_2D, 0, 3, image_width_px, image_height_px, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_str)
    
    self.tag_texture = texture

  
  def initGL(self):
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP) # Clamp texture at borders instead ...
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP) # ... of mirror or replicating
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR) # Slower than GL_NEAREST, but more accurate blending
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR) # Ideally want to use GL_LINEAR_MIPMAP_LINEAR, but does not work in software Linux renderer
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL) # Use image's color instead of blending with surface (lighting)

    glEnable(GL_CULL_FACE) # Use single-faced geometry (front- & back-face culling) to display white tag back

    glClearColor(0.0, 0.0, 0.0, 0.0) # Set black background
    
    glClearDepth(1.0) # Enable clearing of depth filter
    glEnable(GL_DEPTH_TEST) # Enable depth testing
    glDepthFunc(GL_LESS)
    glShadeModel(GL_SMOOTH) # Enable smooth color shading
    
    self.updateFrustum()


  def updateFrustum(self):
    tag_bounding_sphere_radius = math.sqrt(self.tag_width_m*self.tag_width_m/2.0)*1.1
    near = -self.tag_z_m - tag_bounding_sphere_radius
    far = -self.tag_z_m + tag_bounding_sphere_radius
    if near <= 0.0001:
      near = 0.0001
    if far <= near:
      far = near + tag_bounding_sphere_radius
    
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(self.scene_fovy_deg, float(self.scene_width_px)/self.scene_height_px, near, far)
    #print glGetFloatv(GL_PROJECTION_MATRIX)

    glMatrixMode(GL_MODELVIEW)
    

  def handleDrawGLScene(self):
    if self.frustum_changed:
      self.frustum_changed = False
      self.updateFrustum()
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) # Clear color and depth buffers
    
    if self.tag_texture is not None:
      # Apply translation and rotation to GL_MODELVIEW
      glLoadIdentity()
      glTranslatef(self.tag_x_m, self.tag_y_m, self.tag_z_m)
      glRotatef(self.tag_pitch_deg, 1.0, 0.0, 0.0)
      glRotatef(self.tag_yaw_deg, 0.0, 1.0, 0.0)
      glRotatef(self.tag_roll_deg, 0.0, 0.0, 1.0)
      
      glScale(self.tag_width_m, self.tag_width_m, self.tag_width_m)
      
      # Render front-side of tag with texture
      glCullFace(GL_BACK)
      #glBindTexture(GL_TEXTURE_2D, texture) # Apparently PyOpenGL does not support this fn call
      glEnable(GL_TEXTURE_2D)
      glBegin(GL_QUADS)
      glTexCoord2f(0.0, 0.0); glVertex3f(-0.5, -0.5,  0.0)  # Bottom-left of texture and quad
      glTexCoord2f(1.0, 0.0); glVertex3f( 0.5, -0.5,  0.0)  # Bottom-right of texture and quad
      glTexCoord2f(1.0, 1.0); glVertex3f( 0.5,  0.5,  0.0)  # Top-right of texture and quad
      glTexCoord2f(0.0, 1.0); glVertex3f(-0.5,  0.5,  0.0)  # Top-left of texture and quad
      glEnd()
      glDisable(GL_TEXTURE_2D)
      #glDisable(GL_CULL_FACE)

      # Render back-side of tag (as white)
      glCullFace(GL_FRONT)
      #glEnable(GL_CULL_FACE)
      glBegin(GL_QUADS)
      glColor3f(1.0, 1.0, 1.0)
      glVertex3f(-0.5, -0.5,  0.0)  # Bottom-left of texture and quad
      glVertex3f( 0.5, -0.5,  0.0)  # Bottom-right of texture and quad
      glVertex3f( 0.5,  0.5,  0.0)  # Top-right of texture and quad
      glVertex3f(-0.5,  0.5,  0.0)  # Top-left of texture and quad
      glEnd()

    glutSwapBuffers()
    
    if self.postDrawCB is not None:
      self.postDrawCB()
    

  def handleResizeGLScene(self, new_width, new_height):
    if False: # Update scene dimensions = saved frame images will have new dimensions as well
      if width <= 0:
        width = 1
      if height <= 0:
        height = 1
      
      self.scene_width_px = width
      self.scene_height_px = height
    else:
      if new_width != self.scene_width_px or new_height != self.scene_height_px:
        glutReshapeWindow(self.scene_width_px, self.scene_height_px) # Force original scene dimensions
      else:
        return
    
    glViewport(0, 0, self.scene_width_px, self.scene_height_px)
    self.frustum_changed = True
    glutPostRedisplay()
    

  def handleKeyCB(self, *args):
    refresh_scene = True
    key = args[0]
    if key == '\033' or key == 'x': # ESC or x
      sys.exit()
    elif key == '4':
      self.tag_x_m -= 0.1
    elif key == '6':
      self.tag_x_m += 0.1
    elif key == '2':
      self.tag_y_m -= 0.1
    elif key == '8':
      self.tag_y_m += 0.1
    elif key == '3':
      self.tag_z_m += 1.0
      self.frustum_changed = True
    elif key == '9':
      self.tag_z_m -= 1.0
      self.frustum_changed = True
    elif key == 'r':
      self.tag_roll_deg += 15.0
    elif key == 'R':
      self.tag_roll_deg -= 15.0
    elif key == 'p':
      self.tag_pitch_deg += 15.0
    elif key == 'P':
      self.tag_pitch_deg -= 15.0
    elif key == 'y':
      self.tag_yaw_deg += 15.0
    elif key == 'Y':
      self.tag_yaw_deg -= 15.0
    elif key == '+':
      self.tag_width_m *= 2.0
      self.frustum_changed = True
    elif key == '-':
      self.tag_width_m /= 2.0
      self.frustum_changed = True
    elif key == '5':
      self.resetTagPose()
      self.frustum_changed = True
    else:
      print 'Unrecognized key:', key
      refresh_scene = False
      
    if refresh_scene:
      glutPostRedisplay()
      

  def saveBuffer(self):
    now = time.time()
    if (self.temp_save_t is None or (now - self.temp_save_t) > 1.0) and self.temp_save_i < 10:
      pass
    else:
      return
      
    buf = glReadPixels(0, 0, self.scene_width_px, self.scene_height_px, GL_RGB, GL_UNSIGNED_BYTE)
    image = Image.fromstring(mode="RGB", size=(self.scene_width_px, self.scene_height_px), data=buf)
    image = image.transpose(Image.FLIP_TOP_BOTTOM)
    image.save("frame%01d.png" % self.temp_save_i, "PNG")
    print "Saved to frame%01d.png" % self.temp_save_i

    self.temp_save_t = now
    self.temp_save_i += 1


  def spinOnce(self):
    #glutPostRedisplay()
    pass
    # TODO: implement re-display rate


if __name__ == "__main__":
  glutInit(sys.argv)
  
  width = 640
  height = 480
  fovy = 45.0
  tag_filename = "ftag2_6s2f22b_20_00_03_13_30_21.png"
  
  renderer = TagRenderer(width, height, fovy, tag_filename)
  renderer.postDrawCB = renderer.saveBuffer
  glutMainLoop()