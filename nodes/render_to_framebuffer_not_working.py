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

FOVY_DEG = 45.0

xrot = yrot = zrot = 0.0
xtran = 0.0
ytran = 0.0
ztran = -2.0

rboColor = 0
rboDepth = 0
fbo = 0


def GenCalibParams(fovy_deg, width_px, height_px, alpha=0.0):
  # OpenGL projection matrix (x/y/homogeneous components only, from eye frame to normalized near plane)
  # J = [fx  0  0
  #       0 fy  0
  #       0  0  1]
  focal_length_y = 1.0/math.tan(math.radians(float(fovy_deg))/2.0)
  #focal_length_x = focal_length_y/width_px*height_px
  
  # Camera intrinsic matrix
  # X = [w/2   0 w/2
  #        0 h/2 h/2
  #        0   0   1]
  #
  # K = X * J = [  v  0 w/2
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


def LoadTextures():
    #global texture
    image = Image.open("ftag2_6s2f22b_20_00_03_13_30_21.png")
  
    ix = image.size[0]
    iy = image.size[1]
    image = image.convert("RGBX").tostring("raw", "RGBX", 0, -1)
  
    # Create Texture
    texture = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, texture) # 2-D texture (x and y size)
  
    #glPixelStorei(GL_UNPACK_ALIGNMENT, 1) # Read pixels in byte-aligned manner (vs default of word-aligned manner)
    glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, image)
    

def InitGL(width, height):
    #LoadTextures()

    #glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP) # Clamp texture at borders instead ...
    #glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP) # ... of mirror or replicating
    #glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR) # Slower than GL_NEAREST, but more accurate blending
    #glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR) # Ideally want to use GL_LINEAR_MIPMAP_LINEAR, but does not work in software Linux renderer
    #glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL) # Use image's color instead of blending with surface (lighting)

    #glEnable(GL_CULL_FACE) # Use single-faced geometry (front- & back-face culling) to display white tag back

    glClearColor(0.0, 0.0, 0.0, 0.0) # Set black background
    glClearDepth(1.0) # Enable clearing of depth filter
    glEnable(GL_DEPTH_TEST) # Enable depth testing
    glDepthFunc(GL_LESS)
    glShadeModel(GL_SMOOTH) # Enable smooth color shading
  
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(FOVY_DEG, float(width)/float(height), 0.1, 10.0)
    #print glGetFloatv(GL_PROJECTION_MATRIX)

    glMatrixMode(GL_MODELVIEW)


def ReSizeGLScene(width, height):
    if height <= 0:
      height = 1

    glViewport(0, 0, width, height) # Reset viewport
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(FOVY_DEG, float(width)/float(height), 0.1, 10.0)
    
    glMatrixMode(GL_MODELVIEW)
    

def DrawGLScene(double_buffering = True):
  global xrot, yrot, zrot, xtran, ytran, ztran

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) # Clear color and depth buffers
  
  # Apply translation and rotation to GL_MODELVIEW
  glLoadIdentity()
  glTranslatef(xtran, ytran, ztran)
  glRotatef(xrot,1.0,0.0,0.0)
  glRotatef(yrot,0.0,1.0,0.0)
  glRotatef(zrot,0.0,0.0,1.0)
  '''
  # Render front-side of tag with texture
  glCullFace(GL_BACK)
  #glBindTexture(GL_TEXTURE_2D, texture) # Apparently PyOpenGL does not support this fn call
  glEnable(GL_TEXTURE_2D)
  glBegin(GL_QUADS)
  glColor3f(1.0, 1.0, 1.0)
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
  '''
  glColor3f(1.0, 1.0, 1.0)
  glutSolidCube(1.0)
  
  # TODO: remove animation updates
  #xrot  = xrot + 0.4                # X rotation
  #yrot = yrot + 0.25                 # Y rotation
  #zrot = zrot + 0.1                 # Z rotation
  ztran = ztran - 1.0
  #if ztran < -10:
  #  ztran = -1.0

  if double_buffering:
    glutSwapBuffers()
  else:
    glutSwapBuffers()

flag=0
def RenderAndReadGLScene():
  global flag, rboColor, rboDepth, fbo
  
  if flag >= 10:
    print "Terminated"
    sys.exit()
    DrawGLScene(False)
    return
    
  width=640
  height=480
  
  
  glBindFramebuffer(GL_FRAMEBUFFER, fbo)
  
  
  
  DrawGLScene(False)
  
  glFlush()
  
  glClearColor(0.0, 0.1*flag, 0.0, 0.0)
  
  
  buff = glGetIntegerv(GL_READ_BUFFER)
  if buff == GL_BACK:
    print "glReadPixels reads from: GL_BACK"
  elif buff == GL_FRONT:
    print "glReadPixels reads from: GL_FRONT"
  elif buff == GL_COLOR_ATTACHMENT0:
    print "glReadPixels reads from: GL_COLOR_ATTACHMENT0"
  else:
    print "glReadPixels reads from: ? %d" % buff
  
  
  
  glReadBuffer(GL_COLOR_ATTACHMENT0)
  buf = glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE)
  image = Image.fromstring(mode="RGB", size=(width, height), data=buf)
  image = image.transpose(Image.FLIP_TOP_BOTTOM)
  image.save("frame%01d.png" % flag, "PNG")
  print "Saved to frame%01d.png" % flag

  flag += 1
  

# The function called whenever a key is pressed. Note the use of Python tuples to pass in: (key, x, y)  
def keyPressed(*args):
    global xrot, yrot, zrot, xtran, ytran, ztran
  
    # If escape is pressed, kill everything.
    if args[0] == '\033' or args[0] == 'x': # ESC
      sys.exit()
    elif args[0] == '4':
      xtran -= 0.1
    elif args[0] == '6':
      xtran += 0.1
    elif args[0] == '2':
      ytran -= 0.1
    elif args[0] == '8':
      ytran += 0.1
    elif args[0] == '3':
      ztran -= 1
    elif args[0] == '9':
      ztran += 1
    elif args[0] == '7':
      zrot += 15.0
    elif args[0] == '1':
      zrot -= 15.0
    elif args[0] == '5':
      xrot = 0.
      yrot = 0.
      zrot = 0.
      xtran = 0.
      ytran = 0.
      ztran = -2.
    else:
      print args[0]


def mainDisplay():
  glutInit(sys.argv)

  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
  
  # get a 640 x 480 window, at the top-left corner of the screen
  glutInitWindowSize(640, 480)
  glutInitWindowPosition(0, 0)
  
  window = glutCreateWindow("Tag Renderer Test")

  glutDisplayFunc(DrawGLScene)
  glutIdleFunc(DrawGLScene)
  glutReshapeFunc(ReSizeGLScene)
  glutKeyboardFunc(keyPressed)

  InitGL(640, 480)
  
  glutMainLoop()


t = None
def Idle():
  global t
  
  now = time.time()
  if t is None or (now - t) > 1.0:
    t = now
    glutPostRedisplay()

def mainRenderFrameBuffer():
  global rboColor, rboDepth, fbo
  
  width=640
  height=480
  
  glutInit(sys.argv)

  glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE | GLUT_DEPTH)
  
  # Must create stub window, but can hide it
  glutInitWindowSize(1, 1)
  glutInitWindowPosition(0, 0)
  glutCreateWindow("Tag Renderer Stub Window")
  glutHideWindow()
  
  glutDisplayFunc(RenderAndReadGLScene)
  glutIdleFunc(RenderAndReadGLScene)
  #glutReshapeFunc(ReSizeGLScene)
  glutKeyboardFunc(keyPressed)

  # Create color renderbuffer
  rboColor = glGenRenderbuffers(1)
  glBindRenderbuffer(GL_RENDERBUFFER, rboColor)
  glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, width, height)

  # Create depth renderbuffer
  rboDepth = glGenRenderbuffers(1)
  glBindRenderbuffer(GL_RENDERBUFFER, rboDepth)
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height)

  # Create framebuffer
  fbo = glGenFramebuffers(1)
  glBindFramebuffer(GL_FRAMEBUFFER, fbo)
  glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rboColor)
  glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rboDepth)

  # Configure OpenGL to render to framebuffer, and read pixels from it as well
  #glBindFramebuffer(GL_FRAMEBUFFER, fbo)
  glReadBuffer(GL_COLOR_ATTACHMENT0)
  
  status = glCheckFramebufferStatus(GL_FRAMEBUFFER)
  if status == GL_FRAMEBUFFER_UNDEFINED:
    print "Status: GL_FRAMEBUFFER_UNDEFINED"
  elif status == GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
    print "Status: GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT"
  elif status == GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
    print "Status: GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT"
  elif status == GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
    print "Status: GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER"
  elif status == GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
    print "Status: GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER"
  elif status == GL_FRAMEBUFFER_UNSUPPORTED:
    print "Status: GL_FRAMEBUFFER_UNSUPPORTED"
  elif status == GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
    print "Status: GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE"
  elif status == GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS:
    print "Status: GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS"
  elif status == 0:
    print "Status: 0"
  elif status == GL_FRAMEBUFFER_COMPLETE:
    print "Status: GL_FRAMEBUFFER_COMPLETE"
  else:
    print "Status: %d" % status
  
  InitGL(width, height)
  
  glutMainLoop()


if __name__ == "__main__":
  #mainDisplay()
  mainRenderFrameBuffer()
