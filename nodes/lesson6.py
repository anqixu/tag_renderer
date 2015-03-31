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

FOVY_DEG = 45.0

xrot = yrot = zrot = 0.0
xtran = 0.0
ytran = 0.0
ztran = -2.0


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
    LoadTextures()

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
    

def DrawGLScene():
  global xrot, yrot, zrot, xtran, ytran, ztran

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) # Clear color and depth buffers
  
  # Apply translation and rotation to GL_MODELVIEW
  glLoadIdentity()
  glTranslatef(xtran, ytran, ztran)
  glRotatef(xrot,1.0,0.0,0.0)
  glRotatef(yrot,0.0,1.0,0.0)
  glRotatef(zrot,0.0,0.0,1.0)
  
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
  
  # TODO: remove animation updates
  #xrot  = xrot + 0.4                # X rotation
  #yrot = yrot + 0.25                 # Y rotation
  #zrot = zrot + 0.1                 # Z rotation
  #ztran = ztran - 0.05
  #if ztran < -10:
  #  ztran = -1.0

  glutSwapBuffers()


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


def main():
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
  

if __name__ == "__main__":
  main()
