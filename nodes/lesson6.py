#! /usr/bin/env python

import string

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import sys
import math
import Image

FOVY_DEG = 45.0

ESCAPE = '\033'

window = 0

xrot = yrot = zrot = 0.0
xtran = 0.0
ytran = 0.0
ztran = -2.0

texture = 0


def LoadTextures():
    #global texture
    image = Image.open("ftag2_6s2f22b_20_00_03_13_30_21.png")
  
    ix = image.size[0]
    iy = image.size[1]
    image = image.convert("RGBX").tostring("raw", "RGBX", 0, -1)
  
    # Create Texture
    glBindTexture(GL_TEXTURE_2D, glGenTextures(1)) # 2d texture (x and y size)
  
    glPixelStorei(GL_UNPACK_ALIGNMENT,1)
    glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, image)
    
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP) # Clamp texture at borders instead ...
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP) # ... of mirror or replicating
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR) # Slower than GL_NEAREST, but more accurate blending
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR) # Ideally want to use GL_LINEAR_MIPMAP_LINEAR, but does not work in software Linux renderer
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL) # Use image's color instead of blending with surface (lighting)


def InitGL(width, height):
    aspect_ratio = float(width)/float(height)
  
    LoadTextures()
    glEnable(GL_CULL_FACE) # Use single-faced geometry (front- & back-face culling) to display white tag back

    glClearColor(0.0, 0.0, 0.0, 0.0) # Set black background
    glClearDepth(1.0) # Enable clearing of depth filter
    glEnable(GL_DEPTH_TEST) # Enable depth testing
    glDepthFunc(GL_LESS)
    glShadeModel(GL_SMOOTH) # Enable smooth color shading
  
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(FOVY_DEG, aspect_ratio, 0.1, 10.0)
    
    model = glGetFloatv(GL_PROJECTION_MATRIX)
    print model
    
    fy = 1.0/math.tan(math.radians(FOVY_DEG)/2)
    fx = fy/aspect_ratio

    print "opengl xy-projection matrix\n[%.2f\t%.2f\t%.2f\n %.2f\t%.2f\t%.2f\n 0.00\t0.00\t1.00]" % (fx, 0, 0, 0, fy, 0)
    print "camera matrix\n[%.2f\t%.2f\t%.2f\n %.2f\t%.2f\t%.2f\n 0.00\t0.00\t1.00]" % (fx*width/2., 0, width/2.0, 0, fy*height/2., height/2.0)
    
    
    '''
    
    self.distortion = numpy.zeros((5, 1), numpy.float64)
     self.intrinsics[0,0] = 1.0
        self.intrinsics[1,1] = 1.0
        cv2.calibrateCamera(
                   opts, ipts,
                   self.size, self.intrinsics,
                   self.distortion,
                   flags = self.calib_flags)
            self.R = numpy.eye(3, dtype=numpy.float64)
        self.P = numpy.zeros((3, 4), dtype=numpy.float64)
        
    lrost(distortion, intrinsics, R, P) -> (d, k, r, p) -> (distortion, camera, rectification, projection)
    
    
            ncm, _ = cv2.getOptimalNewCameraMatrix(self.intrinsics, self.distortion, self.size, a)
        for j in range(3):
            for i in range(3):
                self.P[j,i] = ncm[j, i]
    '''

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
  global xrot, yrot, zrot, xtran, ytran, ztran, texture

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) # Clear screen and depth buffers
  
  # Apply transformations
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
    if args[0] == ESCAPE:
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
      ztran -= 0.1
    elif args[0] == '9':
      ztran += 0.1
    elif args[0] == 'r':
      xrot = 0.
      yrot = 0.
      zrot = 0.
      xtran = 0.
      ytran = 0.
      ztran = -1.
    else:
      print args[0]


def main():
  global window
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
  print "Hit ESC key to quit."
  main()
