#!/usr/bin/env python
'''
A lighter version for visulize the simulation in gazebo.
See README.md for setup instructions.

Ilian Corneliussen
ilianc@kth.se
'''

import time
from tkinter import *
from OpenGL import GL, GLU
from pyopengltk import OpenGLFrame
import json
import sys
import math
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import numpy as np
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from unicodedata import *


class AttributeDict(dict):
    "This makes it so that you can do d.foo instead of d['foo']."
    # From world_viewer.py
    @classmethod
    def object_hook(cls, d):
        self = cls(d)
        self.__dict__ = self
        return self

# Global variables
pose = PoseStamped()
args=sys.argv[1:]
json_fn = os.path.dirname(__file__) + '/' + args[0]
with open(json_fn) as f:
    WORLD = json.load(f, object_hook=AttributeDict.object_hook)
ANGLE = 0
X_ROTATE = 0
Y_ROTATE = 0
Z_ROTATE = 0
X_ZOOM = 0
Y_ZOOM = 0
Z_ZOOM = 0
X_RUNNING = False
Y_RUNNING = False
Z_RUNNING = False
STEP_SIZE = 0 

def draw_drone(position, orientation):
    global POSE
    # Takes out the position and orientation of the drone
    position, orientation = getPose(position, orientation)
    GL.glColor(0.5, 0.5, 0.5, 0.0)
    GL.glPushMatrix()
    tx, ty, tz = position
    GL.glTranslate(tx, ty, tz)
    pitch, roll, yaw = orientation
    GL.glRotate(yaw, 0, 0, 1)
    GL.glRotate(roll+90, 0, 1, 0)
    GL.glRotate(pitch, 0.8, 0, 0)
    GL.glScale(0.02, 0.05, 0.05)

    # Makes the cube for the drone
    glBegin(GL_QUADS)
    glColor3f(0.5,0.5,0.5)
    glVertex3f( 1.0, 1.0,-1.0)
    glVertex3f(-1.0, 1.0,-1.0)
    glVertex3f(-1.0, 1.0, 1.0)
    glVertex3f( 1.0, 1.0, 1.0) 

    glColor3f(0.5,0.5,0.5)
    glVertex3f( 1.0,-1.0, 1.0)
    glVertex3f(-1.0,-1.0, 1.0)
    glVertex3f(-1.0,-1.0,-1.0)
    glVertex3f( 1.0,-1.0,-1.0) 
    
    glColor3f(0.0,1.0,0.0)
    glVertex3f( 1.0, 1.0, 1.0)
    glVertex3f(-1.0, 1.0, 1.0)
    glVertex3f(-1.0,-1.0, 1.0)
    glVertex3f( 1.0,-1.0, 1.0)

    glColor3f(1.0,0.0,0.0)
    glVertex3f( 1.0,-1.0,-1.0)
    glVertex3f(-1.0,-1.0,-1.0)
    glVertex3f(-1.0, 1.0,-1.0)
    glVertex3f( 1.0, 1.0,-1.0)

    glColor3f(0.5,0.5,0.5)
    glVertex3f(-1.0, 1.0, 1.0) 
    glVertex3f(-1.0, 1.0,-1.0)
    glVertex3f(-1.0,-1.0,-1.0) 
    glVertex3f(-1.0,-1.0, 1.0) 

    glColor3f(0.5,0.5,0.5)
    glVertex3f( 1.0, 1.0,-1.0) 
    glVertex3f( 1.0, 1.0, 1.0)
    glVertex3f( 1.0,-1.0, 1.0)
    glVertex3f( 1.0,-1.0,-1.0)
    glEnd()

    GL.glPopMatrix()

    # Gets the position and orientation for GUI
    POSE.set("Position: %3s, %3s, %3s \nOrientation: %3s, %3s, %3s" % (str(round(tx,2)), str(round(ty,2)), str(round(tz,2)), str(round(pitch,2)), str(round(roll,2)), str(round(yaw,2))))

def draw_grid(cells=10, cell_size=0.1):
    # From world_viewer.py
    GL.glColor(0.4, 0.4, 0.4, 0.5)
    for nx in range(-cells//2, cells//2):
        xs = (nx*cell_size, (nx+1)*cell_size)
        for ny in range(-cells//2, cells//2):
            ys = (ny*cell_size, (ny+1)*cell_size)
            GL.glBegin(GL.GL_LINE_STRIP)
            for x, y in ((0, 0), (1, 0), (1, 1), (0, 1), (0, 0)):
                GL.glVertex(xs[x], ys[y], 0.0)
            GL.glEnd()


def draw_airspace(rmin, rmax):
    # From world_viewer.py
    r = np.array([rmin, rmax])
    GL.glBegin(GL.GL_LINES)
    GL.glColor(1, 0, 0, 1.0)
    for dim in range(3):
        for x in (0, 1):
            for y, z in ((0, 0), (1, 0), (1, 1), (0, 1)):
                rows = np.roll([x, y, z], dim)
                v = r[rows, (0, 1, 2)]
                GL.glVertex(*v)
    GL.glEnd()


def draw_wall(rmin, rmax):
    # From world_viewer.py
    r = np.array([rmin, rmax])
    GL.glBegin(GL.GL_QUADS)
    GL.glColor(0.8, 0.7, 0.5, 1.0)
    for xy, z in ((0, 0), (1, 0), (1, 1), (0, 1)):
        v = r[(xy, xy, z), (0, 1, 2)]
        GL.glVertex(*v)
    GL.glEnd()


def draw_marker(num, position, orientation, w=0.197, h=0.197):
    # From world_viewer.py
    GL.glColor(0, 0, 0.8, 0.9)
    GL.glPushMatrix()
    tx, ty, tz = position
    GL.glTranslate(tx, ty, tz)
    pitch, roll, yaw = orientation
    GL.glRotate(yaw, 0, 0, 1)
    GL.glRotate(roll, 0, 1, 0)
    GL.glRotate(pitch+90, 1, 0, 0)
    GL.glScale(w, h, 1.0)
    GL.glBegin(GL.GL_QUADS)
    GL.glVertex(-0.5, -0.5, -0.001)
    GL.glVertex(-0.5, +0.5, -0.001)
    GL.glVertex(+0.5, +0.5, -0.001)
    GL.glVertex(+0.5, -0.5, -0.001)
    GL.glEnd()
    GL.glPopMatrix()


def draw_roadsign(sign, position, orientation, w=0.2, h=0.2):
    # From world_viewer.py
    GL.glColor(0.8, 0, 0.8, 0.9)
    GL.glPushMatrix()
    tx, ty, tz = position
    GL.glTranslate(tx, ty, tz)
    pitch, roll, yaw = orientation
    GL.glRotate(yaw, 0, 0, 1)
    GL.glRotate(roll, 0, 1, 0)
    GL.glRotate(pitch+90, 1, 0, 0)
    GL.glScale(w, h, 1.0)
    GL.glBegin(GL.GL_QUADS)
    GL.glVertex(-0.5, -0.5, -0.001)
    GL.glVertex(-0.5, +0.5, -0.001)
    GL.glVertex(+0.5, +0.5, -0.001)
    GL.glVertex(+0.5, -0.5, -0.001)
    GL.glEnd()
    GL.glPopMatrix()

def getPose(position, orientation_quatarions):
    # Takes the PoseStamp.msg and takes out nessesary cord. 
    position_out = [0,0,0]
    orientation_out = [0,0,0]
    tmp = euler_from_quaternion((orientation_quatarions.x,
                                orientation_quatarions.y,
                                orientation_quatarions.z,
                                orientation_quatarions.w,))

    position_out[0] = position.x
    position_out[1] = position.y
    position_out[2] = position.z

    orientation_out[0] = math.degrees(tmp[0])
    orientation_out[1] = math.degrees(tmp[1])
    orientation_out[2] = math.degrees(tmp[2])
    return position_out, orientation_out

class AppOgl(OpenGLFrame):
    def initgl(self):
        # Initizates the world
        self.start = time.time()    # for FPS
        self.nframes = 0            # for FPS
        
        # From world_viewer.py: Render a single frame
        w, h = 800, 600
        GL.glMatrixMode(GL.GL_PROJECTION)
        # 45 FoV, aspect ratio, zNear clip, zFar clip.
        GLU.gluPerspective(45, w/h, 0.2, 20.0)
        origin = np.mean(np.array([WORLD.airspace.min, WORLD.airspace.max]), axis=0)

        GL.glEnable(GL.GL_BLEND)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glDepthFunc(GL.GL_LESS)
        GL.glMatrixMode(GL.GL_MODELVIEW)

        # Viewport starts with Y up and Z going away from the camera
        GL.glTranslate(0.0, -0.0, -3)   # NOTE: Change for alternative starting possition
        GL.glRotate(-75.0, 1, 0, 0)     # NOTE: Change for alternative starting angle


    def redraw(self):
        global ANGLE, X_ROTATE, Y_ROTATE, Z_ROTATE, X_ZOOM, Y_ZOOM, Z_ZOOM, WORLD
        # Change world from slecting list (GUI)
        if world_SELECTED.get() == 'demo01':
            json_fn = os.path.dirname(__file__) + '/demo01.world.json'
            with open(json_fn) as f:
                WORLD = json.load(f, object_hook=AttributeDict.object_hook)
        elif world_SELECTED.get() == 'demo02':
            json_fn = os.path.dirname(__file__) + '/demo02.world.json'
            with open(json_fn) as f:
                WORLD = json.load(f, object_hook=AttributeDict.object_hook)
        elif world_SELECTED.get() == 'demo03':
            json_fn = os.path.dirname(__file__) + '/demo03.world.json'
            with open(json_fn) as f:
                WORLD = json.load(f, object_hook=AttributeDict.object_hook)
    
        # Redraws the world and the drone
        origin = np.mean(np.array([WORLD.airspace.min, WORLD.airspace.max]), axis=0)
        t1 = time.time()
        dt = t1-self.start
        t0 = t1

        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glRotate(ANGLE, X_ROTATE, Y_ROTATE, Z_ROTATE)
        GL.glPushMatrix()

        # Makes changes to scene location
        x, y, z = -origin
        if X_RUNNING:
            X_ZOOM += STEP_SIZE
        elif Y_RUNNING:
            Y_ZOOM += STEP_SIZE
        elif Z_RUNNING:
            Z_ZOOM += STEP_SIZE
        GL.glTranslate(x+X_ZOOM, y+Y_ZOOM,z+Z_ZOOM)


        # Redraws the grid, airspace, wall, markers, roadsigns and drone
        draw_grid()
        draw_airspace(WORLD.airspace.min, WORLD.airspace.max)
        for wall in WORLD.walls:
            draw_wall(wall.plane.start, wall.plane.stop)
        for marker in WORLD.markers:
            draw_marker(marker.id, marker.pose.position, marker.pose.orientation)
        for roadsign in WORLD.roadsigns:
            draw_roadsign(roadsign.sign, roadsign.pose.position, roadsign.pose.orientation)

        draw_drone(pose.pose.position, pose.pose.orientation)
        GL.glPopMatrix()

        # Updates FPS and Frame count (GUI)
        if self.nframes > 1:
            t = time.time()-self.start
            FPS.set("FPS: %5.2f \nFrames: %d" % (self.nframes / t, self.nframes))
        self.nframes += 1

# Translation and rotation of scene functions
def turn_roll_right(event):
    global ANGLE, X_ROTATE
    ANGLE = -1 # Change for increased rotation
    X_ROTATE = 1

def turn_roll_left(event):
    global ANGLE, X_ROTATE
    ANGLE = 1 # Change for increased rotation
    X_ROTATE = 1

def turn_pitch_right(event):
    global ANGLE, Y_ROTATE
    ANGLE = -1 # Change for increased rotation
    Y_ROTATE = 1

def turn_pitch_left(event):
    global ANGLE, Y_ROTATE
    ANGLE = 1 # Change for increased rotation
    Y_ROTATE = 1
        
def turn_yaw_right(event):
    global ANGLE, Z_ROTATE
    ANGLE = -1 # Change for increased rotation
    Z_ROTATE = 1

def turn_yaw_left(event):
    global ANGLE, Z_ROTATE
    ANGLE = 1 # Change for increased rotation
    Z_ROTATE = 1

def stop(event):
    global ANGLE, X_ROTATE, Y_ROTATE, Z_ROTATE, X_RUNNING, Y_RUNNING, Z_RUNNING, STEP_SIZE
    # To stop pressed button action
    ANGLE = 0
    X_ROTATE = 0
    Y_ROTATE = 0
    Z_ROTATE = 0
    X_RUNNING = False
    Y_RUNNING = False
    Z_RUNNING = False
    STEP_SIZE = 0

def zoom_in_x(event):
    global  X_RUNNING, STEP_SIZE
    X_RUNNING = True
    STEP_SIZE = -0.01 # Change for increased translation

def zoom_out_x(event):
    global  X_RUNNING, STEP_SIZE
    X_RUNNING = True
    STEP_SIZE = 0.01 # Change for increased translation

def zoom_in_y(event):
    global  Y_RUNNING, STEP_SIZE
    Y_RUNNING = True
    STEP_SIZE = -0.01 # Change for increased translation

def zoom_out_y(event):
    global  Y_RUNNING, STEP_SIZE
    Y_RUNNING = True
    STEP_SIZE = 0.01 # Change for increased translation

def zoom_in_z(event):
    global  Z_RUNNING, STEP_SIZE
    Z_RUNNING = True
    STEP_SIZE = -0.01 # Change for increased translation

def zoom_out_z(event):
    global  Z_RUNNING, STEP_SIZE
    Z_RUNNING = True
    STEP_SIZE = 0.01 # Change for increased translation

def pose_callback(msg):
    # Subscribes on /cf1/pose and update drone pose
    global pose
    pose = msg

# Rospy stuff
rospy.init_node('DroneSimulationLight')
sub_pos = rospy.Subscriber('/cf1/pose', PoseStamped, pose_callback)


def main():
    global FPS, world_SELECTED, POSE

    # Gets unicode data for rightwards and leftwards arrow
    R_arrow = lookup("RIGHTWARDS ARROW")
    L_arrow = lookup("LEFTWARDS ARROW")

    # Frames & root settings
    root = Tk()
    # !Change for larger or smaller starting window.
    # !1080p 16:9 - "1920x1080" is a good option. 
    # !4K(3840x2400) 16:10 -"3840x2400" is a good option. 
    root.geometry("1920x1080")  
    root.title('Drone Simulation Light')
    root.configure(background='black')

    while not rospy.is_shutdown():
        # Makes nessesary frames for GUI  
        frame1=Frame(root, highlightthickness=1)
        frame1.place(relx=0.5, rely=0.5, anchor=CENTER)

        frame2=Frame(root, bg='grey')
        frame2.pack(side=TOP, fill=X)

        frame3=Frame(root, bg='black', highlightthickness=1)
        frame3.place(relx=0.8, rely=0.8, anchor=CENTER)
        

        # NOTE: Frame 1
        # Map OpenGL shower frame
        # !Change width and height for larger or smaller map view window. w=1080, h=880 good for 1080p 16x9 ratio
        # !1080p 16:9 - width=1080, height=880 is a good option.
        # !4k (3840 x 2160) 16:10 - width=3120, height=1950 is a good option.
        app = AppOgl(frame1, bg='black', width=1080, height=880) 
        app.grid(row=0, column=0)
        app.animate = 1
        app.after(100, app.printContext)


        # NOTE: Frame 2
        # world map selection
        world_options = ['Default',
                         'demo01', 
                         'demo02', 
                         'demo03']
        world_SELECTED = StringVar()
        world_SELECTED.set(world_options[0])
        world_drop_menu = OptionMenu(frame2, world_SELECTED, *world_options)

        #Frame 2 layout
        POSE = StringVar()
        pose_label = Label(frame2, textvariable=POSE, width=50, borderwidth=5, bg='grey', highlightthickness=0)
        pose_label.place(relx=0.7, rely=0.5, anchor=CENTER)

        FPS = StringVar()
        fps_label = Label(frame2, textvariable=FPS, width=25, borderwidth=5, bg='grey', highlightthickness=0)
        fps_label.place(relx=0.9, rely=0.5, anchor=CENTER)
        Label(frame2, text="default world: " + args[0], bg='grey').place(relx=0.5, rely=0.5, anchor=CENTER)
        Label(frame2, text='Change world file', bg='grey').grid(row=0, column=1, padx=20, pady=(5, 0))
        world_drop_menu.grid(row=1, column=1, pady=(5, 0))

        # TODO: Under develop: adding a move option in the GUI.
        # var = StringVar()
        # checkbox = Checkbutton(frame2, variable=var, onvalue='on', offvalue='off')
        # checkbox.deselect()

        # move_x = Entry(frame2, width=5)
        # move_y = Entry(frame2, width=5)
        # move_z = Entry(frame2, width=5)
        # move_yaw = Entry(frame2, width=5)
        # Label(frame2, text='Move', bg='grey').grid(row=0, column=2, padx=(25, 0), pady=(5, 0))
        # checkbox.grid(row=1, column=2, pady=(5, 0))
        # Label(frame2, text='X', bg='grey').grid(row=0, column=3, pady=(5, 0))
        # move_x.grid(row=1, column=3, pady=(5, 0))
        # Label(frame2, text='Y', bg='grey').grid(row=0, column=4, pady=(5, 0))
        # move_y.grid(row=1, column=4, pady=(5, 0))
        # Label(frame2, text='Z', bg='grey').grid(row=0, column=5, pady=(5, 0))
        # move_z.grid(row=1, column=5, pady=(5, 0))
        # Label(frame2, text='YAW', bg='grey').grid(row=0, column=6, pady=(5, 0))
        # move_yaw.grid(row=1, column=6, pady=(5, 0))
        
        
        # NOTE: Frame 3
        frame31=Frame(frame3, bg='black')
        frame32=Frame(frame3, bg='black')
        frame31.pack()
        frame32.pack()

        # Rotate roll, pitch and yaw buttons
        button_yaw_left = Button(frame31, text = L_arrow.encode('utf-8'))
        button_yaw_right = Button(frame31, text = R_arrow.encode('utf-8'))
        button_roll_left = Button(frame31, text = L_arrow.encode('utf-8'))
        button_roll_right = Button(frame31, text = R_arrow.encode('utf-8'))
        button_pitch_left = Button(frame31, text = L_arrow.encode('utf-8'))
        button_pitch_right = Button(frame31, text = R_arrow.encode('utf-8'))

        # Translation in x,y and z directions
        button_in_x = Button(frame31, text = '+', width=5)
        button_out_x = Button(frame31, text = '-', width=5)
        button_in_y = Button(frame31, text = '+', width=5)
        button_out_y = Button(frame31, text = '-', width=5)
        button_in_z = Button(frame31, text = '+', width=5)
        button_out_z = Button(frame31, text = '-', width=5)

        # Frame 3 layout
        # Rotation
        Label(frame31, text="ROLL", fg = 'white', bg='black').grid(row=1, column=0)
        button_roll_left.grid(row=1, column=1)
        button_roll_right.grid(row=1, column=2)
        button_roll_left.bind("<Button-1>", turn_roll_left)
        button_roll_left.bind('<ButtonRelease-1>', stop)
        button_roll_right.bind("<Button-1>", turn_roll_right)
        button_roll_right.bind('<ButtonRelease-1>', stop)

        Label(frame31, text="PITCH", fg = 'white', bg='black').grid(row=2, column=0)
        button_pitch_left.grid(row=2, column=1)
        button_pitch_right.grid(row=2, column=2)
        button_pitch_left.bind("<Button-1>", turn_pitch_left)
        button_pitch_left.bind('<ButtonRelease-1>', stop)
        button_pitch_right.bind("<Button-1>", turn_pitch_right)
        button_pitch_right.bind('<ButtonRelease-1>', stop)

        Label(frame31, text="YAW", fg = 'white', bg='black').grid(row=3, column=0)
        button_yaw_left.grid(row=3, column=1)
        button_yaw_right.grid(row=3, column=2)
        button_yaw_left.bind("<Button-1>", turn_yaw_left)
        button_yaw_left.bind('<ButtonRelease-1>', stop)
        button_yaw_right.bind("<Button-1>", turn_yaw_right)
        button_yaw_right.bind('<ButtonRelease-1>', stop)

        # Translation
        Label(frame31, text="X", fg = 'white', bg='black').grid(row=0, column=4, padx=(20,0))
        button_in_x.grid(row=1, column=4, padx=(20,0))
        button_out_x.grid(row=2, column=4, padx=(20,0))
        button_in_x.bind("<Button-1>", zoom_in_x)
        button_in_x.bind('<ButtonRelease-1>', stop)
        button_out_x.bind("<Button-1>", zoom_out_x)
        button_out_x.bind('<ButtonRelease-1>', stop)

        Label(frame31, text="Y", fg = 'white', bg='black').grid(row=0, column=5)
        button_in_y.grid(row=1, column=5)
        button_out_y.grid(row=2, column=5)
        button_in_y.bind("<Button-1>", zoom_in_y)
        button_in_y.bind('<ButtonRelease-1>', stop)
        button_out_y.bind("<Button-1>", zoom_out_y)
        button_out_y.bind('<ButtonRelease-1>', stop)

        Label(frame31, text="Z", fg = 'white', bg='black').grid(row=0, column=6)
        button_in_z.grid(row=1, column=6)
        button_out_z.grid(row=2, column=6)
        button_in_z.bind("<Button-1>", zoom_in_z)
        button_in_z.bind('<ButtonRelease-1>', stop)
        button_out_z.bind("<Button-1>", zoom_out_z)
        button_out_z.bind('<ButtonRelease-1>', stop)

        # Color info about for the map
        Label(frame32, text='Rear of the drone', bg='red', width=15).grid(row=1, column=0, padx=20, pady=(25, 0))
        Label(frame32, text='Front of the drone', bg='green', width=15).grid(row=2, column=0, padx=20, pady=(5, 10))
        Label(frame32, text='Aruco markers', bg='blue', width=15).grid(row=1, column=1, padx=20, pady=(25, 0))
        Label(frame32, text='Signs', bg='purple', width=15).grid(row=2, column=1, padx=20, pady=(5, 10))

        app.mainloop()


if __name__ == '__main__':
    main()
