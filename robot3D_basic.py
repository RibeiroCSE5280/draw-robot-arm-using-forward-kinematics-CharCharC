#!/usr/bin/env python
# coding: utf-8


from vedo import *
import numpy as np
# from PIL import ImageGrab

def RotationMatrix(theta, axis_name):
    """ calculate single rotation of $theta$ matrix around x,y or z
        code from: https://programming-surgeon.com/en/euler-angle-python-en/
    input
        theta = rotation angle(degrees)
        axis_name = 'x', 'y' or 'z'
    output
        3x3 rotation matrix
    """

    c = np.cos(theta * np.pi / 180)
    s = np.sin(theta * np.pi / 180)
	
    if axis_name =='x':
        rotation_matrix = np.array([[1, 0,  0],
                                    [0, c, -s],
                                    [0, s,  c]])
    if axis_name =='y':
        rotation_matrix = np.array([[ c,  0, s],
                                    [ 0,  1, 0],
                                    [-s,  0, c]])
    elif axis_name =='z':
        rotation_matrix = np.array([[c, -s, 0],
                                    [s,  c, 0],
                                    [0,  0, 1]])
    return rotation_matrix


def createCoordinateFrameMesh():
    """Returns the mesh representing a coordinate frame
    Args:
      No input args
    Returns:
      F: vedo.mesh object (arrows for axis)
      
    """         
    _shaft_radius = 0.05
    _head_radius = 0.10
    _alpha = 1
    
    
    # x-axis as an arrow  
    x_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(1, 0, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='red',
                        alpha=_alpha)

    # y-axis as an arrow  
    y_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 1, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='green',
                        alpha=_alpha)

    # z-axis as an arrow  
    z_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 0, 1),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='blue',
                        alpha=_alpha)
    
    originDot = Sphere(pos=[0,0,0], 
                       c="black", 
                       r=0.10)


    # Combine the axes together to form a frame as a single mesh object 
    F = x_axisArrow + y_axisArrow + z_axisArrow + originDot
        
    return F


def getLocalFrameMatrix(R_ij, t_ij): 
    """Returns the matrix representing the local frame
    Args:
      R_ij: rotation of Frame j w.r.t. Frame i 
      t_ij: translation of Frame j w.r.t. Frame i 
    Returns:
      T_ij: Matrix of Frame j w.r.t. Frame i. 
      
    """             
    # Rigid-body transformation [ R t ]
    T_ij = np.block([[R_ij,                t_ij],
                     [np.zeros((1, 3)),       1]])
    
    return T_ij
	
def forward_kinematics(Phi, L1, L2, L3, L4):
    radius = .4
    origin_loc = np.array([[3],
                           [2],
                           [0]])
    
    # Calculate frame 1 w.r.t the origin
    R_01 = RotationMatrix(Phi[0], axis_name="z")
    t_01 = origin_loc
    T_01 = getLocalFrameMatrix(R_01, t_01)

    # Calculate frame 2 w.r.t frame 1
    R_12 = RotationMatrix(Phi[1], axis_name="z")
    t_12 = np.array([[L1 + (2*radius)],
                     [0.0],
                     [0.0]])
    T_12 = getLocalFrameMatrix(R_12, t_12)
    T_02 = T_01 @ T_12

    # Calculate frame 3 w.r.t frame 2
    R_23 = RotationMatrix(Phi[2], axis_name="z")
    t_23 = np.array([[L2 + (2*radius)],
                     [0.0],
                     [0.0]])
    T_23 = getLocalFrameMatrix(R_23, t_23)
    T_03 = T_02 @ T_23

    # Calculate frame 3 w.r.t frame 4
    R_34 = RotationMatrix(Phi[3], axis_name="z")
    t_34 = np.array([[L3 + (radius)],
                     [0.0],
                     [0.0]])
    T_34 = getLocalFrameMatrix(R_34, t_34)
    T_04 = T_03 @ T_34

    # Calculate the end-effector
    e = T_04[:, -1][:3]
    
    return T_01, T_02, T_03, T_04, e

# Gets executed by the timer
def handle_timer(event):
    global T_01, T_02, T_03, T_04, counter1, counter2, counter3, images, num_frames
    # Only animate 50 frames
    if num_frames < 50:
      # Revert each mesh to its original position
      Frame1.apply_transform(np.linalg.inv(T_01))
      Frame2.apply_transform(np.linalg.inv(T_02))
      Frame3.apply_transform(np.linalg.inv(T_03))
      end_effector.apply_transform(np.linalg.inv(T_04))

      # Determines new angles
      if -90 + counter1 < -45:
        phi2 = [-90 + counter1,	120 + counter2,	160 + counter3,	0]
        counter1 += 1
        counter2 += 3
        counter3 += 5
      else:
          phi2 = [-90 + counter1,	120 + counter2,	160 + counter3,	0]
          counter2 += 3
          counter3 += 5
      L1 = 5
      L2 = 8
      L3 = 3
      L4 = 0

      # Calculates new transformation matrices
      T_01, T_02, T_03, T_04, e = forward_kinematics(phi2, L1, L2, L3, L4)
      Frame1.apply_transform(T_01)
      Frame2.apply_transform(T_02)
      Frame3.apply_transform(T_03)
      end_effector.apply_transform(T_04)
      plotter.render()
      # Takes a screenshot for the gif
      # images.append(ImageGrab.grab(bbox = (10,50,1000,800)))
      num_frames += 1


# def main():

  # Set the limits of the graph x, y, and z ranges 
axes = Axes(xrange=(0,20), yrange=(-2,10), zrange=(0,6))

# Lengths of arm parts 
L1 = 5   # Length of link 1
L2 = 8   # Length of link 2
L3 = 3   # Length of link 3
L4 = 0   # Length of link 4

# Joint angles 
phi = [-90,	120, 160,	0]
# Arm radius
radius = .4

# Stores the images for the gif
images = []

# Starting matrices
T_01, T_02, T_03, T_04, e = forward_kinematics(phi, L1, L2, L3, L4)

# Link 1
# Create the coordinate frame mesh and transform
Frame1Arrows = createCoordinateFrameMesh()

# Now, let's create a cylinder and add it to the local coordinate frame
link1_mesh = Cylinder(r=0.4, 
                      height=L1, 
                      pos = ((L1/2)+radius,0,0),
                      c="red", 
                      alpha=.8, 
                      axis=(1,0,0)
                      )

# Also create a sphere to show as an example of a joint
sphere1 = Sphere(r = radius).pos(0,0,0).color("gray").alpha(.8)

# Combine all parts into a single object 
Frame1 = Frame1Arrows + link1_mesh + sphere1

# Transform the part to position it at its correct location and orientation 
Frame1.apply_transform(T_01)  


# Link 2
# Create the coordinate frame mesh and transform
Frame2Arrows = createCoordinateFrameMesh()

# Now, let's create a cylinder and add it to the local coordinate frame
link2_mesh = Cylinder(r=0.4, 
                      height=L2, 
                      pos = ((L2/2)+radius,0,0),
                      c="red", 
                      alpha=.8, 
                      axis=(1,0,0)
                      )

# Also create a sphere to show as an example of a joint
sphere2 = Sphere(r = radius).pos(0,0,0).color("gray").alpha(.8)

# Combine all parts into a single object 
Frame2 = Frame2Arrows + link2_mesh + sphere2

# Transform the part to position it at its correct location and orientation 
Frame2.apply_transform(T_02)  


# Link 3
# Create the coordinate frame mesh and transform
Frame3Arrows = createCoordinateFrameMesh()

# Now, let's create a cylinder and add it to the local coordinate frame
link3_mesh = Cylinder(r=0.4, 
                      height=L3, 
                      pos = ((L3/2)+radius,0,0),
                      c="red", 
                      alpha=.8, 
                      axis=(1,0,0)
                      )

# Also create a sphere to show as an example of a joint
sphere3 = Sphere(r = radius).pos(0,0,0).color("gray").alpha(.8)

# Combine all parts into a single object 
Frame3 = Frame3Arrows + link3_mesh + sphere3

# Transform the part to position it at its correct location and orientation 
Frame3.apply_transform(T_03)  


# End Effector
# Create the coordinate frame mesh and transform
end_effector = createCoordinateFrameMesh()

# Transform the part to position it at its correct location and orientation 
end_effector.apply_transform(T_04)

# Used to update the angles for the animation
counter1 = 1
counter2 = 3
counter3 = 5

# Counts the number of frames in the animation
num_frames = 0

# Creates the animation
plotter = Plotter(size=(700, 500))
# plotter.initialize_interactor()
timer_id = plotter.timer_callback("start", dt=100)
timerevt = plotter.add_callback('timer', handle_timer, enable_picking=True)
plotter.show([Frame1, Frame2, Frame3, end_effector], axes, viewup="z")

# Saves the frames into a gif
# if num_frames > 40:
#   images[0].save('robot.gif', 
#                 save_all = True, append_images = images[1:],  
#                 optimize = False, duration = 10, loop=0)



