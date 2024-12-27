# Introduction to Fusion 360 modeling for ROS 2

In this document, we can see how to model a 2 wheel drive robot in Fusion 360 in-order to export into ROS 2 URDF.

Here is the robot model we are going to design

<p align="center">
  <img src="img/f3d_model.png" alt="Fusion Model">
</p>

Here the important steps in the modeling of this robot

# 1) Setting the Design plane for export

We must follow the right-hand rule for setting the plane before starting the modeling. If we follow this, the exported URDF model will face the + X-axis.

Here is the design plane we have to set before start the design.
<p align="center">
  <img src="img/modeling/coordinate.png" alt="Coordinate" width="50%">
</p>

# 2) Sketching Robot Base

After setting the plane, we can start sketching the base of the robot. After sketching the plane, we can extrude the plane to create the box.

Select Menu, SOLID -> Create Sketch. After pressing this option, it will ask which plane we have to draw the sketch. It will show different planes and choosing the exact plane we want may be confusing at first time. We can use Shift+ Mouse Center button to orbit the 3D view to select the same plane we have seen in the first step.

<p align="center">
  <img src="img/modeling/select_plane.png" alt="select_plane" width="100%">
</p>


After selecting the plane, orbit the 3D scene to make the +X axis always front. 

Now, you can select the 2-point rectangle, and from sketch-palette window on right side, choose center rectange option as shown below. This option, can draw a rectangle from a center point. 

After selecting, center point option, go to the origin of the plane and click the mouse and you can start drawing the rectangle.

<p align="center">
  <img src="img/modeling/draw_square.png" alt="draw_square" width="100%">
</p>

When you start drawing the square, you can find the dimension of the square, you can use Tab key to toggle the dimension. You can edit the dimensions in the box. For this model, you may can give 200 mm for each side.

<p align="center">
  <img src="img/modeling/draw_square_1.png" alt="draw_square" width="100%">
</p>


<p align="center">
  <img src="img/modeling/draw_square_2.png" alt="draw_square" width="100%">
</p>

After drawing giving the correct dimensions, you can the click the finish sketch button on the right side top to complete the sketch.

After completing the sketch, you can click on the top of the sketch for selecting the top area of the sketch.


<p align="center">
  <img src="img/modeling/draw_square_3.png" alt="draw_square" width="100%">
</p>

Now press the key 'Q' (Menu Solid ->Modify->Press Pull), to extrude the sketch surface to make a box. Once you press the 'Q' button you can see an arrow key to extrude the sketch to some height. You can give a values as 50 mm as the height.

<p align="center">
  <img src="img/modeling/draw_square_4.png" alt="draw_square" width="100%">
</p>

After giving the height, select Operation option from Extrude window on right side. The Operation option we have to select is New Body. After this option, we can press Ok.

# 3) Adding Wheels to the Robot base

# 4) Moving Robot to Ground Level

# 5) Adding Lidar base and lidar to Robot

# 6) Adding Caster Wheels to the Base of the robot

# 8) Adding material type and color

# 9) Assigning Joints to Wheels

# 10) Exporting to URDF