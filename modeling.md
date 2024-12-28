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

After making the main chassis of the robot, we can now add wheels to the chassis.
We have to add two active wheels and two passive caster wheels to the chassis.

The following image shows Z-X plane, which shows the side of the chassis in which we 
have to add wheels.

<p align="center">
  <img src="img/modeling/wheel_1.png" alt="wheels" width="100%">
</p>

Now, to create wheels, we can start a sketch and extrude it like we did it for the chassis.
In the following image, we can see have chose Sketch option, and chose circle with 
center diameter circle and then choose the center of chassis like the following image.
Give diameter as 100 mm and finish the sketch.

<p align="center">
  <img src="img/modeling/wheel_2.png" alt="wheels" width="100%">
</p>

Now, you can click on the wheel sketch. You may have to press Shift key to
select each segment of the wheel.

Now, press 'Q' to extrude it and make the distance 25mm as thickness of the wheel.
Important think to note is, we have to make the wheel as a new body in the operation option in extrude.
This make a new wheel as a new body.


<p align="center">
  <img src="img/modeling/wheel_3.png" alt="wheels" width="100%">
</p>

We can create the next wheel with the same process we have done with the first wheel.


# 4) Moving Robot to Ground Level

After creating 2 wheels and the chasis, totally, we have 3 bodies. 
The next step is to move the entire robot above the design plane. You can compute the distance by measuring it and you can
move the robot using Move button. Make sure you have selected all the 3 bodies before you move. You can roughly put 25 mm as Z value in Move window to lift the robot up from the design floor.

You can find the robot position like the following image.
<p align="center">
  <img src="img/modeling/wheel_4.png" alt="wheels" width="100%">
</p>

Here is the side of the lifted robot.

<p align="center">
  <img src="img/modeling/lift_robot.png" alt="lift_robot" width="100%">
</p>

# 5) Adding Caster Wheels to the Base of the robot

After lifting the robot, the next step is to add caster wheels to the robot.
We have to add caster wheel on the front and back of the robot.
As we already discussed, casters are passive wheels which help the robot 
to balance the robot model and also helps for weight distribution.

In order to add two casters, we need to create 2 small cylinder on the bottom side of the 
chassis. In order create a cylinder, we can use circle sketch and extrude it.
The length of the extrude can be 24 mm, few mm less than than the robot height from the ground.


<p align="center">
  <img src="img/modeling/caster_sketch.png" alt="caster_sketch" width="100%">
</p>

Here is how we extrude the caster wheel. Make sure the operation is joint not new body. The caster is a part of the chassis.
<p align="center">
  <img src="img/modeling/caster_cylinder.png" alt="caster_sketch" width="100%">
</p>

After creating the cylinder on both side, we can make the bottom face of the cylinder as spherical.

Select the face of cylinder, and Press 'F' or (Menu Surface->Modify->Fillet) and using mouse, we can create a spherical face from the flat surface.

<p align="center">
  <img src="img/modeling/spherical.png" alt="spherical" width="100%">
</p>

We can do the same operation for both the caster wheels, and the final output will look like the following image. 

<p align="center">
  <img src="img/modeling/caster_full.png" alt="spherical" width="100%">
</p>

# 6) Adding Lidar base and lidar to Robot

Once the caster design are completed, next we can add a lidar to the front of the robot. We have to build a fixed platform first.
After putting the base, we can create a cylinder shape on the top of it to replicate a lidar as show below.

You can draw the box with 70 mm(length) x 60 mm(width) x 45 mm (height).

<p align="center">
  <img src="img/modeling/lidar_box.png" alt="lidar_box" width="100%">
</p>

After creating the box, we can do a a circle on the top of the box and create a cylinder by extruding the circle.
Make sure, you are creating a new body when you extrude the object.

After creating all bodies, you can rename these bodies into meaniful name like below

<p align="center">
  <img src="img/modeling/full_model.png" alt="full_model" width="100%">
</p>

The chassis has to name as base_link which is mandatory, and you can name 
left and right wheel as same name and you can add name to lidar as well.

# 7) Adding material type and color

After creating the model, we may can goto each body and right click on each body and choose Physical materials.

The Physical material can assign each link with a material properties.

<p align="center">
  <img src="img/modeling/body_materials.png" alt="body_materials" width="100%">
</p>

Search for ABS plastic for the base link. Just click and drag the material to the link to apply the material.
Choose Rubber black for wheels and laminate blue for lidar.

<p align="center">
  <img src="img/modeling/rubber_material.png" alt="rubber_materials" width="100%">
</p>


# 8) Converting bodies to Components

After adding the materials properties, we have to convert the individual bodies to Components.
The convertion to bodies to Components is easy to do. 

Here is how we can do it. 

Click on to the Bodies option and choose the option called Components to Bodies
as shown below. Bodies are single shape within a component. A component can hold multiple bodies.
In order to export to URDF, we need to make individual component of the robot.

<p align="center">
  <img src="img/modeling/components.png" alt="components" width="100%">
</p>

# 9) Assigning Joints to Wheels and Joints

After assigning the materials, we need to assign the joints for connecting 
wheels to base_link. We also has to attach the lidar link to the base_link. This is the most 
important step in the modeling. Without assigning joints, the robot can't able to move.

Here are the steps to assign a joint in Fusion 360. We can first check how to create joint between wheels and base_link.

First, we have to hide the base_link component, and press 'J' (Menu Solid->Assemble->Joint) to create a new joint.
After invoking the joint option, we have to provide the components (link) which is in the joint.

We can see how to create a revolute joint between the base_link the two wheels. 
In order to create a joint, we need to 2 components, the first components should be the wheel and the second will be the base_link.

We have to hide the base_link first, and click on the center of the wheel which is attached to the base_link.


<p align="center">
  <img src="img/modeling/joint_1.png" alt="joint" width="100%">
</p>


After click on the center on the wheel, we can see a coordinate on the wheel, next, we can hide the wheels and attach the frame in the base_link.

<p align="center">
  <img src="img/modeling/joint_2.png" alt="joint" width="100%">
</p>

Once it is done, you can press Motion Tab in Edit Joint option to select the type of joint.

<p align="center">
  <img src="img/modeling/joint_3.png" alt="joint" width="100%">
</p>

The revolute joint is what we need for the wheels, so select it and you can preview the motion of the joint as well.

After setting one joint, you can do the same for next wheel as well.

For lidar link, we have to create a rigid link between lidar and base_link.

Congratulation, you have done with the modeling part of the robot. Now we can export the model to URDF.


