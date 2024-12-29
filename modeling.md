# Introduction to Fusion 360 modeling for ROS 2

In this document, we can see how to model a 2-wheel drive robot in Fusion 360 in order to export into ROS 2 URDF.

Here is the robot model we are going to design.

<p align="center">
  <img src="img/f3d_model.png" alt="Fusion Model">
</p>

Here are the important steps in the modeling of this robot

# 1) Setting the Design plane for export

We must follow the *right-hand rule* for setting the plane before starting the modeling. 

If we follow this, the exported URDF model will face the *+X-axis*. 
This is the axis we need when we visualize robots in ROS 2.

Here is the design plan we have to set before starting the design.

<p align="center">
  <img src="img/modeling/coordinate.png" alt="Coordinate" width="50%">
</p>

# 2) Sketching Robot Base

After setting the plane, we can start sketching the robot's base. After sketching the plane, we can extrude the plane to create the box.

Select Menu, *SOLID -> Create Sketch*. After pressing this option, it will ask which plane we have to draw the sketch. 

It will show different planes, and choosing the exact plane we want may be confusing. 

We can use the *Shift+ Mouse Center button* to orbit the 3D view to select the same plane we have seen in the first step.

<p align="center">
  <img src="img/modeling/select_plane.png" alt="select_plane" width="100%">
</p>


After selecting the plane, orbit the 3D scene to make the +X axis always front. 

Now, you can select the *2-point rectangle* from the *SKETCH*. From *Sketch-palette* window on right side, choose *center rectange* option as shown below. 
This option, can draw a rectangle from a center point. 

<p align="center">
  <img src="img/modeling/draw_square.png" alt="draw_square" width="100%">
</p>


After selecting the plane, orbit the 3D scene to make the +X axis always front. 

Now, you can select the *2-point rectangle* from the *SKETCH*. From the *Sketch-palette* window on the right side, choose the *center rectangle* option as shown below. 
This option allows you to draw a rectangle from a center point.


<p align="center">
  <img src="img/modeling/draw_square_1.png" alt="draw_square" width="100%">
</p>

Press the *Tab* key to switch between the square dimensions and enter the dimensions. 
You can give **200 mm** as the width and length for the base_link.

<p align="center">
  <img src="img/modeling/draw_square_2.png" alt="draw_square" width="100%">
</p>

After drawing the correct dimensions, you can click the *Finish sketch* button on the right side of the top to complete the sketch.

After completing the sketch, you can click on the top to select the sketch's top area.

<p align="center">
  <img src="img/modeling/draw_square_3.png" alt="draw_square" width="100%">
</p>

Now press the key * 'Q' (Menu Solid ->Modify->Press Pull)* to extrude the sketch surface to make a box. Once you press the *'Q'* button, you will see an arrow key that will extrude the sketch to some height. You can give a value of 50 mm as the height.

<p align="center">
  <img src="img/modeling/draw_square_4.png" alt="draw_square" width="100%">
</p>

After giving the height as 50 mm, select the *Operation* option from the *Extrude window* on the right side. The *Operation* option we must select is **New Body**. After this option, we can press Ok.

# 3) Adding Wheels to the Robot base

We can add wheels after making the robot's main chassis/base_link.
We must add two active and two passive caster wheels to the chassis.

The following image shows the *Z-X plane*, which shows the side of the chassis in which we have to add wheels.

<p align="center">
  <img src="img/modeling/wheel_1.png" alt="wheels" width="100%">
</p>

To create wheels, we can start a sketch and extrude it like we did for the chassis.
In the following image, we can see that you have chosen the *Sketch option* and chose a circle with 
center diameter circle, and then choose the center of the chassis, as shown in the following image.
Give a diameter of **100 mm** and press *Finish the sketch*.

<p align="center">
  <img src="img/modeling/wheel_2.png" alt="wheels" width="100%">
</p>

Now, you can click on the wheel sketch. You may have to press the *Shift* key to
select each segment of the wheel.

Now, press *'Q'* to extrude it and make the distance *25mm* the thickness of the wheel.

The important thing to note is that we have to make the wheel a new body in the operation option in extrude.
This makes a new wheel a new body.


<p align="center">
  <img src="img/modeling/wheel_3.png" alt="wheels" width="100%">
</p>

We can create the next wheel with the same process we have done with the first wheel.

# 4) Moving Robot to Ground Level

After creating 2 wheels and the base_link, we have 3 independent bodies. 
The next step is to move the entire robot above the design plane. You can compute the distance by measuring it and 
moving the robot using the *Move button*. Make sure you have selected all the 3 bodies before you move. You can roughly put *25 mm* as the *Z value* in the *Move window* to lift the robot from the design floor.

You can find the robot's position in the following image.

<p align="center">
  <img src="img/modeling/wheel_4.png" alt="wheels" width="100%">
</p>

Here is the side view of the lifted robot.

<p align="center">
  <img src="img/modeling/lift_robot.png" alt="lift_robot" width="100%">
</p>

# 5) Adding Caster Wheels to the Base of the robot

After lifting the robot, the next step is to add caster wheels to the robot.
We have to add a caster wheel on the front and back of the robot.
As we already discussed, casters are passive wheels that help the robot balance and distribute weight.

To add two casters, we must create 2 small cylinders on the **bottom** side of the chassis. 

To create a cylinder, we can extrude a circle sketch.
The length of the extrude can be *24 mm*, a few mm less than the robot's height from the ground.

<p align="center">
  <img src="img/modeling/caster_sketch.png" alt="caster_sketch" width="100%">
</p>

Here is how we extrude the caster wheel. Make sure the Operation is a **Joint**, NOT a *New body*. The caster is a part of the chassis.

<p align="center">
  <img src="img/modeling/caster_cylinder.png" alt="caster_sketch" width="100%">
</p>

After creating the cylinder on both sides, we can make the bottom face of the cylinder, spherical.

Select the face of the cylinder, and Press *'F'*  or *(Menu Surface->Modify->Fillet)*, and using a mouse, we can create a spherical face from the flat surface.

<p align="center">
  <img src="img/modeling/spherical.png" alt="spherical" width="100%">
</p>

We can do the same operation for both the caster wheels and the final output will look like the following image. 

<p align="center">
  <img src="img/modeling/caster_full.png" alt="spherical" width="100%">
</p>

# 6) Adding Lidar base and lidar to Robot

Once the caster design is completed, we can add a lidar to the front of the robot. We have to build a fixed platform first.
After putting the base, we can create a cylinder shape to replicate a lidar, as shown below.

You can draw the box with 70 mm(length) x 60 mm(width) x 45 mm (height).

<p align="center">
  <img src="img/modeling/lidar_box.png" alt="lidar_box" width="100%">
</p>

After creating the box, we can make a circle on the top and create a cylinder by extruding the circle.
Make sure you are creating a new body when you extrude the object.

After creating all bodies, you can rename them into meaningful names like the one below.

<p align="center">
  <img src="img/modeling/full_model.png" alt="full_model" width="100%">
</p>

The chassis has to be named as base_link, which is mandatory, and you can name 
left and right wheels have the same name, and you can also add a name to the lidar.

# 7) Adding material type and color

After creating the model, we may go to each *Body*, right-click on each body, and choose *Physical materials*.

The *Physical material* can assign each link with material properties.

<p align="center">
  <img src="img/modeling/body_materials.png" alt="body_materials" width="100%">
</p>

Search for ABS plastic for the base link. Just click and drag the material to the link to apply the material.
Choose material *Rubber black* for wheels and laminate blue for lidar.

<p align="center">
  <img src="img/modeling/rubber_material.png" alt="rubber_materials" width="100%">
</p>


# 8) Converting bodies to Components

After adding the material's properties, we must convert the individual bodies to Components.
The conversion from *bodies* to *components* is easy. 

Here is how we can do it. 

Click on the *Bodies* option and choose the option called *Components* to *Bodies*.
As shown below. Bodies are single shapes within a component. A component can hold multiple bodies.
To export to URDF, we need to make individual *components* of the robot.

<p align="center">
  <img src="img/modeling/components.png" alt="components" width="100%">
</p>

# 9) Assigning Joints to Wheels and Joints

After assigning the materials, we must assign the joints for connecting 
wheels to *base_link*. We also have to attach the lidar link to the base_link. 

This is the most 
important step in the modeling. Without assigning joints, the robot can't able to move.

Here are the steps to assign a joint in Fusion 360. We can first check how to create a joint between wheels and *base_link*.

First, we must hide the *base_link* component and press *'J'* *(Menu Solid->Assemble->Joint)* to create a new joint.
After invoking the *joint* option, we must provide the *components* (link) in the joint.

We can see how to create a revolute joint between the base_link and the two wheels. 

To create a joint, we need two 2 components: the first component should be the wheel, and the second will be the base_link.

We have to hide the base_link first and click on the center of the wheel, which is attached to the base_link.

<p align="center">
  <img src="img/modeling/joint_1.png" alt="joint" width="100%">
</p>


After clicking on the center of the wheel, we can see a coordinate on the wheel. Next, we can hide the wheels and attach the frame in the base_link.
<p align="center">
  <img src="img/modeling/joint_2.png" alt="joint" width="100%">
</p>

Once it is done, you can press the *Motion Tab* in the *Edit Joint option* to select the type of joint.

<p align="center">
  <img src="img/modeling/joint_3.png" alt="joint" width="100%">
</p>

We need the revolute joint for the wheels, so select it, and you can preview the motion of the joint as well.

After setting one joint, you can do the same for the next wheel as well.

For the lidar link, we have to create a rigid link between the lidar and base_link.

Congratulations, you have done with the modeling part of the robot. Now, we can export the model to URDF.

We can continoue with the export instruction in the main [README](README.md)
