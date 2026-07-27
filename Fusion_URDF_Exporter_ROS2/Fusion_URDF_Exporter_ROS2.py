#Author-syuntoku14, Dheena2k2, Lentin Joseph, enezl
#Description-Generate URDF file from Fusion 360

import adsk, adsk.core, adsk.fusion, traceback
import os, re
import sys
from .utils import utils
from .core import Link, Joint, Write, urdf_tree

"""
# length unit is 'cm' and inertial unit is 'kg/cm^2'
# If there is no 'body' in the root component, maybe the corrdinates are wrong.
"""

# joint effort: 100
# joint velocity: 100
# supports "Revolute", "Rigid" and "Slider" joint types

# I'm not sure how prismatic joint acts if there is no limit in fusion model


def is_valid_robot_name(name):
    return bool(re.fullmatch(r'[A-Za-z][A-Za-z0-9_-]*', name))


def make_package_name(robot_name):
    package_name = re.sub(r'[^a-z0-9_]+', '_', robot_name.lower())
    package_name = re.sub(r'_+', '_', package_name).strip('_')
    if not package_name or not package_name[0].isalpha():
        package_name = 'robot_' + package_name
    return package_name + '_description'


def run(context):
    ui = None
    success_msg = 'Successfully created URDF file'
    msg = success_msg

    try:
        # --------------------
        # initialize
        app = adsk.core.Application.get()
        ui = app.userInterface


        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        title = 'Fusion 360 -> ROS 2 URDF'
        if not design:
            ui.messageBox('No active Fusion design', title)
            return

        root = design.rootComponent  # root component
        components = design.allComponents

        robot_name = root.name.split()[0]

        # Show welcome message
        welcome_msg = ("Welcome to the Fusion 'Fusion 360 -> ROS 2 URDF Script' plugin.\n"
                       "\n"
                       "This tool generates a robot_description package with launch files for visualizing your robot in Rviz and spawning the model in Gazebo.\n"
                       "\n"
                       "It has been tested with ROS 2 Jazzy and Gazebo Harmonic, as well as ROS 2 Humble with Gazebo Classic.\n"
                       "\n"
                       "WARNING The current version only works with fusion in english.\n"
                       "\n"
                       "Press OK to continue or Cancel to quit.")
        if ui.messageBox(welcome_msg, title, adsk.core.MessageBoxButtonTypes.OKCancelButtonType) != adsk.core.DialogResults.DialogOK:
            return
        name_choice_msg = ("Do you want to change the name of the robot ?\n"
                           "\n"
                           f"Current name : {robot_name}"
                           "\n")

        change_msg = "Choose a new robot name:\n"
        error_msg = ""
        if ui.messageBox(name_choice_msg, title, adsk.core.MessageBoxButtonTypes.YesNoButtonType) != adsk.core.DialogResults.DialogNo:
            valid = False
            while not valid:
                temp_name, box_val = ui.inputBox(change_msg + error_msg, title)
                if box_val != 0:
                    ui.messageBox('Fusion 360 -> ROS 2 URDF was canceled', title)
                    return
                if is_valid_robot_name(temp_name):
                    robot_name = temp_name
                    valid = True
                else:
                    error_msg = (
                        "\nUse a non-empty name starting with a letter and "
                        "containing only letters, digits, underscores, or dashes.\n"
                    )

        # set the names

        package_name = make_package_name(robot_name)
        # # Show folder browse message
        # browse_msg = "Press Ok to browse the folder for saving the ROS package, cancel to quit."
        # if ui.messageBox(browse_msg, title, adsk.core.MessageBoxButtonTypes.OKCancelButtonType) != adsk.core.DialogResults.DialogOK:
        #     return

        # Browse folder
        save_dir = utils.file_dialog(ui)
        if save_dir == False:
            ui.messageBox('Fusion 360 -> ROS 2 URDF was canceled', title)
            return 0

        save_dir = save_dir + '/' + package_name
        try:
            os.mkdir(save_dir)
        except:
            pass

        # Ask for Gazebo version
        gazebo_msg = "Are you using Gazebo Harmonic? Press Yes for Gazebo Harmonic, No for Gazebo Classic (Legacy)."
        if ui.messageBox(gazebo_msg, title, adsk.core.MessageBoxButtonTypes.YesNoButtonType) == adsk.core.DialogResults.DialogYes:
            #Gazebo Harmonic

            package_dir = os.path.abspath(os.path.dirname(__file__)) + '/package/'

            # --------------------
            # set dictionaries

            # Generate joints_dict. All joints are related to root.
            joints_dict, msg = Joint.make_joints_dict(root, msg)
            if msg != success_msg:
                ui.messageBox(msg, title)
                return 0
            joints_dict, tree_warnings = urdf_tree.make_urdf_tree(joints_dict)

            # Generate inertial_dict
            inertial_dict, msg = Link.make_inertial_dict(root, msg)
            if msg != success_msg:
                ui.messageBox(msg, title)
                return 0
            elif not 'base_link' in inertial_dict:
                msg = 'There is no base_link. Please set base_link and run again.'
                ui.messageBox(msg, title)
                return 0

            links_xyz_dict = {}

            # --------------------
            # Generate URDF
            Write.write_urdf_sim(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
            Write.write_materials_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
            Write.write_ros2control_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
            Write.write_gazebo_sim_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
            Write.write_display_launch(package_name, robot_name, save_dir)
            Write.write_gazebo_sim_launch(package_name, robot_name, save_dir)

            # copy over package files
            utils.create_package(package_name, save_dir, package_dir)
            utils.update_setup_py(save_dir, package_name)
            utils.update_setup_cfg(save_dir, package_name)
            utils.update_package_xml(save_dir, package_name)

            # Generate STl files
            utils.copy_occs(root)
            utils.export_stl(design, save_dir, components)

            success_msg = 'Successfully created URDF file and launch file for Gazebo Harmonic'
            if tree_warnings:
                success_msg += '\n\nURDF tree warnings:\n' + '\n'.join(tree_warnings)
            ui.messageBox(success_msg, title)

        else:
            #Gazebo Classic
            package_dir = os.path.abspath(os.path.dirname(__file__)) + '/package/'

            # --------------------
            # set dictionaries

            # Generate joints_dict. All joints are related to root.
            joints_dict, msg = Joint.make_joints_dict(root, msg)


            if msg != success_msg:
                ui.messageBox(msg, title)
                return 0
            joints_dict, tree_warnings = urdf_tree.make_urdf_tree(joints_dict)

            # Generate inertial_dict
            inertial_dict, msg = Link.make_inertial_dict(root, msg)


            if msg != success_msg:
                ui.messageBox(msg, title)
                return 0
            elif not 'base_link' in inertial_dict:
                msg = 'There is no base_link. Please set base_link and run again.'
                ui.messageBox(msg, title)
                return 0

            links_xyz_dict = {}

            # --------------------
            # Generate URDF
            Write.write_urdf(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)

            Write.write_materials_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
            Write.write_transmissions_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
            Write.write_gazebo_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
            Write.write_display_launch(package_name, robot_name, save_dir)
            Write.write_gazebo_launch(package_name, robot_name, save_dir)

            # copy over package files
            utils.create_package(package_name, save_dir, package_dir)
            utils.update_setup_py(save_dir, package_name)
            utils.update_setup_cfg(save_dir, package_name)
            utils.update_package_xml(save_dir, package_name)

            # Generate STl files
            utils.copy_occs(root)
            utils.export_stl(design, save_dir, components)
            success_msg = 'Successfully created URDF file and launch file for Gazebo Classic'
            if tree_warnings:
                success_msg += '\n\nURDF tree warnings:\n' + '\n'.join(tree_warnings)
            ui.messageBox(success_msg, title)



    except Exception as e:
        if ui:
            ui.messageBox(f'Failed:\n{str(e)}', title)
