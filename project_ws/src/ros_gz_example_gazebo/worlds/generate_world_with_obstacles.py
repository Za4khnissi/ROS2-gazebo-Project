#!/usr/bin/env python3

import os
import random
import sys

def generate_random_position():
    x = random.uniform(-10, 10)
    y = random.uniform(-10, 10)
    z = 0.0
    return x, y, z

def generate_cylinder_sdf(x, y, z):
    return f"""
    <model name="random_cylinder_{x}_{y}">
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>1</radius>
              <length>2</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>1</radius>
              <length>2</length>
            </cylinder>
          </geometry>
        </visual>
      </link>
    </model>
    """

def generate_cube_sdf(x, y, z):
    return f"""
    <model name="random_cube_{x}_{y}">
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
    """

def generate_robot_model_sdf(robot_name, robot_mode, position):
    return f"""
    <model name="{robot_name}">
      <pose>{position[0]} {position[1]} 0.35 0 0 0</pose>
      <include merge="true">
        <uri>package://ros_gz_example_description/models/{robot_name}_{robot_mode}</uri>
      </include>
      <ros>
        <package>ros_gz_example_gazebo</package>
        <output>screen</output>
      </ros>
    </model>
    """

def generate_world_with_obstacles_and_robots(drive_mode_3, drive_mode_4, modified_world_file):
    # Create the initial part of the SDF world, with sun and ground
    sdf_content = """<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="demo">
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
    """

    # Add robot models based on drive modes
    sdf_content += generate_robot_model_sdf("limo_105_3", drive_mode_3, (0, -0.5))
    sdf_content += generate_robot_model_sdf("limo_105_4", drive_mode_4, (0, 0.5))

    # Add random obstacles (10 cylinders and 20 cubes)
    for _ in range(10):
        x, y, z = generate_random_position()
        sdf_content += generate_cylinder_sdf(x, y, z)

    for _ in range(20):
        x, y, z = generate_random_position()
        sdf_content += generate_cube_sdf(x, y, z)

    # Append the closing </world> and </sdf> tags
    sdf_content += "\n  </world>\n</sdf>\n"

    # Write the generated content to the new world file
    with open(modified_world_file, 'w') as f:
        f.write(sdf_content)

    print(f"Modified world file with robots and obstacles at {modified_world_file}")

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: generate_world_with_obstacles.py <drive_mode_3> <drive_mode_4> <modified_world_file>")
        sys.exit(1)

    drive_mode_3 = sys.argv[1]
    drive_mode_4 = sys.argv[2]
    modified_world_file = sys.argv[3]

    generate_world_with_obstacles_and_robots(drive_mode_3, drive_mode_4, modified_world_file)
