#!/usr/bin/env python3

import os
import random
import sys

# Function to check if a position is too close to a robot
def is_position_safe(x, y, robots, min_distance=2.0):
    for robot in robots:
        rx, ry = robot
        distance = ((x - rx) ** 2 + (y - ry) ** 2) ** 0.5
        if distance < min_distance:
            return False
    return True

# Function to generate random positions for internal walls ensuring no collisions with robots
def generate_random_position(robots):
    while True:
        x = random.uniform(-8, 8)
        y = random.uniform(-8, 8)
        z = 0.0
        if is_position_safe(x, y, robots):
            return x, y, z

# Function to generate random angles for internal walls
def generate_random_angle():
    return random.uniform(0, 3.14159)

# Function to generate a wall SDF
def generate_wall_sdf(x, y, z, length, angle):
    return f"""
    <model name="random_wall_{x}_{y}">
      <pose>{x} {y} {z} 0 0 {angle}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{length} 0.5 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{length} 0.5 1</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
    """

# Function to generate the four border walls
def generate_border_walls():
    walls = ""
    # Top wall
    walls += generate_wall_sdf(0, 10, 0.5, 20, 0)
    # Bottom wall
    walls += generate_wall_sdf(0, -10, 0.5, 20, 0)
    # Left wall
    walls += generate_wall_sdf(-10, 0, 0.5, 20, 1.5708)  # 1.5708 rad = 90 degrees
    # Right wall
    walls += generate_wall_sdf(10, 0, 0.5, 20, 1.5708)
    return walls


# Function to generate the random internal walls
def generate_internal_walls(robots):
    walls = ""
    for _ in range(6):  # Generate 6 walls
        x, y, z = generate_random_position(robots)
        angle = generate_random_angle()
        length = random.uniform(5, 7)  # Length of the wall between 5 and 7 units
        walls += generate_wall_sdf(x, y, z, length, angle)
    return walls

# Function to generate the robot models based on drive modes
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

# Function to generate the full SDF file content
def generate_world_with_obstacles(drive_mode_3, drive_mode_4, modified_world_file):
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

    # Add the border walls
    sdf_content += generate_border_walls()

    # Add the internal random walls
    sdf_content += generate_internal_walls(robots=[(0, -0.5), (0, 0.5)])

    # Append the closing </world> and </sdf> tags
    sdf_content += "\n  </world>\n</sdf>\n"

    # Write the generated content to the new world file
    with open(modified_world_file, 'w') as f:
        f.write(sdf_content)

    print(f"Modified world file with robots, borders, and random walls at {modified_world_file}")

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: generate_world_with_obstacles.py <drive_mode_3> <drive_mode_4> <modified_world_file>")
        sys.exit(1)

    drive_mode_3 = sys.argv[1]
    drive_mode_4 = sys.argv[2]
    modified_world_file = sys.argv[3]

    generate_world_with_obstacles(drive_mode_3, drive_mode_4, modified_world_file)
