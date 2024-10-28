#!/usr/bin/env python3

from cmath import cos, sin
import os
import random
import sys

# Function to generate random positions for internal walls
def generate_random_position():
    x = random.uniform(-9, 9) 
    y = random.uniform(-9, 9)
    z = 0.0
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

# Function to check if two walls overlap
def walls_overlap(x1, y1, length1, angle1, x2, y2, length2, angle2, clearance=0.5):
    # Calculate wall endpoints assuming each wall is aligned with its angle
    def wall_endpoints(x, y, length, angle):
        x_end = x + length * 0.5 * cos(angle)
        y_end = y + length * 0.5 * sin(angle)
        x_start = x - length * 0.5 * cos(angle)
        y_start = y - length * 0.5 * sin(angle)
        return (x_start, y_start), (x_end, y_end)

    (x1_start, y1_start), (x1_end, y1_end) = wall_endpoints(x1, y1, length1, angle1)
    (x2_start, y2_start), (x2_end, y2_end) = wall_endpoints(x2, y2, length2, angle2)

    # Check if walls intersect or are within a certain distance (clearance)
    return max(abs(x1 - x2), abs(y1 - y2)) < (length1 + length2) * 0.5 + clearance


# Function to generate the random internal walls
def generate_internal_walls():
    walls = ""
    placed_walls = []  # To track placed walls' positions, lengths, and angles
    for _ in range(10):
        while True:
            x, y, z = generate_random_position()
            angle = generate_random_angle()
            length = random.uniform(5, 10)

            # Check for collision with already placed walls
            collision = False
            for (px, py, pz, plength, pangle) in placed_walls:
                if walls_overlap(x, y, length, angle, px, py, plength, pangle):
                    collision = True
                    break

            # If no collision, add wall and break the loop
            if not collision:
                walls += generate_wall_sdf(x, y, z, length, angle)
                placed_walls.append((x, y, z, length, angle))
                break  # Exit while loop once a non-colliding position is found
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
    sdf_content += generate_internal_walls()

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
