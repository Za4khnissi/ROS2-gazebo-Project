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

def modify_world_with_obstacles(original_world_file, modified_world_file):
    # Read the original world file content
    with open(original_world_file, 'r') as f:
        sdf_content = f.read()

    # Look for the closing </world> tag
    closing_world_tag_index = sdf_content.rfind('</world>')

    if closing_world_tag_index == -1:
        raise ValueError("Invalid SDF: </world> tag not found")

    # Extract the content up to the closing </world> tag
    sdf_without_closing_tag = sdf_content[:closing_world_tag_index]

    # Generate and append obstacles
    for _ in range(10):
        x, y, z = generate_random_position()
        sdf_without_closing_tag += generate_cylinder_sdf(x, y, z)

    for _ in range(20):
        x, y, z = generate_random_position()
        sdf_without_closing_tag += generate_cube_sdf(x, y, z)

    # Append the closing </world> and </sdf> tags with correct indentation
    sdf_without_closing_tag += "\n  </world>\n</sdf>\n"


    # Write the modified content to the new world file
    with open(modified_world_file, 'w') as f:
        f.write(sdf_without_closing_tag)

    print(f"Modified world file with obstacles at {modified_world_file}")

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: generate_world_with_obstacles.py <drive_mode_3> <drive_mode_4> <modified_world_file>")
        sys.exit(1)

    drive_mode_3 = sys.argv[1]
    drive_mode_4 = sys.argv[2]
    modified_world_file = sys.argv[3]

    # Logic to select the world file based on drive modes
    if drive_mode_3 == 'ackermann' and drive_mode_4 == 'diff_drive':
        world_file_name = 'ackermann_diff.sdf'
    elif drive_mode_3 == 'ackermann' and drive_mode_4 == 'ackermann':
        world_file_name = 'ackermann.sdf'
    elif drive_mode_3 == 'diff_drive' and drive_mode_4 == 'ackermann':
        world_file_name = 'diff_ackermann.sdf'
    else:
        world_file_name = 'diff_drive.sdf'

    # Adjust the path to your package directory
    pkg_project_gazebo = os.path.join(
        os.path.expanduser('~'),
        'inf3995',
        'project_ws',
        'src',
        'ros_gz_example_gazebo'
    )
    original_world_file = os.path.join(pkg_project_gazebo, 'worlds', world_file_name)

    modify_world_with_obstacles(original_world_file, modified_world_file)
