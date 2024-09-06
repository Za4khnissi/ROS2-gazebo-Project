import { Injectable } from '@nestjs/common';
import { exec } from 'child_process';

@Injectable()
export class AppService {
  // Update with the actual path to your ROS workspace
  private readonly projectWsPath = '/home/zak/inf3995/project_ws';

  startSimulationMission(id: number): Promise<string> {
    return new Promise((resolve, reject) => {
      exec(`ros2 topic pub /movement_command std_msgs/String "data: 'start'" --once`, (error, stdout, stderr) => {
        if (error) {
          console.error(`Error starting mission: ${error}`);
          reject(`Error starting mission: ${stderr}`);
        }
        resolve(`Simulation Robot ${id} mission started.`);
      });
    });
  }

  stopSimulationMission(id: number): Promise<string> {
    return new Promise((resolve, reject) => {
      exec(`ros2 topic pub /movement_command std_msgs/String "data: 'stop'" --once`, (error, stdout, stderr) => {
        if (error) {
          console.error(`Error stopping mission: ${error}`);
          reject(`Error stopping mission: ${stderr}`);
        }
        resolve(`Simulation Robot ${id} mission stopped.`);
      });
    });
  }

  launchSimulation(): Promise<string> {
    return new Promise((resolve, reject) => {
      const command = `bash -c "source /opt/ros/humble/setup.bash && cd ${this.projectWsPath} && source install/setup.sh && colcon build && ros2 launch ros_gz_example_bringup diff_drive.launch.py"`;
      
      exec(command, (error, stdout, stderr) => {
        if (error) {
          console.error(`Error launching Gazebo simulation: ${error}`);
          reject(`Error launching Gazebo simulation: ${stderr}`);
        }
        resolve('Gazebo simulation launched successfully.');
      });
    });
  }
}
