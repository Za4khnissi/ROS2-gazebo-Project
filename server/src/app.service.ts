import { Injectable } from '@nestjs/common';
import { exec } from 'child_process';

@Injectable()
export class AppService {
  private readonly projectWsPath = '/home/zak/inf3995/project_ws';

  identifyRobot(robotId: number): Promise<string> {
    return new Promise((resolve, reject) => {
      const identifyCommand = `ros2 topic pub /movement_command std_msgs/String "data: 'identify'" --once`;

      exec(identifyCommand, (error, stdout, stderr) => {
        if (error) {
          console.error(`Error identifying robot ${robotId}: ${error}`);
          reject(`Error identifying robot ${robotId}: ${stderr}`);
        } else {
          resolve(`Robot ${robotId} is identifying.`);
        }
      });
    });
  }

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
          console.error(`Error launching Gazebo simulation: ${error.message}`);
          console.error(`stderr: ${stderr}`);
          reject(`Error launching Gazebo simulation: ${error.message}`);
        } else {
          console.log(`stdout: ${stdout}`);
          resolve('Gazebo simulation launched successfully.');
        }
      });
    });
  }

  stopSimulation(): Promise<string> {
    return new Promise((resolve, reject) => {
      const command = `pkill -f 'ros2 launch ros_gz_example_bringup diff_drive.launch.py'`;
  
      exec(command, (error, stdout, stderr) => {
        if (error) {
          console.error(`Error stopping Gazebo simulation: ${error.message}`);
          console.error(`stderr: ${stderr}`);
          reject(`Error stopping Gazebo simulation: ${error.message}`);
        } else {
          console.log(`Gazebo simulation stopped. stdout: ${stdout}`);
          resolve('Gazebo simulation stopped successfully.');
        }
      });
    });
  }
}
