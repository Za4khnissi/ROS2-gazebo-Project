import { Injectable, OnModuleInit, HttpException, HttpStatus } from '@nestjs/common';
import * as ROSLIB from 'roslib';
import { ConfigService } from '@nestjs/config';
import { exec, spawn, ChildProcess } from 'child_process';
import * as path from 'path';

@Injectable()
export class RosService implements OnModuleInit {
  private realRos: ROSLIB.Ros;
  private simulationRos: ROSLIB.Ros;
  private processes: ChildProcess[] = [];

  constructor(private configService: ConfigService) {}

  onModuleInit() {
    this.connectToRobots();
  }

  onModuleDestroy() {
    console.log('Shutting down server. Cleaning up launched processes...');
    this.processes.forEach((process) => {
      process.kill('SIGINT');
    });
    console.log('All launched processes have been terminated.');
  }


  private connectToRobots() {
    const simulationWsUrl = this.configService.get<string>('ROS_WS_URL_SIMULATION');
    const realWsUrl = this.configService.get<string>('ROS_WS_URL_REAL');

    if (simulationWsUrl) {
      this.simulationRos = new ROSLIB.Ros({ url: simulationWsUrl });
      this.setupRosConnection(this.simulationRos, 'simulation');
    }

    if (realWsUrl) {
      this.realRos = new ROSLIB.Ros({ url: realWsUrl });
      this.setupRosConnection(this.realRos, 'real');
    }
  }

  private setupRosConnection(ros: ROSLIB.Ros, type: string) {
    ros.on('connection', () => {
      console.log(`Connected to ${type} ROS.`);
    });

    ros.on('error', error => {
      console.error(`Error connecting to ${type} ROS:`, error);
    });

    ros.on('close', () => {
      console.log(`Connection to ${type} ROS closed.`);
    });
  }


  private validateRobotConnection(robotId: string): ROSLIB.Ros {
    let rosConnection: ROSLIB.Ros;

    if (robotId === '3' || robotId === '4') {
      rosConnection = this.simulationRos;
    } else if (robotId === '1' || robotId === '2') {
      rosConnection = this.realRos;
    } else {
      throw new HttpException(`Invalid Robot ID ${robotId}`, HttpStatus.BAD_REQUEST);
    }

    if (!rosConnection || rosConnection.isConnected === false) {
      throw new HttpException(`Cannot connect to Robot ${robotId}`, HttpStatus.SERVICE_UNAVAILABLE);
    }

    return rosConnection;
  }

  launchSimulation(driveMode3: string, driveMode4: string): Promise<any> {
    return new Promise((resolve, reject) => {
        const scriptPath = path.join(__dirname, '../../launch_robot.sh');
        const command = `${scriptPath} simulation ${driveMode3} ${driveMode4}`;
        
        const simulationProcess = exec(command, (error, stdout, stderr) => {
            if (error) {
                console.error(`Error launching simulation: ${error.message}`);
                reject(new HttpException('Failed to launch simulation', HttpStatus.INTERNAL_SERVER_ERROR));
                return;
            }
            if (stderr) {
                console.error(`stderr: ${stderr}`);
                reject(new HttpException('Simulation stderr: ' + stderr, HttpStatus.INTERNAL_SERVER_ERROR));
                return;
            }

            console.log(`stdout: ${stdout}`);
            setTimeout(() => {
                this.connectToRobots();
            }, 5000);
            resolve({ stdout });
        });

        this.processes.push(simulationProcess);
    });
}

  startRobotMission(robotId: string) {
    const rosConnection = this.validateRobotConnection(robotId);

    const namespace = `limo_105_${robotId}`;
    const serviceName = `/${namespace}/mission`;

    const missionService = new ROSLIB.Service({
      ros: rosConnection,
      name: serviceName,
      serviceType: 'example_interfaces/SetBool',
    });

    const request = new ROSLIB.ServiceRequest({
      data: true,
    });

    missionService.callService(request, (result) => {
      if (result.success) {
        console.log(`Mission started for robot ${robotId}: ${result.message}`);
      } else {
        console.error(`Failed to start mission for robot ${robotId}: ${result.message}`);
      }
    });

    return { message: `Requested to start mission for robot ${robotId}` };
  }

  stopRobotMission(robotId: string) {
    const rosConnection = this.validateRobotConnection(robotId);

    const namespace = `limo_105_${robotId}`;
    const serviceName = `/${namespace}/mission`;

    const missionService = new ROSLIB.Service({
      ros: rosConnection,
      name: serviceName,
      serviceType: 'example_interfaces/SetBool',
    });

    const request = new ROSLIB.ServiceRequest({
      data: false,
    });

    missionService.callService(request, (result) => {
      if (result.success) {
        console.log(`Mission stopped for robot ${robotId}: ${result.message}`);
      } else {
        console.error(`Failed to stop mission for robot ${robotId}: ${result.message}`);
      }
    });

    return { message: `Requested to stop mission for robot ${robotId}` };
  }

  identifyRobot(robotId: string) {
    const rosConnection = this.validateRobotConnection(robotId);

    const namespace = `limo_105_${robotId}`;
    const serviceName = `/${namespace}/identify`;

    const identifyService = new ROSLIB.Service({
      ros: rosConnection,
      name: serviceName,
      serviceType: 'std_srvs/Trigger',
    });

    const request = new ROSLIB.ServiceRequest({});

    identifyService.callService(request, (result) => {
      if (result.success) {
        console.log(`Robot ${robotId} successfully identified: ${result.message}`);
      } else {
        console.error(`Failed to identify Robot ${robotId}: ${result.message}`);
      }
    });

    return { message: `Robot ${robotId} is identifying itself` };
  }
}
