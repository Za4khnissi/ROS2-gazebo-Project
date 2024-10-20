import { Injectable, OnModuleInit, HttpException, HttpStatus } from '@nestjs/common';
import * as ROSLIB from 'roslib';
import { ConfigService } from '@nestjs/config';

@Injectable()
export class RosService implements OnModuleInit {
  private realRos: ROSLIB.Ros;
  private simulationRos: ROSLIB.Ros;

  constructor(private configService: ConfigService) {}

  onModuleInit() {
    this.connectToRobots();
  }

  private connectToRobots() {
    const simulationWsUrl = this.configService.get<string>('ROS_WS_URL_SIMULATION');
    const realWsUrl = this.configService.get<string>('ROS_WS_URL_REAL');

    // Connect to simulation ROS
    if (simulationWsUrl) {
      this.simulationRos = new ROSLIB.Ros({
        url: simulationWsUrl,
      });

      this.simulationRos.on('connection', () => {
        console.log(`Connected to simulation ROS at ${simulationWsUrl}`);
      });

      this.simulationRos.on('error', (error) => {
        console.error(`Error connecting to simulation ROS:`, error);
      });

      this.simulationRos.on('close', () => {
        console.log(`Connection to simulation ROS closed`);
      });
    }

    // Connect to real robots ROS
    if (realWsUrl) {
      this.realRos = new ROSLIB.Ros({
        url: realWsUrl,
      });

      this.realRos.on('connection', () => {
        console.log(`Connected to real robots ROS at ${realWsUrl}`);
      });

      this.realRos.on('error', (error) => {
        console.error(`Error connecting to real robots ROS:`, error);
      });

      this.realRos.on('close', () => {
        console.log(`Connection to real robots ROS closed`);
      });
    }
  }

  private validateRobotConnection(robotId: string): ROSLIB.Ros {
    let rosConnection: ROSLIB.Ros;

    if (robotId === '3' || robotId === '4') {
      // Simulation robot
      rosConnection = this.simulationRos;
    } else if (robotId === '1' || robotId === '2') {
      // Real robot
      rosConnection = this.realRos;
    } else {
      throw new HttpException(`Invalid Robot ID ${robotId}`, HttpStatus.BAD_REQUEST);
    }

    if (!rosConnection || rosConnection.isConnected === false) {
      throw new HttpException(`Cannot connect to Robot ${robotId}`, HttpStatus.SERVICE_UNAVAILABLE);
    }

    return rosConnection;
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

  changeDriveMode(robotId: string, driveMode: string) {
    const rosConnection = this.validateRobotConnection(robotId);
  
    const namespace = `limo_105_${robotId}`;
    const serviceName = `/${namespace}/robot_${robotId}_drive_mode`;
  
    const driveModeService = new ROSLIB.Service({
      ros: rosConnection,
      name: serviceName,
      serviceType: 'std_srvs/SetBool', // Using SetBool service
    });
  
    const request = new ROSLIB.ServiceRequest({
      data: driveMode === 'ackermann',
    });
  
    driveModeService.callService(request, (result) => {
      if (result.success) {
        console.log(`Drive mode changed to ${driveMode} for robot ${robotId}`);
      } else {
        console.error(`Failed to change drive mode for robot ${robotId}`);
      }
    });
  
    return { message: `Requested to change drive mode for robot ${robotId} to ${driveMode}` };
  }
  
  
}
