import { Injectable, OnModuleInit, HttpException, HttpStatus } from '@nestjs/common';
import * as ROSLIB from 'roslib';
import { ConfigService } from '@nestjs/config';

@Injectable()
export class RosService implements OnModuleInit {
  private rosConnections: Record<string, ROSLIB.Ros> = {};

  constructor(private configService: ConfigService) {}

  onModuleInit() {
    //this.connectToRobot('0', this.configService.get<string>('ROS_WS_URL_SIMULATION'));
    this.connectToRobot('1', this.configService.get<string>('ROS_WS_URL_ROBOT1'));
    // this.connectToRobot('2', this.configService.get<string>('ROS_WS_URL_ROBOT2'));
  }

  private connectToRobot(robotId: string, wsUrl: string) {
    const ros = new ROSLIB.Ros({
      url: wsUrl,
    });

    ros.on('connection', () => {
      console.log(`Connected to Robot ${robotId} at WebSocket URL: ${wsUrl}`);
    });

    ros.on('error', (error) => {
      console.error(`Error connecting to Robot ${robotId}:`, error);
    });

    ros.on('close', () => {
      console.log(`Connection to Robot ${robotId} closed`);
    });


    this.rosConnections[robotId] = ros;
  }

  private validateRobotConnection(robotId: string): void {
    const rosConnection = this.rosConnections[robotId];
    if (!rosConnection || rosConnection.isConnected === false) {
      throw new HttpException(`Cannot connect to Robot ${robotId}`, HttpStatus.SERVICE_UNAVAILABLE);
    }
  }

  startRobotMission(robotId: string) {
    this.validateRobotConnection(robotId);

    const missionService = new ROSLIB.Service({
      ros: this.rosConnections[robotId],
      name: '/mission_service',
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
    this.validateRobotConnection(robotId);

    const missionService = new ROSLIB.Service({
      ros: this.rosConnections[robotId],
      name: '/mission_service',
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
    this.validateRobotConnection(robotId);
  
    const identifyService = new ROSLIB.Service({
      ros: this.rosConnections[robotId],
      name: `/identify`,  
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
