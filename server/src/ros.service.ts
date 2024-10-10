import {
  Injectable,
  OnModuleInit,
  OnModuleDestroy,
  HttpException,
  HttpStatus,
} from '@nestjs/common';
import * as rclnodejs from 'rclnodejs';
import { ConfigService } from '@nestjs/config';

@Injectable()
export class RosService implements OnModuleInit, OnModuleDestroy {
  private realRobotNode: rclnodejs.Node;
  private simulationRobotNode: rclnodejs.Node;

  constructor(private configService: ConfigService) {}

  async onModuleInit() {

    const rosDomainId = this.configService.get<string>('ROS_DOMAIN_ID') || '49';
    process.env.ROS_DOMAIN_ID = rosDomainId;

    console.log(`ROS_DOMAIN_ID is set to ${process.env.ROS_DOMAIN_ID}`);

    // Initialize rclnodejs
    await rclnodejs.init();

    // Create nodes for real and simulation robots
    this.connectToRobots();
  }

  private connectToRobots() {

    this.simulationRobotNode = new rclnodejs.Node('simulation_robot_node');
    this.realRobotNode = new rclnodejs.Node('real_robot_node');

    // Spin the nodes
    this.simulationRobotNode.spin();
    this.realRobotNode.spin();

    console.log('ROS 2 nodes initialized for simulation and real robots.');
  }

  private validateRobotConnection(robotId: string): rclnodejs.Node {
    if (robotId === '3' || robotId === '4') {
      return this.simulationRobotNode;
    } else if (robotId === '1' || robotId === '2') {
      return this.realRobotNode;
    } else {
      throw new HttpException(`Invalid Robot ID ${robotId}`, HttpStatus.BAD_REQUEST);
    }
  }

  async startRobotMission(robotId: string) {
    const node = this.validateRobotConnection(robotId);
    const namespace = `limo_105_${robotId}`;
    const serviceName = `/${namespace}/mission`;

    // Create a client for the SetBool service
    const client = node.createClient('example_interfaces/srv/SetBool', serviceName);

    // Prepare the request
    const RequestType = rclnodejs.require('example_interfaces/srv/SetBool').Request;
    const request = new RequestType();
    request.data = true;

    // Wait for the service to become available
    const serviceAvailable = await client.waitForService(5000);
    if (!serviceAvailable) {
      throw new HttpException(`Service ${serviceName} not available`, HttpStatus.SERVICE_UNAVAILABLE);
    }

    // Call the service
    return new Promise((resolve, reject) => {
      client.sendRequest(request, (response) => {
        if (response.success) {
          console.log(`Mission started for robot ${robotId}: ${response.message}`);
          resolve({ message: `Requested to start mission for robot ${robotId}` });
        } else {
          console.error(`Failed to start mission for robot ${robotId}: ${response.message}`);
          reject(new HttpException(response.message, HttpStatus.INTERNAL_SERVER_ERROR));
        }
      });
    });
  }

  async stopRobotMission(robotId: string) {
    const node = this.validateRobotConnection(robotId);
    const namespace = `limo_105_${robotId}`;
    const serviceName = `/${namespace}/mission`;

    // Create a client for the SetBool service
    const client = node.createClient('example_interfaces/srv/SetBool', serviceName);

    // Prepare the request
    const RequestType = rclnodejs.require('example_interfaces/srv/SetBool').Request;
    const request = new RequestType();
    request.data = false;

    // Wait for the service to become available
    const serviceAvailable = await client.waitForService(5000);
    if (!serviceAvailable) {
      throw new HttpException(`Service ${serviceName} not available`, HttpStatus.SERVICE_UNAVAILABLE);
    }

    // Call the service
    return new Promise((resolve, reject) => {
      client.sendRequest(request, (response) => {
        if (response.success) {
          console.log(`Mission stopped for robot ${robotId}: ${response.message}`);
          resolve({ message: `Requested to stop mission for robot ${robotId}` });
        } else {
          console.error(`Failed to stop mission for robot ${robotId}: ${response.message}`);
          reject(new HttpException(response.message, HttpStatus.INTERNAL_SERVER_ERROR));
        }
      });
    });
  }

  async identifyRobot(robotId: string) {
    const node = this.validateRobotConnection(robotId);
    const namespace = `limo_105_${robotId}`;
    const serviceName = `/${namespace}/identify`;

    // Create a client for the Trigger service
    const client = node.createClient('std_srvs/srv/Trigger', serviceName);

    // Prepare the request
    const RequestType = rclnodejs.require('std_srvs/srv/Trigger').Request;
    const request = new RequestType();

    // Wait for the service to become available
    const serviceAvailable = await client.waitForService(5000);
    if (!serviceAvailable) {
      throw new HttpException(`Service ${serviceName} not available`, HttpStatus.SERVICE_UNAVAILABLE);
    }

    // Call the service
    return new Promise((resolve, reject) => {
      client.sendRequest(request, (response) => {
        if (response.success) {
          console.log(`Robot ${robotId} successfully identified: ${response.message}`);
          resolve({ message: `Robot ${robotId} is identifying itself` });
        } else {
          console.error(`Failed to identify Robot ${robotId}: ${response.message}`);
          reject(new HttpException(response.message, HttpStatus.INTERNAL_SERVER_ERROR));
        }
      });
    });
  }

  async onModuleDestroy() {
    // Destroy nodes and shutdown rclnodejs
    this.simulationRobotNode.destroy();
    this.realRobotNode.destroy();
    await rclnodejs.shutdown();
  }
}
