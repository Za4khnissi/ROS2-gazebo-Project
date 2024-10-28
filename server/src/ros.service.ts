import {
  Injectable,
  OnModuleInit,
  OnModuleDestroy,
  HttpException,
  HttpStatus,
} from '@nestjs/common';
import * as rclnodejs from 'rclnodejs';
import { ConfigService } from '@nestjs/config';
import * as fs from 'fs';
import { Subject } from 'rxjs';
import { SyncGateway } from './sync/sync.gateway';

interface ServiceResponse {
  success: boolean;
  message?: string;
}

@Injectable()
export class RosService implements OnModuleInit, OnModuleDestroy {
  private realRobotNode: rclnodejs.Node;
  private simulationRobotNode: rclnodejs.Node;
  private logs: any[] = [];
  private logFile = this.configService.get<string>('PATH_TO_LOGS');
  private logSubject = new Subject<any>();

  constructor(
    private configService: ConfigService, 
    private syncGateway: SyncGateway
  ) {
    this.loadLogs();
  }

  async onModuleInit() {

    const rosDomainId = this.configService.get<string>('ROS_DOMAIN_ID') || '49';
    process.env.ROS_DOMAIN_ID = rosDomainId;

    console.log(`ROS_DOMAIN_ID is set to ${process.env.ROS_DOMAIN_ID}`);

    // Initialize rclnodejs
    await rclnodejs.init();

    // Create nodes for real and simulation robots
    this.connectToRobots();
  }

  private loadLogs() {
    if (fs.existsSync(this.logFile)) {
      this.logs = JSON.parse(fs.readFileSync(this.logFile, 'utf8'));
    }
  }

  private saveLogs(log: { event: string; robot: string; message?: string; timestamp: string }) {
    this.logs.push(log);
    fs.writeFileSync(this.logFile, JSON.stringify(this.logs, null, 2));
    this.logSubject.next(log);
}

  getOldLogs() {
    return this.logs;
  }

  getLogStream() {
    return this.logSubject.asObservable();
  }

  private connectToRobots() {
    this.simulationRobotNode = new rclnodejs.Node('simulation_robot_node');
    this.realRobotNode = new rclnodejs.Node('real_robot_node');
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
  
  async startRobotMission(robotId: string): Promise<{ message: string; success: boolean }> {
    const node = this.validateRobotConnection(robotId);
    const namespace = `limo_105_${robotId}`;
    const serviceName = `/${namespace}/mission`;
  
    const client = node.createClient('example_interfaces/srv/SetBool', serviceName);
    const RequestType = rclnodejs.require('example_interfaces/srv/SetBool').Request;
    const request = new RequestType();
    request.data = true;
  
    const serviceAvailable = await client.waitForService(5000);
    if (!serviceAvailable) {
      throw new HttpException(`Service ${serviceName} not available`, HttpStatus.SERVICE_UNAVAILABLE);
    }
  
    const log = { event: 'start_mission', robot: robotId, timestamp: new Date().toISOString() };
    this.saveLogs(log);
  
    return new Promise((resolve, reject) => {
      client.sendRequest(request, (response: ServiceResponse) => {
        if (response.success) {
          console.log(`Mission started for robot ${robotId}: ${response.message}`);
          this.syncGateway.broadcast('syncUpdate', { event: 'mission_started', robot: robotId });
          resolve({ message: `Mission started for robot ${robotId}`, success: true });
        } else {
          console.error(`Failed to start mission for robot ${robotId}: ${response.message}`);
          this.syncGateway.broadcast('syncUpdate', { event: 'mission_failed', robot: robotId });
          reject(new HttpException(response.message, HttpStatus.INTERNAL_SERVER_ERROR));
        }
      });
    });
  }

  async stopRobotMission(robotId: string): Promise<{ message: string; success: boolean }> {
    const node = this.validateRobotConnection(robotId);
    const namespace = `limo_105_${robotId}`;
    const serviceName = `/${namespace}/mission`;
  
    const client = node.createClient('example_interfaces/srv/SetBool', serviceName);
    const RequestType = rclnodejs.require('example_interfaces/srv/SetBool').Request;
    const request = new RequestType();
    request.data = false;
  
    const serviceAvailable = await client.waitForService(5000);
    if (!serviceAvailable) {
      throw new HttpException(`Service ${serviceName} not available`, HttpStatus.SERVICE_UNAVAILABLE);
    }
  
    const log = { event: 'stop_mission', robot: robotId, timestamp: new Date().toISOString() };
    this.saveLogs(log);
  
    return new Promise((resolve, reject) => {
      client.sendRequest(request, (response: ServiceResponse) => {
        if (response.success) {
          console.log(`Mission stopped for robot ${robotId}: ${response.message}`);
          this.syncGateway.broadcast('syncUpdate', { event: 'mission_stopped', robot: robotId });
          resolve({ message: `Mission stopped for robot ${robotId}`, success: true });
        } else {
          console.error(`Failed to stop mission for robot ${robotId}: ${response.message}`);
          this.syncGateway.broadcast('syncUpdate', { event: 'mission_stop_failed', robot: robotId });
          reject(new HttpException(response.message, HttpStatus.INTERNAL_SERVER_ERROR));
        }
      });
    });
  }

  async identifyRobot(robotId: string): Promise<{ message: string; success: boolean }> {
    const node = this.validateRobotConnection(robotId);
    const namespace = `limo_105_${robotId}`;
    const serviceName = `/${namespace}/identify`;
  
    const client = node.createClient('std_srvs/srv/Trigger', serviceName);
    const RequestType = rclnodejs.require('std_srvs/srv/Trigger').Request;
    const request = new RequestType();
  
    const serviceAvailable = await client.waitForService(5000);
    if (!serviceAvailable) {
      throw new HttpException(`Service ${serviceName} not available`, HttpStatus.SERVICE_UNAVAILABLE);
    }
  
    return new Promise((resolve, reject) => {
      client.sendRequest(request, (response: ServiceResponse) => {
        if (response.success) {
          console.log(`Robot ${robotId} identified successfully: ${response.message}`);
          this.syncGateway.broadcast('syncUpdate', { event: 'identified', robot: robotId });
          resolve({ message: `Robot ${robotId} identified`, success: true });
        } else {
          console.error(`Failed to identify Robot ${robotId}: ${response.message}`);
          this.syncGateway.broadcast('syncUpdate', { event: 'identification_failed', robot: robotId });
          reject(new HttpException(response.message, HttpStatus.INTERNAL_SERVER_ERROR));
        }
      });
    });
  }  
  
  changeDriveMode(robotId: string, driveMode: string) {
    const node = this.validateRobotConnection(robotId);
    const namespace = `limo_105_${robotId}`;
    const serviceName = `/${namespace}/robot_${robotId}_drive_mode`;
    
    const client = node.createClient('std_srvs/srv/SetBool', serviceName);
    const RequestType = rclnodejs.require('std_srvs/srv/SetBool').Request;
    const request = new RequestType();
    request.data = driveMode === 'ackermann';
    
    client.sendRequest(request, (response) => {
      const event = response.success ? 'drive_mode_changed' : 'drive_mode_change_failed';
      const log = { event, robot: robotId, driveMode, timestamp: new Date().toISOString() };
      this.saveLogs(log);
      this.syncGateway.broadcast('syncUpdate', log);
    });
  }
  
  async onModuleDestroy() {
    // Destroy nodes and shutdown rclnodejs
    this.simulationRobotNode.destroy();
    this.realRobotNode.destroy();
    await rclnodejs.shutdown();
  }
}
