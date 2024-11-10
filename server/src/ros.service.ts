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
import { Subject, interval, Subscription, Observable } from 'rxjs';
import { SyncGateway } from './sync/sync.gateway';
import * as path from 'path';

interface ServiceResponse {
  success: boolean;
  message?: string;
}

const BATTERY_THRESHOLD = parseInt(process.env.BATTERY_THRESHOLD || '30');

@Injectable()
export class RosService implements OnModuleInit, OnModuleDestroy {
  private realRobotNode: rclnodejs.Node;
  private simulationRobotNode: rclnodejs.Node;
  private logsFolder = this.configService.get<string>('PATH_TO_LOGS_FOLDER');
  private logSubject = new Subject<any>();
  private missionActive = false;
  private logInterval: Subscription;
  private currentLogs: any[] = [];
  private logFilePaths = new Map<string, string>();
  private lastRobotStatus = {
    '3': 'Waiting',
    '4': 'Waiting',
  };

  constructor(
    private configService: ConfigService, 
    private syncGateway: SyncGateway
  ) {
    this.ensureLogsFolderExists();
  }

  async onModuleInit() {
    const rosDomainId = this.configService.get<string>('ROS_DOMAIN_ID') || '49';
    process.env.ROS_DOMAIN_ID = rosDomainId;

    console.log(`ROS_DOMAIN_ID is set to ${process.env.ROS_DOMAIN_ID}`);

    await rclnodejs.init();

    this.connectToRobots();

    const rosoutNode = new rclnodejs.Node('log_listener');
    rosoutNode.createSubscription(
      'rcl_interfaces/msg/Log',
      '/rosout',
      (msg) => this.handleLogMessage(msg)
    );
    rosoutNode.spin();
  }

  private ensureLogsFolderExists() {
    if (!fs.existsSync(this.logsFolder)) {
      fs.mkdirSync(this.logsFolder);
    }
  }

  private formatTimestamp(date: Date): string {
    return date.toLocaleString('en-CA', {
      year: 'numeric',
      month: '2-digit',
      day: '2-digit',
      hour: '2-digit',
      minute: '2-digit',
      hour12: false,
    });
  }

  getLastStatus() {
    return this.lastRobotStatus;
  }

  private startMissionLogFile(robotId: string) {
    const timestamp = this.formatTimestamp(new Date()).replace(/[: ]/g, '-');
    const logFilePath = path.join(this.logsFolder, `mission_${robotId}_${timestamp}.json`);
    this.logFilePaths.set(robotId, logFilePath);
    console.log(`Logging to ${logFilePath} for robot ${robotId}`);
  }

  private saveLogToFile(log: any) {
    const logFilePath = this.logFilePaths.get(log.robot.split('_')[2]);
    if (logFilePath) {
      log.timestamp = this.formatTimestamp(new Date(log.timestamp));
      fs.appendFileSync(logFilePath, JSON.stringify(log) + '\n');
      this.syncGateway.broadcast('syncUpdate', log);
    }
  }

  private handleLogMessage(msg: any) {
    const namespace = msg.name.split('.')[0];
    if (namespace === 'limo_105_3' || namespace === 'limo_105_4') {
      const logEntry = {
        robot: namespace,
        level: msg.level,
        message: msg.msg,
        timestamp: this.formatTimestamp(new Date()),
      };
      this.currentLogs.push(logEntry);
    }
  }


  private startLogging() {
    this.logInterval = interval(1000).subscribe(() => {
      if (this.missionActive && this.currentLogs.length > 0) {
        this.currentLogs.forEach((log) => {
          this.saveLogToFile(log);
          this.syncGateway.broadcast('syncUpdate', log); 
        });
        this.currentLogs = []; 
      }
    });
  }


  private stopLogging() {
    if (this.logInterval) {
      this.logInterval.unsubscribe();
    }
    this.logFilePaths.clear();
  }

  private connectToRobots() {
    this.simulationRobotNode = new rclnodejs.Node('simulation_robot_node');
    this.realRobotNode = new rclnodejs.Node('real_robot_node');

    this.subscribeToBatteryLevel('limo_105_1', this.realRobotNode);
    this.subscribeToBatteryLevel('limo_105_2', this.realRobotNode);
    this.subscribeToBatteryLevel('limo_105_3', this.simulationRobotNode);
    this.subscribeToBatteryLevel('limo_105_4', this.simulationRobotNode);

    

    const OccupancyGrid = rclnodejs.require('nav_msgs/msg/OccupancyGrid');

    // Add map subscription
    this.simulationRobotNode.createSubscription(
      OccupancyGrid as any,
      '/map',
      (message: any) => {
        // Convert the message to a plain JavaScript object
        const mapData = {
          header: {
            seq: message.header.seq,
            stamp: {
              sec: message.header.stamp.sec,
              nsec: message.header.stamp.nsec
            },
            frame_id: message.header.frame_id
          },
          info: {
            map_load_time: {
              sec: message.info.map_load_time.sec,
              nsec: message.info.map_load_time.nsec
            },
            resolution: message.info.resolution,
            width: message.info.width,
            height: message.info.height,
            origin: {
              position: {
                x: message.info.origin.position.x,
                y: message.info.origin.position.y,
                z: message.info.origin.position.z
              },
              orientation: {
                x: message.info.origin.orientation.x,
                y: message.info.origin.orientation.y,
                z: message.info.origin.orientation.z,
                w: message.info.origin.orientation.w
              }
            }
          },
          data: Array.from(message.data) // Convert Int8Array to regular array
        };

        // Broadcast the map data through the WebSocket
        this.syncGateway.broadcast('map_update', mapData);
      }
    );


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

  private subscribeToBatteryLevel(robotId: string, node: rclnodejs.Node) {
    const topicName = `/${robotId}/battery_level`;
    console.log(`Attempting to subscribe to ${topicName}`);

    node.createSubscription(
      'std_msgs/msg/Float32',
      topicName,
      (msg) => {
        console.log(`Message received on ${topicName}`);
        if (this.isFloat32(msg)) {
          const batteryLevel = msg.data;
          console.log(`Battery Level for ${robotId}: ${batteryLevel}%`);
          if (batteryLevel < BATTERY_THRESHOLD) {
            console.log(`Battery level for ${robotId} is low. Stopping Mission.`);
            this.stopRobotMission(robotId, true);
          }
          this.syncGateway.broadcastBatteryUpdate(robotId, batteryLevel);
        } else {
          console.error(`Received message for ${robotId} is not of type Float32`);
        }
      }
    );
    console.log(`Subscribed to battery level topic for ${robotId}`);
  }

  private isFloat32(msg: any): msg is { data: number } {
    return typeof msg.data === 'number';
  }
  
  async startRobotMission(robotId: string): Promise<{ message: string; success: boolean }> {
    this.missionActive = true;
    this.startMissionLogFile(robotId);
    this.startLogging();

    const node = this.validateRobotConnection(robotId);
    const namespace = `limo_105_${robotId}`;
    const serviceName = `/${namespace}/mission`;

    const client = node.createClient(
      'ros_gz_example_application/srv/MissionCommand',
      serviceName
    );

    const RequestType = rclnodejs.require('ros_gz_example_application/srv/MissionCommand').Request;
    const request = new RequestType();

    request.command = 1;  // Start mission

    const serviceAvailable = await client.waitForService(5000);
    if (!serviceAvailable) {
      throw new HttpException(`Service ${serviceName} not available`, HttpStatus.SERVICE_UNAVAILABLE);
    }

    this.lastRobotStatus[robotId] = 'Moving';
    this.syncGateway.broadcast('syncUpdate', { event: 'mission_started', robot: robotId });

    const log = { event: 'start_mission', robot: robotId, timestamp: this.formatTimestamp(new Date()) };
    this.saveLogToFile(log);

    return new Promise((resolve, reject) => {
      client.sendRequest(request, (response: ServiceResponse) => {
        if (response.success) {
          this.syncGateway.broadcast('syncUpdate', { event: 'mission_started', robot: robotId });
          resolve({ message: `Mission started for robot ${robotId}`, success: true });
        } else {
          this.syncGateway.broadcast('syncUpdate', { event: 'mission_failed', robot: robotId });
          reject(new HttpException(response.message, HttpStatus.INTERNAL_SERVER_ERROR));
        }
      });
    });
  }

  async stopRobotMission(robotId: string, shouldReturn: boolean = false): Promise<{ message: string; success: boolean }> {
    this.missionActive = false;
    this.stopLogging();

    const node = this.validateRobotConnection(robotId);
    const namespace = `limo_105_${robotId}`;
    const serviceName = `/${namespace}/mission`;

    const client = node.createClient(
      'ros_gz_example_application/srv/MissionCommand',
      serviceName
    );

    const RequestType = rclnodejs.require('ros_gz_example_application/srv/MissionCommand').Request;
    const request = new RequestType();
    request.command = shouldReturn ? 3 : 2;  // 3 for return, 2 for stop

    const serviceAvailable = await client.waitForService(5000);
    if (!serviceAvailable) {
      throw new HttpException(`Service ${serviceName} not available`, HttpStatus.SERVICE_UNAVAILABLE);
    }

    const status = shouldReturn ? 'Returning' : 'Stopped';
    const event = shouldReturn ? 'robot_returning' : 'mission_stopped';
    this.lastRobotStatus[robotId] = status;
    this.syncGateway.broadcast('syncUpdate', { event, robot: robotId });

    const log = { 
      event: shouldReturn ? 'stop_and_return' : 'stop_mission', 
      robot: robotId, 
      timestamp: this.formatTimestamp(new Date()) 
    };
    this.saveLogToFile(log);

    return new Promise((resolve, reject) => {
      client.sendRequest(request, (response: ServiceResponse) => {
        if (response.success) {
          console.log(`${shouldReturn ? 'Robot returning' : 'Mission stopped'} for robot ${robotId}: ${response.message}`);
          this.syncGateway.broadcast('syncUpdate', { event, robot: robotId });
          resolve({ 
            message: shouldReturn 
              ? `Robot ${robotId} is returning to start` 
              : `Mission stopped for robot ${robotId}`, 
            success: true 
          });
        } else {
          const failEvent = shouldReturn ? 'return_failed' : 'mission_stop_failed';
          console.error(`Failed to ${shouldReturn ? 'initiate return' : 'stop mission'} for robot ${robotId}: ${response.message}`);
          this.syncGateway.broadcast('syncUpdate', { event: failEvent, robot: robotId });
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

    this.lastRobotStatus[robotId] = 'Identifying';
    this.syncGateway.broadcast('syncUpdate', { event: 'identifying', robot: robotId });
  
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
      this.saveLogToFile(log);
      this.syncGateway.broadcast('syncUpdate', log);
    });
  }

  getOldMissions() {
    const missionFiles = fs.readdirSync(this.logsFolder).filter(file => file.startsWith('mission_') && file.endsWith('.json'));
    
    const missions = missionFiles.map(file => {
      const filePath = path.join(this.logsFolder, file);
      const fileContent = fs.readFileSync(filePath, 'utf-8').split('\n').filter(line => line);
      const logs = fileContent.map(line => JSON.parse(line));
  
      return { mission: file, logs };
    });
  
    return missions;
  }

  getLogStream(): Observable<any> {
    return this.logSubject.asObservable();
  }
  
  async onModuleDestroy() {
    // Destroy nodes and shutdown rclnodejs
    this.simulationRobotNode.destroy();
    this.realRobotNode.destroy();
    await rclnodejs.shutdown();
  }
}
