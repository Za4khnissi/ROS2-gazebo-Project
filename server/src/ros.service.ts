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
    private syncGateway: SyncGateway,
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
    rosoutNode.createSubscription('rcl_interfaces/msg/Log', '/rosout', (msg) =>
      this.handleLogMessage(msg),
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
    const logFilePath = path.join(
      this.logsFolder,
      `mission_${robotId}_${timestamp}.json`,
    );
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

    const OccupancyGrid = rclnodejs.require('nav_msgs/msg/OccupancyGrid');
    const PointCloud2 = rclnodejs.require('sensor_msgs/msg/PointCloud2');

    // 2D Map subscription
    this.simulationRobotNode.createSubscription(
      OccupancyGrid as any,
      '/map',
      (message: any) => {
        const mapData = this.formatOccupancyGrid(message);
        this.syncGateway.broadcast('map_update', mapData);
      },
    );

    // 3D Map (Point Cloud) subscription
    this.simulationRobotNode.createSubscription(
      PointCloud2 as any,
      '/octomap_point_cloud_centers', // Ensure topic matches your setup
      (message: any) => {
        const octomapData = this.formatPointCloud2(message);
        this.syncGateway.broadcast('octomap_update', { points: octomapData });
      },
    );

    this.simulationRobotNode.spin();
    this.realRobotNode.spin();
    console.log('ROS 2 nodes initialized for simulation and real robots.');
  }

  private formatOccupancyGrid(message: any): any {
    return {
      header: {
        seq: message.header.seq,
        stamp: {
          sec: message.header.stamp.sec,
          nsec: message.header.stamp.nsec,
        },
        frame_id: message.header.frame_id,
      },
      info: {
        map_load_time: {
          sec: message.info.map_load_time.sec,
          nsec: message.info.map_load_time.nsec,
        },
        resolution: message.info.resolution,
        width: message.info.width,
        height: message.info.height,
        origin: {
          position: {
            x: message.info.origin.position.x,
            y: message.info.origin.position.y,
            z: message.info.origin.position.z,
          },
          orientation: {
            x: message.info.origin.orientation.x,
            y: message.info.origin.orientation.y,
            z: message.info.origin.orientation.z,
            w: message.info.origin.orientation.w,
          },
        },
      },
      data: Array.from(message.data), // Convert Int8Array to a standard array
    };
  }

  formatPointCloud2(message: any): Array<{ x: number; y: number; z: number }> {
    const points = [];
    const data = new DataView(new Uint8Array(message.data).buffer); // Wrap data in DataView for structured access
    const pointStep = message.point_step; // Number of bytes per point
    for (let i = 0; i < data.byteLength; i += pointStep) {
      // Extract x, y, z coordinates as Float32 from the correct offsets
      const x = data.getFloat32(i, true); // true for little-endian
      const y = data.getFloat32(i + 4, true);
      const z = data.getFloat32(i + 8, true);
      points.push({ x, y, z });
    }

    return points;
  }

  private validateRobotConnection(robotId: string): rclnodejs.Node {
    if (robotId === '3' || robotId === '4') {
      return this.simulationRobotNode;
    } else if (robotId === '1' || robotId === '2') {
      return this.realRobotNode;
    } else {
      throw new HttpException(
        `Invalid Robot ID ${robotId}`,
        HttpStatus.BAD_REQUEST,
      );
    }
  }

  async startRobotMission(
    robotId: string,
  ): Promise<{ message: string; success: boolean }> {
    this.missionActive = true;
    this.startMissionLogFile(robotId);
    this.startLogging();

    const node = this.validateRobotConnection(robotId);
    const namespace = `limo_105_${robotId}`;
    const serviceName = `/${namespace}/mission`;

    const client = node.createClient(
      'example_interfaces/srv/SetBool',
      serviceName,
    );
    const RequestType = rclnodejs.require(
      'example_interfaces/srv/SetBool',
    ).Request;
    const request = new RequestType();
    request.data = true;

    const serviceAvailable = await client.waitForService(5000);
    if (!serviceAvailable) {
      throw new HttpException(
        `Service ${serviceName} not available`,
        HttpStatus.SERVICE_UNAVAILABLE,
      );
    }

    this.lastRobotStatus[robotId] = 'Moving';
    this.syncGateway.broadcast('syncUpdate', {
      event: 'mission_started',
      robot: robotId,
    });

    const log = {
      event: 'start_mission',
      robot: robotId,
      timestamp: this.formatTimestamp(new Date()),
    };
    this.saveLogToFile(log);

    return new Promise((resolve, reject) => {
      client.sendRequest(request, (response: ServiceResponse) => {
        if (response.success) {
          this.syncGateway.broadcast('syncUpdate', {
            event: 'mission_started',
            robot: robotId,
          });
          resolve({
            message: `Mission started for robot ${robotId}`,
            success: true,
          });
        } else {
          this.syncGateway.broadcast('syncUpdate', {
            event: 'mission_failed',
            robot: robotId,
          });
          reject(
            new HttpException(
              response.message,
              HttpStatus.INTERNAL_SERVER_ERROR,
            ),
          );
        }
      });
    });
  }

  async stopRobotMission(
    robotId: string,
  ): Promise<{ message: string; success: boolean }> {
    this.missionActive = false;
    this.stopLogging();

    const node = this.validateRobotConnection(robotId);
    const namespace = `limo_105_${robotId}`;
    const serviceName = `/${namespace}/mission`;

    const client = node.createClient(
      'example_interfaces/srv/SetBool',
      serviceName,
    );
    const RequestType = rclnodejs.require(
      'example_interfaces/srv/SetBool',
    ).Request;
    const request = new RequestType();
    request.data = false;

    const serviceAvailable = await client.waitForService(5000);
    if (!serviceAvailable) {
      throw new HttpException(
        `Service ${serviceName} not available`,
        HttpStatus.SERVICE_UNAVAILABLE,
      );
    }

    this.lastRobotStatus[robotId] = 'Stopped';
    this.syncGateway.broadcast('syncUpdate', {
      event: 'mission_stopped',
      robot: robotId,
    });

    const log = {
      event: 'stop_mission',
      robot: robotId,
      timestamp: this.formatTimestamp(new Date()),
    };
    this.saveLogToFile(log);

    return new Promise((resolve, reject) => {
      client.sendRequest(request, (response: ServiceResponse) => {
        if (response.success) {
          console.log(
            `Mission stopped for robot ${robotId}: ${response.message}`,
          );
          this.syncGateway.broadcast('syncUpdate', {
            event: 'mission_stopped',
            robot: robotId,
          });
          resolve({
            message: `Mission stopped for robot ${robotId}`,
            success: true,
          });
        } else {
          console.error(
            `Failed to stop mission for robot ${robotId}: ${response.message}`,
          );
          this.syncGateway.broadcast('syncUpdate', {
            event: 'mission_stop_failed',
            robot: robotId,
          });
          reject(
            new HttpException(
              response.message,
              HttpStatus.INTERNAL_SERVER_ERROR,
            ),
          );
        }
      });
    });
  }

  async identifyRobot(
    robotId: string,
  ): Promise<{ message: string; success: boolean }> {
    const node = this.validateRobotConnection(robotId);
    const namespace = `limo_105_${robotId}`;
    const serviceName = `/${namespace}/identify`;

    const client = node.createClient('std_srvs/srv/Trigger', serviceName);
    const RequestType = rclnodejs.require('std_srvs/srv/Trigger').Request;
    const request = new RequestType();

    const serviceAvailable = await client.waitForService(5000);
    if (!serviceAvailable) {
      throw new HttpException(
        `Service ${serviceName} not available`,
        HttpStatus.SERVICE_UNAVAILABLE,
      );
    }

    this.lastRobotStatus[robotId] = 'Identifying';
    this.syncGateway.broadcast('syncUpdate', {
      event: 'identifying',
      robot: robotId,
    });

    return new Promise((resolve, reject) => {
      client.sendRequest(request, (response: ServiceResponse) => {
        if (response.success) {
          console.log(
            `Robot ${robotId} identified successfully: ${response.message}`,
          );
          this.syncGateway.broadcast('syncUpdate', {
            event: 'identified',
            robot: robotId,
          });
          resolve({ message: `Robot ${robotId} identified`, success: true });
        } else {
          console.error(
            `Failed to identify Robot ${robotId}: ${response.message}`,
          );
          this.syncGateway.broadcast('syncUpdate', {
            event: 'identification_failed',
            robot: robotId,
          });
          reject(
            new HttpException(
              response.message,
              HttpStatus.INTERNAL_SERVER_ERROR,
            ),
          );
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
      const event = response.success
        ? 'drive_mode_changed'
        : 'drive_mode_change_failed';
      const log = {
        event,
        robot: robotId,
        driveMode,
        timestamp: new Date().toISOString(),
      };
      this.saveLogToFile(log);
      this.syncGateway.broadcast('syncUpdate', log);
    });
  }

  getOldMissions() {
    const missionFiles = fs
      .readdirSync(this.logsFolder)
      .filter((file) => file.startsWith('mission_') && file.endsWith('.json'));

    const missions = missionFiles.map((file) => {
      const filePath = path.join(this.logsFolder, file);
      const fileContent = fs
        .readFileSync(filePath, 'utf-8')
        .split('\n')
        .filter((line) => line);
      const logs = fileContent.map((line) => JSON.parse(line));

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
