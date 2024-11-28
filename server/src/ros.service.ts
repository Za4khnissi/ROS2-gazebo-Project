import {
  Injectable,
  OnModuleDestroy,
  HttpException,
  HttpStatus,
  OnModuleInit,
} from '@nestjs/common';
import { InjectModel } from '@nestjs/mongoose';
import * as rclnodejs from 'rclnodejs';
import { ConfigService } from '@nestjs/config';
import * as fs from 'fs';
import { Subject, interval, Subscription, Observable } from 'rxjs';
import { SyncGateway } from './sync/sync.gateway';
import * as path from 'path';
import { MissionModel } from './mission/mission.model';
import { Model } from 'mongoose';
import { exec } from 'child_process';
import * as readline from 'readline';

interface ServiceResponse {
  success: boolean;
  message?: string;
}

const BATTERY_THRESHOLD = parseInt(process.env.BATTERY_THRESHOLD || '30');



@Injectable()
export class RosService implements OnModuleDestroy,OnModuleInit {
  private realRobotNode: rclnodejs.Node;
  private simulationRobotNode: rclnodejs.Node;
  private logsFolder = this.configService.get<string>('PATH_TO_LOGS_FOLDER');
  private logSubject = new Subject<any>();
  private missionActive = false;
  private logInterval: Subscription;
  private currentLogs: any[] = [];
  private logFilePaths = new Map<string, string>();
  private isRosRunning = false;
  private lastRobotStatus = {
    '3': 'Waiting',
    '4': 'Waiting',
  };
  private totalDistance: { [robotId: string]: number } = {};
  private lastPosition: { [robotId: string]: { x: number; y: number } } = {};
  private mapData: any | null = null;
  private currentMissionId: string | null = null;
  private isSimulation : boolean

  constructor(
    private configService: ConfigService,
    private syncGateway: SyncGateway,
    @InjectModel('Mission') private missionModel: Model<MissionModel>,
  ) {
    this.ensureLogsFolderExists();
    this.isSimulation = this.configService.get<number>('SIMULATION') == 1
    console.log('Simulation mode is set to:', this.isSimulation);
    //console.log(process.env.SIMULATION);
  }

  async onModuleInit() {
      if (!this.isSimulation) await this.connectToRos();
  }

  async startRos(driveModes: { [key: string]: string }): Promise<any> {
    if (this.isRosRunning) {
      throw new HttpException('ROS is already running', HttpStatus.CONFLICT);
    }

    const scriptPath = path.resolve(__dirname, '../../launch_robot.sh');
    const command = `${scriptPath} simulation ${driveModes['3']} ${driveModes['4']}`;
    console.log(`Launching ROS project with command: ${command}`);

    return new Promise((resolve, reject) => {
      // Open a new terminal and run the command
      const terminalCommand = `gnome-terminal -- bash -c "${command}; exec bash"`;

      exec(terminalCommand, (error, stdout) => {
        if (error) {
          console.error(
            'Failed to open terminal and launch ROS project:',
            error,
          );
          reject(
            new HttpException(
              'Failed to open terminal and launch ROS project',
              HttpStatus.INTERNAL_SERVER_ERROR,
            ),
          );
        } else {
          console.log('New terminal opened for ROS logs:', stdout);

          // Wait 1 minute for nodes to initialize
          console.log('Waiting 40 seconds for ROS nodes to initialize...');
          setTimeout(() => {
            this.promptUserForConnection(resolve, reject);
          }, 40000);
        }
      });
    });
  }

  private promptUserForConnection(resolve: any, reject: any) {
    const rl = readline.createInterface({
      input: process.stdin,
      output: process.stdout,
    });

    rl.question(
      'Do you want to connect the server to ROS? (Y/yes): ',
      async (answer) => {
        if (answer.toLowerCase() === 'y' || answer.toLowerCase() === 'yes') {
          console.log('Connecting server to ROS...');
          try {
            await this.connectToRos();
            console.log('Server successfully connected to ROS.');
            resolve({
              message: 'ROS started and server connected successfully',
            });
          } catch (error) {
            console.error('Failed to connect server to ROS:', error);
            reject(
              new HttpException(
                'Failed to connect server to ROS',
                HttpStatus.INTERNAL_SERVER_ERROR,
              ),
            );
          }
        } else {
          console.log('Server not connected to ROS. Exiting initialization.');
          reject(
            new HttpException(
              'User chose not to connect to ROS',
              HttpStatus.BAD_REQUEST,
            ),
          );
        }
        rl.close();
      },
    );
  }

  private async connectToRos() {
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

  private async saveLogToFile(log: any, missionId: string) {
    const logFilePath = this.logFilePaths.get(log.robot.split('_')[2]);
    if (logFilePath) {
      log.timestamp = this.formatTimestamp(new Date(log.timestamp));
      fs.appendFileSync(logFilePath, JSON.stringify(log) + '\n');
      this.syncGateway.broadcast('syncUpdate', log);
    }

    try {
      await this.missionModel.findByIdAndUpdate(
        missionId,
        { $push: { logs: log } },
        { new: true },
      );
    } catch (error) {
      console.error(
        `Failed to save log to mission ${missionId}:`,
        error.message,
      );
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
          this.saveLogToFile(log, this.currentMissionId);
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

    this.subscribeToRobotPosition('limo_105_1', this.realRobotNode);
    this.subscribeToRobotPosition('limo_105_2', this.realRobotNode);
    this.subscribeToRobotPosition('limo_105_3', this.simulationRobotNode);
    this.subscribeToRobotPosition('limo_105_4', this.simulationRobotNode);

    const OccupancyGrid = rclnodejs.require('nav_msgs/msg/OccupancyGrid');
    const PointCloud2 = rclnodejs.require('sensor_msgs/msg/PointCloud2');

    // 2D Map subscription
    this.simulationRobotNode.createSubscription(
      OccupancyGrid as any,
      '/map',
      (message: any) => {
        // Convert the message to a plain JavaScript object
        this.mapData = {
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
          data: Array.from(message.data), // Convert Int8Array to regular array
        };

        // Broadcast the map data through the WebSocket
        this.syncGateway.broadcast('map_update', this.mapData);
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

  private subscribeToBatteryLevel(robotId: string, node: rclnodejs.Node) {
    const topicName = `/${robotId}/battery_level`;
    console.log(`Attempting to subscribe to ${topicName}`);

    node.createSubscription('std_msgs/msg/Float32', topicName, (msg) => {
      //console.log(`Message received on ${topicName}`);
      if (this.isFloat32(msg)) {
        const batteryLevel = msg.data;
        //console.log(`Battery Level for ${robotId}: ${batteryLevel}%`);
        if (
          batteryLevel < BATTERY_THRESHOLD &&
          !['Returning', 'Waiting'].includes(this.lastRobotStatus[robotId])
        ) {
          console.log(`Battery level for ${robotId} is low. Stopping Mission.`);
          this.stopRobotMission(robotId, true);
        }
        this.syncGateway.broadcastBatteryUpdate(robotId, batteryLevel);
      } else {
        console.error(`Received message for ${robotId} is not of type Float32`);
      }
    });
    console.log(`Subscribed to battery level topic for ${robotId}`);
  }

  private subscribeToRobotPosition(robotId: string, node: rclnodejs.Node) {
    const topicName = `/${robotId}/amcl_pose`; // or your specific pose topic
    console.log(`Attempting to subscribe to ${topicName}`);

    const PoseWithCovarianceStamped = rclnodejs.require(
      'geometry_msgs/msg/PoseWithCovarianceStamped',
    );

    node.createSubscription(
      PoseWithCovarianceStamped as any,
      topicName,
      (msg: any) => {
        //console.log(`Position message received on ${topicName}`);

        // Extract position and orientation from the message
        const position = {
          x: msg.pose.pose.position.x,
          y: msg.pose.pose.position.y,
          z: msg.pose.pose.position.z,
        };

        const orientation = {
          x: msg.pose.pose.orientation.x,
          y: msg.pose.pose.orientation.y,
          z: msg.pose.pose.orientation.z,
          w: msg.pose.pose.orientation.w,
        };

        // You can also extract covariance if needed
        // const covariance = msg.pose.covariance;

        // Create a position update object
        const positionUpdate = {
          robotId: robotId,
          position: position,
          orientation: orientation,
          // Include frame_id for reference frame information
          frame_id: msg.header.frame_id,
          // Convert ROS time to JavaScript timestamp
          timestamp:
            msg.header.stamp.sec * 1000 +
            Math.floor(msg.header.stamp.nanosec / 1000000),
        };

        // Broadcast the position update through WebSocket
        this.syncGateway.broadcast('robot_position_update', positionUpdate);
      },
    );
    console.log(`Subscribed to position topic for ${robotId}`);
  }

  private isFloat32(msg: any): msg is { data: number } {
    return typeof msg.data === 'number';
  }

  async startRobotMission(
    robotId: string,
  ): Promise<{ message: string; success: boolean }> {
    this.missionActive = true;
    this.startMissionLogFile(robotId);
    this.startLogging();

    const newMission = new this.missionModel({
      dateDebut: new Date(),
      robots: [robotId],
      isPhysical: robotId === '1' || robotId === '2',
      totalDistance: 0,
      duration: 0,
      logs: [],
      mapData: null,
    });

    const savedMission = await newMission.save();
    this.currentMissionId = savedMission._id.toString();
    console.log(`Mission started and saved in DB with ID: ${savedMission._id}`);

    this.watchDistance(robotId);

    const node = this.validateRobotConnection(robotId);
    const namespace = `limo_105_${robotId}`;
    const serviceName = `/${namespace}/mission`;

    const client = node.createClient(
      'ros_gz_example_application/srv/MissionCommand',
      serviceName,
    );

    const RequestType = rclnodejs.require(
      'ros_gz_example_application/srv/MissionCommand',
    ).Request;
    const request = new RequestType();

    request.command = 1; // Start mission

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
    this.saveLogToFile(log, this.currentMissionId);

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

  private watchDistance(robotId: string) {
    this.totalDistance[robotId] = 0;
    const topicName = `/limo_105_1/odom`;

    const node = this.validateRobotConnection(robotId);
    node.createSubscription('nav_msgs/msg/Odometry', topicName, (msg) => {
      const odomMsg = msg as any;
      const currentPosition = {
        x: odomMsg.pose.pose.position.x,
        y: odomMsg.pose.pose.position.y,
      };
      //console.log("Good odom , currentPosition", currentPosition);
      if (this.lastPosition[robotId]) {
        const delta = Math.sqrt(
          Math.pow(currentPosition.x - this.lastPosition[robotId].x, 2) +
            Math.pow(currentPosition.y - this.lastPosition[robotId].y, 2),
        );
        this.totalDistance[robotId] += delta;
      }

      this.lastPosition[robotId] = currentPosition;
    });
  }

  async stopRobotMission(
    robotId: string,
    shouldReturn: boolean = false,
  ): Promise<{ message: string; success: boolean }> {
    this.missionActive = false;
    this.stopLogging();

    const node = this.validateRobotConnection(robotId);
    const namespace = `limo_105_${robotId}`;
    const serviceName = `/${namespace}/mission`;

    const client = node.createClient(
      'ros_gz_example_application/srv/MissionCommand',
      serviceName,
    );

    const RequestType = rclnodejs.require(
      'ros_gz_example_application/srv/MissionCommand',
    ).Request;
    const request = new RequestType();
    request.command = shouldReturn ? 3 : 2; // 3 for return, 2 for stop

    const serviceAvailable = await client.waitForService(5000);
    if (!serviceAvailable) {
      throw new HttpException(
        `Service ${serviceName} not available`,
        HttpStatus.SERVICE_UNAVAILABLE,
      );
    }

    // mettre a jour la db
    if (!['Returning', 'Stopped'].includes(this.lastRobotStatus[robotId])) {
      // if the robot is not already returning or stopped
      const mission = await this.missionModel.findOne({
        robots: robotId,
        dateFin: null,
      });
      if (mission) {
        const endTime = new Date();
        mission.dateFin = endTime;
        mission.duration =
          (endTime.getTime() - mission.dateDebut.getTime()) / 1000; // En secondes
        mission.totalDistance = this.totalDistance[robotId];
        console.log('Totale duration for robot:', mission.duration);
        console.log('Total distance for robot:', this.totalDistance[robotId]);
        mission.mapData = this.mapData;
        await mission.save();
        console.log(
          `Mission stopped and updated in DB with ID: ${mission._id}`,
        );
      } else {
        console.log(`No active mission found for robot ${robotId}`);
      }
    }

    const status = shouldReturn ? 'Returning' : 'Stopped';
    const event = shouldReturn ? 'robot_returning' : 'mission_stopped';
    this.lastRobotStatus[robotId] = status;
    this.syncGateway.broadcast('syncUpdate', { event, robot: robotId });

    const log = {
      event: shouldReturn ? 'stop_and_return' : 'stop_mission',
      robot: robotId,
      timestamp: this.formatTimestamp(new Date()),
    };
    this.saveLogToFile(log, this.currentMissionId);

    return new Promise((resolve, reject) => {
      client.sendRequest(request, (response: ServiceResponse) => {
        if (response.success) {
          console.log(
            `${shouldReturn ? 'Robot returning' : 'Mission stopped'} for robot ${robotId}: ${response.message}`,
          );
          this.syncGateway.broadcast('syncUpdate', { event, robot: robotId });
          resolve({
            message: shouldReturn
              ? `Robot ${robotId} is returning to start`
              : `Mission stopped for robot ${robotId}`,
            success: true,
          });
        } else {
          const failEvent = shouldReturn
            ? 'return_failed'
            : 'mission_stop_failed';
          console.error(
            `Failed to ${shouldReturn ? 'initiate return' : 'stop mission'} for robot ${robotId}: ${response.message}`,
          );
          this.syncGateway.broadcast('syncUpdate', {
            event: failEvent,
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

  async stopRos(): Promise<any> {
    if (!this.isRosRunning) {
      throw new HttpException('ROS is not running', HttpStatus.BAD_REQUEST);
    }

    try {
      this.simulationRobotNode.destroy();
      this.realRobotNode.destroy();
      await rclnodejs.shutdown();
      this.isRosRunning = false;
      console.log('ROS stopped successfully');
      return { message: 'ROS stopped successfully' };
    } catch (error) {
      console.error('Failed to stop ROS:', error);
      throw new HttpException(
        'Failed to stop ROS',
        HttpStatus.INTERNAL_SERVER_ERROR,
      );
    }
  }

  // On shutdown, cleanly stop ROS and free up the port
  async onModuleDestroy() {
    if (this.isRosRunning) {
      await this.stopRos();
    }
  }
}
