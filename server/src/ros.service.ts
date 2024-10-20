import { Injectable, OnModuleInit, HttpException, HttpStatus } from '@nestjs/common';
import * as ROSLIB from 'roslib';
import { ConfigService } from '@nestjs/config';
import * as fs from 'fs';
import { Subject } from 'rxjs';

@Injectable()
export class RosService implements OnModuleInit {
  private realRos: ROSLIB.Ros;
  private simulationRos: ROSLIB.Ros;
  private logs: any[] = [];
  private logFile = this.configService.get<string>('PATH_TO_LOGS');
  private logSubject = new Subject<any>();

  constructor(private configService: ConfigService) {
    this.loadLogs();
  }

  onModuleInit() {
    this.connectToRobots();
  }

  private loadLogs() {
    if (fs.existsSync(this.logFile)) {
      this.logs = JSON.parse(fs.readFileSync(this.logFile, 'utf8'));
    }
  }

  private saveLogs(log: any) {
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
    const simulationWsUrl = this.configService.get<string>('ROS_WS_URL_SIMULATION');
    const realWsUrl = this.configService.get<string>('ROS_WS_URL_REAL');

    // Connect to simulation ROS
    if (simulationWsUrl) {
      this.simulationRos = new ROSLIB.Ros({
        url: simulationWsUrl,
      });

      this.simulationRos.on('connection', () => {
        const log = { event: 'connection', robot: 'simulation', timestamp: new Date().toISOString() };
        this.saveLogs(log);
        console.log(`Connected to simulation ROS at ${simulationWsUrl}`);
      });

      this.simulationRos.on('error', (error) => {
        const log = { event: 'error', robot: 'simulation', timestamp: new Date().toISOString() };
        this.saveLogs(log);
        console.error(`Error connecting to simulation ROS:`, error);
      });

      this.simulationRos.on('close', () => {
        const log = { event: 'close', robot: 'simulation', timestamp: new Date().toISOString() };
        this.saveLogs(log);
        console.log(`Connection to simulation ROS closed`);
      });
    }

    // Connect to real robots ROS
    if (realWsUrl) {
      this.realRos = new ROSLIB.Ros({
        url: realWsUrl,
      });

      this.realRos.on('connection', () => {
        const log = { event: 'connection', robot: 'real', timestamp: new Date().toISOString() };
        this.saveLogs(log);
        console.log(`Connected to real robots ROS at ${realWsUrl}`);
      });

      this.realRos.on('error', (error) => {
        const log = { event: 'error', robot: 'real', timestamp: new Date().toISOString() };
        this.saveLogs(log);
        console.error(`Error connecting to real robots ROS:`, error);
      });

      this.realRos.on('close', () => {
        const log = { event: 'close', robot: 'real', timestamp: new Date().toISOString() };
        this.saveLogs(log);
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
    const log = { event: 'start_mission', robot: robotId, timestamp: new Date().toISOString() };
    this.saveLogs(log);

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
        const log = { event: 'mission_started', robot: robotId, timestamp: new Date().toISOString() };
        this.saveLogs(log);
        console.log(`Mission started for robot ${robotId}: ${result.message}`);
      } else {
        const log = { event: 'mission_failed', robot: robotId, timestamp: new Date().toISOString() };
        this.saveLogs(log);
        console.error(`Failed to start mission for robot ${robotId}: ${result.message}`);
      }
    });

    return { message: `Requested to start mission for robot ${robotId}` };
  }

  stopRobotMission(robotId: string) {
    const rosConnection = this.validateRobotConnection(robotId);
    const log = { event: 'stop_mission', robot: robotId, timestamp: new Date().toISOString() };
    this.saveLogs(log);

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
        const log = { event: 'mission_stopped', robot: robotId, timestamp: new Date().toISOString() };
        this.saveLogs(log);
        console.log(`Mission stopped for robot ${robotId}: ${result.message}`);
      } else {
        const log = { event: 'mission_stop_failed', robot: robotId, timestamp: new Date().toISOString() };
        this.saveLogs(log);
        console.error(`Failed to stop mission for robot ${robotId}: ${result.message}`);
      }
    });

    return { message: `Requested to stop mission for robot ${robotId}` };
  }

  identifyRobot(robotId: string) {
    const rosConnection = this.validateRobotConnection(robotId);
    const log = { event: 'identify', robot: robotId, timestamp: new Date().toISOString() };
    this.saveLogs(log);

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
        const log = { event: 'identified', robot: robotId, timestamp: new Date().toISOString() };
        this.saveLogs(log);
        console.log(`Robot ${robotId} successfully identified: ${result.message}`);
      } else {
        const log = { event: 'identification_failed', robot: robotId, timestamp: new Date().toISOString() };
        this.saveLogs(log);
        console.error(`Failed to identify Robot ${robotId}: ${result.message}`);
      }
    });

    return { message: `Robot ${robotId} is identifying itself` };
  }

  changeDriveMode(robotId: string, driveMode: string) {
    const rosConnection = this.validateRobotConnection(robotId);
    const log = { event: 'change_drive_mode', robot: robotId, timestamp: new Date().toISOString() };
    this.saveLogs(log);
  
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
        const log = { event: 'drive_mode_changed', robot: robotId, timestamp: new Date().toISOString() };
        this.saveLogs(log);
        console.log(`Drive mode changed to ${driveMode} for robot ${robotId}`);
      } else {
        const log = { event: 'drive_mode_change_failed', robot: robotId, timestamp: new Date().toISOString() };
        this.saveLogs(log);
        console.error(`Failed to change drive mode for robot ${robotId}`);
      }
    });
  
    return { message: `Requested to change drive mode for robot ${robotId} to ${driveMode}` };
  }
  
  
}
