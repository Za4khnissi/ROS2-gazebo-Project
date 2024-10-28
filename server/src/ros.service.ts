import { Injectable, OnModuleInit, HttpException, HttpStatus } from '@nestjs/common';
import * as ROSLIB from 'roslib';
import { ConfigService } from '@nestjs/config';
import * as fs from 'fs';
import { Subject } from 'rxjs';
import { SyncGateway } from './sync/sync.gateway';

@Injectable()
export class RosService implements OnModuleInit {
  private realRos: ROSLIB.Ros;
  private simulationRos: ROSLIB.Ros;
  private logs: any[] = [];
  private logFile = this.configService.get<string>('PATH_TO_LOGS');
  private logSubject = new Subject<any>();

  constructor(private configService: ConfigService, private syncGateway: SyncGateway) {
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
  
    return new Promise<{ message: string; success: boolean }>((resolve) => {
      missionService.callService(request, (result) => {
        const success = result.success;
        const message = success ? `Mission started for robot ${robotId}` : `Mission start failed for robot ${robotId}`;
        this.syncGateway.broadcast('syncUpdate', { event: success ? 'mission_started' : 'mission_failed', robot: robotId });
        this.saveLogs({ event: success ? 'mission_started' : 'mission_failed', robot: robotId, timestamp: new Date().toISOString() });
        resolve({ message, success });
      });
    });
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
  
    return new Promise<{ message: string; success: boolean }>((resolve) => {
      missionService.callService(request, (result) => {
        const success = result.success;
        const message = success ? `Mission stopped for robot ${robotId}` : `Mission stop failed for robot ${robotId}`;
        this.syncGateway.broadcast('syncUpdate', { event: success ? 'mission_stopped' : 'mission_stop_failed', robot: robotId });
        this.saveLogs({ event: success ? 'mission_stopped' : 'mission_stop_failed', robot: robotId, timestamp: new Date().toISOString() });
        resolve({ message, success });
      });
    });
  }

  identifyRobot(robotId: string): Promise<{ message: string; success: boolean }> {
    const rosConnection = this.validateRobotConnection(robotId);
    const namespace = `limo_105_${robotId}`;
    const identifyService = new ROSLIB.Service({
      ros: rosConnection,
      name: `/${namespace}/identify`,
      serviceType: 'std_srvs/Trigger',
    });
    const request = new ROSLIB.ServiceRequest({});
  
    return new Promise((resolve, reject) => {
      identifyService.callService(request, (result) => {
        const success = result.success;
        const message = success
          ? `Robot ${robotId} identified successfully`
          : `Failed to identify Robot ${robotId}`;
  
        const event = success ? 'identified' : 'identification_failed';
        const log = { event, robot: robotId, message, timestamp: new Date().toISOString() };
        this.saveLogs(log);
        this.syncGateway.broadcast('syncUpdate', log);
  
        resolve({ message, success });
      });
    });
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
      serviceType: 'std_srvs/SetBool',
    });
  
    const request = new ROSLIB.ServiceRequest({
      data: driveMode === 'ackermann',
    });
  
    driveModeService.callService(request, (result) => {
      const event = result.success ? 'drive_mode_changed' : 'drive_mode_change_failed';
      const log = { event, robot: robotId, driveMode, timestamp: new Date().toISOString() };
      this.saveLogs(log);
      this.syncGateway.broadcast('syncUpdate', log);
    });
  }
  
  
}
