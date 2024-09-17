import { Test, TestingModule } from '@nestjs/testing';
import { RosService } from './ros.service';
import { ConfigService } from '@nestjs/config';
import { HttpException, HttpStatus } from '@nestjs/common';
import * as ROSLIB from 'roslib';

jest.mock('roslib');

describe('RosService', () => {
  let service: RosService;
  let configService: ConfigService;

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      providers: [
        RosService,
        {
          provide: ConfigService,
          useValue: {
            get: jest.fn().mockReturnValue('ws://localhost:9090'),
          },
        },
      ],
    }).compile();

    service = module.get<RosService>(RosService);
    configService = module.get<ConfigService>(ConfigService);
  });

  describe('onModuleInit', () => {
    it('should not throw an error during module initialization', () => {
      expect(() => service.onModuleInit()).not.toThrow();
    });
  });

  describe('connectToRobot', () => {
    it('should establish a connection to the robot and handle events', () => {
      const robotId = '1';
      const wsUrl = 'ws://localhost:9090';

      const mockRos = {
        on: jest.fn(),
      };

      (ROSLIB.Ros as unknown as jest.Mock).mockImplementation(() => mockRos);

      // Spies on console.log and console.error
      const consoleLogSpy = jest.spyOn(console, 'log').mockImplementation();
      const consoleErrorSpy = jest.spyOn(console, 'error').mockImplementation();

      // Call the private method
      (service as any).connectToRobot(robotId, wsUrl);

      expect(ROSLIB.Ros).toHaveBeenCalledWith({ url: wsUrl });
      expect(mockRos.on).toHaveBeenCalledTimes(3);
      expect(service['rosConnections'][robotId]).toBe(mockRos);

      // Simulate 'connection' event
      const connectionCallback = mockRos.on.mock.calls.find(
        (call) => call[0] === 'connection',
      )[1];
      connectionCallback();

      expect(consoleLogSpy).toHaveBeenCalledWith(
        `Connected to Robot ${robotId} at WebSocket URL: ${wsUrl}`,
      );

      // Simulate 'error' event
      const errorCallback = mockRos.on.mock.calls.find(
        (call) => call[0] === 'error',
      )[1];
      const error = new Error('Test error');
      errorCallback(error);

      expect(consoleErrorSpy).toHaveBeenCalledWith(
        `Error connecting to Robot ${robotId}:`,
        error,
      );

      // Simulate 'close' event
      const closeCallback = mockRos.on.mock.calls.find(
        (call) => call[0] === 'close',
      )[1];
      closeCallback();

      expect(consoleLogSpy).toHaveBeenCalledWith(
        `Connection to Robot ${robotId} closed`,
      );

      // Cleanup
      consoleLogSpy.mockRestore();
      consoleErrorSpy.mockRestore();
    });
  });

  describe('validateRobotConnection', () => {
    it('should pass when robot is connected', () => {
      const robotId = '1';
      service['rosConnections'][robotId] = { isConnected: true } as any;

      expect(() => (service as any).validateRobotConnection(robotId)).not.toThrow();
    });

    it('should throw HttpException when robot is not connected', () => {
      const robotId = '1';
      service['rosConnections'][robotId] = { isConnected: false } as any;

      expect(() => (service as any).validateRobotConnection(robotId)).toThrow(
        new HttpException(
          `Cannot connect to Robot ${robotId}`,
          HttpStatus.SERVICE_UNAVAILABLE,
        ),
      );
    });

    it('should throw HttpException when robot connection is missing', () => {
      const robotId = '1';
      service['rosConnections'][robotId] = undefined;

      expect(() => (service as any).validateRobotConnection(robotId)).toThrow(
        new HttpException(
          `Cannot connect to Robot ${robotId}`,
          HttpStatus.SERVICE_UNAVAILABLE,
        ),
      );
    });
  });

  describe('startRobotMission', () => {
    it('should start the robot mission successfully', () => {
      const robotId = '1';
      service['rosConnections'][robotId] = { isConnected: true } as any;

      const mockCallService = jest.fn((request, callback) => {
        callback({ success: true, message: 'Mission started' });
      });
      (ROSLIB.Service as jest.Mock).mockImplementation(() => ({
        callService: mockCallService,
      }));
      (ROSLIB.ServiceRequest as jest.Mock).mockImplementation(() => ({}));

      const result = service.startRobotMission(robotId);

      expect(mockCallService).toHaveBeenCalledTimes(1);
      expect(result).toEqual({ message: `Requested to start mission for robot ${robotId}` });
    });

    it('should handle service call failure', () => {
      const robotId = '1';
      service['rosConnections'][robotId] = { isConnected: true } as any;

      const mockCallService = jest.fn((request, callback) => {
        callback({ success: false, message: 'Failed to start mission' });
      });
      (ROSLIB.Service as jest.Mock).mockImplementation(() => ({
        callService: mockCallService,
      }));

      const consoleErrorSpy = jest.spyOn(console, 'error').mockImplementation();

      service.startRobotMission(robotId);

      expect(mockCallService).toHaveBeenCalledTimes(1);
      expect(consoleErrorSpy).toHaveBeenCalledWith(
        `Failed to start mission for robot ${robotId}: Failed to start mission`,
      );

      consoleErrorSpy.mockRestore();
    });

    it('should throw HttpException when robot is not connected', () => {
      const robotId = '1';

      expect(() => service.startRobotMission(robotId)).toThrow(
        new HttpException(
          `Cannot connect to Robot ${robotId}`,
          HttpStatus.SERVICE_UNAVAILABLE,
        ),
      );
    });
  });

  describe('stopRobotMission', () => {
    it('should stop the robot mission successfully', () => {
      const robotId = '1';
      service['rosConnections'][robotId] = { isConnected: true } as any;

      const mockCallService = jest.fn((request, callback) => {
        callback({ success: true, message: 'Mission stopped' });
      });
      (ROSLIB.Service as jest.Mock).mockImplementation(() => ({
        callService: mockCallService,
      }));
      (ROSLIB.ServiceRequest as jest.Mock).mockImplementation(() => ({}));

      const result = service.stopRobotMission(robotId);

      expect(mockCallService).toHaveBeenCalledTimes(1);
      expect(result).toEqual({ message: `Requested to stop mission for robot ${robotId}` });
    });

    it('should handle service call failure', () => {
      const robotId = '1';
      service['rosConnections'][robotId] = { isConnected: true } as any;

      const mockCallService = jest.fn((request, callback) => {
        callback({ success: false, message: 'Failed to stop mission' });
      });
      (ROSLIB.Service as jest.Mock).mockImplementation(() => ({
        callService: mockCallService,
      }));

      const consoleErrorSpy = jest.spyOn(console, 'error').mockImplementation();

      service.stopRobotMission(robotId);

      expect(mockCallService).toHaveBeenCalledTimes(1);
      expect(consoleErrorSpy).toHaveBeenCalledWith(
        `Failed to stop mission for robot ${robotId}: Failed to stop mission`,
      );

      consoleErrorSpy.mockRestore();
    });

    it('should throw HttpException when robot is not connected', () => {
      const robotId = '1';

      expect(() => service.stopRobotMission(robotId)).toThrow(
        new HttpException(
          `Cannot connect to Robot ${robotId}`,
          HttpStatus.SERVICE_UNAVAILABLE,
        ),
      );
    });
  });

  describe('identifyRobot', () => {
    it('should identify the robot successfully', () => {
      const robotId = '1';
      service['rosConnections'][robotId] = { isConnected: true } as any;

      const mockCallService = jest.fn((request, callback) => {
        callback({ success: true, message: 'Identified' });
      });
      (ROSLIB.Service as jest.Mock).mockImplementation(() => ({
        callService: mockCallService,
      }));
      (ROSLIB.ServiceRequest as jest.Mock).mockImplementation(() => ({}));

      const result = service.identifyRobot(robotId);

      expect(mockCallService).toHaveBeenCalledTimes(1);
      expect(result).toEqual({ message: `Robot ${robotId} is identifying itself` });
    });

    it('should handle service call failure', () => {
      const robotId = '1';
      service['rosConnections'][robotId] = { isConnected: true } as any;

      const mockCallService = jest.fn((request, callback) => {
        callback({ success: false, message: 'Failed to identify' });
      });
      (ROSLIB.Service as jest.Mock).mockImplementation(() => ({
        callService: mockCallService,
      }));
      (ROSLIB.ServiceRequest as jest.Mock).mockImplementation(() => ({}));

      const consoleErrorSpy = jest.spyOn(console, 'error').mockImplementation();

      const result = service.identifyRobot(robotId);

      expect(mockCallService).toHaveBeenCalledTimes(1);
      expect(consoleErrorSpy).toHaveBeenCalledWith(
        `Failed to identify Robot ${robotId}: Failed to identify`,
      );
      expect(result).toEqual({ message: `Robot ${robotId} is identifying itself` });

      consoleErrorSpy.mockRestore();
    });

    it('should throw HttpException when robot is not connected', () => {
      const robotId = '1';

      expect(() => service.identifyRobot(robotId)).toThrow(
        new HttpException(
          `Cannot connect to Robot ${robotId}`,
          HttpStatus.SERVICE_UNAVAILABLE,
        ),
      );
    });
  });
});
