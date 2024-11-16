import { Test, TestingModule } from '@nestjs/testing';
import { RosService } from './ros.service';
import { ConfigService } from '@nestjs/config';
import { HttpException, HttpStatus } from '@nestjs/common';
import * as ROSLIB from 'roslib';

jest.mock('roslib', () => ({
  Ros: jest.fn().mockImplementation(() => ({
    on: jest.fn(),
    isConnected: true,
  })),
  Service: jest.fn(),
  ServiceRequest: jest.fn(),
}));

describe('RosService', () => {
  let service: RosService;
  let configService: ConfigService;

  beforeEach(async () => {
    jest.clearAllMocks();

    const module: TestingModule = await Test.createTestingModule({
      providers: [
        RosService,
        {
          provide: ConfigService,
          useValue: {
            get: jest.fn().mockImplementation((key) => {
              if (key === 'ROS_WS_URL_SIMULATION')
                return 'ws://simulation:9090';
              if (key === 'ROS_WS_URL_REAL') return 'ws://real:9090';
              return null;
            }),
          },
        },
      ],
    }).compile();

    service = module.get<RosService>(RosService);
    configService = module.get<ConfigService>(ConfigService);
  });

  describe('onModuleInit', () => {
    it('should connect to both simulation and real ROS and handle events', () => {
      // Mocks
      const simulationRosOnMock = jest.fn();
      const realRosOnMock = jest.fn();

      const simulationRosMock = {
        on: simulationRosOnMock,
        isConnected: true,
      };
      const realRosMock = {
        on: realRosOnMock,
        isConnected: true,
      };

      (ROSLIB.Ros as unknown as jest.Mock)
        .mockImplementationOnce(() => simulationRosMock)
        .mockImplementationOnce(() => realRosMock);

      // Spies
      const consoleLogSpy = jest.spyOn(console, 'log').mockImplementation();
      const consoleErrorSpy = jest.spyOn(console, 'error').mockImplementation();

      // Call onModuleInit
      service.onModuleInit();

      // Verify ROS connections are created
      expect(ROSLIB.Ros).toHaveBeenCalledTimes(2);
      expect(ROSLIB.Ros).toHaveBeenCalledWith({ url: 'ws://simulation:9090' });
      expect(ROSLIB.Ros).toHaveBeenCalledWith({ url: 'ws://real:9090' });

      // Verify event handlers are set up
      expect(simulationRosOnMock).toHaveBeenCalledTimes(3);
      expect(realRosOnMock).toHaveBeenCalledTimes(3);

      // Simulate events for simulationRos
      const simulationConnectionCallback = simulationRosOnMock.mock.calls.find(
        (call) => call[0] === 'connection',
      )[1];
      const simulationErrorCallback = simulationRosOnMock.mock.calls.find(
        (call) => call[0] === 'error',
      )[1];
      const simulationCloseCallback = simulationRosOnMock.mock.calls.find(
        (call) => call[0] === 'close',
      )[1];

      // Simulate 'connection' event
      simulationConnectionCallback();
      expect(consoleLogSpy).toHaveBeenCalledWith(
        'Connected to simulation ROS at ws://simulation:9090',
      );

      // Simulate 'error' event
      const simError = new Error('Simulation ROS Error');
      simulationErrorCallback(simError);
      expect(consoleErrorSpy).toHaveBeenCalledWith(
        'Error connecting to simulation ROS:',
        simError,
      );

      // Simulate 'close' event
      simulationCloseCallback();
      expect(consoleLogSpy).toHaveBeenCalledWith(
        'Connection to simulation ROS closed',
      );

      // Simulate events for realRos
      const realConnectionCallback = realRosOnMock.mock.calls.find(
        (call) => call[0] === 'connection',
      )[1];
      const realErrorCallback = realRosOnMock.mock.calls.find(
        (call) => call[0] === 'error',
      )[1];
      const realCloseCallback = realRosOnMock.mock.calls.find(
        (call) => call[0] === 'close',
      )[1];

      // Simulate 'connection' event
      realConnectionCallback();
      expect(consoleLogSpy).toHaveBeenCalledWith(
        'Connected to real robots ROS at ws://real:9090',
      );

      // Simulate 'error' event
      const realError = new Error('Real ROS Error');
      realErrorCallback(realError);
      expect(consoleErrorSpy).toHaveBeenCalledWith(
        'Error connecting to real robots ROS:',
        realError,
      );

      // Simulate 'close' event
      realCloseCallback();
      expect(consoleLogSpy).toHaveBeenCalledWith(
        'Connection to real robots ROS closed',
      );

      // Cleanup
      consoleLogSpy.mockRestore();
      consoleErrorSpy.mockRestore();
    });

    it('should not connect to simulation ROS when simulationWsUrl is undefined', () => {
      // Update ConfigService to return undefined for simulationWsUrl
      configService.get = jest.fn().mockImplementation((key) => {
        if (key === 'ROS_WS_URL_SIMULATION') return undefined;
        if (key === 'ROS_WS_URL_REAL') return 'ws://real:9090';
        return null;
      });

      // Call onModuleInit
      service.onModuleInit();

      // Verify only real ROS connection is created
      expect(ROSLIB.Ros).toHaveBeenCalledTimes(1);
      expect(ROSLIB.Ros).toHaveBeenCalledWith({ url: 'ws://real:9090' });
    });

    it('should not connect to real ROS when realWsUrl is undefined', () => {
      // Update ConfigService to return undefined for realWsUrl
      configService.get = jest.fn().mockImplementation((key) => {
        if (key === 'ROS_WS_URL_SIMULATION') return 'ws://simulation:9090';
        if (key === 'ROS_WS_URL_REAL') return undefined;
        return null;
      });

      // Call onModuleInit
      service.onModuleInit();

      // Verify only simulation ROS connection is created
      expect(ROSLIB.Ros).toHaveBeenCalledTimes(1);
      expect(ROSLIB.Ros).toHaveBeenCalledWith({ url: 'ws://simulation:9090' });
    });

    it('should not connect to any ROS when both URLs are undefined', () => {
      // Update ConfigService to return undefined for both URLs
      configService.get = jest.fn().mockReturnValue(undefined);

      // Call onModuleInit
      service.onModuleInit();

      // Verify no ROS connections are created
      expect(ROSLIB.Ros).not.toHaveBeenCalled();
    });
  });

  describe('validateRobotConnection', () => {
    it('should return simulationRos for robotId "3" when connected', () => {
      (service as any).simulationRos = { isConnected: true } as any;

      const ros = (service as any).validateRobotConnection('3');

      expect(ros).toBe((service as any).simulationRos);
    });

    it('should return realRos for robotId "1" when connected', () => {
      (service as any).realRos = { isConnected: true } as any;

      const ros = (service as any).validateRobotConnection('1');

      expect(ros).toBe((service as any).realRos);
    });

    it('should throw HttpException for invalid robotId', () => {
      expect(() => (service as any).validateRobotConnection('5')).toThrow(
        new HttpException('Invalid Robot ID 5', HttpStatus.BAD_REQUEST),
      );
    });

    it('should throw HttpException when simulationRos is not connected', () => {
      (service as any).simulationRos = { isConnected: false } as any;

      expect(() => (service as any).validateRobotConnection('3')).toThrow(
        new HttpException(
          'Cannot connect to Robot 3',
          HttpStatus.SERVICE_UNAVAILABLE,
        ),
      );
    });

    it('should throw HttpException when realRos is not connected', () => {
      (service as any).realRos = { isConnected: false } as any;

      expect(() => (service as any).validateRobotConnection('1')).toThrow(
        new HttpException(
          'Cannot connect to Robot 1',
          HttpStatus.SERVICE_UNAVAILABLE,
        ),
      );
    });

    it('should throw HttpException when simulationRos is undefined', () => {
      (service as any).simulationRos = undefined;

      expect(() => (service as any).validateRobotConnection('3')).toThrow(
        new HttpException(
          'Cannot connect to Robot 3',
          HttpStatus.SERVICE_UNAVAILABLE,
        ),
      );
    });

    it('should throw HttpException when realRos is undefined', () => {
      (service as any).realRos = undefined;

      expect(() => (service as any).validateRobotConnection('1')).toThrow(
        new HttpException(
          'Cannot connect to Robot 1',
          HttpStatus.SERVICE_UNAVAILABLE,
        ),
      );
    });
  });

  describe('startRobotMission', () => {
    it('should start the mission successfully for robotId "1"', () => {
      (service as any).realRos = { isConnected: true } as any;

      const mockCallService = jest.fn((request, callback) => {
        callback({ success: true, message: 'Mission started' });
      });

      (ROSLIB.Service as jest.Mock).mockImplementation(() => ({
        callService: mockCallService,
      }));

      const result = service.startRobotMission('1');

      expect(mockCallService).toHaveBeenCalledTimes(1);
      expect(result).toEqual({
        message: 'Requested to start mission for robot 1',
      });
    });

    it('should handle service call failure for robotId "1"', () => {
      (service as any).realRos = { isConnected: true } as any;

      const mockCallService = jest.fn((request, callback) => {
        callback({ success: false, message: 'Failed to start mission' });
      });

      (ROSLIB.Service as jest.Mock).mockImplementation(() => ({
        callService: mockCallService,
      }));

      const consoleErrorSpy = jest.spyOn(console, 'error').mockImplementation();

      service.startRobotMission('1');

      expect(mockCallService).toHaveBeenCalledTimes(1);
      expect(consoleErrorSpy).toHaveBeenCalledWith(
        `Failed to start mission for robot 1: Failed to start mission`,
      );

      consoleErrorSpy.mockRestore();
    });

    it('should throw HttpException when robot is not connected', () => {
      (service as any).realRos = { isConnected: false } as any;

      expect(() => service.startRobotMission('1')).toThrow(
        new HttpException(
          'Cannot connect to Robot 1',
          HttpStatus.SERVICE_UNAVAILABLE,
        ),
      );
    });
  });

  describe('stopRobotMission', () => {
    it('should stop the mission successfully for robotId "3"', () => {
      (service as any).simulationRos = { isConnected: true } as any;

      const mockCallService = jest.fn((request, callback) => {
        callback({ success: true, message: 'Mission stopped' });
      });

      (ROSLIB.Service as jest.Mock).mockImplementation(() => ({
        callService: mockCallService,
      }));

      const result = service.stopRobotMission('3');

      expect(mockCallService).toHaveBeenCalledTimes(1);
      expect(result).toEqual({
        message: 'Requested to stop mission for robot 3',
      });
    });

    it('should handle service call failure for robotId "3"', () => {
      (service as any).simulationRos = { isConnected: true } as any;

      const mockCallService = jest.fn((request, callback) => {
        callback({ success: false, message: 'Failed to stop mission' });
      });

      (ROSLIB.Service as jest.Mock).mockImplementation(() => ({
        callService: mockCallService,
      }));

      const consoleErrorSpy = jest.spyOn(console, 'error').mockImplementation();

      service.stopRobotMission('3');

      expect(mockCallService).toHaveBeenCalledTimes(1);
      expect(consoleErrorSpy).toHaveBeenCalledWith(
        `Failed to stop mission for robot 3: Failed to stop mission`,
      );

      consoleErrorSpy.mockRestore();
    });

    it('should throw HttpException when robot is not connected', () => {
      (service as any).simulationRos = { isConnected: false } as any;

      expect(() => service.stopRobotMission('3')).toThrow(
        new HttpException(
          'Cannot connect to Robot 3',
          HttpStatus.SERVICE_UNAVAILABLE,
        ),
      );
    });
  });

  describe('identifyRobot', () => {
    it('should identify the robot successfully for robotId "2"', () => {
      (service as any).realRos = { isConnected: true } as any;

      const mockCallService = jest.fn((request, callback) => {
        callback({ success: true, message: 'Identified' });
      });

      (ROSLIB.Service as jest.Mock).mockImplementation(() => ({
        callService: mockCallService,
      }));

      const result = service.identifyRobot('2');

      expect(mockCallService).toHaveBeenCalledTimes(1);
      expect(result).toEqual({ message: 'Robot 2 is identifying itself' });
    });

    it('should handle service call failure for robotId "2"', () => {
      (service as any).realRos = { isConnected: true } as any;

      const mockCallService = jest.fn((request, callback) => {
        callback({ success: false, message: 'Failed to identify' });
      });

      (ROSLIB.Service as jest.Mock).mockImplementation(() => ({
        callService: mockCallService,
      }));

      const consoleErrorSpy = jest.spyOn(console, 'error').mockImplementation();

      service.identifyRobot('2');

      expect(mockCallService).toHaveBeenCalledTimes(1);
      expect(consoleErrorSpy).toHaveBeenCalledWith(
        `Failed to identify Robot 2: Failed to identify`,
      );

      consoleErrorSpy.mockRestore();
    });

    it('should throw HttpException when robot is not connected', () => {
      (service as any).realRos = { isConnected: false } as any;

      expect(() => service.identifyRobot('2')).toThrow(
        new HttpException(
          'Cannot connect to Robot 2',
          HttpStatus.SERVICE_UNAVAILABLE,
        ),
      );
    });
  });
});
