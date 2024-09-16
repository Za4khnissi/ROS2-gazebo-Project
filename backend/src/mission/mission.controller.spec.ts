import { Test, TestingModule } from '@nestjs/testing';
import { MissionController } from './mission.controller';
import { RosService } from '../ros.service';
import { HttpStatus, HttpException } from '@nestjs/common';

describe('MissionController', () => {
  let controller: MissionController;
  let rosService: RosService;

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      controllers: [MissionController],
      providers: [
        {
          provide: RosService,
          useValue: {
            startRobotMission: jest.fn(),
            stopRobotMission: jest.fn(),
          },
        },
      ],
    }).compile();

    controller = module.get<MissionController>(MissionController);
    rosService = module.get<RosService>(RosService);
  });

  describe('startMission', () => {
    it('should return OK when mission starts successfully', () => {
      const robotId = '1';
      const mockResponse = { message: `Started mission for robot ${robotId}` };
      jest.spyOn(rosService, 'startRobotMission').mockReturnValue(mockResponse);

      const result = controller.startMission(robotId);

      expect(rosService.startRobotMission).toHaveBeenCalledWith(robotId);
      expect(result).toEqual({
        statusCode: HttpStatus.OK,
        message: `Mission started for robot ${robotId}`,
      });
    });

    it('should return NOT_FOUND when robot is not found', () => {
      const robotId = 'non-existent';
      jest.spyOn(rosService, 'startRobotMission').mockReturnValue(null);

      const result = controller.startMission(robotId);

      expect(rosService.startRobotMission).toHaveBeenCalledWith(robotId);
      expect(result).toEqual({
        statusCode: HttpStatus.NOT_FOUND,
        message: `Robot ${robotId} not found`,
      });
    });

    it('should throw HttpException when service throws an exception', () => {
      const robotId = 'error';
      const error = new HttpException('Service error', HttpStatus.INTERNAL_SERVER_ERROR);
      jest.spyOn(rosService, 'startRobotMission').mockImplementation(() => {
        throw error;
      });

      expect(() => controller.startMission(robotId)).toThrow(HttpException);
    });
  });

  describe('stopMission', () => {
    it('should return OK when mission stops successfully', () => {
      const robotId = '1';
      const mockResponse = { message: `Stopped mission for robot ${robotId}` };
      jest.spyOn(rosService, 'stopRobotMission').mockReturnValue(mockResponse);

      const result = controller.stopMission(robotId);

      expect(rosService.stopRobotMission).toHaveBeenCalledWith(robotId);
      expect(result).toEqual({
        statusCode: HttpStatus.OK,
        message: `Mission stopped for robot ${robotId}`,
      });
    });

    it('should return NOT_FOUND when robot is not found', () => {
      const robotId = 'non-existent';
      jest.spyOn(rosService, 'stopRobotMission').mockReturnValue(null);

      const result = controller.stopMission(robotId);

      expect(rosService.stopRobotMission).toHaveBeenCalledWith(robotId);
      expect(result).toEqual({
        statusCode: HttpStatus.NOT_FOUND,
        message: `Robot ${robotId} not found`,
      });
    });

    it('should throw HttpException when service throws an exception', () => {
      const robotId = 'error';
      const error = new HttpException('Service error', HttpStatus.INTERNAL_SERVER_ERROR);
      jest.spyOn(rosService, 'stopRobotMission').mockImplementation(() => {
        throw error;
      });

      expect(() => controller.stopMission(robotId)).toThrow(HttpException);
    });
  });
});
