import { Test, TestingModule } from '@nestjs/testing';
import { RobotController } from './robot.controller';
import { RosService } from '../ros.service';
import { HttpStatus, HttpException } from '@nestjs/common';

describe('RobotController', () => {
  let controller: RobotController;
  let rosService: RosService;

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      controllers: [RobotController],
      providers: [
        {
          provide: RosService,
          useValue: {
            identifyRobot: jest.fn(),
          },
        },
      ],
    }).compile();

    controller = module.get<RobotController>(RobotController);
    rosService = module.get<RosService>(RosService);
  });

  describe('identifyRobot', () => {
    it('should return OK when robot identifies itself successfully', async () => {
      const robotId = '1';
      const mockResponse = { success: true, message: `Robot ${robotId} is identifying itself` };

      jest.spyOn(rosService, 'identifyRobot').mockResolvedValue(mockResponse);

      const result = await controller.identifyRobot(robotId);

      expect(rosService.identifyRobot).toHaveBeenCalledWith(robotId);
      expect(result).toEqual({
        statusCode: HttpStatus.OK,
        message: `Robot ${robotId} is identigying itself`,
      });
    });

    it('should throw NOT_FOUND when robot is not found', async () => {
      const robotId = 'non-existent';
      const mockResponse = {
        success: false,
        message: `Robot ${robotId} not found`,
      };

      jest.spyOn(rosService, 'identifyRobot').mockResolvedValue(mockResponse);

      await expect(controller.identifyRobot(robotId)).rejects.toThrow(
        new HttpException(
          `Failed to identify Robot ${robotId}`,
          HttpStatus.NOT_FOUND,
        ),
      );
      expect(rosService.identifyRobot).toHaveBeenCalledWith(robotId);
    });

    it('should throw INTERNAL_SERVER_ERROR when service throws an exception', async () => {
      const robotId = 'error';
      const error = new Error('Service error');

      jest.spyOn(rosService, 'identifyRobot').mockRejectedValue(error);

      await expect(controller.identifyRobot(robotId)).rejects.toThrow(
        new HttpException(error.message, HttpStatus.INTERNAL_SERVER_ERROR),
      );
      expect(rosService.identifyRobot).toHaveBeenCalledWith(robotId);
    });
  });
});
