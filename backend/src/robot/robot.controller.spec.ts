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
    it('should return OK when robot identifies itself successfully', () => {
      const robotId = '1';
      const mockResponse = { message: `Robot ${robotId} is identifying itself` };
      jest.spyOn(rosService, 'identifyRobot').mockReturnValue(mockResponse);

      const result = controller.identifyRobot(robotId);

      expect(rosService.identifyRobot).toHaveBeenCalledWith(robotId);
      expect(result).toEqual({
        statusCode: HttpStatus.OK,
        message: `Robot ${robotId} is identifying itself`,
      });
    });

    it('should return NOT_FOUND when robot is not found', () => {
      const robotId = 'non-existent';
      jest.spyOn(rosService, 'identifyRobot').mockReturnValue(null);

      const result = controller.identifyRobot(robotId);

      expect(rosService.identifyRobot).toHaveBeenCalledWith(robotId);
      expect(result).toEqual({
        statusCode: HttpStatus.NOT_FOUND,
        message: `Robot ${robotId} not found`,
      });
    });

    it('should throw HttpException when service throws an exception', () => {
      const robotId = 'error';
      const error = new HttpException('Service error', HttpStatus.INTERNAL_SERVER_ERROR);
      jest.spyOn(rosService, 'identifyRobot').mockImplementation(() => {
        throw error;
      });

      expect(() => controller.identifyRobot(robotId)).toThrow(HttpException);
    });
  });
});
