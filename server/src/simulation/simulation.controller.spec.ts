import { Test, TestingModule } from '@nestjs/testing';
import { SimulationController } from './simulation.controller';
import { RosService } from '../ros.service';
import { HttpStatus, HttpException } from '@nestjs/common';

describe('SimulationController', () => {
  let controller: SimulationController;
  let rosService: RosService;

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      controllers: [SimulationController],
      providers: [
        {
          provide: RosService,
          useValue: {
            startRos: jest.fn(),
          },
        },
      ],
    }).compile();

    controller = module.get<SimulationController>(SimulationController);
    rosService = module.get<RosService>(RosService);
  });

  describe('startRos', () => {
    it('should start ROS successfully with valid drive modes', async () => {
      const driveModes = {
        '3': 'diff_drive',
        '4': 'ackermann',
      };
      const mockResponse = { success: true, message: 'ROS started' };

      jest.spyOn(rosService, 'startRos').mockResolvedValue(mockResponse);

      const result = await controller.startRos({ driveModes });

      expect(rosService.startRos).toHaveBeenCalledWith(driveModes);
      expect(result).toEqual({
        statusCode: HttpStatus.OK,
        message: 'ROS started successfully',
        result: mockResponse,
      });
    });

    it('should throw BAD_REQUEST when drive modes are invalid', async () => {
      const invalidDriveModes = {
        '3': 'invalid_mode',
        '4': 'ackermann',
      };

      await expect(controller.startRos({ driveModes: invalidDriveModes })).rejects.toThrow(
        new HttpException('Invalid drive modes', HttpStatus.BAD_REQUEST),
      );

      expect(rosService.startRos).not.toHaveBeenCalled();
    });

    it('should throw BAD_REQUEST when drive modes are missing', async () => {
      const missingDriveModes = {
        '3': 'diff_drive',
      };

      await expect(controller.startRos({ driveModes: missingDriveModes })).rejects.toThrow(
        new HttpException('Invalid drive modes', HttpStatus.BAD_REQUEST),
      );

      expect(rosService.startRos).not.toHaveBeenCalled();
    });

    it('should throw INTERNAL_SERVER_ERROR when ROS service fails', async () => {
      const driveModes = {
        '3': 'diff_drive',
        '4': 'ackermann',
      };
      const error = new Error('ROS service failed');

      jest.spyOn(rosService, 'startRos').mockRejectedValue(error);

      await expect(controller.startRos({ driveModes })).rejects.toThrow(
        new HttpException('Failed to start ROS', HttpStatus.INTERNAL_SERVER_ERROR),
      );

      expect(rosService.startRos).toHaveBeenCalledWith(driveModes);
    });
  });
});

