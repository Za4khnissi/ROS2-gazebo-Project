import { Test, TestingModule } from '@nestjs/testing';
import { RobotController } from './robot.controller';

describe('RobotController', () => {
  let controller: RobotController;

  beforeEach(async () => {
    const module: TestingModule = await Test.createTestingModule({
      controllers: [RobotController],
    }).compile();

    controller = module.get<RobotController>(RobotController);
  });

  it('should be defined', () => {
    expect(controller).toBeDefined();
  });
});
