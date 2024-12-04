import { Test, TestingModule } from '@nestjs/testing';
import { LogsController } from './logs.controller';
import { RosService } from '../ros.service';

describe('LogsController', () => {
  let controller: LogsController;
  let mockRosService: Partial<RosService>;

  beforeEach(async () => {
    // Mocking the RosService methods used in LogsController
    mockRosService = {
      getOldMissions: jest.fn().mockReturnValue(['mission1', 'mission2']),
      getLastStatus: jest.fn().mockReturnValue('lastStatus'),
    };

    const module: TestingModule = await Test.createTestingModule({
      controllers: [LogsController],
      providers: [
        {
          provide: RosService,
          useValue: mockRosService,
        },
      ],
    }).compile();

    controller = module.get<LogsController>(LogsController);
  });

  it('should be defined', () => {
    expect(controller).toBeDefined();
  });

  it('should return old missions', () => {
    const result = controller.getOldMissions();
    expect(result).toEqual(['mission1', 'mission2']);
    expect(mockRosService.getOldMissions).toHaveBeenCalled();
  });

  it('should return the last status', () => {
    const result = controller.getLastStatus();
    expect(result).toBe('lastStatus');
    expect(mockRosService.getLastStatus).toHaveBeenCalled();
  });
});
