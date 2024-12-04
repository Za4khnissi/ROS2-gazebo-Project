import { Test, TestingModule } from '@nestjs/testing';
import { MissionController } from './mission.controller';
import { RosService } from '../ros.service';
import { DatabaseService } from '../database/database.service';
import { HttpStatus, HttpException } from '@nestjs/common';
import { MissionModel } from './mission.model';


describe('MissionController', () => {
  let controller: MissionController;
  let rosService: RosService;
  let databaseService: DatabaseService;

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
        {
          provide: DatabaseService,
          useValue: {
            getAllMissions: jest.fn(),
            getMissionById: jest.fn(),
            createMission: jest.fn(),
          },
        },
      ],
    }).compile();

    controller = module.get<MissionController>(MissionController);
    rosService = module.get<RosService>(RosService);
    databaseService = module.get<DatabaseService>(DatabaseService);
  });

  describe('startMission', () => {
    it('should return OK when mission starts successfully', async () => {
      const robotId = '1';
      const mockResponse = { success: true, message: `Started mission for robot ${robotId}` };
      jest.spyOn(rosService, 'startRobotMission').mockResolvedValue(mockResponse);

      const result = await controller.startMission(robotId);

      expect(rosService.startRobotMission).toHaveBeenCalledWith(robotId);
      expect(result).toEqual({
        statusCode: HttpStatus.OK,
        message: mockResponse.message,
      });
    });

    it('should return NOT_FOUND when robot is not found', async () => {
      const robotId = 'non-existent';
      const mockResponse = { success: false, message: 'Robot not found' };
      jest.spyOn(rosService, 'startRobotMission').mockResolvedValue(mockResponse);

      const result = await controller.startMission(robotId);

      expect(rosService.startRobotMission).toHaveBeenCalledWith(robotId);
      expect(result).toEqual({
        statusCode: HttpStatus.NOT_FOUND,
        message: mockResponse.message,
      });
    });
  });

  describe('stopMission', () => {
    it('should return OK when mission stops successfully', async () => {
      const robotId = '1';
      const mockResponse = { success: true, message: `Stopped mission for robot ${robotId}` };
      jest.spyOn(rosService, 'stopRobotMission').mockResolvedValue(mockResponse);

      const result = await controller.stopMission(robotId);

      expect(rosService.stopRobotMission).toHaveBeenCalledWith(robotId);
      expect(result).toEqual({
        statusCode: HttpStatus.OK,
        message: mockResponse.message,
      });
    });
  });

  describe('returnFromMission', () => {
    it('should return OK when return process starts successfully', async () => {
      const robotId = '1';
      const mockResponse = { success: true, message: `Return process started for robot ${robotId}` };
      jest.spyOn(rosService, 'stopRobotMission').mockResolvedValue(mockResponse);

      const result = await controller.returnFromMission(robotId);

      expect(rosService.stopRobotMission).toHaveBeenCalledWith(robotId, true);
      expect(result).toEqual({
        statusCode: HttpStatus.OK,
        message: mockResponse.message,
      });
    });
  });

  describe('getAllMissions', () => {
    it('should return a list of all missions', async () => {
      const mockMissions = [
        {
          _id: '1',
          dateDebut: new Date('2024-01-01T00:00:00Z'),
          dateFin: new Date('2024-01-02T00:00:00Z'),
          duration: 24,
          robots: ['robot1', 'robot2'],
          isPhysical: true,
          totalDistance: 100,
          mapData: { key: 'value' },
          logs: [],
        },
        {
          _id: '2',
          dateDebut: new Date('2024-02-01T00:00:00Z'),
          dateFin: new Date('2024-02-03T00:00:00Z'),
          duration: 48,
          robots: ['robot3'],
          isPhysical: false,
          totalDistance: 200,
          mapData: null,
          logs: [],
        },
      ];

      jest.spyOn(databaseService, 'getAllMissions').mockResolvedValue(mockMissions as MissionModel[]);
      const result = await controller.getAllMissions();

      expect(databaseService.getAllMissions).toHaveBeenCalled();
      expect(result).toEqual({
        statusCode: HttpStatus.OK,
        missions: mockMissions,
      });
    });
  });


  describe('getMissionById', () => {
    it('should return a mission by ID', async () => {
      const missionId = '1';
      const mockMission = {
        _id: missionId,
        dateDebut: new Date('2024-01-01T00:00:00Z'),
        dateFin: new Date('2024-01-02T00:00:00Z'),
        duration: 24,
        robots: ['robot1', 'robot2'],
        isPhysical: true,
        totalDistance: 100,
        mapData: { key: 'value' },
        logs: [],
      } as MissionModel;


      jest.spyOn(databaseService, 'getMissionById').mockResolvedValue(mockMission);
      const result = await controller.getMissionById(missionId);
      expect(databaseService.getMissionById).toHaveBeenCalledWith(missionId);


      expect(result).toEqual({
        statusCode: HttpStatus.OK,
        mission: mockMission,
      });
    });
  });


  describe('createMission', () => {
    it('should create a mission successfully', async () => {
      const missionDto = {
        dateDebut: new Date('2024-01-01T00:00:00Z'),
        robots: ['robot1', 'robot2'],
        isPhysical: true,
        dateFin: new Date('2024-01-02T00:00:00Z'),
        duration: 24,
        totalDistance: 100,
        mapData: { mapName: 'TestMap', resolution: 0.5 },
        logs: [
          {
            robotId: 'robot1',
            event: 'start',
            level: 'info',
            message: 'Mission started',
            timestamp: new Date('2024-01-01T00:00:00Z'),
          },
        ],
      } as MissionModel;

      const mockMission = {
        _id: '1',
        ...missionDto,
      } as MissionModel;

      jest.spyOn(databaseService, 'createMission').mockResolvedValue(mockMission);

      const result = await controller.createMission(missionDto);

      expect(databaseService.createMission).toHaveBeenCalledWith(missionDto);
      expect(result).toEqual({
        statusCode: HttpStatus.CREATED,
        mission: mockMission,
      });
    });
  });

});
