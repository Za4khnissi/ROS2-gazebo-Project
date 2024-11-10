import { Controller, Param, Get,Post, Body, HttpStatus, HttpException ,InternalServerErrorException } from '@nestjs/common';
import { ApiTags, ApiOkResponse, ApiNotFoundResponse, ApiOperation,ApiCreatedResponse } from '@nestjs/swagger';
import { RosService } from '../ros.service';
import {DatabaseService} from '../database/database.service';
import { MissionModel } from './mission.model';


@ApiTags('Mission')
@Controller('mission')
export class MissionController {
  constructor(private readonly rosService: RosService, private readonly databaseService : DatabaseService) {}

  @Get(':robotId/start')
  @ApiOkResponse({ description: 'Mission started successfully' })
  @ApiNotFoundResponse({ description: 'Robot not found' })
  async startMission(@Param('robotId') robotId: string) {
    const response = await this.rosService.startRobotMission(robotId);
    if (!response.success) {
      return { statusCode: HttpStatus.NOT_FOUND, message: `Robot ${robotId} not found or mission start failed` };
    }
    return { statusCode: HttpStatus.OK, message: response.message };
  }

  @Get(':robotId/stop')
  @ApiOkResponse({ description: 'Mission stopped successfully' })
  @ApiNotFoundResponse({ description: 'Robot not found' })
  async stopMission(@Param('robotId') robotId: string) {
    const response = await this.rosService.stopRobotMission(robotId);
    if (!response.success) {
      return { statusCode: HttpStatus.NOT_FOUND, message: `Robot ${robotId} not found or mission stop failed` };
    }
    return { statusCode: HttpStatus.OK, message: response.message };
  }

  @Get(':robotId/return')
  @ApiOkResponse({ description: 'Return process started successfully' })
  @ApiNotFoundResponse({ description: 'Robot not found' })
  async returnFromMission(@Param('robotId') robotId: string) {
    const response = await this.rosService.stopRobotMission(robotId, true);
    if (!response.success) {
      return { statusCode: HttpStatus.NOT_FOUND, message: `Robot ${robotId} not found or return process failed` };
    }
    return { statusCode: HttpStatus.OK, message: response.message };
  }

  @Get()
@ApiOperation({ summary: 'Get all missions' })
@ApiOkResponse({ description: 'List of all missions' })
async getAllMissions() {
  const missions = await this.databaseService.getAllMissions();
  console.log('Missions from controller:', missions);  
  return { statusCode: HttpStatus.OK, missions };
}


  @Get(':id')
  @ApiOperation({ summary: 'Get a mission by ID' })
  @ApiOkResponse({ description: 'Mission found' })
  @ApiNotFoundResponse({ description: 'Mission not found' })
  async getMissionById(@Param('id') missionId: string) {
    const mission = await this.databaseService.getMissionById(missionId);
    return { statusCode: HttpStatus.OK, mission };
  }

  @Post()
  @ApiOperation({ summary: 'Create a new mission' })
  @ApiCreatedResponse({ description: 'Mission created successfully' })
  async createMission(@Body() missionDto: MissionModel) {
    try {
      const mission = await this.databaseService.createMission(missionDto);
      return { statusCode: HttpStatus.CREATED, mission };
    } catch (error) {
      throw new HttpException('Failed to create mission', HttpStatus.BAD_REQUEST);
    }
  }

}
