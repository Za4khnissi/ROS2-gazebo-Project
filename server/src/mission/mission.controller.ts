import { Controller, Param, Get, HttpStatus } from '@nestjs/common';
import { ApiTags, ApiOkResponse, ApiNotFoundResponse } from '@nestjs/swagger';
import { RosService } from '../ros.service';

@ApiTags('Mission')
@Controller('mission')
export class MissionController {
  constructor(private readonly rosService: RosService) {}

  @Get(':robotId/start')
  @ApiOkResponse({ description: 'Mission started successfully' })
  @ApiNotFoundResponse({ description: 'Robot not found' })
  async startMission(@Param('robotId') robotId: string) {
    const response = await this.rosService.startRobotMission(robotId);
    if (!response.success) {
      return {
        statusCode: HttpStatus.NOT_FOUND,
        message: `Robot ${robotId} not found or mission start failed`,
      };
    }
    return { statusCode: HttpStatus.OK, message: response.message };
  }

  @Get(':robotId/stop')
  @ApiOkResponse({ description: 'Mission stopped successfully' })
  @ApiNotFoundResponse({ description: 'Robot not found' })
  async stopMission(@Param('robotId') robotId: string) {
    const response = await this.rosService.stopRobotMission(robotId);
    if (!response.success) {
      return {
        statusCode: HttpStatus.NOT_FOUND,
        message: `Robot ${robotId} not found or mission stop failed`,
      };
    }
    return { statusCode: HttpStatus.OK, message: response.message };
  }
}
