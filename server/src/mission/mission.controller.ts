import { Controller, Param, Get, HttpStatus } from '@nestjs/common';
import { ApiTags, ApiOkResponse, ApiNotFoundResponse, ApiOperation } from '@nestjs/swagger';
import { RosService } from '../ros.service';

@ApiTags('Mission')
@Controller('mission')
export class MissionController {
  constructor(private readonly rosService: RosService) {}

  @Get(':robotId/start')
  @ApiOkResponse({ description: 'Mission started successfully' })
  @ApiNotFoundResponse({ description: 'Robot not found' })
  startMission(@Param('robotId') robotId: string) {
    const response = this.rosService.startRobotMission(robotId);
    if (!response) {
      return { statusCode: HttpStatus.NOT_FOUND, message: `Robot ${robotId} not found` };
    }
    return { statusCode: HttpStatus.OK, message: `Mission started for robot ${robotId}` };
  }

  @Get(':robotId/stop')
  @ApiOkResponse({ description: 'Mission stopped successfully' })
  @ApiNotFoundResponse({ description: 'Robot not found' })
  stopMission(@Param('robotId') robotId: string) {
    const response = this.rosService.stopRobotMission(robotId);
    if (!response) {
      return { statusCode: HttpStatus.NOT_FOUND, message: `Robot ${robotId} not found` };
    }
    return { statusCode: HttpStatus.OK, message: `Mission stopped for robot ${robotId}` };
  }
}
