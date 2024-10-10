import { Controller, Param, Get, HttpStatus, InternalServerErrorException } from '@nestjs/common';
import { ApiTags, ApiOkResponse, ApiNotFoundResponse, ApiOperation } from '@nestjs/swagger';
import { RosService } from '../ros.service';

@ApiTags('Mission')
@Controller('mission')
export class MissionController {
  constructor(private readonly rosService: RosService) {}

  @Get(':robotId/start')
  @ApiOkResponse({ description: 'Mission started successfully' })
  @ApiNotFoundResponse({ description: 'Robot not found' })
  async startMission(@Param('robotId') robotId: string) {
    try {
      const response = await this.rosService.startRobotMission(robotId);
      if (!response) {
        return {
          statusCode: HttpStatus.NOT_FOUND,
          message: `Robot ${robotId} not found`,
        };
      }
      return {
        statusCode: HttpStatus.OK,
        message: `Mission started for robot ${robotId}`,
      };
    } catch (error) {
      // Log the error if necessary
      console.error(`Failed to start mission for robot ${robotId}:`, error);
      throw new InternalServerErrorException(
        `Failed to start mission for robot ${robotId}. Please try again later.`,
      );
    }
  }

  @Get(':robotId/stop')
  @ApiOkResponse({ description: 'Mission stopped successfully' })
  @ApiNotFoundResponse({ description: 'Robot not found' })
  async stopMission(@Param('robotId') robotId: string) {
    try {
      const response = await this.rosService.stopRobotMission(robotId);
      if (!response) {
        return {
          statusCode: HttpStatus.NOT_FOUND,
          message: `Robot ${robotId} not found`,
        };
      }
      return {
        statusCode: HttpStatus.OK,
        message: `Mission stopped for robot ${robotId}`,
      };
    } catch (error) {
      // Log the error if necessary
      console.error(`Failed to stop mission for robot ${robotId}:`, error);
      throw new InternalServerErrorException(
        `Failed to stop mission for robot ${robotId}. Please try again later.`,
      );
    }
  }
}
