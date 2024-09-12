import { Controller, Param, Get, HttpStatus, HttpException } from '@nestjs/common';
import { ApiTags, ApiOkResponse, ApiNotFoundResponse, ApiOperation } from '@nestjs/swagger';
import { RosService } from '../ros.service';

@ApiTags('Robot')
@Controller('robot')
export class RobotController {
  constructor(private readonly rosService: RosService) {}

  @Get(':robotId/identify')
  @ApiOperation({ summary: 'Identify a specific robot by playing a sound' })
  @ApiOkResponse({ description: 'Robot identified successfully' })
  @ApiNotFoundResponse({ description: 'Robot not found' })
  identifyRobot(@Param('robotId') robotId: string) {
    try {
      const response = this.rosService.identifyRobot(robotId);
      if (!response) {
        throw new HttpException(`Robot ${robotId} not found`, HttpStatus.NOT_FOUND);
      }
      return { statusCode: HttpStatus.OK, message: response.message };
    } catch (error) {
      throw new HttpException(error.message, HttpStatus.INTERNAL_SERVER_ERROR);
    }
  }
}
