import { Controller, Param, Get, HttpStatus } from '@nestjs/common';
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
    const response = this.rosService.identifyRobot(robotId);
    if (!response) {
      return { statusCode: HttpStatus.NOT_FOUND, message: `Robot ${robotId} not found` };
    }
    return { statusCode: HttpStatus.OK, message: `Robot ${robotId} is identifying itself` };
  }
}
