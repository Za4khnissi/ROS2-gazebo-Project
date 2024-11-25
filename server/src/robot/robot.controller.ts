import {
  Controller,
  Param,
  Get,
  HttpStatus,
  HttpException,
} from '@nestjs/common';
import {
  ApiTags,
  ApiOkResponse,
  ApiNotFoundResponse,
  ApiOperation,
} from '@nestjs/swagger';
import { RosService } from '../ros.service';

@ApiTags('Robot')
@Controller('robot')
export class RobotController {
  constructor(private readonly rosService: RosService) {}

  @Get(':robotId/identify')
  @ApiOperation({ summary: 'Identify a specific robot by playing a sound' })
  @ApiOkResponse({ description: 'Robot identified successfully' })
  @ApiNotFoundResponse({ description: 'Robot not found' })
  async identifyRobot(@Param('robotId') robotId: string) {
    try {
      const response = await this.rosService.identifyRobot(robotId);
      if (!response.success) {
        throw new HttpException(
          `Failed to identify Robot ${robotId}`,
          HttpStatus.NOT_FOUND,
        );
      }
      return {
        statusCode: HttpStatus.OK,
        message: `Robot ${robotId} is identigying itself`,
      };
    } catch (error) {
      throw new HttpException(error.message, HttpStatus.INTERNAL_SERVER_ERROR);
    }
  }
}
