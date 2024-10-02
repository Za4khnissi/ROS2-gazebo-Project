import { Controller, Post, Param, Body, HttpException, HttpStatus } from '@nestjs/common';
import { RosService } from '../ros.service';
import { ApiTags, ApiOperation, ApiOkResponse, ApiBadRequestResponse, ApiInternalServerErrorResponse } from '@nestjs/swagger';

@ApiTags('Simulation')
@Controller('robot')
export class SimulationController {
  constructor(private readonly rosService: RosService) {}

  @Post(':robotId/change_drive_mode')
  @ApiOperation({ summary: 'Change drive mode for a specific robot during simulation' })
  @ApiOkResponse({ description: 'Drive mode changed successfully' })
  @ApiBadRequestResponse({ description: 'Bad Request: Invalid Robot ID or drive mode' })
  @ApiInternalServerErrorResponse({ description: 'Internal Server Error: Failed to change drive mode' })
  async changeDriveMode(
    @Param('robotId') robotId: string,
    @Body() body: { drive_mode: string }
  ) {
    const { drive_mode } = body;
    
    if (!drive_mode || !['diff_drive', 'ackermann'].includes(drive_mode)) {
      throw new HttpException('Invalid drive mode', HttpStatus.BAD_REQUEST);
    }
    
    try {
      const result = await this.rosService.changeDriveMode(robotId, drive_mode);
      return { statusCode: HttpStatus.OK, message: `Drive mode for Robot ${robotId} changed to ${drive_mode}`, result };
    } catch (error) {
      console.error(`Error changing drive mode for Robot ${robotId}:`, error);
      throw new HttpException('Failed to change drive mode', HttpStatus.INTERNAL_SERVER_ERROR);
    }
  }
}
