import { Controller, Post, Body, HttpException, HttpStatus } from '@nestjs/common';
import { RosService } from '../ros.service';
import { ApiTags, ApiOperation, ApiOkResponse, ApiBadRequestResponse, ApiInternalServerErrorResponse } from '@nestjs/swagger';

@ApiTags('Simulation')
@Controller('start-simulation')
export class SimulationController {
  constructor(private readonly rosService: RosService) {}

  @Post()
  @ApiOperation({ summary: 'Start simulation with specified drive modes' })
  @ApiOkResponse({ description: 'Simulation launched successfully' })
  @ApiBadRequestResponse({ description: 'Bad Request: Missing drive modes' })
  @ApiInternalServerErrorResponse({ description: 'Internal Server Error: Failed to launch simulation' })
  async startSimulation(@Body() body: { drive_mode_3: string, drive_mode_4: string }) {
    const { drive_mode_3, drive_mode_4 } = body;
    
    if (!drive_mode_3 || !drive_mode_4) {
      throw new HttpException('Drive modes for both robots must be provided', HttpStatus.BAD_REQUEST);
    }
    
    try {
      const result = await this.rosService.launchSimulation(drive_mode_3, drive_mode_4);
      return { statusCode: HttpStatus.OK, message: 'Simulation launched successfully', result };
    } catch (error) {
      console.error('Error launching simulation:', error);
      throw new HttpException('Failed to launch simulation', HttpStatus.INTERNAL_SERVER_ERROR);
    }
  }
}
