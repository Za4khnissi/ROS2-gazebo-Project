import {
  Controller,
  Post,
  Body,
  HttpException,
  HttpStatus,
} from '@nestjs/common';
import { RosService } from '../ros.service';
import {
  ApiTags,
  ApiOperation,
  ApiOkResponse,
  ApiBadRequestResponse,
} from '@nestjs/swagger';

@ApiTags('Simulation')
@Controller('simulation') // Updated to match Angular's expected route
export class SimulationController {
  constructor(private readonly rosService: RosService) {}

  @Post('start_ros')
  @ApiOperation({
    summary: 'Start the ROS system with the specified drive modes',
  })
  @ApiOkResponse({
    description: 'ROS started successfully and server connected',
  })
  @ApiBadRequestResponse({
    description: 'Invalid drive modes or user declined connection',
  })
  async startRos(@Body() body: { driveModes: { [key: string]: string } }) {
    try {
      const { driveModes } = body; // Extract the nested driveModes
      console.log('Received drive modes:', driveModes);
      // Validate the drive modes
      if (
        !driveModes['3'] ||
        !driveModes['4'] ||
        !['diff_drive', 'ackermann'].includes(driveModes['3']) ||
        !['diff_drive', 'ackermann'].includes(driveModes['4'])
      ) {
        throw new HttpException('Invalid drive modes', HttpStatus.BAD_REQUEST);
      }
      // Call the ROS service to start the system
      const response = await this.rosService.startRos(driveModes);
      return {
        statusCode: HttpStatus.OK,
        message: 'ROS started successfully',
        result: response,
      };
    } catch (error) {
      console.error('Failed to start ROS:', error);
      throw error instanceof HttpException
        ? error
        : new HttpException(
            'Failed to start ROS',
            HttpStatus.INTERNAL_SERVER_ERROR,
          );
    }
  }
}
