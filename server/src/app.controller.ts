import { Controller, Post, Param } from '@nestjs/common';
import { AppService } from './app.service';

@Controller('robot')
export class AppController {
  constructor(private readonly appService: AppService) {}

  @Post('identify/:type/:id')
  identifyRobot(@Param('type') type: string, @Param('id') id: number) {
    if (type === 'physical') {
      return this.appService.identifyPhysicalRobot(id);
    } else if (type === 'simulation') {
      return this.appService.identifySimulationRobot(id);
    }
  }

  @Post('start-mission/:type/:id')
  startMission(@Param('type') type: string, @Param('id') id: number) {
    if (type === 'physical') {
      return this.appService.startPhysicalMission(id);
    } else if (type === 'simulation') {
      return this.appService.startSimulationMission(id);
    }
  }

  @Post('stop-mission/:type/:id')
  stopMission(@Param('type') type: string, @Param('id') id: number) {
    if (type === 'physical') {
      return this.appService.stopPhysicalMission(id);
    } else if (type === 'simulation') {
      return this.appService.stopSimulationMission(id);
    }
  }
}
