import { Controller, Post, Param, Get } from '@nestjs/common';
import { AppService } from './app.service';

@Controller('robot')
export class AppController {
  constructor(private readonly appService: AppService) {}

  @Post('start-mission/simulation/:id')
  async startMission(@Param('id') id: number) {
    return await this.appService.startSimulationMission(id);
  }

  @Post('stop-mission/simulation/:id')
  async stopMission(@Param('id') id: number) {
    return await this.appService.stopSimulationMission(id);
  }

  @Get('launch-simulation')
  async launchSimulation() {
    return await this.appService.launchSimulation();
  }
}
