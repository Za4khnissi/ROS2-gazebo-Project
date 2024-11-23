// logs.controller.ts
import { Controller, Get } from '@nestjs/common';
import { RosService } from '../ros.service';

@Controller('logs')
export class LogsController {
  constructor(private readonly rosService: RosService) {}

  @Get('old')
  getOldMissions() {
    return this.rosService.getOldMissions();
  }

  @Get('/last')
  getLastStatus() {
    return this.rosService.getLastStatus();
  }
}
