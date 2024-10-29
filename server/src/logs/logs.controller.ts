// logs.controller.ts
import { Controller, Get, Sse } from '@nestjs/common';
import { RosService } from '../ros.service';
import { map, Observable } from 'rxjs';

@Controller('logs')
export class LogsController {
  constructor(private readonly rosService: RosService) {}

  @Get('old')
  getOldMissions() {
    return this.rosService.getOldMissions();
  }
}