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

  @Sse('stream')
  getLogStream(): Observable<MessageEvent> {
    return this.rosService.getLogStream().pipe(
      map(log => ({ data: log } as MessageEvent))
    );
  }
}