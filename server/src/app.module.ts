import { Module } from '@nestjs/common';
import { AppController } from './app.controller';
import { AppService } from './app.service';
import { MissionModule } from './mission/mission.module';

@Module({
  imports: [MissionModule],
  controllers: [AppController],
  providers: [AppService],
})
export class AppModule {}
