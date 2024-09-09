import { Module } from '@nestjs/common';
//import { AppController } from './app.controller';
import { AppService } from './app.service';
import { ConfigModule } from '@nestjs/config';
import { MissionController } from './mission/mission.controller';
import { RobotController } from './robot/robot.controller';
import { RosService } from './ros.service';

@Module({
  
  imports: [ConfigModule.forRoot({ isGlobal: true })],
  controllers: [MissionController, RobotController],
  providers: [RosService],
})
export class AppModule {}
