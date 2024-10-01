import { Module } from '@nestjs/common';
import { ConfigModule } from '@nestjs/config';
import { MissionController } from './mission/mission.controller';
import { RobotController } from './robot/robot.controller';
import { RosService } from './ros.service';
import { SimulationController } from './simulation/simulation.controller';

@Module({
  
  imports: [ConfigModule.forRoot({ isGlobal: true })],
  controllers: [MissionController, RobotController, SimulationController],
  providers: [RosService],
})
export class AppModule {}
