import { Module } from '@nestjs/common';
import { ConfigModule } from '@nestjs/config';
import { MissionController } from './mission/mission.controller';
import { MongooseModule } from '@nestjs/mongoose';
import { RobotController } from './robot/robot.controller';
import { RosService } from './ros.service';
import { SimulationController } from './simulation/simulation.controller';
import { LogsController } from './logs/logs.controller';
import { SyncGateway } from './sync/sync.gateway';
import { Mission, MissionSchema } from './mission/mission.model';
import { DatabaseService } from './database/database.service'; 


@Module({
  
  imports: [
    ConfigModule.forRoot({ isGlobal: true }),
    MongooseModule.forRoot('mongodb+srv://axellestevialetieutchemeni:projet3@cluster0.urbxk.mongodb.net/?retryWrites=true&w=majority&appName=Cluster0'), 
    MongooseModule.forFeature([{ name: Mission.name, schema: MissionSchema }]), 
  ],

  controllers: [MissionController, RobotController, SimulationController, LogsController],
  providers: [RosService, SyncGateway,DatabaseService],
})
export class AppModule {}
