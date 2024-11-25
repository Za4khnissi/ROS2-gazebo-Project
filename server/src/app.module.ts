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
    MongooseModule.forRoot(
      'mongodb+srv://axellestevialetieutchemeni:projet3@cluster0.urbxk.mongodb.net/?retryWrites=true&w=majority&appName=Cluster0',
      {
        serverSelectionTimeoutMS: 30000, // Délai pour la sélection du serveur à 30 secondes
        socketTimeoutMS: 45000, // Délai pour les opérations socket à 45 secondes
        connectTimeoutMS: 30000, // Délai pour établir la connexion à 30 secondes
      },
    ),
    MongooseModule.forFeature([{ name: Mission.name, schema: MissionSchema }]),
  ],
  controllers: [
    MissionController,
    RobotController,
    SimulationController,
    LogsController,
  ],
  providers: [RosService, SyncGateway, DatabaseService],
})
export class AppModule {}
