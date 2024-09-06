import { Injectable } from '@nestjs/common';

@Injectable()
export class AppService {
  identifyPhysicalRobot(id: number): { message: string } {
    return { message: `Physical Robot ${id} identified.` };
  }
  identifySimulationRobot(id: number): { message: string } {
    return { message: `Simulation Robot ${id} identified.` };
  }
  startPhysicalMission(id: number): { message: string } {
    return { message: `Physical Robot ${id} mission started.` };
  }
  startSimulationMission(id: number): { message: string } {
    return { message: `Simulation Robot ${id} mission started.` };
  }
  stopPhysicalMission(id: number): { message: string } {
    return { message: `Physical Robot ${id} mission stopped.` };
  }
  stopSimulationMission(id: number): { message: string } {
    return { message: `Simulation Robot ${id} mission stopped.` };
  }
}
