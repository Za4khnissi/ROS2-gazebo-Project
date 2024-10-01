import { Component } from '@angular/core';
import { SimulationService } from '@app/services/sim.service';
import { Router } from '@angular/router';

@Component({
  selector: 'app-simulation-robot',
  templateUrl: './simulation.component.html',
  styleUrls: ['./simulation.component.css']
})
export class SimulationComponent {
  robot1Status: string = 'Waiting';
  robot2Status: string = 'Waiting';
  simulationStatus: boolean = false;

  // Initial drive modes
  driveMode3: string = 'Diff Drive';
  driveMode4: string = 'Diff Drive';

  constructor(private simService: SimulationService, private router: Router) {}

  identifyRobot(robotId: number) {
    this.simService.identifyRobot(robotId).subscribe({
      next: (response) => {
        if (robotId === 3) {
          this.robot1Status = 'Identifying... ' + response.message;
        } else if (robotId === 4) {
          this.robot2Status = 'Identifying... ' + response.message;
        }
      },
      error: (error) => {
        console.error('Error identifying robot:', error);
      }
    });
  }

  startMission(robotId: number) {
    this.simService.startMission(robotId).subscribe({
      next: (response) => {
        if (robotId === 3) {
          this.robot1Status = response.message;
        } else if (robotId === 4) {
          this.robot2Status = response.message;
        }
      },
      error: (error) => {
        console.error('Error starting mission:', error);
      }
    });
  }

  stopMission(robotId: number) {
    this.simService.stopMission(robotId).subscribe({
      next: (response) => {
        if (robotId === 3) {
          this.robot1Status = response.message;
        } else if (robotId === 4) {
          this.robot2Status = response.message;
        }
      },
      error: (error) => {
        console.error('Error stopping mission:', error);
      }
    });
  }

  toggleDriveMode(robotId: number) {
    if (robotId === 3) {
      this.driveMode3 = this.driveMode3 === 'Diff Drive' ? 'Ackermann' : 'Diff Drive';
    } else if (robotId === 4) {
      this.driveMode4 = this.driveMode4 === 'Diff Drive' ? 'Ackermann' : 'Diff Drive';
    }
  }

  launchSimulation() {
    const mode3 = this.driveMode3.toLowerCase().replace(' ', '_');
    const mode4 = this.driveMode4.toLowerCase().replace(' ', '_');
    this.simService.launchSimulation(mode3, mode4).subscribe({
      next: (response) => {
        console.log('Simulation launched:', response);
        this.simulationStatus = true;
      },
      error: (error) => {
        console.error('Error launching simulation:', error);
      }
    });
  }
}
