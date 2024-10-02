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
    let newDriveMode = '';
    if (robotId === 3) {
      this.driveMode3 = this.driveMode3 === 'Diff Drive' ? 'Ackermann' : 'Diff Drive';
      newDriveMode = this.driveMode3;
    } else if (robotId === 4) {
      this.driveMode4 = this.driveMode4 === 'Diff Drive' ? 'Ackermann' : 'Diff Drive';
      newDriveMode = this.driveMode4;
    }

    // Send drive mode change request
    this.simService.changeDriveMode(robotId, newDriveMode.toLowerCase().replace(' ', '_')).subscribe({
      next: (response) => {
        console.log(`Drive mode for Robot ${robotId} changed to ${newDriveMode}`);
      },
      error: (error) => {
        console.error(`Error changing drive mode for Robot ${robotId}:`, error);
      }
    });
  }
}
