import { Component, OnInit } from '@angular/core';
import { SimulationService } from '@app/services/sim.service';
import { Router } from '@angular/router';

@Component({
  selector: 'app-simulation-robot',
  templateUrl: './simulation.component.html',
  styleUrls: ['./simulation.component.css']
})
export class SimulationComponent implements OnInit {
  robot1Status: string = 'Waiting';
  robot2Status: string = 'Waiting';
  simulationStatus: boolean = false;

  constructor(private simService: SimulationService, private router: Router) {}

  ngOnInit() {
    if (!this.simulationStatus) {
      this.launchSimulation();
    }
  }

  toggleSimulation() {
    if (!this.simulationStatus) {
      this.launchSimulation();
    } else {
      this.stopSimulation();
    }
  }

  launchSimulation() {
    if (!this.simulationStatus) {
      this.simService.launchSimulation().subscribe({
        next: (response) => {
          this.simulationStatus = true;
          console.log('Simulation launched:', response.message);
        },
        error: (error) => {
          console.error('Error launching simulation:', error);
        }
      });
    } else {
      console.log('Simulation is already running. No need to launch again.');
    }
  }

  stopSimulation() {
    if (this.simulationStatus) {
      this.simService.stopSimulation().subscribe({
        next: (response) => {
          this.simulationStatus = false; // Set simulation as stopped
          console.log('Simulation stopped:', response.message);
        },
        error: (error) => {
          console.error('Error stopping simulation:', error);
        }
      });
    } else {
      console.log('No simulation is running. No need to stop.');
    }
  }

  identifyRobot(robotId: number) {
    this.simService.identifyRobot(robotId).subscribe({
      next: (response) => {
        if (robotId === 1) {
          this.robot1Status = 'Identifying... ' + response.message;
        } else {
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
        if (robotId === 1) {
          this.robot1Status = response.message;
        } else {
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
        if (robotId === 1) {
          this.robot1Status = response.message;
        } else {
          this.robot2Status = response.message;
        }
      },
      error: (error) => {
        console.error('Error stopping mission:', error);
      }
    });
  }
}
