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

  constructor(private simService: SimulationService, private router: Router) {}

  ngOnInit() {
    this.simService.launchSimulation().subscribe({
      next: (response) => {
        console.log(response.message);
        this.router.navigate(['/control']);
      },
      error: (error) => {
        console.error('Error launching simulation:', error);
      }
    });
  }

  identifyRobot(robotId: number) {
    this.simService.identifyRobot(robotId).subscribe({
      next: (response) => {
        if (robotId === 1) {
          this.robot1Status = response.message;
        } else {
          this.robot2Status = response.message;
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
