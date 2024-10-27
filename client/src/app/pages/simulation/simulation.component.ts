import { Component, OnInit } from '@angular/core';
import { RobotService } from '@app/services/robot.service';
import { Router } from '@angular/router';
import { NgFor } from '@angular/common';

@Component({
  selector: 'app-simulation-robot',
  standalone: true,
  templateUrl: './simulation.component.html',
  styleUrls: ['./simulation.component.css'],
  imports: [NgFor]
})
export class SimulationComponent implements OnInit {
  robot1Status: string = 'Waiting';
  robot2Status: string = 'Waiting';
  simulationStatus: boolean = false;
  driveMode3: string = 'Diff Drive';
  driveMode4: string = 'Diff Drive';

  logs: any[] = [];

  constructor(private simService: RobotService, private router: Router) {}

  ngOnInit(): void {
    this.simService.listenForLogs().subscribe((log) => {
      console.log('New log received:', log);
      this.logs.push(log);
    });
  }

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

    this.simService.changeDriveMode(robotId, newDriveMode.toLowerCase().replace(' ', '_')).subscribe({
      next: (response) => {
        console.log(`Drive mode for Robot ${robotId} changed to ${newDriveMode}`);
      },
      error: (error) => {
        console.error(`Error changing drive mode for Robot ${robotId}:`, error);
      }
    });
  }

  loadOldLogs() {
    this.simService.getOldLogs().subscribe((logs) => {
      this.logs = logs;
    });
  }
}
