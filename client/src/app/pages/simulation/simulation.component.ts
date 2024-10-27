import { Component, OnInit } from '@angular/core';
import { RobotService } from '@app/services/robot.service';
import { NgFor, NgClass } from '@angular/common';

@Component({
  selector: 'app-simulation-robot',
  templateUrl: './simulation.component.html',
  styleUrls: ['./simulation.component.css'],
  standalone: true,
  imports: [NgFor, NgClass]
})
export class SimulationComponent implements OnInit {
  robot1Status: string = 'Waiting';
  robot2Status: string = 'Waiting';
  simulationStatus: boolean = false;
  driveMode3: string = 'Diff Drive';
  driveMode4: string = 'Diff Drive';

  logs: any[] = [];
  showOldLogs: boolean = false;

  constructor(private simService: RobotService) {}

  ngOnInit(): void {
    this.simService.listenForLogs().subscribe((log) => {
      log.isOld = false;
      this.logs.push(log);
    });
  }

  identifyRobot(robotId: number) {
    this.simService.identifyRobot(robotId).subscribe({
      error: (error) => {
        console.error('Error identifying robot:', error);
      }
    });
  }

  startMission(robotId: number) {
    this.simService.startMission(robotId).subscribe({
      error: (error) => {
        console.error('Error starting mission:', error);
      }
    });
  }

  stopMission(robotId: number) {
    this.simService.stopMission(robotId).subscribe({
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
      error: (error) => {
        console.error(`Error changing drive mode for Robot ${robotId}:`, error);
      }
    });
  }

  toggleOldLogs() {
    if (!this.showOldLogs) {
      // Load old logs and display them
      this.simService.getOldLogs().subscribe((logs) => {
        logs.forEach((log) => (log.isOld = true));
        this.logs = [...logs, ...this.logs];
        this.showOldLogs = true;
      });
    } else {
      // Hide old logs by filtering them out
      this.logs = this.logs.filter((log) => !log.isOld);
      this.showOldLogs = false;
    }
  }
}
