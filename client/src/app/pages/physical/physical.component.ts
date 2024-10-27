import { Component, OnInit } from '@angular/core';
import { RobotService } from '@app/services/robot.service';
import { NgFor, NgClass } from '@angular/common';

@Component({
  selector: 'app-physical-robot',
  templateUrl: './physical.component.html',
  styleUrls: ['./physical.component.css'],
  standalone: true,
  imports: [NgFor, NgClass]
})
export class PhysicalRobotComponent implements OnInit {
  robot1Status: string = 'Waiting';
  robot2Status: string = 'Waiting';

  logs: any[] = [];
  showOldLogs: boolean = false;

  constructor(private robotService: RobotService) {}

  ngOnInit(): void {
    this.robotService.listenForLogs().subscribe((log) => {
      log.isOld = false;
      this.logs.push(log);
    });
  }

  identifyRobot(robotId: number) {
    this.robotService.identifyRobot(robotId).subscribe({
      error: (error) => {
        console.error('Error identifying robot:', error);
      }
    });
  }

  startMission(robotId: number) {
    this.robotService.startMission(robotId).subscribe({
      error: (error) => {
        console.error('Error starting mission:', error);
      }
    });
  }

  stopMission(robotId: number) {
    this.robotService.stopMission(robotId).subscribe({
      error: (error) => {
        console.error('Error stopping mission:', error);
      }
    });
  }

  toggleOldLogs() {
    if (!this.showOldLogs) {
      // Load old logs and display them
      this.robotService.getOldLogs().subscribe((logs) => {
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
