import { Component, OnInit } from '@angular/core';
import { RobotService } from '@app/services/robot.service';
import { WebSocketService } from '@app/services/web-socket.service';
import { NgFor, NgClass, NgIf } from '@angular/common';

@Component({
  selector: 'app-simulation-robot',
  templateUrl: './simulation.component.html',
  styleUrls: ['./simulation.component.css'],
  standalone: true,
  imports: [NgFor, NgClass, NgIf],
})
export class SimulationComponent implements OnInit {
  robot1Status: string = 'Waiting';
  robot2Status: string = 'Waiting';
  simulationStatus: boolean = false;
  driveMode3: string = 'Diff Drive';
  driveMode4: string = 'Diff Drive';
  driveModeAvailable: boolean = false;

  logs: any[] = [];
  showOldLogs: boolean = false;

  constructor(private simService: RobotService, private webSocketService: WebSocketService) {}

  ngOnInit(): void {
    this.simService.listenForLogs().subscribe((log) => {
      log.isOld = false;
      this.logs.push(log);
    });

    this.webSocketService.listen('syncUpdate').subscribe((data) => {
      console.log('Received sync update:', data);
      this.handleSyncUpdate(data);
    });
  }

  private handleSyncUpdate(data: any) {
    this.logs.push(data);
    if (data.robot && data.event) {
      if (data.robot === '1') this.robot1Status = data.event;
      if (data.robot === '2') this.robot2Status = data.event;
      if (data.robot === '3') this.driveMode3 = data.driveMode || this.driveMode3;
      if (data.robot === '4') this.driveMode4 = data.driveMode || this.driveMode4;
    }
  }

  identifyRobot(robotId: number) {
    this.simService.identifyRobot(robotId).subscribe();
  }

  startMission(robotId: number) {
    this.simService.startMission(robotId).subscribe();
  }

  stopMission(robotId: number) {
    this.simService.stopMission(robotId).subscribe();
  }

  toggleDriveMode(robotId: number) {
    let newDriveMode = robotId === 3 ? this.driveMode3 : this.driveMode4;
    newDriveMode = newDriveMode === 'Diff Drive' ? 'Ackermann' : 'Diff Drive';

    this.simService.changeDriveMode(robotId, newDriveMode.toLowerCase().replace(' ', '_')).subscribe();
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
