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
  missions: any[] = [];

  constructor(private simService: RobotService, private webSocketService: WebSocketService) {}

  ngOnInit(): void {
    this.webSocketService.listen('syncUpdate').subscribe((data: any) => {
      this.logs.push(data);

      if (data.robot && data.event) {
        switch (data.robot) {
          case '1':
            this.robot1Status = data.event;
            break;
          case '2':
            this.robot2Status = data.event;
            break;
          case '3':
            this.driveMode3 = data.driveMode || this.driveMode3;
            break;
          case '4':
            this.driveMode4 = data.driveMode || this.driveMode4;
            break;
        }
      }
    });
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
      this.simService.getOldLogs().subscribe((missions) => {
        this.missions = missions.map((mission: any) => ({
          ...mission,
          expanded: false,
        }));
        this.showOldLogs = true;
      });
    } else {
      this.missions = [];
      this.showOldLogs = false;
    }
  }

  toggleMissionLogs(mission: any) {
    mission.expanded = !mission.expanded;
  }
}
