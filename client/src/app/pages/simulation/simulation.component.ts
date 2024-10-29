import { Component, OnInit, ChangeDetectorRef } from '@angular/core';
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

  constructor(
    private simService: RobotService, 
    private webSocketService: WebSocketService,
    private cdr: ChangeDetectorRef
  ) {}

  ngOnInit(): void {
    this.webSocketService.listen('syncUpdate').subscribe((data: any) => {
      console.log('Received event:', data);
      this.logs.push(data);
  
      if (data.robot && data.event) {
        const status = this.getStatusMessage(data.event);
        switch (data.robot) {
          case '3':
            this.robot1Status = status;
            console.log('Updated robot1Status to:', this.robot1Status);
            break;
          case '4':
            this.robot2Status = status;
            console.log('Updated robot2Status to:', this.robot2Status);
            break;
        }
        this.cdr.detectChanges();
      }
    });
  }
  

  private getStatusMessage(event: string): string {
    switch (event) {
      case 'identifying':
        return 'Identifying';
      case 'identified':
        return 'Identified';
      case 'identification_failed':
        return 'Identification Failed';
      case 'mission_started':
        return 'Moving';
      case 'mission_stopped':
        return 'Stopped';
      case 'mission_failed':
        return 'Mission Start Failed';
      case 'mission_stop_failed':
        return 'Mission Stop Failed';
      default:
        return 'Waiting';
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
