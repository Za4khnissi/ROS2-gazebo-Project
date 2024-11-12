import { Component, OnInit, ChangeDetectorRef } from '@angular/core';
import { RobotService } from '@app/services/robot.service';
import { WebSocketService } from '@app/services/web-socket.service';
import { NgFor, NgClass, NgIf } from '@angular/common';
import { Router } from '@angular/router';

@Component({
  selector: 'app-physical-robot',
  templateUrl: './physical.component.html',
  styleUrls: ['./physical.component.css'],
  standalone: true,
  imports: [NgFor, NgClass, NgIf],
})
export class PhysicalRobotComponent implements OnInit {
  robot1Status: string = 'Waiting';
  robot2Status: string = 'Waiting';

  logs: any[] = [];
  showOldLogs: boolean = false;
  missions: any[] = [];

  constructor(
    private robotService: RobotService, 
    private webSocketService: WebSocketService,
    private router: Router,
    private cdr: ChangeDetectorRef
  ) {}

  ngOnInit(): void {
    this.webSocketService.listen('syncUpdate').subscribe((data: any) => {
      console.log('Received event:', data);
      this.logs.push(data);
  
      if (data.robot && data.event) {
        const status = this.getStatusMessage(data.event);
        switch (data.robot) {
          case '1':
            this.robot1Status = status;
            console.log('Updated robot1Status to:', this.robot1Status);
            break;
          case '2':
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
    this.robotService.identifyRobot(robotId).subscribe();
  }

  startMission(robotId: number, IsSimu: boolean) {
    this.robotService.startMission(robotId).subscribe();
  }

  stopMission(robotId: number) {
    this.robotService.stopMission(robotId).subscribe();
  }

  returnFromMission(robotId: number) {
    this.robotService.returnFromMission(robotId).subscribe();
  }

  toggleOldLogs() {
    if (!this.showOldLogs) {
      this.robotService.getOldLogs().subscribe((missions) => {
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
