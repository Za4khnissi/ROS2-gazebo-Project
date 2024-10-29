import { Component, OnInit } from '@angular/core';
import { RobotService } from '@app/services/robot.service';
import { WebSocketService } from '@app/services/web-socket.service';
import { NgFor, NgClass, NgIf } from '@angular/common';

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

  constructor(private robotService: RobotService, private webSocketService: WebSocketService) {}

  ngOnInit(): void {
    this.robotService.listenForLogs().subscribe((log) => {
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
    }
  }

  identifyRobot(robotId: number) {
    this.robotService.identifyRobot(robotId).subscribe();
  }

  startMission(robotId: number) {
    this.robotService.startMission(robotId).subscribe();
  }

  stopMission(robotId: number) {
    this.robotService.stopMission(robotId).subscribe();
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
