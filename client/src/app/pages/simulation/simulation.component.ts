import { Component, OnInit, ChangeDetectorRef } from '@angular/core';
import { RobotService } from '@app/services/robot.service';
import { WebSocketService } from '@app/services/web-socket.service';
import { NgFor, NgClass, NgIf } from '@angular/common';
import { MapComponent } from '../../components/map/map.component';
import { Router } from '@angular/router';
import { BatteryStatusComponent } from '../battery-status/battery-status.component';
import { OctomapComponent } from '@app/components/map/octomap.component';
import { FormsModule } from '@angular/forms';

@Component({
  selector: 'app-simulation-robot',
  templateUrl: './simulation.component.html',
  styleUrls: ['./simulation.component.css'],
  standalone: true,
  imports: [NgFor, NgClass, NgIf, MapComponent, BatteryStatusComponent, OctomapComponent, FormsModule]
})
export class SimulationComponent implements OnInit {
  robot1Status = 'Waiting';
  robot2Status = 'Waiting';
  simulationStatus = false;
  driveMode3 = 'Diff Drive'; // Default mode
  driveMode4 = 'Diff Drive'; // Default mode
  selectedDriveModes = { 3: 'Diff Drive', 4: 'Diff Drive' };
  mode: 'simulation' | 'physical' = 'simulation';

  logs: any[] = [];
  showOldLogs = false;
  missions: any[] = [];
  is3DView = false;

  constructor(
    private simService: RobotService, 
    private webSocketService: WebSocketService,
    private router: Router,
    private cdr: ChangeDetectorRef
  ) {}

  ngOnInit(): void {
    this.simService.getLastStatus().subscribe((lastStatus) => {
      this.robot1Status = lastStatus['3'];
      this.robot2Status = lastStatus['4'];
      this.cdr.detectChanges(); // Ensure view updates with the initial statuses
  
      // Start listening for real-time updates
      this.webSocketService.listen('syncUpdate').subscribe((data: any) => {
        console.log('Received event:', data);
  
        if (data.event !== 'drive_mode_changed' && data.event !== 'drive_mode_change_failed') {
          this.logs.push(data);
        }
  
        if (data.robot && data.event) {
          const status = this.getStatusMessage(data.event);
          switch (data.robot) {
            case '3':
              this.robot1Status = status;
              break;
            case '4':
              this.robot2Status = status;
              break;
          }
          this.cdr.detectChanges();
        }
      });
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

  returnFromMission(robotId: number) {
    this.simService.returnFromMission(robotId).subscribe();
  }

  toggleDriveMode(robotId: number) {
    let newDriveMode = robotId === 3 ? this.driveMode3 : this.driveMode4;
    // eslint-disable-next-line @typescript-eslint/no-unused-vars
    newDriveMode = newDriveMode === 'Diff Drive' ? 'Ackermann' : 'Diff Drive';
  }

  startRos() {
    const payload: { driveModes: Record<string, string> } = {
      driveModes: {
        '3': this.selectedDriveModes[3].toLowerCase().replace(' ', '_'),
        '4': this.selectedDriveModes[4].toLowerCase().replace(' ', '_'),
      },
    };
  
    console.log('Payload being sent to the server:', payload); // Log payload for debugging
  
    this.simService.startRos(payload).subscribe({
      next: (response) => {
        console.log('ROS started successfully:', response);
        this.simulationStatus = true; // Update simulation status if needed
      },
      error: (err) => {
        console.error('Failed to start ROS:', err);
      },
    });
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

  toggleViewMode() {
    this.is3DView = !this.is3DView;
  }
}
