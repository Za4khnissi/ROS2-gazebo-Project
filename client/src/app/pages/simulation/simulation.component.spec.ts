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
  imports: [NgFor, NgClass, NgIf, MapComponent, BatteryStatusComponent, OctomapComponent, FormsModule],
})
export class SimulationComponent implements OnInit {
  // États des robots
  robotStatus: { [key: number]: string } = { 3: 'Waiting', 4: 'Waiting' };
  simulationStatus = false;

  // Modes de conduite par défaut
  driveModes: { [key: number]: string } = { 3: 'Diff Drive', 4: 'Diff Drive' };

  // Autres propriétés
  logs: any[] = [];
  missions: any[] = [];
  showOldLogs = false;
  is3DView = false;

  // Confirmation d'action
  confirmStopMissionId: number | null = null;

  constructor(
    private simService: RobotService,
    private webSocketService: WebSocketService,
    private router: Router,
    private cdr: ChangeDetectorRef
  ) {}

  ngOnInit(): void {
    this.initializeRobotStatuses();
    this.listenToUpdates();
  }

  private initializeRobotStatuses(): void {
    // Récupère le dernier état des robots
    this.simService.getLastStatus().subscribe((lastStatus) => {
      this.robotStatus[3] = lastStatus['3'] || 'Waiting';
      this.robotStatus[4] = lastStatus['4'] || 'Waiting';
      this.cdr.detectChanges();
    });
  }

  private listenToUpdates(): void {
    // Écoute les mises à jour en temps réel via WebSocket
    this.webSocketService.listen('syncUpdate').subscribe((data: any) => {
      console.log('Received event:', data);

      // Ajoute les logs, sauf certains événements spécifiques
      if (!['drive_mode_changed', 'drive_mode_change_failed'].includes(data.event)) {
        this.logs.push(data);
      }

      // Met à jour le statut du robot correspondant
      if (data.robot && data.event) {
        const status = this.getStatusMessage(data.event);
        this.robotStatus[+data.robot] = status;
        this.cdr.detectChanges();
      }
    });
  }

  private getStatusMessage(event: string): string {
    const statuses: { [key: string]: string } = {
      identifying: 'Identifying',
      identified: 'Identified',
      identification_failed: 'Identification Failed',
      mission_started: 'Moving',
      mission_stopped: 'Stopped',
      mission_failed: 'Mission Start Failed',
      mission_stop_failed: 'Mission Stop Failed',
      robot_returning: 'Returning',
    };
    return statuses[event] || 'Waiting';
  }

  // Actions liées aux robots
  identifyRobot(robotId: number): void {
    this.simService.identifyRobot(robotId).subscribe();
  }

  startMission(robotId: number): void {
    this.simService.startMission(robotId).subscribe();
  }

  stopMission(robotId: number): void {
    this.confirmStopMissionId = robotId; // Demande confirmation
  }

  stopConfirmedMission(): void {
    if (this.confirmStopMissionId !== null) {
      this.simService.stopMission(this.confirmStopMissionId).subscribe({
        next: () => {
          console.log(`Mission arrêtée avec succès pour le Robot ${this.confirmStopMissionId}.`);
          this.confirmStopMissionId = null; // Réinitialise après succès
        },
        error: (err) => {
          console.error(`Erreur lors de l'arrêt de la mission pour le robot ${this.confirmStopMissionId}:`, err);
          this.confirmStopMissionId = null; // Réinitialise même en cas d'erreur
        },
      });
    }
  }

  returnFromMission(robotId: number): void {
    this.simService.returnFromMission(robotId).subscribe();
  }

  toggleDriveMode(robotId: number): void {
    this.driveModes[robotId] = this.driveModes[robotId] === 'Diff Drive' ? 'Ackermann' : 'Diff Drive';
  }

  startRos(): void {
    const payload = {
      driveModes: {
        '3': this.driveModes[3].toLowerCase().replace(' ', '_'),
        '4': this.driveModes[4].toLowerCase().replace(' ', '_'),
      },
    };

    console.log('Payload envoyé:', payload); // Debug
    this.simService.startRos(payload).subscribe({
      next: (response) => {
        console.log('ROS démarré avec succès:', response);
        this.simulationStatus = true; // Met à jour le statut si besoin
      },
      error: (err) => {
        console.error('Échec du démarrage de ROS:', err);
      },
    });
  }

  // Gestion des logs
  toggleOldLogs(): void {
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

  toggleMissionLogs(mission: any): void {
    mission.expanded = !mission.expanded;
  }

  // Vue 2D / 3D
  toggleViewMode(): void {
    this.is3DView = !this.is3DView;
  }

  cancelStopMission(): void {
    this.confirmStopMissionId = null; // Annule l'arrêt
  }
}
