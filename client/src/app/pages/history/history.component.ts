import { Component } from '@angular/core';
import { CommonModule } from '@angular/common';
import { RobotService } from '@app/services/robot.service';
import {MissionModel, ApiMissionResponse } from '@app/models/mission.model';

@Component({
  selector: 'app-history',
  standalone: true,
  imports: [CommonModule],
  templateUrl: './history.component.html',
  styleUrl: './history.component.css'
})
export class HistoryComponent {

  missions: MissionModel[] = [];
  constructor(private robotService: RobotService) {}
  
  ngOnInit(): void {
    this.robotService.getAllMissions().subscribe(
      (data: ApiMissionResponse) => {
        this.missions = data.missions; 
        //console.log('Missions récupérées :', this.missions);
      },
      (error) => {
        console.error('Erreur lors de la récupération des missions :', error);
      }
    );
  }

}
