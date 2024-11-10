import { Component } from '@angular/core';
import { CommonModule } from '@angular/common';
import { RobotService } from '@app/services/robot.service';
import { MissionModel, ApiMissionResponse } from '@app/models/mission.model';

@Component({
  selector: 'app-history',
  standalone: true,
  imports: [CommonModule],
  templateUrl: './history.component.html',
  styleUrls: ['./history.component.css']
})
export class HistoryComponent {

  missions: MissionModel[] = [];
  selectedMapData: any; 

  constructor(private robotService: RobotService) {}

  ngOnInit(): void {
    this.robotService.getAllMissions().subscribe(
      (data: ApiMissionResponse) => {
        this.missions = data.missions;
      },
      (error) => {
        console.error('Erreur lors de la récupération des missions :', error);
      }
    );
  }

  onMapClick(missionId: string) {
    const mission = this.missions.find(m => m._id === missionId);
    console.log("Mission sélectionnée:", mission);  
    
    if (mission && mission.mapData) {
      this.selectedMapData = mission.mapData;
      this.drawMap();
    } else {
      console.warn(`Aucune donnée de carte disponible pour la mission avec l'ID : ${missionId}`);
    }
    
  }
  

  private drawMap() {
    console.log("Dessin de la carte avec les données:", this.selectedMapData);
    if (!this.selectedMapData || !this.selectedMapData.info || !this.selectedMapData.data) return;
  
    const canvas = document.getElementById('mapCanvas') as HTMLCanvasElement;
    if (!canvas) return;
  
    const ctx = canvas.getContext('2d')!;
    const { width, height } = this.selectedMapData.info;
    const data = this.selectedMapData.data;
    if (!data || data.length === 0) {
      console.warn("Les données de carte sont vides ou non définies.");
      return;
    }
  
    canvas.width = width;
    canvas.height = height;
  
    const imageData = ctx.createImageData(width, height);
    for (let i = 0; i < data.length; i++) {
      const value = data[i];
      const idx = i * 4;
  
      if (value === -1) { 
        imageData.data[idx] = 128;
        imageData.data[idx + 1] = 128;
        imageData.data[idx + 2] = 128;
        imageData.data[idx + 3] = 255;
      } else if (value === 0) { 
        imageData.data[idx] = 255;
        imageData.data[idx + 1] = 255;
        imageData.data[idx + 2] = 255;
        imageData.data[idx + 3] = 255;
      } else { 
        const intensity = Math.floor(255 * (1 - value / 100));
        imageData.data[idx] = intensity;
        imageData.data[idx + 1] = intensity;
        imageData.data[idx + 2] = intensity;
        imageData.data[idx + 3] = 255;
      }
    }
  
    ctx.putImageData(imageData, 0, 0);
  }
  
  
}
