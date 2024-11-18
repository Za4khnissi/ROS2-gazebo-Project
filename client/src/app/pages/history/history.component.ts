import { Component } from '@angular/core';
import { CommonModule } from '@angular/common';
import { RobotService } from '@app/services/robot.service';
import { MissionModel, ApiMissionResponse } from '@app/models/mission.model';
import { FormsModule } from '@angular/forms';
import { SortingInterface } from './sortingInterface';

@Component({
  selector: 'app-history',
  standalone: true,
  imports: [CommonModule, FormsModule],
  templateUrl: './history.component.html',
  styleUrls: ['./history.component.css']
})
export class HistoryComponent {
  missions: MissionModel[] = [];
  columns: Array<keyof MissionModel> = ['dateDebut', 'isPhysical', 'duration', 'totalDistance', 'mapData'];
  sorting: SortingInterface = { column: 'dateDebut', order: 'desc' };
  selectedMapData: any;
  searchTerm: string = ''; // Search term for filtering

  constructor(private robotService: RobotService) {}

  ngOnInit(): void {
   this.fetchMissions();
  }

  fetchMissions(): void {
    this.robotService.getAllMissions(this.sorting).subscribe(
      (data: ApiMissionResponse) => {
        this.missions = data.missions;
        this.sortMissions();
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

  // Method to filter missions based on searchTerm
  filteredMissions() {
    const searchTerm = this.searchTerm.toLowerCase();
  
    return this.missions.filter(mission => {
      // Convertir chaque valeur en chaîne de caractères pour éviter les erreurs de comparaison
      const dateString = mission.dateDebut ? mission.dateDebut.toString().toLowerCase() : '';
      const isPhysicalString = mission.isPhysical ? 'physique' : 'simulation';
      const durationString = mission.duration ? mission.duration.toString() : '';
      const distanceString = mission.totalDistance ? mission.totalDistance.toString() : '';
  
      // Vérifier si le terme de recherche est contenu dans l'un des champs
      return (
        dateString.includes(searchTerm) ||
        isPhysicalString.includes(searchTerm) ||
        durationString.includes(searchTerm) ||
        distanceString.includes(searchTerm)
      );
    });
  }

  isDescendingSorting(column: string): boolean {
    return this.sorting.column === column && this.sorting.order === 'desc';
  }

  isAscendingSorting(column: string): boolean {
    return this.sorting.column === column && this.sorting.order === 'asc';
  }

  sortTable(column: string): void {
    if (this.sorting.column === column) {
      this.sorting.order = this.sorting.order === 'asc' ? 'desc' : 'asc';
    } else {
      this.sorting.column = column;
      this.sorting.order = 'asc';
    }

    this.fetchMissions();
  }

  sortMissions(): void {
    const { column, order } = this.sorting;
  
    this.missions.sort((a, b) => {
      let valueA: any = a[column as keyof MissionModel];
      let valueB: any = b[column as keyof MissionModel];
  
      // Assurez-vous que les valeurs sont comparables (par exemple, les dates doivent être comparées correctement)
      if (typeof valueA === 'string') {
        valueA = valueA.toLowerCase();
        valueB = valueB.toLowerCase();
      }
  
      if (order === 'asc') {
        return valueA < valueB ? -1 : valueA > valueB ? 1 : 0;
      } else {
        return valueA > valueB ? -1 : valueA < valueB ? 1 : 0;
      }
    });
  }
}
