export interface ApiMissionResponse {
    statusCode: number;
    missions: MissionModel[];
  }
  
  export interface MissionModel {
    id: string; 
    dateDebut: Date; 
    dateFin: Date;
    duration: number;
    robots: string[]; 
    isPhysical: boolean; 
    totalDistance: number; 
    createdAt: Date; 
    updatedAt: Date; 
  }
  