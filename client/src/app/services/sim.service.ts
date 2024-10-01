import { Injectable } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { Observable } from 'rxjs';

@Injectable({
  providedIn: 'root'
})
export class SimulationService {
  private apiUrl = 'http://localhost:3000';

  constructor(private http: HttpClient) {}

  identifyRobot(robotId: number): Observable<{ message: string }> {
    return this.http.get<{ message: string }>(`${this.apiUrl}/robot/${robotId}/identify`);
  }

  startMission(robotId: number): Observable<{ message: string }> {
    return this.http.get<{ message: string }>(`${this.apiUrl}/mission/${robotId}/start`);
  }

  stopMission(robotId: number): Observable<{ message: string }> {
    return this.http.get<{ message: string }>(`${this.apiUrl}/mission/${robotId}/stop`);
  }

  launchSimulation(driveMode3: string, driveMode4: string): Observable<any> {
    return this.http.post(`${this.apiUrl}/start-simulation`, { drive_mode_3: driveMode3, drive_mode_4: driveMode4 });
  }
}
